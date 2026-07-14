// OpenOBD — CAN-Bus-Datenlogger (VW Golf 8.5 eTSI)  —  v1: Maximum-Capture
// Basis: github.com/roypeter/esp32-obd2-logger, stark umgebaut.
// Zielhardware: LilyGO T-CAN485 (ESP32 + SN65HVD231 + microSD).
//
// v1-Philosophie: SO VIELE DATEN WIE MOEGLICH. Der Logger
//   1) pollt aktiv die Standard-OBD2-PIDs (dekodiert -> saubere CSV + Dashboard)
//   2) schneidet PASSIV JEDES empfangene CAN-Frame mit (Roh-Log, SavvyCAN-Format)
//   3) zeigt live im Browser jede gesehene CAN-ID (CAN-Explorer)
//
// Bereits erledigt ggü. Original: OLED raus, Gang-Schaetzung raus (Auto liefert
// den Gang selbst), alle Antwort-IDs 0x7E8..0x7EF, CAN-Bus-Off-Recovery,
// Auto-Record beim Boot, Roh-Mitschnitt (Tx UND Rx, 64-bit-Zeitstempel),
// Live-Explorer, Datei-Schluss nach Write, PID-Discovery (+ Re-Run per Button),
// Handy-Zeit-Sync + Time Hunt, Sessions mit Pagination + Loeschen,
// Dashboard zeigt nur Werte, die wirklich empfangen wurden.
//
// Threading-Modell: NUR loop() (Core 1) schreibt rawBuf und Session-Dateien.
// Der Webserver (Core 0) stoesst Session-Wechsel/Re-Discovery ueber Flags an;
// saemtliche SD-Zugriffe laufen unter fsMutex.
//
// Noch offen (naechste Schritte, siehe README): JSON-Config + Settings-Seite,
// datengetriebene Dashboards aus dem Roh-Log, Anchor-Aufloesung beim Download.

#include <Arduino.h>
#include <WiFi.h>
#include <NimBLEDevice.h> // BLE-Broadcast (Status + Werte) statt Webserver
#include <esp_system.h>  // esp_restart() fuer den Software-Watchdog
#include <SD.h>
#include <LittleFS.h>
#include <SPI.h>
#include "config.h"      // Konfiguration im Projektordner (wird beim Flashen eingebaut)
#include <PubSubClient.h> // Dev: MQTT-Broadcast der Roh-Frames (optional)
#include "driver/twai.h"
#include "esp_timer.h"   // 64-bit-Mikrosekunden: kein Ueberlauf nach 71 min (anders als micros())
#include <time.h>

// ===== LilyGO T-CAN485 GPIO-Map (offiz. Repo Xinyuan-LilyGO/T-CAN485) =============
#define PIN_5V_EN    16   // MUSS HIGH: 5V-Boost fuer CAN-Transceiver + SD-Slot
#define CAN_TX_PIN   27   // per Boot-Selbsttest bestaetigt — die config.h-Angabe 26/27 war vertauscht!
#define CAN_RX_PIN   26
#define CAN_SE_PIN   23   // LOW = normaler High-Speed-Modus
#define SD_SCLK_PIN  14
#define SD_MISO_PIN  2
#define SD_MOSI_PIN  15
#define SD_CS_PIN    13
// DS3231 kommt spaeter an I2C — Default-Pins, beim Anloeten ggf. anpassen:
#define RTC_SDA_PIN  32
#define RTC_SCL_PIN  33
// =================================================================================

// --- Konfiguration: kommt aus include/config.h (Compile-Zeit), NICHT von SD ---
#define RESPONSE_TIMEOUT_MS 50
#define SLOW_POLL_RATIO 5                  // 1 langsame PID je 5 Polls
struct Config {
  bool devMode       = CFG_DEV_MODE;
  bool webUi         = CFG_WEB_UI;
  bool led           = CFG_LED;
  bool autoRecord    = CFG_AUTO_RECORD;
  bool logRaw        = CFG_LOG_RAW;
  bool logDecoded    = CFG_LOG_DECODED;
  int  logIntervalMs = CFG_LOG_INTERVAL_MS;
  bool discovery     = CFG_DISCOVERY;
  char wifiMode[6]   = CFG_WIFI_MODE;
  char ntp[48]       = CFG_NTP;
  char tz[40]        = CFG_TZ;
  bool streamTcp     = CFG_STREAM_TCP;
  int  tcpPort       = CFG_TCP_PORT;
  bool mqttEn        = CFG_MQTT;
  char mqttHost[48]  = CFG_MQTT_HOST;
  int  mqttPort      = CFG_MQTT_PORT;
  char mqttTopic[40] = CFG_MQTT_TOPIC;
} cfg;

SemaphoreHandle_t fsMutex;
volatile uint32_t lastLoopTick = 0;   // Software-Watchdog: loop() stempelt hier, wdtTask prueft

// Storage-Abstraktion — SD bevorzugt, LittleFS als Fallback
FS* storage = nullptr;
bool useSD = false;
uint64_t storageTotalBytes() { return useSD ? SD.totalBytes() : LittleFS.totalBytes(); }
uint64_t storageUsedBytes()  { return useSD ? SD.usedBytes()  : LittleFS.usedBytes();  }

// --- Aktiv gepollte Standard-PIDs ---
const uint8_t fast_pids[] = {
  0x0C, 0x0D, 0x0B, 0x04, 0x0E, 0x47,
  0x5A, 0x4C, 0x61, 0x62,
  0x06, 0x07, 0x15, 0x44, 0x56,
  0x5E, 0x34, 0x43,
  0x11, 0x45, 0xA4        // Drossel, Rel-Drossel, Getriebe-Ist-Gang (0xA4 nur mitpollen -> Roh-Log)
};
const int NUM_FAST = sizeof(fast_pids) / sizeof(fast_pids[0]);
const uint8_t slow_pids[] = { 0x05, 0x0F, 0x33, 0x3C, 0x5C, 0x2F, 0x42, 0x46, 0x63, 0x2E,
                              0x1F, 0x31, 0x51, 0x53, 0xA6 };  // Laufzeit, Strecke, Kraftstoffart, Evap-Druck, Kilometerstand
const int NUM_SLOW = sizeof(slow_pids) / sizeof(slow_pids[0]);

// --- Per-PID-Timeout-Tracking / Auto-Disable ---
#define MAX_PIDS 48
#define PID_DISABLE_THRESHOLD 10
struct PidStats {
  uint8_t pid;
  unsigned long timeouts;
  unsigned long responses;
  uint8_t consecutiveTimeouts;
  bool disabled;
};
PidStats pidStats[MAX_PIDS];
int numTrackedPids = 0;

PidStats* getPidStats(uint8_t pid) {
  for (int i = 0; i < numTrackedPids; i++)
    if (pidStats[i].pid == pid) return &pidStats[i];
  if (numTrackedPids < MAX_PIDS) {
    PidStats* s = &pidStats[numTrackedPids++];
    s->pid = pid; s->timeouts = 0; s->responses = 0;
    s->consecutiveTimeouts = 0; s->disabled = false;
    return s;
  }
  return nullptr;
}
bool isPidDisabled(uint8_t pid) { PidStats* s = getPidStats(pid); return s && s->disabled; }

// --- PID-Discovery: welche Standard-PIDs unterstuetzt DIESES Auto wirklich? ---
bool pidSupported[256];
bool pidSeen[256];            // hat dieser PID je geantwortet? (sonst zeigt das Dashboard "--")
bool discoveryDone = false;   // false = keine Auto-Antwort -> Fallback auf bekannte Liste
int  numSupported = 0;

bool canDecode(uint8_t pid) {  // koennen wir den PID schon in Klartext uebersetzen?
  for (int i = 0; i < NUM_FAST; i++) if (fast_pids[i] == pid) return true;
  for (int i = 0; i < NUM_SLOW; i++) if (slow_pids[i] == pid) return true;
  return false;
}

// Alle PID-Sperren/Statistiken zuruecksetzen. Wichtig nach einem Adress-/Bitraten-
// Wechsel: PIDs, die unter der FALSCHEN Konfig als tot markiert wurden, muessen
// unter der neuen Konfig wieder gepollt werden (sonst nur noch PID 0x00-Spam).
void resetPidStats() { numTrackedPids = 0; for (int i = 0; i < 256; i++) pidSeen[i] = false; }

// Ist dieses Frame eine OBD-Antwort? 11 bit: 0x7E8-0x7EF. 29 bit: 0x18DAF1xx
// (Antwort irgendeines Steuergeraets an Tester-Adresse 0xF1).
bool isObdResponse(const twai_message_t &rx) {
  if (!rx.extd) return rx.identifier >= 0x7E8 && rx.identifier <= 0x7EF;
  return (rx.identifier >> 8) == 0x18DAF1;
}

// --- Zeit: Handy-Sync + Anker (kein DS3231 noetig) ---------------------------------
// Jede Log-Zeile traegt millis() (monotone Uptime). Schickt das Handy beim Oeffnen des
// Dashboards die Echtzeit, merken wir uns EINEN Anker (Uptime <-> Echtzeit); damit ist
// die reale Zeit JEDER Zeile der Session bestimmbar -- auch der Zeilen VOR dem Sync.
// Der Anker liegt als Sidecar /data/sNNN.anchor (Roh-/Decoded-CSV bleiben sauber).
bool timeSynced = false;
long long anchorEpochMs = 0;       // reale Zeit (UTC, ms) zum Ankerzeitpunkt
unsigned long anchorUptimeMs = 0;  // millis() zum Ankerzeitpunkt
// Lokalzeit kommt aus der POSIX-TZ (cfg.tz) via setenv("TZ")+localtime_r -> autom. Sommer-/Winterzeit
char anchorFile[40];
long long lastKnownEpochMs = 0;   // letzte bekannte Echtzeit (aus /lasttime.txt) — Ordner-Datierung ohne RTC

long long nowEpochMs() { return timeSynced ? anchorEpochMs + (long long)(millis() - anchorUptimeMs) : 0; }

void localNow(struct tm &t) {
  time_t utc = (time_t)(nowEpochMs() / 1000);
  localtime_r(&utc, &t);   // wendet die per setenv("TZ") gesetzte Zone inkl. DST an
}

void clockString(char* buf, size_t n) {
  if (timeSynced) { struct tm t; localNow(t); strftime(buf, n, "%Y-%m-%d %H:%M:%S", &t); }
  else {
    unsigned long s = millis() / 1000;
    snprintf(buf, n, "REL_D%lu %02lu:%02lu:%02lu", s/86400+1, (s%86400)/3600, (s%3600)/60, s%60);
  }
}

void writeTimeAnchor() {  // Sidecar zur aktuellen Session
  if (!storage || anchorFile[0] == 0) return;
  if (xSemaphoreTake(fsMutex, pdMS_TO_TICKS(50))) {
    File f = storage->open(anchorFile, "w");
    if (f) { f.printf("uptime_ms=%lu\nepoch_ms=%lld\n", anchorUptimeMs, anchorEpochMs); f.close(); }
    xSemaphoreGive(fsMutex);
  }
}

// --- Roh-Mitschnitt: Tabelle aller je gesehenen CAN-IDs (fuer den Live-Explorer) ---
#define MAX_FRAMES 160
struct FrameInfo {
  uint32_t id;
  bool ext;
  uint8_t dlc;
  uint8_t data[8];
  uint32_t count;
  unsigned long firstMs;
  unsigned long lastMs;
};
FrameInfo frames[MAX_FRAMES];
int numFrames = 0;
unsigned long totalFramesSeen = 0;

FrameInfo* getOrAddFrame(uint32_t id, bool ext) {
  for (int i = 0; i < numFrames; i++)
    if (frames[i].id == id && frames[i].ext == ext) return &frames[i];
  if (numFrames < MAX_FRAMES) {
    FrameInfo* f = &frames[numFrames++];
    f->id = id; f->ext = ext; f->dlc = 0; f->count = 0;
    f->firstMs = millis(); f->lastMs = f->firstMs;
    for (int i = 0; i < 8; i++) f->data[i] = 0;
    return f;
  }
  return nullptr;  // Tabelle voll — nur noch totalFramesSeen zaehlt
}

// --- Roh-Log-Puffer: FIXER char-Buffer statt String -> keine Heap-Fragmentierung
//     bei stundenlangem Betrieb. Gebatcht auf SD geschrieben (SavvyCAN-CSV). ---
char rawBuf[4096];
size_t rawLen = 0;
const size_t RAW_FLUSH_BYTES = 3072;      // ab hier auf SD schreiben (Rest = Sicherheitsmarge)
bool storageFull = false;                 // gesetzt wenn Karte fast voll -> sauberer Logging-Stopp
uint64_t cachedFreeBytes = 0;             // freier Platz, nur alle 10s aktualisiert (usedBytes() ist teuer!)
unsigned long lastFreeCheck = 0;
unsigned long rawLinesDropped = 0;        // gezaehlte verworfene Roh-Zeilen (Puffer/Storage voll)

// --- Zuletzt dekodierte Werte ---
int rpm=0, speed_kmh=0, coolant_temp=0, manifold_kpa=0, intake_temp=0, engine_load=0;
float timing_advance=0; int throttle_pos=0, accel_pedal=0, cmd_throttle=0, demand_torque=0, actual_torque=0;
float short_fuel_trim=0, long_fuel_trim=0; int baro_kpa=0;
float o2_s2_voltage=0, o2_s2_stft=0, cmd_equiv_ratio=0; int catalyst_temp=0; float o2_secondary_ltft=0;
int oil_temp=0, fuel_level=0; float fuel_rate=0, module_voltage=0; int ambient_temp=0, ref_torque=0;
float o2s1_lambda=0, o2s1_current=0, absolute_load=0; int evap_purge=0;
float totalDistKm=0, totalFuelL=0;   // in die CSV aufgenommen (Baustelle #5 des Originals)
// Harvest-Neuzugaenge (2026-07-13): weitere Standard-PIDs
int throttle11=0, rel_throttle=0, fuel_type=-1, gear=0;
unsigned long runtime_s=0, dist_clr=0;
float evap_press=0, odometer_km=0;
unsigned long lastLog = 0;

// --- Session / Aufzeichnung ---
int sessionNum = 0;
char decodedFile[40];   // /data/YYYY/MM/s001.csv
char rawFile[40];       // /dev/YYYY/MM/raw_s001.csv
volatile bool recording = true;             // Auto-Record; per Dashboard pausierbar
volatile bool newSessionRequested = false;  // vom Webserver (Core 0) gesetzt, von loop() (Core 1) ausgefuehrt
volatile bool rediscoverRequested = false;  // dito — Single-Writer-Prinzip, s. Kopfkommentar
volatile int  requestedCanMode = -1;        // vom Webserver: 0=Normal, 1=Nur-Lauschen; -1 = nichts angefordert
int  currentCanMode = 0;                    // aktiver CAN-Modus
bool canSelfTestOk = false;                 // Boot-Selbsttest: Controller + Transceiver intakt?
int  canSeWorking = LOW;                    // welche CAN_SE-Polaritaet der Transceiver braucht (Boardrevision)
gpio_num_t canTxPin = (gpio_num_t)CAN_TX_PIN;  // vom Selbsttest bestaetigte Pin-Belegung
gpio_num_t canRxPin = (gpio_num_t)CAN_RX_PIN;  // (Quellen widersprechen sich: 26/27 vs 27/26)
uint32_t obdReqId = 0x7DF;                     // OBD-Anfrageadresse: 0x7DF funktional ODER 0x7E0 physisch (Motor-STG)
uint8_t  obdPad = 0x00;                        // Fuellbyte fuer Bytes 3-7 (manche VAG-STG wollen 0x55)
int      canBitrateK = 500;                    // aktive Bitrate in kbit/s (500 Standard, 250 Alternative)
volatile bool probeRequested = false;          // Auto-Probe vom Webserver angefordert, loop() fuehrt aus
const char* CSV_HEADER = "Timestamp_ms,RPM,Speed_kmh,Coolant_C,OilTemp_C,MAP_kPa,IAT_C,Load_pct,TimingAdv_deg,Throttle_pct,AccelPedal_pct,CmdThrottle_pct,DemandTorque_pct,ActualTorque_pct,STFT_pct,LTFT_pct,Baro_kPa,O2S2_V,O2S2_STFT_pct,CmdEquivRatio,CatalystTemp_C,O2_SecLTFT_pct,FuelLevel_pct,FuelRate_Lph,ModuleVoltage_V,AmbientTemp_C,RefTorque_Nm,O2S1_Lambda,O2S1_Current_mA,AbsLoad_pct,EvapPurge_pct,TotalDist_km,TotalFuel_L,Throttle11_pct,RelThrottle_pct,RunTime_s,DistSinceClear_km,FuelType,EvapPress_kPa,Odometer_km,Gear";
const char* RAW_HEADER = "Time Stamp,ID,Extended,Dir,Bus,LEN,D1,D2,D3,D4,D5,D6,D7,D8";  // SavvyCAN

// --- Poll-State ---
bool requestPending = false;
unsigned long requestSentTime = 0;
uint8_t pendingPid = 0;
int pollCounter = 0, fastIndex = 0, slowIndex = 0;

// --- Debug-Ringpuffer ---
#define LOG_LINES 50
#define LOG_LINE_LEN 120
char logBuf[LOG_LINES][LOG_LINE_LEN];
int logIndex = 0;
unsigned long txCount = 0, rxCount = 0, rxTimeoutCount = 0;
unsigned long lastStatusLog = 0;
unsigned long txFailCount = 0, lastTxFailLog = 0;
unsigned long txBackoffUntil = 0;   // nach TX-Fehler kurz Pause statt Dauerfeuer

void addLog(const char* fmt, ...) {
  char* line = logBuf[logIndex % LOG_LINES];
  int off = snprintf(line, LOG_LINE_LEN, "[%lu] ", millis());
  va_list args; va_start(args, fmt);
  vsnprintf(line + off, LOG_LINE_LEN - off, fmt, args);
  va_end(args);
  Serial.println(line);
  logIndex++;
}

// --- Boot-Schreibtest: deckt defekte, schreibgeschuetzte oder falsch
//     formatierte Karten SOFORT auf, statt still nichts aufzuzeichnen ---
// --- Ordner-Helfer: legt einen ganzen Pfad an (jede Ebene einzeln; FAT legt nicht rekursiv an) ---
void ensureDirTree(const char* full) {
  if (!storage) return;
  char tmp[48]; strlcpy(tmp, full, sizeof(tmp));
  for (char* p = tmp + 1; *p; p++) {
    if (*p == '/') { *p = 0; if (!storage->exists(tmp)) storage->mkdir(tmp); *p = '/'; }
  }
  if (!storage->exists(tmp)) storage->mkdir(tmp);
}

// --- Jahr/Monat der besten bekannten Zeit -> "YYYY/MM" (sonst "unsorted") ---
void currentYM(char* buf, size_t n) {
  long long e = timeSynced ? nowEpochMs() : lastKnownEpochMs;
  if (e <= 0) { strlcpy(buf, "unsorted", n); return; }
  time_t t = (time_t)(e / 1000);
  struct tm tmv; localtime_r(&t, &tmv);   // lokale Zeit inkl. DST
  snprintf(buf, n, "%04d/%02d", tmv.tm_year + 1900, tmv.tm_mon + 1);
}

// --- Letzte bekannte Echtzeit persistieren/laden (Ersatz fuer fehlende RTC) ---
void saveLastTime() {
  if (!storage || !timeSynced) return;
  if (xSemaphoreTake(fsMutex, pdMS_TO_TICKS(50))) {
    File f = storage->open("/lasttime.txt", "w");
    if (f) { f.printf("epoch_ms=%lld\n", nowEpochMs()); f.close(); }
    xSemaphoreGive(fsMutex);
  }
}
void loadLastTime() {
  if (!storage || !storage->exists("/lasttime.txt")) return;
  if (xSemaphoreTake(fsMutex, pdMS_TO_TICKS(300))) {
    File f = storage->open("/lasttime.txt", "r");
    if (f) {
      while (f.available()) {
        String line = f.readStringUntil('\n');
        if (line.startsWith("epoch_ms=")) lastKnownEpochMs = atoll(line.substring(9).c_str());
      }
      f.close();
    }
    xSemaphoreGive(fsMutex);
  }
  if (lastKnownEpochMs > 0) addLog("Letzte bekannte Zeit geladen (provisorisch bis Sync)");
}

// --- Boot-Schreibtest + Basisordner anlegen (/data = sauber, /dev = roh) ---
bool ensureDataDirWritable() {
  if (!storage) return false;
  if (!storage->exists("/data")) storage->mkdir("/data");
  if (!storage->exists("/dev"))  storage->mkdir("/dev");
  File tf = storage->open("/.wtest", "w");
  if (!tf) { addLog("FEHLER: Schreibtest open fehlgeschlagen"); return false; }
  size_t n = tf.print("ok");
  tf.close();
  storage->remove("/.wtest");
  if (n != 2) { addLog("FEHLER: Schreibtest write fehlgeschlagen"); return false; }
  addLog("Speicher-Schreibtest OK");
  return true;
}

// --- Session anlegen: saubere Daten in data/YYYY/MM, Roh-Daten in dev/YYYY/MM.
//     Zaehler /session.txt + Index /sessions.csv liegen im Root. ---
void openNewSession() {
  if (!storage) return;
  if (xSemaphoreTake(fsMutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
    addLog("Session-Wechsel abgebrochen: Speicher belegt");
    return;
  }
  int fileNum = 0;   // Zaehler aus Root; Migration: altes /data/session.txt weiter nutzen
  if (storage->exists("/session.txt")) { File sf = storage->open("/session.txt", "r"); if (sf) { fileNum = sf.parseInt(); sf.close(); } }
  else if (storage->exists("/data/session.txt")) { File sf = storage->open("/data/session.txt", "r"); if (sf) { fileNum = sf.parseInt(); sf.close(); } }
  sessionNum = fileNum + 1;
  { File sf = storage->open("/session.txt", "w"); if (sf) { sf.print(sessionNum); sf.close(); } }

  char ym[16]; currentYM(ym, sizeof(ym));
  char dataDir[32], devDir[32];
  snprintf(dataDir, sizeof(dataDir), "/data/%s", ym);
  snprintf(devDir,  sizeof(devDir),  "/dev/%s",  ym);
  ensureDirTree(dataDir); ensureDirTree(devDir);
  snprintf(decodedFile, sizeof(decodedFile), "%s/s%03d.csv",     dataDir, sessionNum);
  snprintf(rawFile,     sizeof(rawFile),     "%s/raw_s%03d.csv", devDir,  sessionNum);
  snprintf(anchorFile,  sizeof(anchorFile),  "%s/s%03d.anchor",  dataDir, sessionNum);
  if (cfg.logDecoded) { File f1 = storage->open(decodedFile, "w"); if (f1) { f1.println(CSV_HEADER); f1.close(); } }
  if (cfg.logRaw)     { File f2 = storage->open(rawFile, "w");     if (f2) { f2.println(RAW_HEADER); f2.close(); } }
  { File mf = storage->open("/sessions.csv", "a");   // Index: num,epoch_ms,decodedPfad,rawPfad
    if (mf) { mf.printf("%d,%lld,%s,%s\n", sessionNum, timeSynced ? nowEpochMs() : lastKnownEpochMs, decodedFile, rawFile); mf.close(); } }
  xSemaphoreGive(fsMutex);
  if (timeSynced) writeTimeAnchor();   // nimmt den Mutex selbst — daher NACH der Freigabe
  addLog("Session %d -> %s", sessionNum, decodedFile);
}

// --- Roh-Puffer auf SD schreiben ---
void flushRaw() {
  if (rawLen == 0) return;
  if (!storage || storageFull) { rawLen = 0; return; }   // voll -> Puffer verwerfen statt blind schreiben
  if (xSemaphoreTake(fsMutex, pdMS_TO_TICKS(50))) {
    File f = storage->open(rawFile, "a");
    if (f) { f.write((const uint8_t*)rawBuf, rawLen); f.close(); rawLen = 0; }
    xSemaphoreGive(fsMutex);
  }
}

// Freien Speicher aktualisieren — teuer (usedBytes() scannt die FAT), daher NUR
// alle 10s aus loop() aufrufen. Setzt storageFull, um Logging sauber zu stoppen.
void checkFreeSpace() {
  if (!storage) { storageFull = true; return; }
  uint64_t total = storageTotalBytes(), used = storageUsedBytes();
  cachedFreeBytes = (total > used) ? total - used : 0;
  bool full = cachedFreeBytes < 262144;   // <256 KB Reserve = voll
  if (full && !storageFull) addLog("SPEICHER FAST VOLL (%llu KB frei) -> Logging gestoppt", cachedFreeBytes / 1024);
  if (!full && storageFull) addLog("Speicher wieder frei -> Logging laeuft");
  storageFull = full;
}

// Eine Zeile ins Roh-Log — Rx UND Tx (Request/Response-Paare sind fuers
// Reverse-Engineering Gold wert). Zeitstempel 64 bit: kein Ueberlauf nach 71 min.
// Nur aus loop()/setup() aufrufen (Single-Writer, s. Kopfkommentar).
void logRawLine(uint32_t id, bool ext, uint8_t dlc, const uint8_t* data, bool tx) {
  if (!cfg.logRaw || !recording || !storage || storageFull) return;   // nichts puffern, wenn nicht geschrieben wird
  char line[96];
  int o = snprintf(line, sizeof(line), "%lld,%X,%d,%s,0,%d",
                   (long long)esp_timer_get_time(), id, ext ? 1 : 0, tx ? "Tx" : "Rx", dlc);
  for (int i = 0; i < dlc && i < 8; i++) o += snprintf(line+o, sizeof(line)-o, ",%02X", data[i]);
  for (int i = dlc; i < 8; i++)          o += snprintf(line+o, sizeof(line)-o, ",");
  if (o < 0 || o > (int)sizeof(line) - 2) return;       // Schutz vor snprintf-Ueberlauf
  line[o++] = '\n';
  if (rawLen + (size_t)o > sizeof(rawBuf)) flushRaw();   // Platz schaffen
  if (rawLen + (size_t)o > sizeof(rawBuf)) { rawLinesDropped++; return; }  // immer noch voll -> droppen
  memcpy(rawBuf + rawLen, line, o); rawLen += o;
  if (rawLen >= RAW_FLUSH_BYTES) flushRaw();
}


// ---- Dev-Streaming: Roh-Frames drahtlos (TCP-Zeilen fuer Python/SavvyCAN-Capture, optional MQTT).
// Nur opt-in (config dev.*). loop() (Core 1) fuellt eine Queue, DIESER Task (Core 0) sendet -> keine
// Netz-I/O im CAN-Pfad. Bei Ueberlauf werden Frames verworfen (Dev-Mitschnitt, unkritisch). ----
struct StreamFrame { int64_t ts; uint32_t id; uint8_t ext, dlc, tx; uint8_t d[8]; };
QueueHandle_t streamQ = nullptr;
WiFiClient    mqttNet;
PubSubClient  mqtt(mqttNet);

void streamTask(void *pv) {
  WiFiServer *tcp = nullptr;
  WiFiClient  cli;
  if (cfg.streamTcp) { tcp = new WiFiServer(cfg.tcpPort); tcp->begin(); addLog("Dev-TCP-Stream: Port %d", cfg.tcpPort); }
  if (cfg.mqttEn && cfg.mqttHost[0]) { mqtt.setServer(cfg.mqttHost, cfg.mqttPort); mqtt.setSocketTimeout(1); }
  unsigned long lastMqtt = 0;
  for (;;) {
    if (tcp && (!cli || !cli.connected())) { WiFiClient nc = tcp->available(); if (nc) cli = nc; }
    if (cfg.mqttEn && cfg.mqttHost[0] && !mqtt.connected() && millis() - lastMqtt > 3000) {
      lastMqtt = millis(); mqtt.connect("openobd");
    }
    if (mqtt.connected()) mqtt.loop();
    StreamFrame fr; int budget = 250;
    while (budget-- > 0 && streamQ && xQueueReceive(streamQ, &fr, 0) == pdTRUE) {
      char line[80];
      int o = snprintf(line, sizeof(line), "%lld,%X,%d,%s,0,%d",
                       (long long)fr.ts, fr.id, fr.ext ? 1 : 0, fr.tx ? "Tx" : "Rx", fr.dlc);
      for (int i = 0; i < fr.dlc && i < 8; i++) o += snprintf(line + o, sizeof(line) - o, ",%02X", fr.d[i]);
      if (o > (int)sizeof(line) - 2) o = sizeof(line) - 2;
      line[o++] = '\n'; line[o] = 0;
      if (cli && cli.connected() && cli.availableForWrite() >= o) cli.write((const uint8_t*)line, o);
      if (mqtt.connected()) mqtt.publish(cfg.mqttTopic, line);
    }
    vTaskDelay(2);
  }
}

// ---------- Vorwaerts-Deklarationen (Arduino generiert fuer .cpp keine Prototypen) ----------
bool initCan(int mode);
void reinitCan(int mode);
bool probeSupportedPids();
void discoverSupportedPids();
void writePidsFile();
void captureFrame(twai_message_t &rx);
bool queryPidMask(uint8_t basePid, uint32_t &mask);





















// ---------- PID-Discovery ----------
void captureFrame(twai_message_t &rx);   // vorwaerts deklariert — Discovery schneidet auch mit

// Fragt Service-01-PID `basePid` ab und liefert die 32-Bit-Unterstuetzungsmaske.
bool queryPidMask(uint8_t basePid, uint32_t &mask) {
  twai_message_t rx;
  while (twai_receive(&rx, 0) == ESP_OK) captureFrame(rx);  // Queue leeren, aber NICHTS wegwerfen
  twai_message_t m = {};
  m.identifier = obdReqId; m.extd = (obdReqId > 0x7FF); m.data_length_code = 8;
  m.data[0] = 0x02; m.data[1] = 0x01; m.data[2] = basePid;
  for (int i = 3; i < 8; i++) m.data[i] = obdPad;
  if (twai_transmit(&m, pdMS_TO_TICKS(50)) != ESP_OK) return false;
  logRawLine(m.identifier, m.extd, 8, m.data, true);
  unsigned long start = millis();
  while (millis() - start < 300) {
    if (twai_receive(&rx, pdMS_TO_TICKS(20)) == ESP_OK) {
      captureFrame(rx);
      if (isObdResponse(rx) && rx.data[1] == 0x41 && rx.data[2] == basePid) {
        mask = ((uint32_t)rx.data[3] << 24) | ((uint32_t)rx.data[4] << 16) |
               ((uint32_t)rx.data[5] << 8)  |  (uint32_t)rx.data[6];
        return true;
      }
    }
  }
  return false;
}

// Eine Discovery-Runde mit der aktuell gesetzten obdReqId. Gibt true zurueck,
// wenn das Auto ueberhaupt geantwortet hat (mind. ein Bereich beantwortet).
bool probeSupportedPids() {
  for (int i = 0; i < 256; i++) pidSupported[i] = false;
  numSupported = 0;
  bool found = false;
  const uint8_t bases[] = { 0x00, 0x20, 0x40, 0x60, 0x80, 0xA0, 0xC0, 0xE0 };
  for (uint8_t bi = 0; bi < 8; bi++) {
    uint8_t base = bases[bi];
    uint32_t mask = 0; bool ok = false;
    for (int r = 0; r < 3 && !ok; r++) ok = queryPidMask(base, mask);
    if (!ok) { if (base == 0x00) break; else continue; }
    found = true;
    for (int bit = 0; bit < 32; bit++) {
      int p = base + 1 + bit;                    // 0xE0+32 waere 256 -> Bounds-Check zwingend
      if (p < 256 && (mask & (1UL << (31 - bit)))) { pidSupported[p] = true; numSupported++; }
    }
    if (!(mask & 0x1UL)) break;       // naechster Bereich nicht angekuendigt -> fertig
  }
  return found;
}

void writePidsFile() {
  if (storage && xSemaphoreTake(fsMutex, pdMS_TO_TICKS(500))) {  // lesbare Liste auf SD (Briefing #1)
    File f = storage->open("/pids.txt", "w");
    if (f) {
      f.printf("# OpenOBD — Anfrage-ID 0x%X @ %dk — unterstuetzte Service-01-PIDs\n", obdReqId, canBitrateK);
      for (int p = 1; p < 256; p++)
        if (pidSupported[p]) f.printf("0x%02X%s\n", p, canDecode((uint8_t)p) ? "  (dekodiert)" : "  (roh)");
      f.close();
    }
    xSemaphoreGive(fsMutex);
  }
}

void discoverSupportedPids() {
  discoveryDone = false;
  resetPidStats();   // alte Sperren loeschen — sonst blieben unter falscher Konfig getoetete PIDs tot
  // 0x7DF=11bit funktional, 0x7E0=11bit physisch, 0x18DB33F1=29bit funktional (VW Golf 8.5!)
  uint32_t tryIds[] = { obdReqId, 0x7DF, 0x7E0, 0x18DB33F1 };
  for (int i = 0; i < 4; i++) {
    uint32_t rid = tryIds[i];
    if (i > 0 && rid == tryIds[0]) continue;   // Duplikat der bereits probierten ID ueberspringen
    obdReqId = rid;
    addLog("Discovery: probiere Anfrage-ID 0x%X ...", rid);
    if (probeSupportedPids()) {
      discoveryDone = true;
      resetPidStats();   // frischer Start fuers Polling mit der gefundenen Konfig
      addLog("Discovery OK auf 0x%X — %d PIDs unterstuetzt", rid, numSupported);
      break;
    }
  }
  if (!discoveryDone) addLog("Discovery: keine Antwort -> Fallback-Liste (Auto-Probe versuchen!)");
  writePidsFile();
}

// ---------- CAN-Init & Selbsttest ----------
// Treiber (neu) starten. mode: 0 = Normal (Polling + ACK), 1 = Nur-Lauschen (sendet nichts)
bool initCan(int mode) {
  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(canTxPin, canRxPin,
                              mode ? TWAI_MODE_LISTEN_ONLY : TWAI_MODE_NORMAL);
  g.rx_queue_len = 128;  // grosser RX-Puffer: faengt Frames ab, falls loop() beim SD-Schreiben kurz stockt
  // Die TWAI_TIMING_CONFIG_*-Makros expandieren zu {..}-Initialisierern und
  // duerfen daher nicht im Ternary stehen -> ueber zwei Variablen waehlen.
  twai_timing_config_t t500 = TWAI_TIMING_CONFIG_500KBITS();
  twai_timing_config_t t250 = TWAI_TIMING_CONFIG_250KBITS();
  twai_timing_config_t t = (canBitrateK == 250) ? t250 : t500;
  twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  bool ok = (twai_driver_install(&g, &t, &f) == ESP_OK) && (twai_start() == ESP_OK);
  if (ok) currentCanMode = mode;
  addLog("TWAI %s (%s, %dk)", ok ? "gestartet" : "FEHLER", mode ? "NUR LAUSCHEN" : "Normal", canBitrateK);
  return ok;
}

void reinitCan(int mode) { twai_stop(); twai_driver_uninstall(); initCan(mode); }

// Selbsttest OHNE Auto: Im No-ACK-Modus ein Frame an sich selbst senden. Der Weg
// geht physisch durch den Transceiver und zurueck — besteht der Test, sind
// Controller + Transceiver intakt. Parametrisiert ueber Pin-Belegung + Modus-Pin,
// damit der Sweep alle plausiblen Verdrahtungen durchprobieren kann.
bool canSelfTestCfg(gpio_num_t tx, gpio_num_t rx, int seLevel) {
  digitalWrite(CAN_SE_PIN, seLevel);
  delay(20);   // Transceiver-Modus umschalten lassen
  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(tx, rx, TWAI_MODE_NO_ACK);
  twai_timing_config_t t = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  if (twai_driver_install(&g, &t, &f) != ESP_OK) return false;
  bool ok = false;
  if (twai_start() == ESP_OK) {
    twai_message_t m = {};
    m.identifier = 0x555; m.extd = 0; m.self = 1;   // Self-Reception
    m.data_length_code = 8;
    for (int i = 0; i < 8; i++) m.data[i] = i;
    if (twai_transmit(&m, pdMS_TO_TICKS(100)) == ESP_OK) {
      twai_message_t rx_m;
      unsigned long start = millis();
      while (millis() - start < 300) {
        if (twai_receive(&rx_m, pdMS_TO_TICKS(50)) == ESP_OK && rx_m.identifier == 0x555) { ok = true; break; }
      }
    }
    twai_stop();
  }
  twai_driver_uninstall();
  return ok;
}

// Alle plausiblen Konfigurationen durchprobieren: beide Pin-Reihenfolgen (die
// Quellen widersprechen sich) x beide CAN_SE-Pegel. Erste Variante, die den
// Loopback besteht, wird uebernommen. Besteht KEINE -> echter Hardware-Defekt.
bool canSelfTestSweep() {
  const gpio_num_t A = (gpio_num_t)26, B = (gpio_num_t)27;
  struct { gpio_num_t tx, rx; int se; } combos[] = {
    {B, A, LOW}, {A, B, LOW}, {B, A, HIGH}, {A, B, HIGH}   // bekannte Belegung 27/26 zuerst
  };
  for (auto &c : combos) {
    bool ok = canSelfTestCfg(c.tx, c.rx, c.se);
    addLog("  Selbsttest TX=%d RX=%d SE=%s -> %s",
           (int)c.tx, (int)c.rx, c.se ? "HIGH" : "LOW", ok ? "OK" : "fail");
    if (ok) { canTxPin = c.tx; canRxPin = c.rx; canSeWorking = c.se; return true; }
  }
  return false;
}

// ================= UDS (Service 0x22) — aktive VAG-Diagnose (Phase 2) =================
// Der Golf gibt 48V/DSG/ACM NICHT per Broadcast preis -> aktiv abfragen.
// 29-bit ISO 15765-4:  Anfrage an STG 0xNN = 0x18DA{NN}F1,  Antwort = 0x18DAF1{NN}.
#define UDS_REQ(ecu)  (0x18DA00F1UL | ((uint32_t)(ecu) << 8))   // z.B. 0x10 -> 0x18DA10F1
#define UDS_RSP(ecu)  (0x18DAF100UL | (uint32_t)(ecu))          // z.B. 0x10 -> 0x18DAF110
#define ECU_ENGINE 0x01
#define ECU_DSG    0x10

struct UdsItem {
  uint8_t     ecu;      // Steuergeraet (0x01 Motor, 0x10 DSG, ...)
  uint16_t    did;      // Data Identifier (Service 0x22)
  const char* name;
  uint8_t     off;      // Byte-Offset im Datenteil (nach 62 DIDhi DIDlo)
  uint8_t     len;      // 1 oder 2 Bytes
  float       scale, offset;
  const char* unit;
  long        value;    // Laufzeit: letzter Rohwert
  bool        seen;
};

// >>> HIER gefundene DIDs eintragen (aktuell nur 1 unbestaetigtes BEISPIEL). <<<
// Sobald echte DIDs drinstehen: udsPollEnabled = true setzen.
UdsItem udsTable[] = {
  // ecu,      did,    name,          off,len, scale, offset, unit
  { ECU_DSG, 0xF18C, "DSG-Teilenr.",  0,  1,  1.0f,  0.0f,   "" },   // BEISPIEL/UNBESTAETIGT
};
const int NUM_UDS = sizeof(udsTable) / sizeof(udsTable[0]);
bool udsPollEnabled = false;          // AUS bis echte DIDs vorhanden -> Logger bleibt unberuehrt
char udsTestResult[80] = "Noch nicht getestet.";
volatile bool udsTestRequested = false;

// --- SF-Request bauen + senden (non-blocking). true = auf den Bus gelegt. ---
bool requestUdsData(uint8_t ecu, uint16_t did) {
  twai_message_t m = {};
  m.identifier = UDS_REQ(ecu); m.extd = 1; m.data_length_code = 8;
  m.data[0] = 0x03;                 // ISO-TP Single Frame, 3 Nutzbytes folgen
  m.data[1] = 0x22;                 // Service ReadDataByIdentifier
  m.data[2] = did >> 8; m.data[3] = did & 0xFF;
  m.data[4] = m.data[5] = m.data[6] = m.data[7] = 0xAA;   // Padding wie das Fahrzeug
  if (twai_transmit(&m, pdMS_TO_TICKS(10)) != ESP_OK) return false;
  logRawLine(m.identifier, true, 8, m.data, true);        // Request in den Roh-Log (Dir=Tx)
  return true;
}

// --- ISO-TP Single-Frame-Positivantwort parsen (<=7 Byte). true bei 0x62 + passender DID. ---
bool parseUdsSF(twai_message_t &rx, uint16_t did, const uint8_t*& data, uint8_t& len) {
  if ((rx.data[0] & 0xF0) != 0x00) return false;          // Bit7-4 == 0 -> Single Frame
  uint8_t sf = rx.data[0] & 0x0F;                         // Nutzlaenge (max 7)
  if (sf < 3 || rx.data[1] != 0x62) return false;         // keine 0x22-Positivantwort
  if (((rx.data[2] << 8) | rx.data[3]) != did) return false;
  data = &rx.data[4];
  len  = sf - 3;                                          // Bytes nach '62 DIDhi DIDlo'
  return true;
}

// --- Non-blocking State-Machine: eine Abfrage zur Zeit, nur wenn OBD-Slot frei ---
enum UdsState { UDS_IDLE, UDS_WAIT };
UdsState udsState = UDS_IDLE;
int udsIdx = 0;
unsigned long udsSentAt = 0, lastUdsPoll = 0;
uint32_t udsExpRsp = 0; uint16_t udsExpDid = 0;
#define UDS_TIMEOUT_MS 80
#define UDS_POLL_GAP_MS 100

void serviceUds() {
  if (!udsPollEnabled || NUM_UDS == 0) return;
  if (udsState == UDS_IDLE) {
    if (requestPending) return;                             // OBD nicht stoeren
    if (millis() - lastUdsPoll < UDS_POLL_GAP_MS) return;
    UdsItem &it = udsTable[udsIdx];
    if (requestUdsData(it.ecu, it.did)) {
      udsExpRsp = UDS_RSP(it.ecu); udsExpDid = it.did;
      udsSentAt = millis(); udsState = UDS_WAIT;
    }
  } else if (millis() - udsSentAt >= UDS_TIMEOUT_MS) {      // keine Antwort -> naechste DID
    udsState = UDS_IDLE; lastUdsPoll = millis();
    udsIdx = (udsIdx + 1) % NUM_UDS;
  }
}

// Aus der RX-Schleife: passt das Frame zur offenen UDS-Anfrage?
void handleUdsFrame(twai_message_t &rx) {
  if (udsState != UDS_WAIT || rx.identifier != udsExpRsp) return;
  const uint8_t* d; uint8_t len;
  if (parseUdsSF(rx, udsExpDid, d, len)) {
    UdsItem &it = udsTable[udsIdx];
    long v = (it.off < len) ? d[it.off] : 0;
    if (it.len == 2 && (size_t)it.off + 1 < len) v = (d[it.off] << 8) | d[it.off + 1];
    it.value = v; it.seen = true;
    addLog("UDS STG 0x%02X DID %04X = %ld", it.ecu, it.did, v);
  } else if (rx.data[1] == 0x7F) {
    addLog("UDS DID %04X abgelehnt (NRC 0x%02X)", udsExpDid, rx.data[3]);
  } else if ((rx.data[0] & 0xF0) == 0x10) {
    addLog("UDS DID %04X: Multiframe-Antwort -> braucht Flow-Control (spaetere Phase)", udsExpDid);
  } else return;                                            // fremdes Frame (z.B. OBD 0x41) -> weiter warten
  udsState = UDS_IDLE; lastUdsPoll = millis();
  udsIdx = (udsIdx + 1) % NUM_UDS;
}

// --- Startup-Test: antwortet ein STG auf eine Standard-UDS-Anfrage? ---
// Kurzes Blocking ist hier ok (nur beim Boot / per Knopfdruck, wie die Discovery).
// JEDE Antwort auf die Antwort-ID (auch ablehnend 0x7F) beweist: STG spricht UDS.
bool udsPing(uint8_t ecu, uint16_t did) {
  twai_message_t rx;
  while (twai_receive(&rx, 0) == ESP_OK) captureFrame(rx);  // Queue leeren, nichts verlieren
  if (!requestUdsData(ecu, did)) return false;
  uint32_t rsp = UDS_RSP(ecu);
  unsigned long t0 = millis();
  while (millis() - t0 < 300) {
    if (twai_receive(&rx, pdMS_TO_TICKS(20)) == ESP_OK) {
      captureFrame(rx);
      if (rx.identifier == rsp && (rx.data[1] == 0x62 || rx.data[1] == 0x7F)) return true;
    }
  }
  return false;
}

void testUdsConnection() {
  addLog("UDS-Test: Anfrage an DSG 0x18DA10F1, DID 0xF186 ...");
  bool ok = udsPing(ECU_DSG, 0xF186);   // 0xF186 = aktive Diagnose-Session (kurze SF-Antwort)
  snprintf(udsTestResult, sizeof(udsTestResult),
           ok ? "DSG antwortet auf UDS (0x18DAF110)!" : "Keine UDS-Antwort vom DSG (0x10).");
  addLog("%s", udsTestResult);
}




// ---------- Zeit: POSIX-Zone setzen (autom. Sommer-/Winterzeit) ----------
void applyTz() { setenv("TZ", cfg.tz, 1); tzset(); }

void setTimeAnchor(long long epochMs) {   // Anker: UTC-Epoche <-> Uptime; Lokalzeit macht localtime_r (DST)
  anchorEpochMs = epochMs; anchorUptimeMs = millis(); timeSynced = true;
  writeTimeAnchor();
}

// struct tm (UTC) -> Unix-Sekunden, ohne libc-Zeitzone (fuer den HTTP-Date-Header)
static long long tmToEpoch(struct tm* t) {
  static const int cum[] = {0,31,59,90,120,151,181,212,243,273,304,334};
  long y = t->tm_year + 1900;
  long days = (y-1970)*365 + (y-1969)/4 - (y-1901)/100 + (y-1601)/400 + cum[t->tm_mon] + (t->tm_mday-1);
  if (t->tm_mon >= 2 && ((y%4==0 && y%100!=0) || y%400==0)) days += 1;
  return (long long)days*86400 + t->tm_hour*3600 + t->tm_min*60 + t->tm_sec;
}

// NTP (UDP) — mehrere Server, falls einer nicht antwortet.
bool tryNtp() {
  configTzTime(cfg.tz, "pool.ntp.org", "time.google.com", "time.cloudflare.com");
  struct tm ti;
  if (getLocalTime(&ti, 8000)) { setTimeAnchor((long long)time(nullptr)*1000LL); addLog("Zeit per NTP"); return true; }
  return false;
}

// HTTP-Date-Fallback: viele Auto-/Mobilfunk-Hotspots blocken NTP (UDP 123), lassen aber HTTP
// durch (z.B. Internetradio). Dann holen wir die Uhrzeit aus dem HTTP-'Date'-Header (TCP).
bool tryHttpTime() {
  WiFiClient c; c.setTimeout(4000);
  if (!c.connect("google.com", 80)) return false;
  c.print("HEAD / HTTP/1.1\r\nHost: google.com\r\nConnection: close\r\n\r\n");
  unsigned long t0 = millis(); bool ok = false;
  while (c.connected() && millis() - t0 < 4000) {
    String line = c.readStringUntil('\n');
    if (line.startsWith("Date:") || line.startsWith("date:")) {
      struct tm tm = {};
      if (strptime(line.c_str() + 6, "%a, %d %b %Y %H:%M:%S", &tm)) {
        setTimeAnchor(tmToEpoch(&tm) * 1000LL); addLog("Zeit per HTTP-Date"); ok = true;
      }
      break;
    }
    if (line == "\r" || line.length() == 0) break;   // Header-Ende
  }
  c.stop();
  return ok;
}

// ---------- WLAN: NUR kurz beim Boot fuer die NTP-Zeit, danach AUS ----------
// Kein Webserver, kein AP mehr -> WLAN nur, wenn ein Auto-WLAN konfiguriert ist ("join").
// Danach schalten wir WLAN ab, damit nur BLE laeuft (kein Funk-Konflikt, minimale Last).
void setupWifi() {
  if (strcmp(cfg.wifiMode, "join") == 0) {
    struct { const char* s; const char* p; } nets[] = {
      { CFG_WIFI_1_SSID, CFG_WIFI_1_PASS }, { CFG_WIFI_2_SSID, CFG_WIFI_2_PASS },
      { CFG_WIFI_3_SSID, CFG_WIFI_3_PASS }, { CFG_WIFI_4_SSID, CFG_WIFI_4_PASS },
    };
    WiFi.mode(WIFI_STA);
    // Netze der Reihe nach: verbinden -> Zeit holen (NTP, sonst HTTP-Date). Klappt Zeit -> fertig,
    // sonst naechstes Netz. So faengt ein Netz ohne NTP nicht die ganze Zeitquelle ab.
    for (auto& net : nets) {
      if (!net.s[0]) continue;
      addLog("WLAN: verbinde mit '%s' ...", net.s);
      WiFi.begin(net.s, net.p);
      unsigned long t0 = millis();
      while (WiFi.status() != WL_CONNECTED && millis() - t0 < 9000) delay(200);
      if (WiFi.status() != WL_CONNECTED) { addLog("  nicht verbunden -> naechstes"); continue; }
      addLog("  verbunden (%s), hole Zeit ...", WiFi.localIP().toString().c_str());
      if (tryNtp() || tryHttpTime()) break;    // Zeit da -> fertig
      addLog("  Zeit ueber dieses Netz nicht moeglich -> naechstes");
      WiFi.disconnect(true);
    }
  }
  // WLAN nur anlassen, wenn Dev-Streaming (TCP/MQTT) es braucht — sonst aus (nur BLE, minimale Last)
  if (cfg.streamTcp || cfg.mqttEn) {
    addLog("WLAN bleibt an (Dev-Streaming)");
  } else {
    WiFi.disconnect(true, true); WiFi.mode(WIFI_OFF);
    addLog("WLAN aus -> nur noch BLE");
  }
}

// (Config kommt aus include/config.h zur Compile-Zeit — kein Laden von SD noetig.)

// ================= BLE-Broadcast (statt Webserver) =================
// Sendet Status + Werte als lesbare ASCII-Charakteristiken. Eine BLE-Scanner-App
// (nRF Connect, Bluetooth Inspector) verbindet sich und sieht die Werte live.
// Best-effort & entkoppelt: faellt BLE aus, laeuft das SD-Logging unberuehrt weiter.
static NimBLECharacteristic *chStatus = nullptr, *chSystem = nullptr, *chTrip = nullptr, *chPids = nullptr, *chConn = nullptr;
uint32_t bleHeartbeat = 0;
char pidsStr[220] = "discovery laeuft...";   // Liste der erkannten PIDs (nach Discovery gefuellt)

static void bleName(NimBLECharacteristic* c, const char* name) {
  NimBLEDescriptor* d = c->createDescriptor("2901", NIMBLE_PROPERTY::READ, 24);
  if (d) d->setValue(name);
}

void bleInit() {
  NimBLEDevice::init("OpenOBD");
  NimBLEServer* srv = NimBLEDevice::createServer();
  NimBLEService* svc = srv->createService("6f70656e-6f62-6400-0000-0000000000ff");
  chStatus = svc->createCharacteristic("6f70656e-6f62-6400-0000-000000000001", NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  chSystem = svc->createCharacteristic("6f70656e-6f62-6400-0000-000000000002", NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  chTrip   = svc->createCharacteristic("6f70656e-6f62-6400-0000-000000000003", NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  chPids   = svc->createCharacteristic("6f70656e-6f62-6400-0000-000000000004", NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  chConn   = svc->createCharacteristic("6f70656e-6f62-6400-0000-000000000005", NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  bleName(chStatus, "Status");
  bleName(chSystem, "System");
  bleName(chTrip, "Trip");
  bleName(chPids, "PIDs");
  bleName(chConn, "Verbindung");
  chStatus->setValue("start"); chSystem->setValue("--"); chTrip->setValue("--");
  chPids->setValue(pidsStr); chConn->setValue("--");
  svc->start();
  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  adv->addServiceUUID(svc->getUUID());
  adv->setScanResponse(true);
  NimBLEDevice::startAdvertising();
  addLog("BLE aktiv: 'OpenOBD' (mit Scanner-App verbinden)");
}

// 1x/Sekunde aus loop(): Status + Werte als ASCII aktualisieren + notifizieren
void bleUpdate() {
  if (!chStatus) return;
  char clk[40]; clockString(clk, sizeof(clk));
  const char* tstr = timeSynced ? (strchr(clk, ' ') ? strchr(clk, ' ') + 1 : clk) : "unsync";
  char b[80];
  // Status: Herzschlag (zaehlt hoch = lebt), Uptime, freier RAM, verworfene Roh-Zeilen (Last-Indikator)
  snprintf(b, sizeof(b), "HB%lu UP%lus HEAP%uk DROP%lu",
           (unsigned long)(++bleHeartbeat), millis() / 1000,
           (unsigned)(ESP.getFreeHeap() / 1024), rawLinesDropped);
  chStatus->setValue((uint8_t*)b, strlen(b)); chStatus->notify();
  // System: SD-frei, Aufnahme, Session, Uhrzeit (oder 'unsync' = kein NTP/Sync)
  snprintf(b, sizeof(b), "SD%luMB REC%d S%d T:%s",
           (unsigned long)(cachedFreeBytes / (1024UL * 1024UL)), recording ? 1 : 0, sessionNum, tstr);
  chSystem->setValue((uint8_t*)b, strlen(b)); chSystem->notify();
  // Trip: verstrichene km + Liter dieser Session + Momentanverbrauch (L/100 wenn faehrt)
  double now100 = (speed_kmh > 3 && pidSeen[0x5E]) ? (fuel_rate / speed_kmh * 100.0) : 0.0;
  snprintf(b, sizeof(b), "KM%.1f L%.2f NOW%.1f", totalDistKm, totalFuelL, now100);
  chTrip->setValue((uint8_t*)b, strlen(b)); chTrip->notify();
  // Verbindung: bekommt das Geraet Daten vom Auto? (rxCount>0 = Bus lebt / Auto antwortet)
  twai_status_info_t st; unsigned long tec = 0;
  if (twai_get_status_info(&st) == ESP_OK) tec = st.tx_error_counter;
  snprintf(b, sizeof(b), "%s RX%lu TEC%lu %s", rxCount > 0 ? "verbunden" : "kein Bus",
           rxCount, tec, discoveryDone ? "DISC-OK" : "DISC-?");
  chConn->setValue((uint8_t*)b, strlen(b)); chConn->notify();
}

// ---------- Software-Watchdog: rebootet automatisch, falls loop() je haengt ----------
// Laeuft auf Core 0 (loop() auf Core 1) -> greift auch, wenn Core 1 blockiert.
void wdtTask(void* pv) {
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(2000));
    if (lastLoopTick != 0 && (uint32_t)(millis() - lastLoopTick) > 15000) esp_restart();
  }
}

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);

  // T-CAN485 hochfahren (vor SD + TWAI): 5V-Boost an, Transceiver in Normalmodus
  pinMode(PIN_5V_EN, OUTPUT); digitalWrite(PIN_5V_EN, HIGH);
  pinMode(CAN_SE_PIN, OUTPUT); digitalWrite(CAN_SE_PIN, LOW);
  delay(10);

  fsMutex = xSemaphoreCreateMutex();
  applyTz();          // Zeitzone (inkl. DST) fuer localtime_r setzen

  SPI.begin(SD_SCLK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
  if (SD.begin(SD_CS_PIN)) {
    storage = &SD; useSD = true;
    addLog("SD OK — %llu MB", SD.cardSize() / (1024 * 1024));
  } else {
    addLog("SD nicht gefunden");
  }
  // Schreibtest: eine erkannte, aber unbeschreibbare SD (falsches Format, defekt)
  // darf nicht stillschweigend "aufzeichnen" — dann lieber lauter Fallback.
  if (!storage || !ensureDataDirWritable()) {
    if (storage) addLog("SD NICHT BESCHREIBBAR -> als FAT32/MBR formatieren oder Karte tauschen");
    storage = nullptr; useSD = false;
    if (LittleFS.begin(true)) {
      storage = &LittleFS;
      if (ensureDataDirWritable()) addLog("Fallback: interner Flash (LittleFS, ~1.5 MB)");
      else { storage = nullptr; addLog("KEIN nutzbarer Speicher!"); }
    } else {
      addLog("KEIN nutzbarer Speicher!");
    }
  }
  addLog("Modus: %s (aus config.h)", cfg.devMode ? "DEV" : "Normal");
  if (storage) loadLastTime();   // letzte bekannte Zeit -> provisorisches Datum fuer die Ordner
  setupWifi();        // ZUERST: im Join-Modus steht damit die echte NTP-Zeit fuer die Ordner-Datierung
  if (storage) openNewSession();
  recording = cfg.autoRecord;
  checkFreeSpace();   // Startwert fuer freien Speicher (danach alle 10s in loop)

  bleInit();          // BLE-Broadcast (Status + Werte) statt Webserver
  xTaskCreatePinnedToCore(wdtTask, "WDT", 2048, NULL, 3, NULL, 0);   // Software-Watchdog (Core 0)

  if (cfg.streamTcp || cfg.mqttEn) {   // Dev-Streaming nur bei Bedarf hochfahren
    streamQ = xQueueCreate(256, sizeof(StreamFrame));
    xTaskCreatePinnedToCore(streamTask, "Stream", 6144, NULL, 1, NULL, 0);
    addLog("Dev-Streaming: TCP=%d MQTT=%d", cfg.streamTcp, cfg.mqttEn);
  }

  // CAN-Selbsttest (ohne Auto): alle Pin-/Modus-Kombinationen durchprobieren.
  digitalWrite(PIN_5V_EN, HIGH); delay(80);   // sicherstellen, dass die 5V-Schiene steht
  addLog("CAN-Selbsttest: probiere alle Konfigurationen...");
  canSelfTestOk = canSelfTestSweep();
  if (canSelfTestOk)
    addLog("CAN-SELBSTTEST OK -> Board GESUND. Nutze TX=%d RX=%d SE=%s",
           (int)canTxPin, (int)canRxPin, canSeWorking ? "HIGH" : "LOW");
  else
    addLog("CAN-SELBSTTEST: KEINE Konfiguration bestanden -> Transceiver/5V wirklich defekt");
  digitalWrite(CAN_SE_PIN, canSeWorking);     // funktionierenden Pegel fuer Normalbetrieb setzen
  initCan(0);                 // Normal-Modus, 500 kbps, alle IDs

  delay(200);                 // Bus kurz atmen lassen
  if (cfg.discovery) discoverSupportedPids();  // fragt das Auto, welche PIDs es unterstuetzt
  { // erkannte PIDs fuer die BLE-Charakteristik zusammenbauen
    int o = snprintf(pidsStr, sizeof(pidsStr), "%d PIDs:", numSupported);
    for (int p = 1; p < 256 && o < (int)sizeof(pidsStr) - 4; p++)
      if (pidSupported[p]) o += snprintf(pidsStr + o, sizeof(pidsStr) - o, " %02X", p);
    if (chPids) chPids->setValue(pidsStr);
  }
  testUdsConnection();        // Phase 2: einmalig pruefen, ob das DSG UDS spricht
  lastLoopTick = millis();    // Software-Watchdog jetzt scharf schalten (Boot ist durch)
}

uint8_t getNextPid() {
  pollCounter++;
  if (pollCounter % SLOW_POLL_RATIO == 0) {
    for (int i = 0; i < NUM_SLOW; i++) {
      uint8_t pid = slow_pids[slowIndex]; slowIndex = (slowIndex + 1) % NUM_SLOW;
      if (!isPidDisabled(pid) && (!discoveryDone || pidSupported[pid])) return pid;
    }
  }
  for (int i = 0; i < NUM_FAST; i++) {
    uint8_t pid = fast_pids[fastIndex]; fastIndex = (fastIndex + 1) % NUM_FAST;
    if (!isPidDisabled(pid) && (!discoveryDone || pidSupported[pid])) return pid;
  }
  return 0x00;
}

void sendObd2Request(uint8_t pid) {
  twai_message_t m = {};
  m.identifier = obdReqId; m.extd = (obdReqId > 0x7FF); m.data_length_code = 8;
  m.data[0] = 0x02; m.data[1] = 0x01; m.data[2] = pid;
  for (int i = 3; i < 8; i++) m.data[i] = obdPad;
  if (twai_transmit(&m, pdMS_TO_TICKS(10)) == ESP_OK) {
    txCount++; requestPending = true; requestSentTime = millis(); pendingPid = pid;
    logRawLine(m.identifier, m.extd, 8, m.data, true);   // Request mitschneiden (Dir=Tx)
  } else {
    txFailCount++;   // ohne verbundenen CAN-Bus normal (kein ACK) — gedrosselt loggen
    txBackoffUntil = millis() + 100;   // 100 ms Sendepause statt Busy-Loop
    if (millis() - lastTxFailLog >= 5000) {
      lastTxFailLog = millis();
      addLog("TX fail x%lu (kein CAN-Bus verbunden?)", txFailCount);
    }
  }
}

void parseObd2Response(twai_message_t &rx) {
  uint8_t pid = rx.data[2];
  pidSeen[pid] = true;   // ab jetzt zeigt das Dashboard den echten Wert statt "--"
  switch (pid) {
    case 0x0C: rpm = ((rx.data[3]*256)+rx.data[4])/4; break;
    case 0x0D: speed_kmh = rx.data[3]; break;
    case 0x05: if(rx.data[3]==0xFF){pidSeen[pid]=false;break;} coolant_temp = rx.data[3]-40; break;
    case 0x0B: manifold_kpa = rx.data[3]; break;
    case 0x0F: if(rx.data[3]==0xFF){pidSeen[pid]=false;break;} intake_temp = rx.data[3]-40; break;
    case 0x04: engine_load = (rx.data[3]*100)/255; break;
    case 0x0E: timing_advance = rx.data[3]/2.0-64.0; break;
    case 0x47: throttle_pos = (rx.data[3]*100)/255; break;
    case 0x5A: accel_pedal = (rx.data[3]*100)/255; break;
    case 0x4C: cmd_throttle = (rx.data[3]*100)/255; break;
    case 0x61: demand_torque = rx.data[3]-125; break;
    case 0x62: actual_torque = rx.data[3]-125; break;
    case 0x06: short_fuel_trim = (rx.data[3]-128)*100.0/128.0; break;
    case 0x07: long_fuel_trim = (rx.data[3]-128)*100.0/128.0; break;
    case 0x33: baro_kpa = rx.data[3]; break;
    case 0x15: o2_s2_voltage = rx.data[3]/200.0; o2_s2_stft = (rx.data[4]-128)*100.0/128.0; break;
    case 0x44: cmd_equiv_ratio = ((rx.data[3]*256)+rx.data[4])*2.0/65536.0; break;
    case 0x3C: catalyst_temp = ((rx.data[3]*256)+rx.data[4])/10-40; break;
    case 0x56: o2_secondary_ltft = (rx.data[3]-128)*100.0/128.0; break;
    case 0x5C: if(rx.data[3]==0xFF){pidSeen[pid]=false;break;} oil_temp = rx.data[3]-40; break;
    case 0x2F: fuel_level = (rx.data[3]*100)/255; break;
    case 0x5E: fuel_rate = ((rx.data[3]*256)+rx.data[4])/20.0; break;
    case 0x42: module_voltage = ((rx.data[3]*256)+rx.data[4])/1000.0; break;
    case 0x46: if(rx.data[3]==0xFF){pidSeen[pid]=false;break;} ambient_temp = rx.data[3]-40; break;
    case 0x63: ref_torque = (rx.data[3]*256)+rx.data[4]; break;
    case 0x34: o2s1_lambda = ((rx.data[3]*256)+rx.data[4])/32768.0;
               o2s1_current = ((rx.data[5]*256)+rx.data[6])/256.0-128.0; break;
    case 0x43: absolute_load = ((rx.data[3]*256)+rx.data[4])*100.0/255.0; break;
    case 0x2E: evap_purge = (rx.data[3]*100)/255; break;
    // --- Harvest-Neuzugaenge (Standard-SAE-Formeln) ---
    case 0x11: throttle11 = (rx.data[3]*100)/255; break;
    case 0x45: rel_throttle = (rx.data[3]*100)/255; break;
    case 0x1F: runtime_s = (rx.data[3]*256)+rx.data[4]; break;
    case 0x31: dist_clr = (rx.data[3]*256)+rx.data[4]; break;
    case 0x51: fuel_type = rx.data[3]; break;
    case 0x53: evap_press = ((rx.data[3]*256)+rx.data[4])/200.0; break;
    case 0xA6: odometer_km = (((uint32_t)rx.data[3]<<24)|((uint32_t)rx.data[4]<<16)|
                              ((uint32_t)rx.data[5]<<8)|rx.data[6])/10.0; break;
    case 0xA4: gear = rx.data[4] >> 4; break;   // High-Nibble Byte B = Gang 1-7 (0 = kein Gang / N)
  }
}

// Jedes empfangene Frame: Explorer-Tabelle aktualisieren + Roh-Log puffern
void captureFrame(twai_message_t &rx) {
  totalFramesSeen++;
  FrameInfo* f = getOrAddFrame(rx.identifier, rx.extd);
  if (f) {
    f->count++; f->lastMs = millis(); f->dlc = rx.data_length_code;
    for (int i = 0; i < 8; i++) f->data[i] = rx.data[i];
  }
  logRawLine(rx.identifier, rx.extd, rx.data_length_code, rx.data, false);
  if (streamQ) {   // Dev-Streaming: Frame an den Sende-Task (Core 0) reichen, non-blocking
    StreamFrame sf; sf.ts = esp_timer_get_time(); sf.id = rx.identifier; sf.ext = rx.extd;
    sf.dlc = rx.data_length_code; sf.tx = 0; memcpy(sf.d, rx.data, 8);
    xQueueSend(streamQ, &sf, 0);
  }
}

void loop() {
  lastLoopTick = millis();   // Software-Watchdog fuettern (haengt loop() je -> Auto-Reboot)

  // 1) Naechste Anfrage senden, falls keine offen (nicht im Lausch-Modus)
  if (currentCanMode == 0 && !requestPending && (int32_t)(millis() - txBackoffUntil) >= 0) sendObd2Request(getNextPid());
  serviceUds();   // Phase 2: UDS-Abfrage einschieben (nur wenn aktiviert + OBD-Slot frei)

  // 2) Timeout der offenen Anfrage
  if (requestPending && (millis() - requestSentTime >= RESPONSE_TIMEOUT_MS)) {
    requestPending = false; rxTimeoutCount++;
    PidStats* s = getPidStats(pendingPid);
    if (s) {
      s->timeouts++; s->consecutiveTimeouts++;
      if (!s->disabled && s->consecutiveTimeouts >= PID_DISABLE_THRESHOLD) {
        s->disabled = true; addLog("PID 0x%02X disabled", s->pid);
      }
    }
  }

  // 3) RX-Queue leeren — JEDES Frame mitschneiden, OBD-Antworten zusaetzlich dekodieren
  twai_message_t rx;
  int budget = 40;
  while (budget-- > 0 && twai_receive(&rx, 0) == ESP_OK) {
    rxCount++;
    captureFrame(rx);
    handleUdsFrame(rx);        // Phase 2: UDS-Antwort (Service 0x62) abholen, falls Anfrage offen
    // OBD2-Antwort? 11 bit 0x7E8..0x7EF oder 29 bit 0x18DAF1xx, Mode-01-Antwort = 0x41
    if (isObdResponse(rx) && rx.data[1] == 0x41) {
      parseObd2Response(rx);
      PidStats* s = getPidStats(rx.data[2]);
      if (s) { s->responses++; s->consecutiveTimeouts = 0; }
      if (requestPending && rx.data[2] == pendingPid) requestPending = false;
    }
  }

  // BLE-Broadcast 1x/s (Herzschlag + Status + Werte)
  static unsigned long lastBle = 0;
  if (millis() - lastBle >= 1000) { lastBle = millis(); bleUpdate(); }

  // 4) Alle LOG_INTERVAL_MS: Decoded-CSV schreiben + Roh-Puffer flushen + Status
  if (millis() - lastLog >= (unsigned long)cfg.logIntervalMs) {
    float dt_h = cfg.logIntervalMs / 3600000.0;
    totalDistKm += speed_kmh * dt_h;
    totalFuelL += fuel_rate * dt_h;
    lastLog = millis();

    if (millis() - lastFreeCheck >= 10000) { lastFreeCheck = millis(); checkFreeSpace(); saveLastTime(); }

    if (millis() - lastStatusLog >= 5000) {
      lastStatusLog = millis();
      addLog("TX:%lu RX:%lu TO:%lu Frames:%lu IDs:%d", txCount, rxCount, rxTimeoutCount, totalFramesSeen, numFrames);
      twai_status_info_t st;
      if (twai_get_status_info(&st) == ESP_OK) {
        addLog("CAN state:%d TEC:%lu REC:%lu RX:%lu %s", st.state, st.tx_error_counter,
               st.rx_error_counter, rxCount,
               (st.state == TWAI_STATE_RUNNING && st.tx_error_counter < 96)
                 ? "-> am Bus" : "-> NICHT am Bus (Stecker/CANH-CANL?)");
        // CAN-Bus-Off-Recovery (Baustelle #2 des Originals)
        if (st.state == TWAI_STATE_BUS_OFF) { twai_initiate_recovery(); addLog("CAN bus-off -> Recovery"); }
        else if (st.state == TWAI_STATE_STOPPED) { twai_start(); addLog("CAN Recovery -> Neustart"); }
      }
    }

    if (recording && storage && !storageFull) {
      flushRaw();  // Roh-Frames sichern (no-op wenn logRaw aus)
      if (cfg.logDecoded) {
        if (xSemaphoreTake(fsMutex, pdMS_TO_TICKS(50))) {
          File file = storage->open(decodedFile, "a");
          if (file) {
            file.printf("%lu,%d,%d,%d,%d,%d,%d,%d,%.1f,%d,%d,%d,%d,%d,%.1f,%.1f,%d,%.3f,%.1f,%.3f,%d,%.1f,%d,%.1f,%.2f,%d,%d,%.3f,%.2f,%.1f,%d,%.2f,%.2f,%d,%d,%lu,%lu,%d,%.2f,%.1f,%d\n",
              millis(), rpm, speed_kmh, coolant_temp, oil_temp, manifold_kpa, intake_temp, engine_load,
              timing_advance, throttle_pos, accel_pedal, cmd_throttle, demand_torque, actual_torque,
              short_fuel_trim, long_fuel_trim, baro_kpa, o2_s2_voltage, o2_s2_stft, cmd_equiv_ratio,
              catalyst_temp, o2_secondary_ltft, fuel_level, fuel_rate, module_voltage, ambient_temp,
              ref_torque, o2s1_lambda, o2s1_current, absolute_load, evap_purge, totalDistKm, totalFuelL,
              throttle11, rel_throttle, runtime_s, dist_clr, fuel_type, evap_press, odometer_km, gear);
            file.close();
          }
          xSemaphoreGive(fsMutex);
        }
      }
    }
  }
}
