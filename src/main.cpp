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
#include <WebServer.h>
#include <ESPmDNS.h>     // openobd.local, wenn wir im Auto-WLAN haengen
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
  char ssid[33]      = CFG_WIFI_SSID;
  char pass[65]      = CFG_WIFI_PASS;
  bool fallbackAp    = CFG_FALLBACK_AP;
  char apSsid[33]    = CFG_AP_SSID;
  char apPass[65]    = CFG_AP_PASS;
  char ntp[48]       = CFG_NTP;
  int  tzMin         = CFG_TZ_MIN;
  bool streamTcp     = CFG_STREAM_TCP;
  int  tcpPort       = CFG_TCP_PORT;
  bool mqttEn        = CFG_MQTT;
  char mqttHost[48]  = CFG_MQTT_HOST;
  int  mqttPort      = CFG_MQTT_PORT;
  char mqttTopic[40] = CFG_MQTT_TOPIC;
} cfg;

WebServer server(80);
SemaphoreHandle_t fsMutex;

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
int tzOffsetMin = 0;               // lokaler Offset (Minuten oestlich UTC), vom Handy
char anchorFile[40];
long long lastKnownEpochMs = 0;   // letzte bekannte Echtzeit (aus /lasttime.txt) — Ordner-Datierung ohne RTC

long long nowEpochMs() { return timeSynced ? anchorEpochMs + (long long)(millis() - anchorUptimeMs) : 0; }

void localNow(struct tm &t) {
  time_t local = (time_t)(nowEpochMs() / 1000) + (time_t)tzOffsetMin * 60;
  gmtime_r(&local, &t);
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
    if (f) { f.printf("uptime_ms=%lu\nepoch_ms=%lld\ntz_min=%d\n", anchorUptimeMs, anchorEpochMs, tzOffsetMin); f.close(); }
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
char probeReport[2000] = "Noch kein Auto-Probe gelaufen.\nAm Auto (Motor an): Button druecken.";
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
  time_t t = (time_t)(e / 1000) + (time_t)tzOffsetMin * 60;
  struct tm tmv; gmtime_r(&t, &tmv);
  snprintf(buf, n, "%04d/%02d", tmv.tm_year + 1900, tmv.tm_mon + 1);
}

// --- Letzte bekannte Echtzeit persistieren/laden (Ersatz fuer fehlende RTC) ---
void saveLastTime() {
  if (!storage || !timeSynced) return;
  if (xSemaphoreTake(fsMutex, pdMS_TO_TICKS(50))) {
    File f = storage->open("/lasttime.txt", "w");
    if (f) { f.printf("epoch_ms=%lld\ntz_min=%d\n", nowEpochMs(), tzOffsetMin); f.close(); }
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
        else if (line.startsWith("tz_min=")) tzOffsetMin = line.substring(7).toInt();
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

void webServerTask(void *parameter) {
  for (;;) { server.handleClient(); vTaskDelay(1); }
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

// Dashboard-SPA: feste Vollbild-Ansicht (kein Scrollen), Kategorie-Tabs, Gauges,
// Live-Graph, Landscape-optimiert. Technik/Discovery liegt im Dev-Tab. Statisch aus
// PROGMEM (spart RAM); alle Werte kommen live von /data.
void handleRoot() {
  if (!cfg.webUi) {   // Dev-Modus: kein Cockpit, nur Technik-Links
    server.send(200, "text/html",
      "<html><head><meta name='viewport' content='width=device-width,initial-scale=1'></head>"
      "<body style='font-family:sans-serif;background:#0f1020;color:#eee;padding:22px'>"
      "<h2>OpenOBD &mdash; Dev-Modus</h2>"
      "<p style='color:#aaa'>Dashboard deaktiviert (Dev-Modus geflasht, <code>config.h</code>). "
      "Roh-Mitschnitt + Diagnose laufen.</p>"
      "<p><a href='/explore' style='color:#8ad'>CAN Explorer</a> &middot; "
      "<a href='/uds' style='color:#8ad'>UDS</a> &middot; "
      "<a href='/sessions' style='color:#8ad'>Sessions</a> &middot; "
      "<a href='/pids' style='color:#8ad'>PIDs</a> &middot; "
      "<a href='/log' style='color:#8ad'>Log</a></p></body></html>");
    return;
  }
  static const char PAGE[] PROGMEM = R"PAGE(<!doctype html><html lang=de><head>
<meta charset=utf-8><meta name=viewport content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">
<title>OpenOBD</title><style>
:root{--bg:#0a0e17;--pan:#121826;--pan2:#0d1320;--line:#1d2635;--ink:#eaf0f8;--mut:#7f8ba1;--faint:#4a5568;--acc:#31d2e6;--good:#37c98a;--warn:#f2b13d;--crit:#f0566c}
*{box-sizing:border-box;margin:0;-webkit-tap-highlight-color:transparent}
html,body{height:100%}
body{background:var(--bg);color:var(--ink);font-family:system-ui,-apple-system,sans-serif;overflow:hidden}
#app{display:flex;flex-direction:column;height:100dvh;padding:8px 10px;gap:8px}
#top{display:flex;align-items:center;gap:10px}
.ham{font-size:19px;line-height:1;background:var(--pan);border:1px solid var(--line);color:var(--ink);border-radius:10px;padding:9px 13px;font-weight:700}
#top b{font-size:14px;letter-spacing:.5px}#top b i{color:var(--acc);font-style:normal}
#cur{font-size:13px;font-weight:700;color:var(--acc)}
#clk{margin-left:auto;font-variant-numeric:tabular-nums;color:var(--mut);font-size:12px}
#stt{font-size:11px;font-weight:700}
#menu{position:fixed;inset:0;background:rgba(6,9,15,.975);z-index:30;display:none;padding:16px;grid-template-columns:repeat(3,1fr);gap:12px;align-content:center}
#menu.open{display:grid}
.mbtn{background:linear-gradient(180deg,var(--pan),var(--pan2));border:1px solid var(--line);border-radius:18px;display:flex;align-items:center;justify-content:center;min-height:86px;font-size:20px;font-weight:700;color:var(--ink)}
.mbtn.on{background:var(--acc);color:#081018;border-color:transparent}
#view{flex:1;min-height:0;display:grid;gap:8px}
#view.cat{grid-template-columns:repeat(4,1fr);grid-auto-rows:1fr}
#view.status{display:block;overflow-y:auto}
.strows{display:grid;grid-template-columns:1fr 1fr;gap:8px}
.technik{margin-top:8px;display:grid;grid-template-columns:repeat(3,1fr);gap:8px}
.technik h4{grid-column:1/-1;margin:8px 2px 0;font-size:10px;letter-spacing:1px;text-transform:uppercase;color:var(--mut);font-weight:700}
.lk{display:flex;align-items:center;justify-content:center;text-align:center;text-decoration:none;background:var(--pan);border:1px solid var(--line);color:var(--acc);border-radius:10px;padding:12px;font-size:13px;font-weight:600}
.lk.act{color:var(--good)}
#view.liveview{display:block}
.tile{background:linear-gradient(180deg,var(--pan),var(--pan2));border:1px solid var(--line);border-radius:13px;padding:9px 12px;display:flex;flex-direction:column;min-height:0;overflow:hidden}
.tl{font-size:10px;font-weight:700;letter-spacing:1px;text-transform:uppercase;color:var(--mut);display:flex;justify-content:space-between;align-items:center;gap:6px}
.tag{font-size:9px;font-weight:800;color:var(--acc);background:#0e2830;padding:2px 6px;border-radius:8px;letter-spacing:.3px;text-transform:none;flex:none}
.tv{font-weight:700;font-variant-numeric:tabular-nums;font-size:clamp(17px,4.4vh,32px);line-height:1.05;margin-top:2px;font-family:ui-monospace,Menlo,monospace}
.tu{font-size:.42em;color:var(--mut);font-weight:600;margin-left:3px}
.bar{height:5px;border-radius:3px;background:var(--line);margin-top:auto;overflow:hidden}
.bar>i{display:block;height:100%;width:0;background:var(--acc);border-radius:3px;transition:width .4s,background .4s}
.wide{grid-column:1/-1}
.live-wrap{display:grid;grid-template-rows:1.15fr .9fr .85fr;gap:8px;height:100%;min-height:0}
.live-top{display:grid;grid-template-columns:1fr 1fr;gap:8px;min-height:0}
.live-tiles{display:grid;grid-template-columns:repeat(6,1fr);gap:8px;min-height:0}
canvas{flex:1;min-height:0;width:100%}
.g{align-items:center;justify-content:center;position:relative}
.g svg{width:100%;height:auto;max-height:84%}
.g .ab{fill:none;stroke:var(--line);stroke-width:11;stroke-linecap:round}
.g .av{fill:none;stroke:var(--acc);stroke-width:11;stroke-linecap:round;transition:stroke-dashoffset .5s}
.g .rd{position:absolute;top:50%;left:0;right:0;text-align:center}
.g .rd .n{font:700 clamp(22px,6vh,46px)/1 ui-monospace,Menlo,monospace;font-variant-numeric:tabular-nums}
.g .rd .u{font-size:11px;color:var(--mut);margin-left:4px}
.g .cap{position:absolute;top:9px;left:12px;font-size:10px;font-weight:700;letter-spacing:1px;text-transform:uppercase;color:var(--mut)}
.srow{display:flex;align-items:center;gap:10px;padding:11px 13px;background:var(--pan);border:1px solid var(--line);border-radius:11px}
.srow .dt{width:9px;height:9px;border-radius:50%;background:var(--faint);flex:none}
.srow .k{color:var(--mut);font-size:12px;font-weight:600;width:38%}
.srow .sv{font-family:ui-monospace,Menlo,monospace;font-size:13px;color:var(--ink)}
@media(orientation:portrait){
 #view.cat{grid-template-columns:repeat(2,1fr)}
 .strows{grid-template-columns:1fr}
 .technik{grid-template-columns:repeat(2,1fr)}
 #menu{grid-template-columns:repeat(2,1fr)}
 .live-tiles{grid-template-columns:repeat(3,1fr)}
 .live-wrap{grid-template-rows:1fr 1fr 1.1fr}
}
</style></head><body><div id=app>
<div id=top><button class="ham" id="ham">&#9776;</button><b>Open<i>OBD</i></b><span id="cur">Live</span><span id="clk">--</span><span id="stt">--</span></div>
<div id="menu"></div><div id="view"></div></div>
<script>
// key:[Label,Einheit,BalkenMax,Kennzeichen?]  (Kennzeichen: 'jetzt'=Momentan, 'Ø'=Durchschnitt)
const M={
 rpm:['Drehzahl','1/min',7000],spd:['Tempo','km/h',200],
 tmp:['Kuehlmittel','°C',120],oil:['Oeltemp','°C',150],iat:['Ansaugluft','°C',80],
 map:['Saugrohr','kPa',250],boost:['Ladedruck','kPa',150],baro:['Luftdruck','kPa',110],
 load:['Last','%',100],absld:['Abs-Last','%',100],tmg:['Zuendung','°',64],
 thr:['Drossel','%',100],cthr:['Soll-Drossel','%',100],pdl:['Gaspedal','%',100],
 atq:['Ist-Moment','%',100],dtq:['Soll-Moment','%',100],rtq:['Ref-Moment','Nm',400],ps:['Leistung','PS',200],
 frate:['Verbrauch','L/h',30,'jetzt'],l100:['Verbrauch','L/100',20,'jetzt'],avg100:['Verbrauch','L/100',20,'Ø'],
 fused:['Sprit','L',0,'Trip'],dist:['Strecke','km',0,'Trip'],fuel:['Tank','%',100],
 o2lam:['Lambda','',2],o2cur:['O2-Strom','mA',20],o2s2v:['O2-Sonde 2','V',1.5],o2s2t:['O2-2 Trim','%',25],
 ceq:['Soll-Lambda','',2],stft:['Kurz-Trim','%',25],ltft:['Lang-Trim','%',25],o2slt:['Sek-LTFT','%',25],
 cat:['Kat-Temp','°C',900],evap:['Evap','%',100],
 mvolt:['Bordspannung','V',16],amb:['Aussentemp','°C',50],
 odo:['Kilometerstand','km',0,'gesamt'],distclr:['Strecke ges.','km',0,'Trip'],runtime:['Laufzeit','s',0],
 thr11:['Drosselklappe','%',100],relthr:['Rel. Drossel','%',100],evappr:['Evap-Druck','kPa',0],
 gear:['Gang','',0]
};
const CATS={
 Motor:['rpm','tmp','oil','iat','map','boost','load','absld','tmg'],
 Fahrt:['spd','gear','thr11','relthr','thr','cthr','pdl','atq','dtq','rtq','ps','runtime'],
 Verbrauch:['l100','avg100','frate','fused','dist','odo','distclr','fuel'],
 Abgas:['o2lam','o2cur','o2s2v','o2s2t','ceq','stft','ltft','o2slt','cat','evap','evappr'],
 Elektrik:['mvolt','amb','baro']
};
const LIVE=['l100','avg100','ps','tmp','boost','mvolt'];
const ZONE={tmp:[80,106],oil:[70,120],mvolt:[13.2,15.0],l100:[0,6],avg100:[0,6]};
// Technik-Sektion im Status-Tab (einmal gebaut, damit Taps nicht verloren gehen)
const TECHNIK='<div class="technik"><h4>Ansehen</h4>'+
 '<a class="lk" href="/sessions">Sessions</a><a class="lk" href="/explore">CAN-Explorer</a>'+
 '<a class="lk" href="/uds">UDS-Scan</a><a class="lk" href="/pids">PIDs</a>'+
 '<a class="lk" href="/timeouts">PID-Status</a><a class="lk" href="/log">Log</a>'+
 '<h4>Aktionen</h4>'+
 '<a class="lk act" href="#" onclick="return act(\'/api/rediscover\',this,\'Discovery neu\')">Discovery neu</a>'+
 '<a class="lk act" href="#" onclick="return act(\'/api/record?cmd=new\',this,\'Neue Session\')">Neue Session</a>'+
 '<a class="lk act" href="#" onclick="return act(\'/api/record?cmd=toggle\',this,\'Rec An/Aus\')">Rec An/Aus</a>'+
 '<a class="lk act" href="#" onclick="return act(\'/api/canmode\',this,\'CAN Normal/Lausch\')">CAN Normal/Lausch</a>'+
 '<a class="lk act" href="#" onclick="return probe(this)">Auto-Probe</a></div>';
function act(u,el,name){fetch(u).catch(function(){});el.textContent='✓';setTimeout(function(){el.textContent=name;},900);return false;}
function probe(el){fetch('/api/probe').catch(function(){});el.textContent='laeuft ~20s…';setTimeout(function(){location.href='/probereport';},22000);return false;}
function zcol(k,v){var z=ZONE[k];if(!z||isNaN(v))return'var(--acc)';
 if(v>=z[0]&&v<=z[1])return'var(--good)';var p=(z[1]-z[0])*0.18;
 return(v>=z[0]-p&&v<=z[1]+p)?'var(--warn)':'var(--crit)';}
const TABS=['Live'].concat(Object.keys(CATS)).concat(['Status']);
let cur='Live',hist={rpm:[],spd:[],load:[]};
const view=document.getElementById('view'),menu=document.getElementById('menu'),ham=document.getElementById('ham'),curEl=document.getElementById('cur');
ham.onclick=function(){menu.innerHTML=TABS.map(function(n){return '<button class="mbtn'+(n===cur?' on':'')+'" data-t="'+n+'">'+n+'</button>';}).join('');menu.classList.add('open');};
menu.onclick=function(e){var b=e.target.closest?e.target.closest('.mbtn'):null;if(!b){menu.classList.remove('open');return;}
 cur=b.dataset.t;curEl.textContent=cur;menu.classList.remove('open');render();};
function tile(k){var m=M[k];var tag=m[3]?'<span class="tag">'+m[3]+'</span>':'';
 return '<div class="tile" data-k="'+k+'"><div class="tl"><span>'+m[0]+'</span>'+tag+'</div>'+
 '<div class="tv"><span class="n">--</span><span class="tu">'+m[1]+'</span></div>'+
 (m[2]?'<div class="bar"><i></i></div>':'')+'</div>';}
function gaugeHTML(k,max){return '<div class="tile g" data-g="'+k+'" data-max="'+max+'">'+
 '<div class="cap">'+M[k][0]+'</div>'+
 '<svg viewBox="0 0 200 112"><path class="ab" d="M16,100 A84,84 0 0,1 184,100"/>'+
 '<path class="av" d="M16,100 A84,84 0 0,1 184,100"/></svg>'+
 '<div class="rd"><span class="n">--</span><span class="u">'+M[k][1]+'</span></div></div>';}
function render(){
 view.className='';
 if(cur==='Live'){view.classList.add('liveview');
  view.innerHTML='<div class="live-wrap"><div class="live-top">'+gaugeHTML('spd',200)+gaugeHTML('rpm',7000)+'</div>'+
   '<div class="tile wide"><div class="tl"><span>Verlauf &mdash; Drehzahl · Tempo · Last</span></div><canvas id="gc"></canvas></div>'+
   '<div class="live-tiles">'+LIVE.map(tile).join('')+'</div></div>';
 }else if(cur==='Status'){view.classList.add('status');view.innerHTML='<div class="strows" id="strows"><div class="tl">wird geladen&hellip;</div></div>'+TECHNIK;}
 else{view.classList.add('cat');view.innerHTML=CATS[cur].map(tile).join('');}
}
function updateGauge(g,val,max){var av=g.querySelector('.av');if(!av._len)av._len=av.getTotalLength();
 var f=isNaN(val)?0:Math.max(0,Math.min(1,val/max));
 av.style.strokeDasharray=av._len;av.style.strokeDashoffset=av._len*(1-f);
 g.querySelector('.n').textContent=isNaN(val)?'--':Math.round(val);}
function drawGraph(){var c=document.getElementById('gc');if(!c||!c.clientWidth)return;
 var w=c.width=c.clientWidth,h=c.height=c.clientHeight,x=c.getContext('2d');x.clearRect(0,0,w,h);
 var ln=function(a,mx,col){if(a.length<2)return;x.strokeStyle=col;x.lineWidth=2;x.beginPath();
  a.forEach(function(v,i){var px=i/(a.length-1)*w,py=h-Math.min(1,Math.max(0,v/mx))*(h-6)-3;i?x.lineTo(px,py):x.moveTo(px,py);});x.stroke();};
 ln(hist.rpm,7000,'#f0566c');ln(hist.spd,200,'#31d2e6');ln(hist.load,100,'#37c98a');}
function drawStatus(d){var on='var(--good)',off='var(--warn)';
 var row=function(k,val,ok){return '<div class="srow"><span class="dt" style="background:'+(ok?on:off)+'"></span>'+
  '<span class="k">'+k+'</span><span class="sv">'+val+'</span></div>';};
 var el=document.getElementById('strows'); if(!el)return;
 el.innerHTML=
  row('WLAN',d.wmode==='join'?('Auto-WLAN · '+d.ip):('Hotspot · '+d.ip),d.wmode==='join')+
  row('Internet',d.net?'verbunden':'offline',d.net)+
  row('Zeit',(d.clock||'--')+(d.synced?'':' (nicht sync)'),d.synced)+
  row('Speicher',d.sdtype==='-'?'keiner':(d.sdtype+' · '+d.sdfree+' / '+d.sdtot+' MB frei'),d.sdtype!=='-')+
  row('Aufnahme',(d.rec?'laeuft':'pausiert')+' · Session '+d.sess,d.rec)+
  row('CAN-Bus',d.canlive?('live · '+d.canmode+' · RX '+d.canrx):'keine Daten',d.canlive)+
  row('Firmware',d.fw||'-',true);
}
function update(d){
 // Handy-Zeit-Sync (Fallback ohne NTP): solange das Board nicht synct, Zeit schicken
 if(d.synced===false){fetch('/api/settime?epoch='+Date.now()+'&tz='+(-new Date().getTimezoneOffset())).catch(function(){});}
 document.getElementById('clk').textContent=d.clock||'--';
 var s=document.getElementById('stt');
 s.textContent=(d.sdtype&&d.sdtype!=='-'?d.sdtype:'kein SD')+' · '+(d.canlive?'CAN ✓':'CAN ✗');
 s.style.color=d.canlive?'var(--good)':'var(--warn)';
 [].forEach.call(document.querySelectorAll('.tile[data-k]'),function(t){var k=t.dataset.k,val=d[k];
  t.querySelector('.n').textContent=(val===undefined||val==='--')?'--':val;
  var bi=t.querySelector('.bar>i');if(bi){var num=parseFloat(val),mx=M[k][2];
   bi.style.width=(isNaN(num)?0:Math.max(0,Math.min(100,Math.abs(num)/mx*100)))+'%';
   bi.style.background=zcol(k,num);}});
 [].forEach.call(document.querySelectorAll('.g[data-g]'),function(g){updateGauge(g,parseFloat(d[g.dataset.g]),+g.dataset.max);});
 ['rpm','spd','load'].forEach(function(k){var n=parseFloat(d[k]);if(!isNaN(n)){hist[k].push(n);if(hist[k].length>90)hist[k].shift();}});
 if(cur==='Live')drawGraph();
 if(cur==='Status')drawStatus(d);
}
render();
setInterval(function(){fetch('/data').then(function(r){return r.json();}).then(update).catch(function(){});},1000);
</script></body></html>)PAGE";
  server.send_P(200, "text/html", PAGE);
}

void handleData() {
  // Wert erst zeigen, wenn der zugehoerige PID wirklich geantwortet hat -> sonst "--".
  auto v = [](uint8_t pid, String val) { return pidSeen[pid] ? val : String("\"--\""); };
  String j; j.reserve(1400); j = "{";
  // --- Motor ---
  j += "\"rpm\":"    + v(0x0C, String(rpm));
  j += ",\"spd\":"   + v(0x0D, String(speed_kmh));
  j += ",\"tmp\":"   + v(0x05, String(coolant_temp));
  j += ",\"oil\":"   + v(0x5C, String(oil_temp));
  j += ",\"iat\":"   + v(0x0F, String(intake_temp));
  j += ",\"map\":"   + v(0x0B, String(manifold_kpa));
  j += ",\"baro\":"  + v(0x33, String(baro_kpa));
  j += ",\"load\":"  + v(0x04, String(engine_load));
  j += ",\"absld\":" + v(0x43, String(absolute_load, 1));
  j += ",\"tmg\":"   + v(0x0E, String(timing_advance, 1));
  // --- Fahrt ---
  j += ",\"thr\":"   + v(0x47, String(throttle_pos));
  j += ",\"cthr\":"  + v(0x4C, String(cmd_throttle));
  j += ",\"pdl\":"   + v(0x5A, String(accel_pedal));
  j += ",\"dtq\":"   + v(0x61, String(demand_torque));
  j += ",\"atq\":"   + v(0x62, String(actual_torque));
  j += ",\"rtq\":"   + v(0x63, String(ref_torque));
  // --- Verbrauch ---
  j += ",\"frate\":" + v(0x5E, String(fuel_rate, 1));
  j += ",\"fuel\":"  + v(0x2F, String(fuel_level));
  j += ",\"dist\":"  + String(totalDistKm, 2);
  j += ",\"fused\":" + String(totalFuelL, 2);
  // --- Abgas / Umwelt ---
  j += ",\"o2lam\":" + v(0x34, String(o2s1_lambda, 3));
  j += ",\"o2cur\":" + v(0x34, String(o2s1_current, 1));
  j += ",\"o2s2v\":" + v(0x15, String(o2_s2_voltage, 3));
  j += ",\"o2s2t\":" + v(0x15, String(o2_s2_stft, 1));
  j += ",\"ceq\":"   + v(0x44, String(cmd_equiv_ratio, 3));
  j += ",\"stft\":"  + v(0x06, String(short_fuel_trim, 1));
  j += ",\"ltft\":"  + v(0x07, String(long_fuel_trim, 1));
  j += ",\"o2slt\":" + v(0x56, String(o2_secondary_ltft, 1));
  j += ",\"cat\":"   + v(0x3C, String(catalyst_temp));
  j += ",\"evap\":"  + v(0x2E, String(evap_purge));
  j += ",\"amb\":"   + v(0x46, String(ambient_temp));
  // --- Elektrik ---
  j += ",\"mvolt\":" + v(0x42, String(module_voltage, 2));
  // --- Harvest-Neuzugaenge ---
  j += ",\"thr11\":"   + v(0x11, String(throttle11));
  j += ",\"relthr\":"  + v(0x45, String(rel_throttle));
  j += ",\"runtime\":" + v(0x1F, String(runtime_s));
  j += ",\"distclr\":" + v(0x31, String(dist_clr));
  j += ",\"evappr\":"  + v(0x53, String(evap_press, 2));
  j += ",\"odo\":"     + v(0xA6, String(odometer_km, 1));
  j += ",\"fueltype\":"+ v(0x51, String(fuel_type));
  if (pidSeen[0xA4]) j += ",\"gear\":\"" + (gear > 0 ? String(gear) : String("N")) + "\"";
  else               j += ",\"gear\":\"--\"";
  // --- Abgeleitet: Momentanverbrauch L/100km, Ladedruck ueber Umgebung, geschaetzte Leistung ---
  String l100 = "\"--\"";
  if (pidSeen[0x5E] && pidSeen[0x0D] && speed_kmh > 3) l100 = String(fuel_rate / speed_kmh * 100.0, 1);
  j += ",\"l100\":" + l100;
  String avg100 = "\"--\"";   // Trip-Durchschnitt aus aufsummierter Strecke/Sprit
  if (totalDistKm > 0.1) avg100 = String(totalFuelL / totalDistKm * 100.0, 1);
  j += ",\"avg100\":" + avg100;
  String boost = "\"--\"";
  if (pidSeen[0x0B] && pidSeen[0x33]) boost = String(manifold_kpa - baro_kpa);
  j += ",\"boost\":" + boost;
  String ps = "\"--\"";
  if (pidSeen[0x62] && pidSeen[0x63] && pidSeen[0x0C]) {
    float nm = ref_torque * actual_torque / 100.0f;      // Ist-Moment in Nm (Anteil vom Ref-Moment)
    ps = String(nm * rpm / 9549.0f * 1.35962f, 0);       // P[kW] -> PS
  }
  j += ",\"ps\":" + ps;
  // --- System / Status ---
  char clk[40]; clockString(clk, sizeof(clk));
  bool joined = (WiFi.status() == WL_CONNECTED);
  IPAddress ip = joined ? WiFi.localIP() : WiFi.softAPIP();
  twai_status_info_t st; unsigned long tec = 0;
  if (twai_get_status_info(&st) == ESP_OK) tec = st.tx_error_counter;
  j += ",\"clock\":\"" + String(clk) + "\"";
  j += ",\"synced\":"  + String(timeSynced ? "true" : "false");
  j += ",\"wmode\":\"" + String(joined ? "join" : "ap") + "\"";
  j += ",\"ip\":\""    + ip.toString() + "\"";
  j += ",\"net\":"     + String((joined && timeSynced) ? "true" : "false");
  j += ",\"sdtype\":\""+ String(storage ? (useSD ? "SD" : "Flash") : "-") + "\"";
  j += ",\"sdtot\":"   + String((unsigned long)(storageTotalBytes() / (1024UL * 1024UL)));
  j += ",\"sdfree\":"  + String((unsigned long)(cachedFreeBytes / (1024UL * 1024UL)));
  j += ",\"rec\":"     + String(recording ? "true" : "false");
  j += ",\"sess\":"    + String(sessionNum);
  j += ",\"canlive\":" + String(rxCount > 0 ? "true" : "false");
  j += ",\"canmode\":\""+ String(currentCanMode ? "Lauschen" : "Normal") + "\"";
  j += ",\"canrx\":"   + String(rxCount);
  j += ",\"cantec\":"  + String(tec);
  j += ",\"fw\":\"OpenOBD v2\"";
  j += "}";
  server.send(200, "application/json", j);
}

// Live-CAN-Explorer: zeigt jede gesehene CAN-ID, Rate und letzte Bytes
void handleExplore() {
  String html = "<html><head><meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>CAN Explorer</title></head>"
    "<body style='font-family:monospace;padding:16px;background:#0f1020;color:#eee;'>"
    "<h2 style='font-family:sans-serif;'>CAN Explorer <a href='/' style='font-size:0.6em;color:#8ad;'>&larr; Dashboard</a></h2>"
    "<div id='meta' style='color:#aaa;margin-bottom:10px;'></div>"
    "<table style='width:100%;border-collapse:collapse;font-size:0.9em;'>"
    "<tr style='color:#aaa;text-align:left;border-bottom:1px solid #333;'>"
    "<th>ID</th><th>Ext</th><th>Hz</th><th>Count</th><th>Letzte Bytes</th></tr>"
    "<tbody id='rows'></tbody></table>"
    "<script>"
    "setInterval(()=>fetch('/api/frames').then(r=>r.json()).then(d=>{"
      "document.getElementById('meta').textContent='Eindeutige IDs: '+d.ids.length+'  |  Frames gesamt: '+d.total;"
      "d.ids.sort((a,b)=>parseInt(a.id,16)-parseInt(b.id,16));"
      "document.getElementById('rows').innerHTML=d.ids.map(f=>"
        "'<tr style=\"border-bottom:1px solid #1c1e30;\"><td style=\"color:#e9a844;\">0x'+f.id+"
        "'</td><td>'+(f.ext?'Y':'')+'</td><td>'+f.hz+'</td><td>'+f.count+'</td>"
        "<td style=\"color:#8ad;\">'+f.bytes+'</td></tr>').join('');"
    "}).catch(()=>{}),500);"
    "</script></body></html>";
  server.send(200, "text/html", html);
}

void handleApiFrames() {
  String j = "{\"total\":" + String(totalFramesSeen) + ",\"ids\":[";
  unsigned long now = millis();
  for (int i = 0; i < numFrames; i++) {
    FrameInfo* f = &frames[i];
    unsigned long span = now - f->firstMs; if (span < 1) span = 1;
    float hz = f->count * 1000.0 / span;
    char idhex[9]; snprintf(idhex, sizeof(idhex), "%03X", f->id);
    char bytes[26]; int o = 0;
    for (int b = 0; b < f->dlc && b < 8; b++) o += snprintf(bytes+o, sizeof(bytes)-o, "%02X ", f->data[b]);
    if (i) j += ",";
    j += "{\"id\":\"" + String(idhex) + "\",\"ext\":" + String(f->ext ? 1 : 0)
       + ",\"hz\":" + String(hz,1) + ",\"count\":" + String(f->count)
       + ",\"bytes\":\"" + String(bytes) + "\"}";
  }
  j += "]}";
  server.send(200, "application/json", j);
}

void handleRecord() {
  String cmd = server.hasArg("cmd") ? server.arg("cmd") : "";
  if (cmd == "toggle") recording = !recording;
  else if (cmd == "start") recording = true;
  else if (cmd == "stop") recording = false;
  else if (cmd == "new") newSessionRequested = true;   // ausgefuehrt von loop() — kein SD/rawBuf-Zugriff vom Web-Task
  server.send(200, "application/json", String("{\"recording\":") + (recording ? "true" : "false") + "}");
}

// Pfad einer Session aus dem Index /sessions.csv holen (Dateien liegen in datierten Ordnern).
// Erwartet, dass der Aufrufer fsMutex bereits haelt.
bool sessionPath(int s, bool raw, char* out, size_t n) {
  out[0] = 0;
  if (!storage) return false;
  File mf = storage->open("/sessions.csv", "r");
  if (!mf) return false;
  while (mf.available()) {
    String line = mf.readStringUntil('\n');
    int c1 = line.indexOf(','); if (c1 < 0) continue;
    if (line.substring(0, c1).toInt() != s) continue;
    int c2 = line.indexOf(',', c1 + 1); int c3 = line.indexOf(',', c2 + 1);
    if (c2 < 0 || c3 < 0) break;
    String p = raw ? line.substring(c3 + 1) : line.substring(c2 + 1, c3);
    p.trim(); strlcpy(out, p.c_str(), n);
    break;
  }
  mf.close();
  return out[0] != 0;
}

void handleDownload() {  // ?s=N&raw=1
  if (!storage) { server.send(503, "text/plain", "Kein Speicher verfuegbar"); return; }
  int s = server.hasArg("s") ? server.arg("s").toInt() : sessionNum;
  bool raw = server.hasArg("raw");
  if (xSemaphoreTake(fsMutex, pdMS_TO_TICKS(1000))) {
    char path[48];
    if (!sessionPath(s, raw, path, sizeof(path))) { xSemaphoreGive(fsMutex); server.send(404, "text/plain", "Not found"); return; }
    File f = storage->open(path, "r");
    if (!f) { xSemaphoreGive(fsMutex); server.send(404, "text/plain", "Not found"); return; }
    char fname[40]; snprintf(fname, sizeof(fname), "openobd_%ss%03d.csv", raw ? "raw_" : "", s);
    server.sendHeader("Content-Disposition", String("attachment; filename=\"") + fname + "\"");
    server.streamFile(f, "text/csv");
    f.close();
    xSemaphoreGive(fsMutex);
  } else server.send(503, "text/plain", "Busy");
}

// Sessions-Liste aus dem Index (neueste zuerst, bis zu 40). Datei-Groessen live.
void handleSessions() {
  if (!storage) { server.send(503, "text/plain", "Kein Speicher verfuegbar"); return; }
  String html = "<html><head><meta name='viewport' content='width=device-width,initial-scale=1'></head>"
    "<body style='font-family:sans-serif;padding:20px;background:#0f1020;color:#eee;max-width:680px;margin:0 auto;'>"
    "<h2 style='text-align:center;'>Sessions</h2>"
    "<div style='text-align:center;margin-bottom:15px;'><a href='/' style='color:#8ad;'>Dashboard</a></div>"
    "<table style='width:100%;border-collapse:collapse;'>"
    "<tr style='border-bottom:1px solid #333;color:#aaa;text-align:left;'><th style='padding:8px;'>Session</th>"
    "<th style='padding:8px;'>Datum</th><th style='padding:8px;text-align:right;'>Clean</th>"
    "<th style='padding:8px;text-align:right;'>Raw</th><th style='padding:8px;text-align:right;'></th></tr>";
  const int MAXR = 40;
  String ring[MAXR]; int cnt = 0;
  if (xSemaphoreTake(fsMutex, pdMS_TO_TICKS(1000))) {
    File mf = storage->open("/sessions.csv", "r");
    if (mf) { while (mf.available()) { String l = mf.readStringUntil('\n'); l.trim(); if (l.length()) { ring[cnt % MAXR] = l; cnt++; } } mf.close(); }
    int shown = cnt < MAXR ? cnt : MAXR;
    for (int i = 0; i < shown; i++) {
      String l = ring[(cnt - 1 - i) % MAXR];          // neueste zuerst
      int c1 = l.indexOf(','), c2 = l.indexOf(',', c1 + 1), c3 = l.indexOf(',', c2 + 1);
      if (c1 < 0 || c2 < 0 || c3 < 0) continue;
      int num = l.substring(0, c1).toInt();
      long long ep = atoll(l.substring(c1 + 1, c2).c_str());
      String dec = l.substring(c2 + 1, c3), rw = l.substring(c3 + 1);
      char dstr[20] = "--";
      if (ep > 0) { time_t t = (time_t)(ep / 1000) + (time_t)tzOffsetMin * 60; struct tm tv; gmtime_r(&t, &tv); strftime(dstr, sizeof(dstr), "%Y-%m-%d", &tv); }
      size_t dz = 0, rz = 0;
      File fd = storage->open(dec.c_str(), "r"); if (fd) { dz = fd.size(); fd.close(); }
      File fr = storage->open(rw.c_str(), "r");  if (fr) { rz = fr.size(); fr.close(); }
      bool active = (num == sessionNum);
      html += "<tr style='border-bottom:1px solid #1c1e30;'><td style='padding:8px;'>" +
              String(active ? "<b>s" : "s") + String(num) + String(active ? " (aktiv)</b>" : "") + "</td>"
              "<td style='padding:8px;color:#aaa;'>" + String(dstr) + "</td>"
              "<td style='padding:8px;text-align:right;'><a href='/download?s=" + String(num) + "' style='color:#8ad;'>" + String(dz / 1024) + " KB</a></td>"
              "<td style='padding:8px;text-align:right;'><a href='/download?raw=1&s=" + String(num) + "' style='color:#e9a844;'>" + String(rz / 1024) + " KB</a></td>"
              "<td style='padding:8px;text-align:right;'>" +
              String(active ? "" : ("<a href='/delete?s=" + String(num) + "' onclick='return confirm(\"Session " + String(num) + " loeschen?\")' style='color:#DC3545;'>Del</a>")) +
              "</td></tr>";
    }
    xSemaphoreGive(fsMutex);
  }
  html += "</table>";
  if (cnt > MAXR) html += "<p style='color:#666;text-align:center;'>&hellip; nur die " + String(MAXR) + " neuesten von " + String(cnt) + " gezeigt</p>";
  html += "</body></html>";
  server.send(200, "text/html", html);
}

// Session loeschen (decoded + raw + anchor); aktive Session ist geschuetzt
void handleDelete() {
  if (!storage) { server.send(503, "text/plain", "Kein Speicher verfuegbar"); return; }
  if (!server.hasArg("s")) { server.send(400, "text/plain", "?s=N fehlt"); return; }
  int s = server.arg("s").toInt();
  if (s == sessionNum) { server.send(400, "text/plain", "Aktive Session kann nicht geloescht werden"); return; }
  if (xSemaphoreTake(fsMutex, pdMS_TO_TICKS(1000))) {
    char dp[48], rp[48];
    if (sessionPath(s, false, dp, sizeof(dp)) && storage->exists(dp)) {
      storage->remove(dp);
      String ap = String(dp); ap.replace(".csv", ".anchor");   // Anchor liegt neben der Clean-CSV
      if (storage->exists(ap.c_str())) storage->remove(ap.c_str());
    }
    if (sessionPath(s, true, rp, sizeof(rp)) && storage->exists(rp)) storage->remove(rp);
    // Index /sessions.csv ohne diese Session neu schreiben
    File mf = storage->open("/sessions.csv", "r");
    String keep = "";
    if (mf) {
      while (mf.available()) {
        String l = mf.readStringUntil('\n'); if (l.length() < 2) continue;
        int c1 = l.indexOf(','); if (c1 > 0 && l.substring(0, c1).toInt() == s) continue;
        l.trim(); keep += l; keep += "\n";
      }
      mf.close();
    }
    File wf = storage->open("/sessions.csv", "w"); if (wf) { wf.print(keep); wf.close(); }
    xSemaphoreGive(fsMutex);
    server.sendHeader("Location", "/sessions");
    server.send(302);
  } else server.send(503, "text/plain", "Busy");
}

void handleLog() {
  String out; out.reserve(LOG_LINES * LOG_LINE_LEN);
  int total = logIndex < LOG_LINES ? logIndex : LOG_LINES;
  int start = logIndex < LOG_LINES ? 0 : logIndex - LOG_LINES;
  for (int i = start; i < start + total; i++) { out += logBuf[i % LOG_LINES]; out += '\n'; }
  server.send(200, "text/plain", out);
}

void handleTimeouts() {
  String html = "<html><head><meta name='viewport' content='width=device-width,initial-scale=1'></head>"
    "<body style='font-family:sans-serif;padding:20px;background:#0f1020;color:#eee;max-width:640px;margin:0 auto;'>"
    "<h2 style='text-align:center;'>PID-Status</h2>"
    "<div style='text-align:center;margin-bottom:15px;'><a href='/' style='color:#8ad;'>Dashboard</a></div>"
    "<table style='width:100%;border-collapse:collapse;'>"
    "<tr style='border-bottom:1px solid #333;color:#aaa;text-align:left;'><th style='padding:8px;'>PID</th>"
    "<th style='padding:8px;text-align:right;'>OK</th><th style='padding:8px;text-align:right;'>Timeouts</th>"
    "<th style='padding:8px;text-align:right;'>Status</th></tr>";
  for (int i = 0; i < numTrackedPids; i++) {
    PidStats* s = &pidStats[i];
    char pidHex[8]; snprintf(pidHex, sizeof(pidHex), "0x%02X", s->pid);
    const char* st = s->disabled ? "<span style='color:#DC3545;'>Disabled</span>" : "<span style='color:#28a745;'>Active</span>";
    html += "<tr style='border-bottom:1px solid #1c1e30;'><td style='padding:8px;'>" + String(pidHex) + "</td>"
            "<td style='padding:8px;text-align:right;'>" + String(s->responses) + "</td>"
            "<td style='padding:8px;text-align:right;'>" + String(s->timeouts) + "</td>"
            "<td style='padding:8px;text-align:right;'>" + String(st) + "</td></tr>";
  }
  html += "</table></body></html>";
  server.send(200, "text/html", html);
}

// Handy schickt beim Dashboard-Oeffnen Date.now() (ms) + Zeitzonen-Offset -> Anker setzen
void handleSetTime() {
  if (server.hasArg("epoch")) {
    anchorEpochMs = atoll(server.arg("epoch").c_str());
    anchorUptimeMs = millis();
    tzOffsetMin = server.hasArg("tz") ? server.arg("tz").toInt() : 0;
    bool first = !timeSynced; timeSynced = true;
    writeTimeAnchor();
    if (first) addLog("Zeit synchronisiert (Handy)");
  }
  char c[40]; clockString(c, sizeof(c));
  server.send(200, "application/json", String("{\"synced\":") + (timeSynced ? "true" : "false") + ",\"clock\":\"" + c + "\"}");
}

static uint8_t bcd(int v) { return ((v / 10) << 4) | (v % 10); }

// Time Hunt: gleicht die (per Handy bekannte) Echtzeit gegen alle CAN-Frames ab und
// markiert Kandidaten, die die Fahrzeug-Uhr enthalten koennten (binaer + BCD).
void handleTimeHunt() {
  String html = "<html><head><meta name='viewport' content='width=device-width,initial-scale=1'></head>"
    "<body style='font-family:monospace;padding:20px;background:#0f1020;color:#eee;'>"
    "<h2 style='font-family:sans-serif;'>Time Hunt &mdash; CAN-Uhr-Suche <a href='/' style='font-size:0.6em;color:#8ad;'>Dashboard</a></h2>";
  if (!timeSynced) {
    html += "<p style='color:#e9a844;'>Zuerst das Dashboard oeffnen, damit die Handy-Zeit synct. Dann gleiche ich die "
            "aktuelle Uhrzeit gegen alle CAN-Frames ab und markiere moegliche Uhr-Kandidaten.</p>";
  } else {
    struct tm t; localNow(t);
    int hh=t.tm_hour, mm=t.tm_min, ss=t.tm_sec, dd=t.tm_mday, mo=t.tm_mon+1, y2=(t.tm_year+1900)%100;
    char nb[40]; clockString(nb, sizeof(nb));
    html += "<p style='color:#aaa;'>Referenz jetzt: <b>" + String(nb) + "</b>. Suche Bytes = HH:MM, MM:SS oder Tag/Monat/Jahr.</p>"
            "<table style='width:100%;border-collapse:collapse;'><tr style='color:#aaa;text-align:left;border-bottom:1px solid #333;'>"
            "<th>ID</th><th>Offset</th><th>Treffer</th><th>Bytes</th></tr>";
    int hits = 0;
    for (int i = 0; i < numFrames; i++) {
      FrameInfo* f = &frames[i]; int d = f->dlc;
      for (int o = 0; o < d; o++) {
        const char* hit = nullptr;
        if (o+1 < d && ((f->data[o]==hh && f->data[o+1]==mm) || (f->data[o]==bcd(hh) && f->data[o+1]==bcd(mm)))) hit = "HH:MM";
        else if (o+2 < d && ((f->data[o]==dd && f->data[o+1]==mo && f->data[o+2]==y2) ||
                             (f->data[o]==bcd(dd) && f->data[o+1]==bcd(mo) && f->data[o+2]==bcd(y2)))) hit = "Tag-Monat-Jahr";
        else if (o+1 < d && ((f->data[o]==mm && f->data[o+1]==ss) || (f->data[o]==bcd(mm) && f->data[o+1]==bcd(ss)))) hit = "MM:SS";
        if (hit) {
          char idhex[8]; snprintf(idhex, sizeof(idhex), "%03X", f->id);
          char bytes[26]; int bo=0; for (int b=0;b<d&&b<8;b++) bo+=snprintf(bytes+bo,sizeof(bytes)-bo,"%02X ",f->data[b]);
          html += "<tr style='border-bottom:1px solid #1c1e30;'><td style='color:#e9a844;'>0x"+String(idhex)+
                  "</td><td>"+String(o)+"</td><td style='color:#0f7;'>"+String(hit)+"</td><td style='color:#8ad;'>"+String(bytes)+"</td></tr>";
          hits++;
        }
      }
    }
    html += "</table>";
    if (!hits) html += "<p style='color:#e9a844;margin-top:12px;'>Noch kein Treffer. Motor an, ein paar Sekunden warten, F5. Kommt dauerhaft "
                       "nichts, gibt der Golf seine Uhr am OBD-Port vermutlich nicht als Broadcast raus &mdash; dann bleibt der Handy-Sync die Quelle.</p>";
    else html += "<p style='color:#666;font-size:0.85em;margin-top:12px;'>Heuristik &mdash; Treffer koennen Zufall sein. Sicher ist ein Kandidat, "
                 "wenn er ueber die Zeit korrekt weitertickt (im CAN Explorer beobachten).</p>";
  }
  html += "</body></html>";
  server.send(200, "text/html", html);
}

void handlePids() {
  String html = "<html><head><meta name='viewport' content='width=device-width,initial-scale=1'></head>"
    "<body style='font-family:sans-serif;padding:20px;background:#0f1020;color:#eee;max-width:640px;margin:0 auto;'>"
    "<h2 style='text-align:center;'>PID-Discovery</h2>"
    "<div style='text-align:center;margin-bottom:12px;'><a href='/' style='color:#8ad;'>Dashboard</a> &nbsp; "
    "<button onclick=\"fetch('/api/rediscover').then(()=>{document.body.style.opacity=0.5;setTimeout(()=>location.reload(),6000)})\" "
    "style='padding:6px 14px;border:0;border-radius:5px;background:#e9a844;color:#111;'>Discovery neu ausf&uuml;hren</button></div>";
  if (!discoveryDone) {
    html += "<p style='text-align:center;color:#e9a844;'>Noch keine Antwort vom Auto &mdash; es werden die bekannten Standard-PIDs gepollt (Fallback).</p>";
  } else {
    html += "<p style='text-align:center;color:#aaa;'>Dein Fahrzeug meldet <b style='color:#0f7;'>" + String(numSupported) +
            "</b> unterstuetzte Standard-PIDs.</p>"
            "<table style='width:100%;border-collapse:collapse;font-family:monospace;'>"
            "<tr style='color:#aaa;text-align:left;border-bottom:1px solid #333;'><th style='padding:6px;'>PID</th><th style='padding:6px;'>Status</th></tr>";
    for (int p = 1; p < 256; p++) {
      if (!pidSupported[p]) continue;
      const char* tag = canDecode((uint8_t)p)
        ? "<span style='color:#0f7;'>dekodiert</span>"
        : "<span style='color:#e9a844;'>roh (noch nicht dekodiert)</span>";
      char hx[8]; snprintf(hx, sizeof(hx), "0x%02X", p);
      html += "<tr style='border-bottom:1px solid #1c1e30;'><td style='padding:6px;color:#8ad;'>" +
              String(hx) + "</td><td style='padding:6px;'>" + String(tag) + "</td></tr>";
    }
    html += "</table><p style='color:#666;font-size:0.85em;margin-top:14px;'>Gelb = das Auto liefert diesen Wert, "
            "OpenOBD uebersetzt ihn nur noch nicht in Klartext. Kommt in einer spaeteren Phase.</p>";
  }
  html += "</body></html>";
  server.send(200, "text/html", html);
}

void handleRediscover() {
  rediscoverRequested = true;   // loop() fuehrt die Discovery aus (blockiert Polling kurz)
  server.send(200, "application/json", "{\"ok\":true}");
}

// Auto-Probe: alle plausiblen CAN-Konfigurationen in einem Rutsch durchtesten.
// Sucht die Kombination, bei der der Golf tatsaechlich antwortet — damit man
// nicht fuer jede Hypothese einzeln zum Auto laufen muss. Laeuft in loop() (Core 1).
void runAutoProbe() {
  static const int    rates[]  = { 500, 250 };
  static const uint32_t reqIds[] = { 0x7DF, 0x7E0, 0x18DB33F1 };  // funktional, physisch, 29-bit
  static const uint8_t  pads[]   = { 0x00, 0x55 };
  int n = 0;
  n += snprintf(probeReport+n, sizeof(probeReport)-n, "Auto-Probe laeuft...\n");
  int savedRate = canBitrateK; uint32_t savedId = obdReqId; uint8_t savedPad = obdPad;
  bool win = false;

  for (int ri = 0; ri < 2 && !win; ri++) {
    canBitrateK = rates[ri];
    reinitCan(0);
    delay(150);
    // passiver Bus-Verkehr auf dieser Bitrate? (starker Hinweis auf richtige Rate)
    unsigned long t0 = millis(); int seen = 0; twai_message_t rx;
    while (millis() - t0 < 400) if (twai_receive(&rx, pdMS_TO_TICKS(20)) == ESP_OK) { captureFrame(rx); seen++; }
    n += snprintf(probeReport+n, sizeof(probeReport)-n, "\n[%dk] passiv gesehen: %d Frames\n", rates[ri], seen);

    for (int ii = 0; ii < 3 && !win; ii++) {
      for (int pi = 0; pi < 2 && !win; pi++) {
        obdReqId = reqIds[ii]; obdPad = pads[pi];
        bool ok = probeSupportedPids();   // nutzt reqId/pad/extd automatisch
        if (ok) {
          n += snprintf(probeReport+n, sizeof(probeReport)-n,
                        "  ID 0x%-8X Pad 0x%02X -> ANTWORT! (%d PIDs)\n", reqIds[ii], pads[pi], numSupported);
          win = true;
        } else {
          n += snprintf(probeReport+n, sizeof(probeReport)-n,
                        "  ID 0x%-8X Pad 0x%02X -> still\n", reqIds[ii], pads[pi]);
        }
      }
    }
  }

  if (win) {
    n += snprintf(probeReport+n, sizeof(probeReport)-n,
                  "\nGEFUNDEN: %dk, ID 0x%X, Pad 0x%02X, %d PIDs. Uebernommen.\n",
                  canBitrateK, obdReqId, obdPad, numSupported);
    discoveryDone = true;
    resetPidStats();   // vorher (unter 0x7DF) deaktivierte PIDs wieder freigeben -> echtes Polling
    writePidsFile();
  } else {
    canBitrateK = savedRate; obdReqId = savedId; obdPad = savedPad;
    n += snprintf(probeReport+n, sizeof(probeReport)-n,
                  "\nKEINE Kombination lieferte eine Antwort.\n"
                  "-> Motor lief? Stecker fest? Dann evtl. Gateway sperrt OBD.\n");
  }
  reinitCan(0);   // mit finaler (Gewinner- oder zurueckgesetzter) Konfig neu starten
}

void handleProbe() {
  probeRequested = true;   // loop() fuehrt aus (blockiert ~10-20s)
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleProbeReport() { server.send(200, "text/plain", probeReport); }

void handleCanMode() {  // ?m=normal|listen, ohne Arg: umschalten — ausgefuehrt von loop()
  String m = server.hasArg("m") ? server.arg("m") : "toggle";
  if (m == "listen") requestedCanMode = 1;
  else if (m == "normal") requestedCanMode = 0;
  else requestedCanMode = (currentCanMode == 0) ? 1 : 0;
  server.send(200, "application/json", "{\"ok\":true}");
}

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

// ================= Hoeflicher UDS-Scan ("nur lesen, was das Auto freiwillig gibt") =========
// Sendet AUSSCHLIESSLICH Leseanfragen (Service 0x22) in der Standard-Sitzung. Wird eine DID
// abgelehnt (NRC) -> wird das akzeptiert, wir gehen weiter. KEINE Sitzungssteuerung (0x10),
// KEIN Security-Access (0x27), KEIN Schreiben (0x2E/0x31). Standard-Identifikations-DIDs
// benennen die Steuergeraete selbst, statt zu raten. Nur per Knopf (kurzes Blocking, wie Probe).
struct PoliteTarget { uint8_t ecu; uint16_t did; const char* name; };
const PoliteTarget politeList[] = {
  { 0x01, 0xF197, "Motor 0x01 Systemname" }, { 0x01, 0xF187, "Motor 0x01 Teilenummer" },
  { 0x01, 0xF189, "Motor 0x01 SW-Version" }, { 0x01, 0xF190, "Motor 0x01 VIN" },
  { 0x02, 0xF197, "STG 0x02 Systemname" },   { 0x02, 0xF187, "STG 0x02 Teilenummer" },
  { 0x02, 0xF189, "STG 0x02 SW-Version" },
  { 0x10, 0xF197, "STG 0x10 Systemname" },   { 0x10, 0xF187, "STG 0x10 Teilenummer" },
  { 0x10, 0xF189, "STG 0x10 SW-Version" },
};
const int NUM_POLITE = sizeof(politeList) / sizeof(politeList[0]);
char politeReport[1500] = "Noch kein hoeflicher Scan gelaufen. Am Auto (Zuendung an): Knopf druecken.";
volatile bool politeScanRequested = false;

void runPoliteScan() {
  int n = 0;
  n += snprintf(politeReport + n, sizeof(politeReport) - n,
                "Hoeflicher UDS-Scan (nur Lesen 0x22, Standard-Sitzung):\n\n");
  for (int i = 0; i < NUM_POLITE && n < (int)sizeof(politeReport) - 90; i++) {
    uint8_t ecu = politeList[i].ecu; uint16_t did = politeList[i].did;
    twai_message_t rx;
    while (twai_receive(&rx, 0) == ESP_OK) captureFrame(rx);   // Queue leeren, nichts verlieren
    char res[80]; strcpy(res, "keine Antwort");
    if (requestUdsData(ecu, did)) {
      uint32_t rsp = UDS_RSP(ecu);
      unsigned long t0 = millis();
      while (millis() - t0 < 300) {
        if (twai_receive(&rx, pdMS_TO_TICKS(20)) != ESP_OK) continue;
        captureFrame(rx);
        if (rx.identifier != rsp) continue;
        uint8_t pci = rx.data[0] & 0xF0;
        if (pci == 0x00 && rx.data[1] == 0x62) {              // Single-Frame-Positivantwort
          uint8_t len = rx.data[0] & 0x0F;
          char hex[26] = "", asc[12] = ""; int ho = 0, ao = 0;
          for (int b = 4; b < 1 + len && b < 8; b++) {        // Datenbytes nach '62 DIDhi DIDlo'
            ho += snprintf(hex + ho, sizeof(hex) - ho, "%02X", rx.data[b]);
            char c = (char)rx.data[b]; if (ao < 11) asc[ao++] = (c >= 32 && c < 127) ? c : '.';
          }
          asc[ao] = 0;
          snprintf(res, sizeof(res), "OK  %s  \"%s\"", hex, asc);
          break;
        } else if (pci == 0x10 && rx.data[2] == 0x62) {       // First Frame -> lange Antwort
          snprintf(res, sizeof(res), "existiert (mehrteilig >7B, Flow-Control folgt spaeter)");
          break;
        } else if (rx.data[1] == 0x7F) {                      // Ablehnung -> hoeflich akzeptieren
          snprintf(res, sizeof(res), "abgelehnt (NRC 0x%02X) - ok, weiter", rx.data[3]);
          break;
        }
      }
    } else strcpy(res, "Senden fehlgeschlagen");
    n += snprintf(politeReport + n, sizeof(politeReport) - n, "%-24s: %s\n", politeList[i].name, res);
    delay(60);   // hoeflich: den Bus nicht fluten
  }
  // Bericht auf SD sichern (einmalig, unter Mutex)
  if (storage && xSemaphoreTake(fsMutex, pdMS_TO_TICKS(300))) {
    File f = storage->open("/uds_scan.txt", "w");
    if (f) { f.print(politeReport); f.close(); }
    xSemaphoreGive(fsMutex);
  }
  addLog("Hoeflicher UDS-Scan fertig");
}

void handlePoliteScan() { politeScanRequested = true; server.send(200, "application/json", "{\"ok\":true}"); }

void handleUdsTest() { udsTestRequested = true; server.send(200, "application/json", "{\"ok\":true}"); }

void handleUds() {
  String h = "<html><head><meta name='viewport' content='width=device-width,initial-scale=1'></head>"
    "<body style='font-family:sans-serif;padding:18px;background:#0f1020;color:#eef;'>"
    "<h2>UDS &mdash; Phase 2 <a href='/' style='font-size:.6em;color:#8ad;'>Dashboard</a></h2>"
    "<p style='color:#aaa;'>Aktive VAG-Diagnose (Service 0x22, 29-bit). Poller: <b style='color:" +
    String(udsPollEnabled ? "#0f7'>AN" : "#e94'>AUS") + "</b></p>"
    "<button onclick=\"fetch('/api/udstest');this.textContent='teste...';setTimeout(()=>location.reload(),1600)\" "
    "style='padding:12px 20px;border:0;border-radius:10px;background:#e9a844;color:#111;font-weight:bold;'>UDS-Test DSG (0x18DA10F1)</button>"
    "<p style='color:#0f7;font-weight:bold;'>" + String(udsTestResult) + "</p>"
    "<hr style='border-color:#222;margin:14px 0;'>"
    "<h3 style='margin:0 0 4px;'>Hoeflicher Scan</h3>"
    "<p style='color:#aaa;font-size:.9em;margin:0 0 8px;'>Liest nur (Service 0x22) und benennt die "
    "Steuergeraete selbst. Abgelehnte DIDs werden akzeptiert &mdash; kein Zwang, keine Sitzung, "
    "kein Schreiben. Ergebnis auch als <code>/uds_scan.txt</code> auf der SD.</p>"
    "<button onclick=\"fetch('/api/politescan');this.textContent='scanne ~4s...';setTimeout(()=>location.reload(),4500)\" "
    "style='padding:12px 20px;border:0;border-radius:10px;background:#37c98a;color:#04120c;font-weight:bold;'>Hoeflichen Scan starten</button>"
    "<pre style='background:#0b111d;border:1px solid #1d2635;border-radius:10px;padding:12px;overflow:auto;white-space:pre-wrap;'>" + String(politeReport) + "</pre>"
    "<table style='width:100%;border-collapse:collapse;font-family:monospace;margin-top:8px;'>"
    "<tr style='color:#aaa;text-align:left;border-bottom:1px solid #333;'><th>STG</th><th>DID</th><th>Name</th><th>Wert</th></tr>";
  for (int i = 0; i < NUM_UDS; i++) {
    UdsItem &it = udsTable[i]; char b[120];
    snprintf(b, sizeof(b), "<tr style='border-bottom:1px solid #1c1e30;'><td>0x%02X</td><td style='color:#e9a844'>0x%04X</td>"
             "<td>%s</td><td style='color:#8ad'>%s %s</td></tr>",
             it.ecu, it.did, it.name, it.seen ? String(it.value).c_str() : "--", it.unit);
    h += b;
  }
  h += "</table><p style='color:#666;font-size:.85em;margin-top:12px;'>Gefundene DIDs in <code>udsTable[]</code> "
       "eintragen, dann <code>udsPollEnabled=true</code>. Multiframe-Antworten (&gt;7 Byte) folgen in einer spaeteren Phase.</p>"
       "</body></html>";
  server.send(200, "text/html", h);
}

// ---------- Zeit per NTP (nur wenn wir Internet ueber das Auto-WLAN haben) ----------
// Setzt denselben Anker wie der Handy-Sync (UTC-Epoche <-> Uptime); der lokale
// Offset wird erst in localNow() gewendet. -> Keine Uhr-Hardware noetig.
void syncTimeNtp() {
  configTime(0, 0, cfg.ntp);   // UTC holen
  struct tm ti;
  if (getLocalTime(&ti, 5000)) {
    anchorEpochMs  = (long long)time(nullptr) * 1000LL;
    anchorUptimeMs = millis();
    tzOffsetMin    = cfg.tzMin;
    timeSynced     = true;
    writeTimeAnchor();
    addLog("Zeit per NTP gesetzt (%s)", cfg.ntp);
  } else {
    addLog("NTP-Sync fehlgeschlagen (kein Internet im WLAN?)");
  }
}

// ---------- WLAN: entweder ins Auto-WLAN einwaehlen (Internet + NTP + Handy behaelt
//            Netz) oder eigenen Hotspot aufmachen. Bei Join-Fehler: Fallback-AP. ----------
void setupWifi() {
  bool joined = false;
  if (strcmp(cfg.wifiMode, "join") == 0 && cfg.ssid[0]) {
    WiFi.mode(WIFI_STA);
    WiFi.begin(cfg.ssid, cfg.pass);
    addLog("WLAN: verbinde mit '%s' ...", cfg.ssid);
    unsigned long t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 9000) delay(200);
    joined = (WiFi.status() == WL_CONNECTED);
    if (joined) { addLog("WLAN verbunden, IP: %s", WiFi.localIP().toString().c_str()); syncTimeNtp(); }
    else        { addLog("WLAN-Beitritt fehlgeschlagen"); }
  }
  if (!joined) {
    if (strcmp(cfg.wifiMode, "join") == 0 && !cfg.fallbackAp) {
      addLog("WLAN: join gescheitert, kein Fallback -> offline");
    } else {
      WiFi.mode(WIFI_AP);
      WiFi.softAP(cfg.apSsid, cfg.apPass);
      addLog("WLAN AP '%s', IP: %s", cfg.apSsid, WiFi.softAPIP().toString().c_str());
    }
  }
  if (MDNS.begin("openobd")) { MDNS.addService("http", "tcp", 80); addLog("mDNS: http://openobd.local"); }
}

// (Config kommt aus include/config.h zur Compile-Zeit — kein Laden von SD noetig.)

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);

  // T-CAN485 hochfahren (vor SD + TWAI): 5V-Boost an, Transceiver in Normalmodus
  pinMode(PIN_5V_EN, OUTPUT); digitalWrite(PIN_5V_EN, HIGH);
  pinMode(CAN_SE_PIN, OUTPUT); digitalWrite(CAN_SE_PIN, LOW);
  delay(10);

  fsMutex = xSemaphoreCreateMutex();

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

  server.on("/", HTTP_GET, handleRoot);
  server.on("/data", HTTP_GET, handleData);
  server.on("/explore", HTTP_GET, handleExplore);
  server.on("/api/frames", HTTP_GET, handleApiFrames);
  server.on("/api/record", HTTP_GET, handleRecord);
  server.on("/download", HTTP_GET, handleDownload);
  server.on("/sessions", HTTP_GET, handleSessions);
  server.on("/log", HTTP_GET, handleLog);
  server.on("/timeouts", HTTP_GET, handleTimeouts);
  server.on("/pids", HTTP_GET, handlePids);
  server.on("/api/settime", HTTP_GET, handleSetTime);
  server.on("/timehunt", HTTP_GET, handleTimeHunt);
  server.on("/delete", HTTP_GET, handleDelete);
  server.on("/api/rediscover", HTTP_GET, handleRediscover);
  server.on("/api/canmode", HTTP_GET, handleCanMode);
  server.on("/api/probe", HTTP_GET, handleProbe);
  server.on("/probereport", HTTP_GET, handleProbeReport);
  server.on("/uds", HTTP_GET, handleUds);
  server.on("/api/udstest", HTTP_GET, handleUdsTest);
  server.on("/api/politescan", HTTP_GET, handlePoliteScan);
  server.begin();

  xTaskCreatePinnedToCore(webServerTask, "WebServer", 8192, NULL, 1, NULL, 0);

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
  testUdsConnection();        // Phase 2: einmalig pruefen, ob das DSG UDS spricht
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
  // 0) Vom Webserver angeforderte Aktionen hier ausfuehren — nur loop() darf
  //    rawBuf und Session-Dateien anfassen (Race-Fix, s. Kopfkommentar)
  if (newSessionRequested) {
    newSessionRequested = false;
    flushRaw(); openNewSession(); recording = true;
  }
  if (rediscoverRequested) {
    rediscoverRequested = false;
    discoverSupportedPids();   // blockiert das Polling wenige Sekunden — bewusst
  }
  if (probeRequested) {
    probeRequested = false;
    runAutoProbe();            // testet alle Bitraten/Adressen/Paddings (~10-20s)
    requestPending = false; txBackoffUntil = 0;
  }
  if (udsTestRequested) {
    udsTestRequested = false;
    testUdsConnection();       // Phase 2: UDS-Verbindungstest (kurzes Blocking, wie Discovery)
    requestPending = false; txBackoffUntil = 0;
  }
  if (politeScanRequested) {
    politeScanRequested = false;
    runPoliteScan();           // hoeflicher Lese-Scan (kurzes Blocking, nur 0x22)
    requestPending = false; txBackoffUntil = 0;
  }
  if (requestedCanMode >= 0) {
    int m = requestedCanMode; requestedCanMode = -1;
    if (m != currentCanMode) {
      twai_stop(); twai_driver_uninstall();
      initCan(m);
      requestPending = false; txBackoffUntil = 0;
    }
  }

  // 1) Naechste Anfrage senden, falls keine offen (nicht im Lausch-Modus)
  if (currentCanMode == 0 && !requestPending && millis() >= txBackoffUntil) sendObd2Request(getNextPid());
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
