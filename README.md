# OpenOBD

CAN-Bus-Datenlogger für den **VW Golf 8.5 eTSI** — lokal-first, kein Abo, Open Source.
Maximaler Datenfang: pollt Standard-OBD2 **und** schneidet jedes CAN-Frame roh mit.
Fork von [roypeter/esp32-obd2-logger](https://github.com/roypeter/esp32-obd2-logger).

> **Status: v2.** Läuft am Fahrzeug: 29-Bit-OBD (~28 PIDs), Roh-Log, Konfig in `include/config.h`
> (Compile-Zeit), Normal-/Dev-Modus, WLAN-Join + NTP, Cockpit-Dashboard, sanftes UDS, Dev-Streaming.
> Aufbau, Konfiguration & Bedienung: siehe unten.

---

## Was v1 kann

- **Aktiv-Polling** der Standard-OBD2-PIDs → dekodierte, saubere Werte in `s{NNN}.csv`.
- **Roh-Mitschnitt jedes CAN-Frames — empfangen UND gesendet** → `raw_s{NNN}.csv` im
  **SavvyCAN-Format** (direkt importierbar). Request/Response-Paare bleiben zusammen,
  Zeitstempel 64 bit (kein Überlauf auf langen Fahrten).
- **Live-CAN-Explorer** im Browser (`/explore`): jede gesehene CAN-ID, Rate (Hz),
  Zähler, letzte Bytes — live. So siehst du sofort, was dein Golf hergibt.
- **Auto-Record beim Boot** + Pause/Weiter + „Neue Session" im Dashboard.
- **Robust:** Datei-Schluss nach jedem Write (kein Datenverlust bei Zündung-aus),
  CAN-Bus-Off-Recovery, alle Antwort-IDs `0x7E8–0x7EF`, Fallback SD → interner Flash.
- **PID-Discovery** (`/pids`): fragt den Golf per Bitmaske (`0x00`/`0x20`/…), welche
  Standard-PIDs er wirklich unterstützt, pollt nur die — und listet auch die
  unterstützten, aber noch nicht dekodierten PIDs. Fallback auf die bekannte Liste,
  wenn kein Auto antwortet (Bench-Test). Ergebnis auch in `/pids.txt`.
  **„Discovery neu ausführen"-Button**, falls das Steuergerät beim Boot noch schlief.
- **Ehrliches Dashboard:** Werte erscheinen erst, wenn der PID wirklich geantwortet
  hat — vorher `--` statt irreführender Nullen.
- **Sessions-Verwaltung:** paginierte Liste, Download (decoded + raw), Löschen alter
  Sessions direkt im Browser (aktive Session ist geschützt).
- **Threadsicher:** nur die Log-Schleife schreibt auf SD/Puffer; Web-Aktionen
  (Neue Session, Re-Discovery) laufen über Flags. Alle SD-Zugriffe unter Mutex.
- **OLED und Gang-Schätzung entfernt** (Auto liefert den Gang selbst).

### Neu in v2
- **Konfiguration in `include/config.h`** — fest beim Flashen eingebaut (keine `config.json` auf SD). Zwei Flash-Ziele: **Normal** & **Dev** (siehe unten).
- **WLAN-Join + NTP** — der Logger wählt sich ins Auto-WLAN ein: Handy behält Internet, Zeit kommt automatisch (kein DS3231 nötig), Zugriff über `http://openobd.local`.
- **Cockpit-Dashboard** — dunkles Instrument-Cluster: Gauges, Ampel-Kacheln, Verlaufskurve, abgeleitete Werte (Verbrauch L/100 *jetzt* & *Ø*, Leistung PS, Ladedruck), Kategorie-Tabs + Status-Tab, Hamburger-Menü mit großen (fahrsicheren) Schaltflächen.
- **Sanftes UDS** (`/uds`) — „höflicher Scan": liest nur (Service 0x22), akzeptiert Ablehnungen, keine Sitzung/kein Zwang/kein Schreiben; benennt die Steuergeräte selbst.
- **Dev-Streaming** — Roh-Frames drahtlos per TCP (Python/SavvyCAN-Capture) oder MQTT, ohne SD zu ziehen.

Noch offen: Multiframe-UDS (VIN/lange Namen via Flow-Control), native SavvyCAN-GVRET-Verbindung, VAG-Spezial-Dashboards (Energiefluss/DSG/48V) — sobald die DIDs bekannt sind.

---

## Hardware

| Teil | Status |
|---|---|
| LilyGO T-CAN485 (ESP32 + SN65HVD231 + microSD) | ⏳ bestellt |
| USB-C-Datenkabel | ⏳ |
| **microSD ≤ 32 GB, FAT32** | ⏳ |
| DS3231SN + AT24C32 RTC-Modul (I2C) | für Phase „Zeitstempel" |
| Powerbank (erster Bench-Test) | ✅ |
| OBD2-Breakout-Kabel 16-Pin | ⏳ bestellt |

> **Karte:** 32 GB reicht für **Jahre** Fahrdaten (auch mit Roh-Log). 64 GB ginge auch,
> muss dann aber zwingend **FAT32** formatiert sein — die ESP32-SD-Lib kann kein exFAT.

### Pin-Belegung (im Code hinterlegt, Quelle: offizielles LilyGO-Repo)

| Funktion | GPIO |
|---|---|
| 5V-Boost-Enable (versorgt CAN + SD) | 16 |
| CAN TX / RX / Silent-Enable | 27 / 26 / 23 (per Selbsttest bestätigt; config.h 26/27 war vertauscht) |
| SD SCLK / MISO / MOSI / CS | 14 / 2 / 15 / 13 |
| **DS3231 I2C SDA / SCL (Default)** | **32 / 33** |

> ⚠️ Die DS3231-Pins **32/33** sind ein sinnvoller Default — beim Anlöten bitte gegen
> die tatsächlich freien Header-Pins deines Boards abgleichen und ggf. die zwei
> `#define RTC_SDA_PIN` / `RTC_SCL_PIN` oben in `src/main.cpp` anpassen.

---

## Toolchain (einmalig)

**PlatformIO.** Entweder die **VS-Code-Extension „PlatformIO IDE"**, oder per CLI:

```bash
brew install platformio        # macOS
# oder: pipx install platformio
pio --version
```

## Bauen & Flashen

```bash
pio run -e t-can485                                   # Normal kompilieren (geht OHNE Board)
pio run -e t-can485      --target upload --target monitor   # Normal flashen + Konsole
pio run -e t-can485-dev  --target upload --target monitor   # Dev flashen + Konsole
```
> Konfig vorher in `include/config.h` einstellen (WLAN, Intervalle, …). Modus Normal/Dev
> kommt übers Flash-Ziel — nicht von der SD.

> Upload hängt? **BOOT** halten, **RST** tippen, **BOOT** loslassen, dann Upload.
> `pio device list` zeigt den Port; manche Klone brauchen den CP210x/CH340-Treiber.

---

## Konfiguration (`include/config.h`) & Betriebsmodi

Die Konfiguration liegt **im Projekt** in `include/config.h` und wird **beim Flashen**
fest eingebaut — **keine `config.json` auf der SD**, kein Karten-Rein-Raus. Einstellen,
flashen, fertig.

Für die zwei Modi gibt es zwei **Flash-Ziele** — du stellst nichts um, flashst nur das
richtige Ziel:

| Modus | Flash-Ziel | Dashboard | Roh-Log |
|---|---|---|---|
| **Normal** | `-e t-can485` | ✅ Cockpit | aus (minimal) |
| **Dev** | `-e t-can485-dev` | aus (nur Technik-Links) | ✅ voll + Stream |

```bash
python3 -m platformio run -e t-can485      --target upload --target monitor   # Normal
python3 -m platformio run -e t-can485-dev  --target upload --target monitor   # Dev
```
Das Dev-Ziel setzt `CFG_DEV_MODE=true` und schaltet damit automatisch Dashboard aus +
Roh-Log & TCP-Stream an. Alles Weitere stellst du in `include/config.h` ein:

| Einstellung | Bedeutung |
|---|---|
| `CFG_DEV_MODE` | Grundmodus (meist übers Flash-Ziel gesetzt) |
| `CFG_LOG_DECODED` / `CFG_LOG_RAW` | dekodiertes / Roh-CSV schreiben |
| `CFG_LOG_INTERVAL_MS` | Schreibrate des dekodierten Logs |
| `CFG_DISCOVERY` | beim Boot PID-Discovery fahren |
| `CFG_WIFI_MODE` | `"ap"` (eigener Hotspot) oder `"join"` (ins Auto-WLAN) |
| `CFG_WIFI_SSID` / `CFG_WIFI_PASS` | Zugangsdaten des Auto-Hotspots (bei `"join"`) |
| `CFG_FALLBACK_AP` | bei fehlgeschlagenem Join eigenen AP aufmachen |
| `CFG_NTP` / `CFG_TZ_MIN` | NTP-Server + lokaler Offset (CET=60, CEST=120) |
| `CFG_TCP_PORT` | Port des Dev-TCP-Roh-Streams |
| `CFG_MQTT` / `CFG_MQTT_HOST` … | Roh-Frames per MQTT publizieren |

### WLAN & Zeit
- `wifi.mode:"join"` → der ESP32 hängt sich ins **Auto-WLAN**. Dein Handy behält
  Internet, der Logger holt sich die **Uhrzeit per NTP** (kein DS3231 nötig), Zugriff
  über **`http://openobd.local`**. Klappt der Beitritt nicht, macht er (bei
  `fallback_ap:true`) automatisch seinen eigenen Hotspot auf.
- `wifi.mode:"ap"` → eigener Hotspot `OpenOBD` / `openobd1234`, `http://192.168.4.1`;
  Zeit dann per Handy-Sync beim Öffnen des Dashboards.

### Dashboard
Dunkles Cockpit, Vollbild ohne Scrollen (Hoch- & Querformat), großes **Hamburger-Menü**.
Tabs: **Live** (2 Gauges + Verlaufskurve + abgeleitete Kacheln), **Motor / Fahrt /
Verbrauch / Abgas / Elektrik** (jeder Wert als Zahl + Balken + Ampelfarbe), **Status**
(WLAN/Internet/Zeit/Speicher/Aufnahme/CAN mit An-Aus-Anzeige). Kennzeichen an den
Kacheln: **„jetzt"** = Momentanwert, **„Ø"** = Trip-Durchschnitt.

### Dev-Streaming (Bus drahtlos ansehen, ohne SD zu ziehen)
Im Dev-Modus streamt der Logger jedes empfangene Frame drahtlos (Netz-I/O läuft auf
Core 0, damit die CAN-Erfassung nicht stockt):
- **TCP** (`CFG_TCP_PORT`, Standard 3333) — Zeilen im SavvyCAN-Format. Auslesen z. B.
  `nc <ip> 3333`, aus Python per `socket`, oder in eine Datei umleiten und in SavvyCAN
  importieren.
- **MQTT** (`CFG_MQTT`) — publiziert jedes Frame als Zeile auf `CFG_MQTT_TOPIC`
  (für eigene Python-Tools, siehe `analysis/`).

> Hinweis: native **SavvyCAN-Netzwerkverbindung (GVRET)** und **Multiframe-UDS** sind als
> nächste Iteration geplant und noch am realen Gerät zu verifizieren.

### UDS — höflicher Scan (`/uds`)
„Nur lesen, was das Auto freiwillig gibt." Der Knopf **Höflichen Scan starten** schickt
ausschließlich Leseanfragen (Service 0x22) in der Standard-Sitzung und **benennt die
Steuergeräte selbst** (Identifikations-DIDs). Abgelehnte DIDs werden akzeptiert —
**keine Sitzungssteuerung, kein Security-Access, kein Schreiben**. Ergebnis auch als
`/uds_scan.txt` auf der SD.

### Dateistruktur auf der SD
Logs werden nach **Jahr/Monat** einsortiert; saubere und Roh-Daten getrennt:

```
/pids.txt  /session.txt          unterstützte PIDs · Session-Zähler
/sessions.csv  /lasttime.txt     Sessions-Index · letzte bekannte Zeit (RTC-Ersatz)
/uds_scan.txt                    Ergebnis des höflichen UDS-Scans
/data/2026/07/s001.csv           saubere/dekodierte Logs (+ s001.anchor)
/dev/2026/07/raw_s001.csv        Roh-Logs (SavvyCAN), nur im Dev-Modus
```
> Konfig liegt **nicht** auf der SD, sondern in `include/config.h` (siehe oben).

> **Zeit ohne RTC:** Das Board kennt beim Booten (AP-Modus, ohne Handy/Internet) die
> Uhrzeit nicht. OpenOBD merkt sich die **letzte bekannte Zeit** (`/lasttime.txt`) und
> datiert die Ordner damit provisorisch, bis ein echter Sync (NTP im Auto-WLAN, oder
> Handy beim Dashboard-Öffnen) sie korrigiert. Mit `wifi.mode:"join"` steht die echte
> Zeit sofort beim Boot.

---

## Verkabelung: Breakout-Kabel ↔ T-CAN485

Farbbelegung des vorhandenen 16-adrigen OBD2-Breakout-Kabels (lt. Hersteller-Pinout):

| OBD2-Pin | Aderfarbe | Funktion | Anschluss |
|---|---|---|---|
| 6 | **Grün** | CAN-H | direkt Schraubklemme **CANH** |
| 14 | **Grün-Weiß** | CAN-L | direkt Schraubklemme **CANL** |
| 5 | **Hellblau** | Masse (Signal) | direkt Schraubklemme **GND** |
| 4 | **Orange** | Masse (Chassis) | per Wago an Wandler-Schwarz (−) |
| 1 | **Braun** | vermutl. Zündungsplus* | per Wago → 2A-Sicherung → Wago → Wandler-Rot (+) |
| 16 | **Rot** | ⚡ 12V **Dauerplus** | **NICHT anschließen!** |
| übrige 10 Adern | — | K-Line etc. | einzeln isolieren |

\* erst mit Multimeter bestätigen (Zündung an ≈ 12 V / aus = 0 V, gegen Orange messen).
Alle drei Wago-Verbindungen sind 2-adrig — das freie dritte Loch der 221-413 bleibt leer.

> ⚠️ **Die drei Gebote:**
> 1. **Grün ≠ Grün-Weiß** — CAN-H und CAN-L vertauscht = es funktioniert einfach nichts.
> 2. **Rot (Pin 16) niemals an das Board** — das sind ungeregelte 12–14,5 V Dauerplus.
>    Strom kommt in der Testphase ausschließlich per USB-C (Powerbank).
> 3. **Alle 13 unbenutzten Adern einzeln isolieren** (Tape/Schrumpfschlauch oder
>    Wago-Klemme), BEVOR das Kabel ins Auto gesteckt wird — eine blanke rote Ader,
>    die zufällig eine andere berührt, kann teuer werden.

**Multimeter-Test für später (Power-Strategie, Briefing Punkt 2):** Zündung aus →
Spannung zwischen **Braun (Pin 1)** und **Orange (Pin 4)** messen. Zündung an → nochmal.
0 V ↔ ~12 V = Braun ist Zündungsplus → Stromquelle für den Festeinbau.

### Stromversorgung Festeinbau (lötfrei)

Der T-CAN485 hat zwar einen 5–12-V-VIN-Eingang, aber das Bordnetz liefert bei laufendem
Motor **13,8–14,7 V** (plus Spitzen) — dauerhaft über der Spezifikation. **Deshalb nie
direkt anschließen**, sondern über einen Wandler:

- **KFZ-Spannungswandler 12V/24V → 5V USB** (vergossener Einbau-Block mit offenen
  Eingangsadern, ~10 €) — verträgt 9–30 V und filtert Bordnetz-Spitzen
- Verdrahtung komplett mit **Wago-221-Klemmen** (kein Löten):
  Braun (Pin 1) → 2A-Sicherung → Wandler-Rot; Orange (Pin 4) → Wandler-Schwarz;
  Wandler-USB-C → T-CAN485. Masse fürs Board kommt separat über Hellblau (Pin 5)
  direkt in die GND-Schraubklemme.
- Testphase-Alternative: USB-Ladegerät in der 12-V-Steckdose des Golf
  (zündungsgeschaltet) + langes USB-C-Kabel

## Erster Bench-Test (ohne Auto)

1. microSD rein, Board per USB-C an Powerbank/Rechner.
2. Serielle Konsole sollte zeigen: `SD OK`, `WiFi AP 'OpenOBD'`, `TWAI gestartet`.
3. Am Handy ins WLAN **`OpenOBD`** (Passwort **`openobd1234`**).
4. Browser: **http://192.168.4.1**
5. Dashboard erscheint (Werte auf „--", weil kein CAN). **CAN Explorer** öffnen —
   ohne Auto bleibt die Tabelle leer, das ist korrekt.

## Erster Test im Auto

1. Board stromlos verkabeln: **OBD2 Pin 6 → CAN-H**, **Pin 14 → CAN-L**,
   **Pin 4/5 → GND**. Strom für den Test noch über USB-C/Powerbank.
2. Zündung an, Board mit Strom, ins WLAN `OpenOBD`, **http://192.168.4.1**.
3. **`/explore` (CAN Explorer)** — hier kommt der Aha-Moment: jede CAN-ID, die der
   Golf am Port sendet, live mit Rate und Bytes. **Das ist die Datenbasis für alles
   Weitere.**
4. **PID-Status** (`/timeouts`): welche Standard-PIDs beantwortet der Golf (Active) und
   welche nicht (Disabled).
5. **Sessions**: `s{NNN}.csv` (dekodiert) und `raw_s{NNN}.csv` (roh, SavvyCAN) laden.

> Am OBD-Port eines modernen Golf (Gateway) kann es sein, dass passiv wenig „von allein"
> kommt und das meiste über Polling läuft — wie viel, zeigt der erste Test. Der Roh-Log
> fängt alles ein, was da ist.

---

## Roadmap

1. ~~**PID-Discovery**~~ — ✅ (`/pids`).
2. ~~**Zeitstempel**~~ — ✅ Handy-Sync **und** NTP (WLAN-Join); Anker löst die Session rückwirkend auf.
3. ~~**JSON-Config**~~ — ✅ `config.json` (Normal-/Dev-Modus, Logging, WLAN, Zeit, Dev-Streaming).
4. ~~**Cockpit-Dashboard**~~ — ✅ Gauges/Kacheln/Kurve + abgeleitete Werte + Status-Tab.
5. **UDS vertiefen** — Multiframe (VIN/lange Namen via Flow-Control), gefundene DIDs korrelieren (`analysis/`), dann VAG-Spezial-Dashboards (Energiefluss, ACT, Segeln, DSG-Temp, 48V).
6. **SavvyCAN-GVRET** (native WLAN-Verbindung) am Gerät verifizieren.
7. **Refactor** in Module (CAN / Storage / Web / UDS / PID-Tabelle).

---

## Später: Git & GitHub

Noch **nicht** veröffentlicht. Wenn es so weit ist:

```bash
git init && git add . && git commit -m "OpenOBD v1: Maximum-Capture"
```

`.gitignore` ignoriert bereits `.pio/` und `*.csv`.
