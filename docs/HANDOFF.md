# OpenOBD — Projekt-Handoff & Kontext

> **Zweck dieses Dokuments:** Vollständiger Wissensstand zum Weitermachen an anderer
> Stelle / in einer frischen Session. Stand: 2026-07-13 (v2).
> **Zum Einlesen (in dieser Reihenfolge):** dieses Dokument → `README.md` (Bedien-/Flash-Anleitung)
> → `PROJECT BRIEF.md` (Ursprungsvision) → `src/main.cpp` überfliegen.

---

## 0. Update v2 (2026-07-13) — was seit dem ersten Handoff dazukam

- **Konfiguration in `include/config.h`** (Compile-Zeit, NICHT auf SD — bewusste User-Entscheidung:
  keine config.json, kein Karten-Rein-Raus). `struct Config cfg` in `src/main.cpp` wird aus den
  `CFG_*`-Makros initialisiert. Zwei Flash-Ziele in `platformio.ini`: `t-can485` (Normal) und
  `t-can485-dev` (setzt `-D CFG_DEV_MODE=true`). `CFG_DEV_MODE` leitet `CFG_WEB_UI`/`CFG_LOG_RAW`/
  `CFG_STREAM_TCP` ab. (Alte SD-`loadConfig()`/`writeDefaultConfig()` + ArduinoJson wurden entfernt.)
- **Normal- vs. Dev-Modus:** Normal = Cockpit + minimal (nur dekodiertes CSV). Dev = kein Dashboard,
  volles Roh-Log, Discovery/UDS. Gates: `web_ui`→handleRoot, `log_raw`→logRawLine, `log_decoded`→Decoded-Write.
- **WLAN neu (`setupWifi`):** `wifi.mode:"join"` wählt sich ins Auto-WLAN (STA) → Handy behält
  Internet, **Zeit per NTP** (`syncTimeNtp`, setzt denselben Anker wie der Handy-Sync), Zugriff via
  **mDNS `openobd.local`**; Fallback-AP bei Fehler. `"ap"` = eigener Hotspot wie bisher.
- **Cockpit-Dashboard in Firmware** (PROGMEM in `handleRoot`): dunkles Cluster, 2 Gauges (ohne Nadel),
  Verlaufskurve, Ampel-Kacheln, **Hamburger-Menü** (große fahrsichere Buttons), Tabs Live/Motor/Fahrt/
  Verbrauch/Abgas/Elektrik/**Status**. Kennzeichen „jetzt" (Momentan) / „Ø" (Trip-Durchschnitt).
  `/data` erweitert um **alle** dekodierten PIDs + **abgeleitete** (l100 jetzt, avg100 Ø, ps, boost) + System-Status.
- **Sanftes UDS** (`/uds`, „höflicher Scan", `runPoliteScan`): nur Service 0x22 in Standard-Sitzung,
  Ablehnung wird akzeptiert, **kein 0x10/0x27/0x2E/0x31**. Benennt STG über Identifikations-DIDs.
  Ergebnis → `/uds_scan.txt`. Ethik: „nur lesen, was das Auto freiwillig gibt."
- **SD-Struktur (v2):** Logs nach Jahr/Monat: saubere Daten `/data/YYYY/MM/sNNN.csv` (+`.anchor`),
  Roh `/dev/YYYY/MM/raw_sNNN.csv`. Meta im Root (KEINE config.json — Konfig ist in config.h): `pids.txt`, `session.txt`,
  `sessions.csv` (Index num,epoch,decoded,raw → Download/Liste/Delete), `lasttime.txt` (RTC-Ersatz:
  letzte bekannte Zeit persistiert, beim Boot als provisorisches Datum geladen; NTP/Handy korrigiert).
  `openNewSession`+`currentYM`+`ensureDirTree`; `handleSessions/Download/Delete` gehen über den Index.
- **Dev-Streaming** (`streamTask` auf Core 0, Queue aus loop): Roh-Frames drahtlos per **TCP**
  (`dev.stream_tcp`, SavvyCAN-Zeilen) und **MQTT** (`dev.mqtt`, PubSubClient). Netz-I/O bewusst NICHT
  im CAN-Pfad. `platformio.ini`: lib_deps ArduinoJson + PubSubClient.
- **Kompiliert:** Flash ~75 %, RAM ~21 %. **Offen/zu verifizieren am Gerät:** WLAN-Join+NTP,
  AP-Isolation des Auto-Hotspots, Dev-Streaming gegen Python/SavvyCAN, höflicher Scan am Auto.
  **Noch nicht gebaut:** Multiframe-UDS (VIN/lange Namen via Flow-Control), native SavvyCAN-GVRET,
  VAG-Spezial-Dashboards (Energiefluss/DSG/48V) — brauchen erst die echten DIDs.

Neue/relevante Dateien: `include/config.h` (Konfiguration, Compile-Zeit), `analysis/` (Python-Analyse-Kit).

---

## 1. Ziel

Eigenständiger, permanent verbauter **CAN-Bus-Datenlogger für einen VW Golf 8.5 eTSI**
(1.5 TSI, 48V-Mildhybrid, DSG, MQB Evo, EZ 06/2025). Hardware: **LilyGO T-CAN485**
(ESP32 + SN65HVD231-CAN-Transceiver + microSD). Fork von `roypeter/esp32-obd2-logger`.

**Leitprinzipien des Users:** lokal-first, kein Abo/Cloud, Open Source, so viele Daten
wie möglich, robust, schlank, erweiterbar, generisch aber aufs Fahrzeug zugeschnitten,
so wenig löten wie möglich (stecken/schrauben).

**Endziel:** Live-Dashboard übers Handy (WLAN) + automatisches SD-Logging jeder Fahrt +
später VAG-spezifische Werte (48V-Akku, DSG-Temperatur, Ladedruck, Zylinderabschaltung).

---

## 2. Hardware & Verkabelung (alles verifiziert)

### T-CAN485 GPIO-Belegung (im Code, `src/main.cpp` oben)
| Funktion | GPIO | Hinweis |
|---|---|---|
| 5V-Boost-Enable | 16 | MUSS HIGH — versorgt CAN-Transceiver **und** SD-Slot |
| **CAN TX** | **27** | ⚠️ **vertauscht ggü. offizieller config.h (die sagt 26)!** siehe Erkenntnis #1 |
| **CAN RX** | **26** | |
| CAN Silent-Enable | 23 | LOW = normaler High-Speed-Modus |
| SD SCLK/MISO/MOSI/CS | 14/2/15/13 | |
| DS3231 I2C SDA/SCL (Default) | 32/33 | RTC noch NICHT verbaut (User will nicht löten) |

### OBD2-Breakout-Kabel → T-CAN485 (Farben des vorhandenen Kabels)
| OBD-Pin | Farbe | → Anschluss |
|---|---|---|
| 6 | **Grün** | Schraubklemme CAN-H |
| 14 | **Grün-Weiß** | Schraubklemme CAN-L |
| 5 | **Hellblau** | Schraubklemme GND |
| 1 | Braun | Zündungsplus (Kandidat, Multimeter-Check offen) — später Stromquelle |
| 16 | Rot | 12V Dauerplus — **NIE anschließen** |

### Stromversorgung
- **Test:** Powerbank (5V/3A) an USB-C. Funktioniert.
- **Festeinbau (Teile gekauft, noch nicht verbaut):** KFZ-Wandler **12/24V→5V USB-C** (vergossen),
  gespeist von **Zündungsplus** (Pin 1). Verdrahtung lötfrei über **Wago 221-413** + **2A-Inline-Sicherung**.
  Konzept: Gerät läuft nur bei Zündung an → kein Sleep-Code, kein Batterie-Drain.
- **SD-Karte:** **32 GB, FAT32.** (8GB-Karte war defekt; 64GB=SDXC bräuchte manuelle FAT32-Formatierung.)

---

## 3. Die wichtigsten Erkenntnisse (hart erkämpft — nicht neu debuggen!)

1. **CAN TX/RX waren vertauscht.** Quellen widersprachen sich (config.h 26/27 vs. Doku 27/26).
   Korrekt für dieses Board: **TX=27, RX=26.** Ein eingebauter Boot-Selbsttest (probiert alle
   Pin-/Modus-Kombis per Loopback) hat es gefunden. Das war die Ursache, warum am Auto zunächst
   *null* Frames ankamen — NICHT defekte Hardware, NICHT Verkabelung.

2. **★ Der Golf spricht OBD nur über 29-Bit-Adressierung (ISO 15765-4).** 11-Bit (0x7DF/0x7E0)
   → **keine Antwort.** Funktional korrekt ist:
   - **Anfrage (funktional):** `0x18DB33F1`
   - **Antworten:** `0x18DAF101` (Motor-STG), `0x18DAF102` (2. Motor-Subsystem), `0x18DAF110` (DSG)
   - Bitrate **500 kbit/s**, Fahrzeug-Padding **0xAA**.

3. **Physische UDS-Adressierung (29-Bit):** Anfrage an STG `0xNN` = `0x18DA{NN}F1`,
   Antwort = `0x18DAF1{NN}`. STG-Adressen: **0x01 Motor, 0x02 Motor-Sub, 0x10 DSG.**
   (Achtung: `0x18DAF110` ist die DSG-**Antwort**, die **Anfrage** geht an `0x18DA10F1`.)

4. **VAG-Daten sind NICHT im Broadcast** (Deep-Research-Report bestätigt + Rohdaten). Am OBD-Port
   liegt nur, was wir aktiv abfragen (Gateway riegelt ab). 48V/BSG vermutlich ins Motor-STG integriert.
   → VAG-Werte nur über **aktives UDS Service 0x22** erreichbar.

5. **UDS-Reads sind in der Standard-Sitzung gesperrt.** DSG antwortet auf `03 22 F1 86` mit
   `03 7F 22 11` (NRC 0x11 serviceNotSupported). Klassisches VW-Verhalten → **erst erweiterte
   Diagnose-Sitzung** (`10 03`) + **TesterPresent** (`3E 00`) nötig, dann `0x22` lesen.

6. **Datenqualität:** Einige Temp-PIDs liefern `0xFF` (= „nicht verfügbar", z.B. Öl/Außentemp).
   Der Code fängt das ab und zeigt `--` statt falscher 215 °C.

7. **Was Standard-OBD schon liefert (~28 PIDs, dekodiert & plausibel):** RPM, Speed, Kühlmittel,
   MAP/Ladedruck (bis 131 kPa!), IAT, Last, Zündung, Drossel, Ist-Moment, Ref-Moment, STFT/LTFT,
   O2-Sonden, Lambda, Katalysator-Temp, Tank, Verbrauch, Bordspannung (14,3V), Distanz u.a.

---

## 4. Aktueller Firmware-Stand (`src/main.cpp`, kompiliert, Flash ~71%)

- **Dual-Core:** Webserver auf Core 0, Polling+Logging auf Core 1.
- **29-Bit-OBD-Polling + PID-Discovery** (probiert 0x7DF→0x7E0→0x18DB33F1; setzt PID-Sperren
  bei Erfolg zurück). Auto-Disable nach 10 Timeouts.
- **Zwei-Ebenen-Logging:** `sNNN.csv` (dekodiert, 33 Spalten) + `raw_sNNN.csv`
  (jedes Frame, **Tx & Rx**, SavvyCAN-Format, 64-Bit-µs-Zeitstempel).
- **CAN-Selbsttest beim Boot** (Loopback, findet Pin-/Modus-Konfig).
- **Auto-Probe** (Dev-Tab): testet alle Bitraten × Adressen × Paddings, übernimmt Gewinner.
- **Zeit:** Handy-Sync beim Dashboard-Öffnen (`/api/settime`), Anker-Prinzip (uptime↔epoch,
  Sidecar `sNNN.anchor`) datiert die ganze Session rückwirkend; Fallback `REL_D{n}`.
- **Robustheit:** fester `rawBuf[4096]` (kein String→keine Heap-Fragmentierung), Free-Space-Check
  nur alle 10s + `storageFull`-Stopp, Datei-Schluss nach jedem Write, Bus-Off-Recovery, RX-Queue 128.
- **Dashboard-SPA** (aus PROGMEM, kein Scrollen, Landscape): Tabs Live/Motor/Fahrt/Verbrauch/
  Elektrik/**Dev**. Kacheln mit Balken + Live-Graph. Dev-Tab bündelt alle Technik-Funktionen.
- **UDS-Framework (Phase 2, non-blocking):** `requestUdsData(ecu,did)`, `parseUdsSF()`,
  `udsTable[]`, State-Machine `serviceUds()`+`handleUdsFrame()` (per `udsPollEnabled=false`
  standardmäßig AUS), `testUdsConnection()` (Boot-Test DSG), Seite `/uds`.
- **WLAN:** AP `OpenOBD` / Passwort `openobd1234`, `http://192.168.4.1`.

---

## 5. Architektur & Regeln (bei jeder Änderung einhalten)

- **Non-blocking loop:** kein `delay()` im Hot-Path. Lange Aktionen (Discovery, Probe, UDS-Test)
  laufen nur als Einmal-Aktionen, vom Webserver per **Flag** angestoßen, in `loop()` ausgeführt.
- **Single-Writer-Prinzip:** NUR `loop()` (Core 1) schreibt `rawBuf`/Session-Dateien. Der Webserver
  (Core 0) fasst diese NIE direkt an — er setzt Flags (`newSessionRequested`, `probeRequested`, …).
- **Alle SD-Zugriffe unter `fsMutex`** (mit Timeout, damit nichts endlos hängt).
- **Feste Puffer statt `String`** in Hot-Paths (Heap-Fragmentierung bei Dauerbetrieb vermeiden).
- **UDS strikt read-only während der Fahrt:** nur `0x10` (Session), `0x3E` (TesterPresent),
  `0x22` (Read). **NIEMALS** `0x2E` (Write) oder `0x31` (Routinen) fahrend.
- **Datengetrieben & generisch:** PID- und DID-Tabellen als Zeilen erweiterbar, keine Hardcodes.
- **Jede Änderung kompilieren + verifizieren** (`pio run`; Dashboard-JS mit `node --check`).

---

## 6. Toolchain: Bauen & Flashen

- **PlatformIO**, Board `esp32dev`, **keine externen Libs** (alles im arduino-esp32-Framework).
  PlatformIO ist auf dem Mac installiert, aber nicht im PATH → `python3 -m platformio` nutzen.
- Nur kompilieren: `python3 -m platformio run -d ~/Projekte/OpenOBD`
- Flashen + Konsole: `python3 -m platformio run -d ~/Projekte/OpenOBD --target upload --target monitor`
- USB-Serial-Chip: **CH9102F** (macOS erkennt ihn meist nativ als `/dev/cu.wchusbserial*`).
- Upload hängt? BOOT halten, RST tippen, BOOT loslassen, Upload starten.

---

## 7. Roadmap / nächste Schritte (Reihenfolge)

1. **UDS-Sitzung freischalten** (der unmittelbar nächste Baustein): `udsOpenSession(ecu)` via
   `02 10 03` (Extended Session) + **TesterPresent-Keepalive** `02 3E 00` alle ~2s. Danach sollte
   `0x22` Daten liefern (oder NRC `0x31` = „DID gibt's nicht" → gültige Adresse, falsche DID).
2. **DID-Scanner** (non-blocking, read-only): pro STG (0x01/0x02/0x10) DID-Bereiche mit `0x22`
   abklopfen, alle `0x62`-Antworten auf SD + `/uds` protokollieren → findet 48V/DSG/ACM-DIDs.
3. **Bedeutung zuordnen:** gefundene DID-Werte gegen Fahrdynamik korrelieren (CSS-Electronics-Methodik).
4. **VAG-Dashboards** bauen (Energiefluss, DSG-Temp, 48V-SoC, Segeln, ACM) — sobald DIDs bekannt.
5. **Multiframe ISO-TP** (First Frame `1x` + Flow Control `30 00`) für lange Antworten (z.B. VIN 0xF190).
6. **Modul-Refactor:** `main.cpp` ist derzeit alles-in-einem (~1200 Z.) → aufteilen (can/storage/web/uds/pids).
7. **Festeinbau-Stromversorgung** verkabeln (Wandler + Sicherung + Wago) + Multimeter-Check Pin 1.
8. **Optional/später:** DS3231 nachrüsten, JSON-Config-Persistenz, Session-Rotation für 24/7-Dauerbetrieb.

---

## 8. Reverse-Engineering-Methodik (für Phase 2/3)

Referenz: CSS-Electronics „CAN Bus Reverse Engineering with AI/LLM" +
GitHub `CSS-Electronics/can-bus-reverse-engineering-skills`. Pipeline:
Bit-Flip-Heatmap (Counter/Checksummen ausschließen) → Korrelation gegen Referenzsignal
(Spearman, Lag-Ausgleich) → Bitsearch (Startbit/Länge/Byteorder/Vorzeichen) → Scale/Offset → DBC.
Deren „CAN-based reference" = OBD-PIDs + Rohdaten in einer Aufnahme — genau was OpenOBD liefert.
**Aber:** Am Golf gibt's keine freien Broadcasts → VAG-Werte müssen aktiv per UDS (0x22) geholt
werden, nicht per passiver Korrelation. „Harvest Drive": 5-10+ min mit viel Variation
(kalt→warm, beschleunigen/bremsen/segeln, Stadt+Autobahn, Start-Stopp).

---

## 9. Dateistruktur & Daten

```
OpenOBD/
├── src/main.cpp          ← die gesamte Firmware (~1200 Z.)
├── platformio.ini        ← Build-Config (board esp32dev, keine Libs)
├── README.md             ← Bedien-/Flash-/Verkabelungs-Anleitung (aktuell)
├── PROJECT BRIEF.md      ← Ursprungsvision (Original-Anforderungen)
├── HANDOFF.md            ← dieses Dokument
├── reference/main_original.ino  ← unveränderter Nexon-Ausgangscode
└── results/              ← Fahrdaten von der SD (sNNN.csv + raw_sNNN.csv)
```
- **results/**: Sessions s001–s034. **s026–s030** = gute Standard-OBD-Fahrten (echte Werte).
  **raw_s034** enthält den UDS-Test (DSG-Antwort `03 7F 22 11`).
- **Zeitstempel-Auflösung:** decoded=`millis()`, raw=`esp_timer` µs → über `sNNN.anchor` in Echtzeit umrechenbar.
- **FAT-Dateidatum 1979** ist normal (kein RTC auf FS-Ebene).

---

## 10. Bekannte offene Punkte / Minor

- `handleData` baut 1×/s einen kurzlebigen `String` (freigegeben, unkritisch für Fahrten;
  für 24/7 evtl. auf festen Puffer).
- Roh-Log wächst groß (32GB reicht Jahre; Rotation erst für Dauerverbau nötig).
- Ein PID, der nur zeitweise verfügbar ist, kann nach 10 Timeouts für die Session deaktiviert werden.
- DS3231-I2C-Pins (32/33) sind Defaults, beim Anlöten gegen freien Header-Pin prüfen.

---

*Erstellt von Claude (Opus 4.8) am 2026-07-12. Bei Fortsetzung an anderer Stelle: dieses Dokument
zuerst lesen, dann `src/main.cpp` überfliegen — der Code ist ausführlich kommentiert, jede
CAN-Besonderheit ist an Ort und Stelle erklärt.*
