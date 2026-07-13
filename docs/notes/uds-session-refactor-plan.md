# Implementierungsplan: Modularisierung, UDS-Session-Management & Erweiterte Fahrzeugdaten (Steuerung, Bremse, Gang, Mild-Hybrid)

Dieser Plan beschreibt die vollständige Zerlegung der monolithischen `src/main.cpp` in sauber strukturierte Module und die Implementierung eines robusten, absolut fahrzeugsicheren (schreibgeschützten) UDS-Session-Managements zur Auslesung erweiterter Daten (Lenkwinkel, Bremsdruck, aktueller Gang, Mild-Hybrid Ladezustand) über Service `0x22` auf dem VW Golf 8.5.

---

## 1. Hintergrund & Motivation

Das OpenOBD-Projekt liest derzeit erfolgreich Standard-OBD2-PIDs über 29-Bit-Anfragen (`0x18DB33F1`) aus und loggt alle CAN-Frames roh auf SD-Karte.
Für tiefere Fahrdaten (Lenkwinkel, Bremspedal, DSG-Gang, Mild-Hybrid Rekuperation/SoC) sind herstellerspezifische UDS-DIDs (Data Identifier) nötig. Diese sind jedoch im Standard-Diagnosemodus gesperrt (NRC `0x11` - *serviceNotSupported*).

**Die Lösung:**
1. **Erweiterte Diagnose-Sitzung (Service 0x10 03):** Muss für jedes relevante Steuergerät (ECU) geöffnet werden.
2. **TesterPresent (Service 3E 80):** Muss alle ~2 Sekunden gesendet werden, um die Sitzung aktiv zu halten. Durch Nutzung der Subfunktion `0x80` (*Suppress Positive Response*) bleibt der Bus extrem leise und entlastet, da die Steuergeräte nicht antworten müssen.
3. **Schreibschutz (Read-Only):** Sämtlicher UDS-Verkehr beschränkt sich strikt auf Leseanfragen (`0x22`), Sitzungssteuerung (`0x10`) und Keep-Alives (`0x3E`). Es werden keinerlei Schreibzugriffe (`0x2E`), Aktuatortests (`0x30`), Codierungen oder Routine-Ausführungen (`0x31`) durchgeführt.
4. **Modularisierung:** Die aktuelle `src/main.cpp` (~1400 Zeilen) ist unübersichtlich. Eine Aufteilung in Module (`config`, `storage`, `can`, `uds`, `web`, `main`) verbessert die Wartbarkeit, erleichtert das Testen und hält den Logger extrem leichtgewichtig.

---

## 2. Scope & Impact

- **Zielplattform:** LilyGO T-CAN485 (ESP32-WROOM-32).
- **Fahrzeug:** VW Golf 8.5 eTSI (MQB Evo Plattform).
- **Sicherheit:** 100% schreibgeschützt auf Automobilebene (keine Modifikation von Fahrzeugzuständen).
- **Speicherbedarf:** Keine nennenswerte Erhöhung; verbesserte Heap-Stabilität durch Vermeidung temporärer Strings im Hot-Path.
- **Rückwärtskompatibilität:** Die bestehenden Standard-OBD2-Polling-Mechanismen, das Live-Dashboard und das Rohdatenschreiben (SavvyCAN-Format) bleiben voll erhalten.

---

## 3. Architektur & Modul-Aufteilung

Wir teilen den Code in folgende Dateien auf:

### 3.1 `src/config.h`
- Definition aller GPIO-Pins (CAN, SD, I2C).
- Globale Konstanten (WLAN-SSID, Passwort, serielle Baudrate, Puffer-Größen).
- Deklaration gemeinsamer Datenstrukturen.

### 3.2 `src/storage_manager.h` / `src/storage_manager.cpp`
- Initialisierung von SD-Karte und LittleFS.
- Thread-sicheres Schreiben unter Verwendung des `fsMutex`.
- Schreiben der `raw_sNNN.csv` (SavvyCAN) und `sNNN.csv` (decoded).
- Verwaltung des Session-Zählers (`session.txt`) und des Zeit-Ankers (`sNNN.anchor`).

### 3.3 `src/can_manager.h` / `src/can_manager.cpp`
- Initialisierung des ESP32-TWAI-Treibers mit den korrekten Pins (TX=27, RX=26).
- Standard-OBD2-Polling-Schleife (`fast_pids`, `slow_pids`).
- PID-Discovery (`0x00`, `0x20` etc.).
- Auto-Probe-System zur Konfigurations-Erkennung.
- Thread-sichere Pufferung (`rawBuf`) empfangener Frames.

### 3.4 `src/uds_manager.h` / `src/uds_manager.cpp`
- **UdsSessionManager (State-Machine):**
  - Verwaltet den Sitzungsstatus pro Steuergerät (`0x01` Motor, `0x02` DSG, `0x03` Bremse, `0x44` Lenkung, `0x19` Gateway).
  - Sendet `02 10 03 AA AA AA AA AA` zur Aktivierung der Extended Session.
  - Sendet periodisch (alle 2s) `02 3E 80 AA AA AA AA AA` (TesterPresent Suppress Response) an alle Steuergeräte, die sich im Extended-Modus befinden.
- **Intervallgesteuerter DID-Scheduler:**
  - Jede herstellerspezifische DID hat ein eigenes Sendeintervall (z.B. Lenkwinkel alle 100ms, Bremsdruck alle 100ms, Gang alle 200ms, Mild-Hybrid SoC alle 2000ms).
  - Sendet Leseanfragen (`0x22`) nur, wenn das Intervall abgelaufen ist.
- **ISO-TP Multiframe Support:**
  - Sendet automatisch Flow-Control-Frames (`30 00 00 AA AA AA AA AA`), wenn ein Steuergerät mit einem First-Frame (`1x xx`) antwortet, um lange Antworten lesen zu können.
- **Integrierter Web-DID-Scanner:**
  - Erlaubt das automatisierte Abklopfen von DID-Bereichen (z.B. `0x0100` bis `0x4FFF` und `0xF100` bis `0xF5FF`) für eine gewählte ECU.
  - Speichert positive Rückmeldungen (`62 {DID} ...`) direkt in `/data/dids_ecu_{ECU}.txt` auf SD-Karte. So können neue Signale direkt am echten Auto entdeckt werden.

### 3.5 `src/web_manager.h` / `src/web_manager.cpp`
- Webserver-Konfiguration auf Core 0.
- Dashboards, Sessions-Liste, `/timeouts`, `/log` und die neue UDS-Diagnoseseite `/uds`.
- Neue API-Endpunkte für den DID-Scanner (`/api/didscan/start`, `/api/didscan/status`).

### 3.6 `src/main.cpp`
- Minimaler Einstiegspunkt (Bootstrap).
- Startet die Tasks: Core 0 (Web-Interface), Core 1 (CAN-Polling, UDS-Session-Engine und Logging).

---

## 4. Konkrete UDS-DIDs für MQB Evo (Golf 8.5)

Wir implementieren folgende Start-DIDs in die Polling-Tabelle (`udsTable`):

| Steuergerät (ECU) | DID (Hex) | Name | Intervall | Datenaufbau / Parser-Formel |
|---|---|---|---|---|
| **0x44 (Lenkhilfe)** | `0x0448` | `Lenkwinkel` | 100 ms | 16-Bit Signed Integer (Grad, -500° bis +500°) |
| **0x03 (Bremse)** | `0x3F07` | `Bremsdruck` | 100 ms | 16-Bit Unsigned (bar, z.B. 0 bis 150 bar) |
| **0x03 (Bremse)** | `0x01F9` | `Bremspedalschalter` | 200 ms | 1 Byte Boolean (0 = Aus, 1 = Ein) |
| **0x02 (DSG)** | `0x04FE` | `Eingelegter Gang` | 200 ms | 1 Byte: 1-7 (Gänge), 8 (R), 13 (N), 14 (P) |
| **0x01 (Motor)** | `0x1164` | `Mild-Hybrid SoC` | 2000 ms | 1 Byte oder Formel `Value / 2.5` (%) |

---

## 5. Implementierungs-Schritte (Phasen)

### Phase 1: Vorbereitung & Modularisierung (Sicherer Zwischenschritt)
1. Erstellen der neuen Header- und Quelldateien.
2. Schrittweises Verschieben des Codes aus `src/main.cpp`.
3. Verifizieren, dass der Code fehlerfrei kompiliert (`pio run`).
4. Boot-Test auf dem Gerät (WLAN AP, SD-Karte, originaler OBD2-Poller läuft stabil).

### Phase 2: UDS-Sitzungssteuerung & TesterPresent
1. Implementierung der ECU-Sitzungsüberwachung (`EcuSessionState`).
2. Implementierung der periodischen `TesterPresent (3E 80)`-Aussendung (Suppress Response) für aktive ECUs.
3. Ergänzung der UDS-Zustandsmaschine, um vor einer Leseanfrage (`22`) sicherzustellen, dass die Extended Session (`10 03`) geöffnet ist. Bei Sitzungsverlust automatische Re-Aktivierung.
4. Testen des DSG-Verbindungstests im Web-Interface.

### Phase 3: DID-Scheduler & Daten-Parsing
1. Implementierung der intervallgesteuerten Abfrage-Logik für DIDs in der `udsTable`.
2. Hinzufügen der MQB-Standard-DIDs (Lenkwinkel, Bremsdruck, Gang, Hybrid SoC).
3. Integration der UDS-Werte in das Live-Dashboard (Übertragung via JSON-Schnittstelle `/data` und Integration von Kacheln im Web-GUI).
4. Speichern der UDS-Daten in einer erweiterten `sNNN.csv` oder einer separaten `uds_sNNN.csv` (um die Spaltenkompatibilität der Standard-CSV nicht zu brechen).

### Phase 4: DID-Scanner & Verifikation
1. Implementierung der non-blocking Suchschleife für DIDs auf einer ECU im Hintergrund.
2. Erstellung einer kleinen Web-UI auf `/uds`, um den Scan zu starten/stoppen, den Fortschritt anzuzeigen und gefundene DIDs aufzulisten.
3. Integration von Flow-Control-Frames (`30 00`) zur korrekten Handhabung von Multiframe-Rückmeldungen.
4. Kompilierung und vollständige Validierung des gesamten Quellcodes.

---

## 6. Verifikation & Tests

- **Schritt 1 (Kompilierung):** `python3 -m platformio run` muss für alle Module ohne Warnungen durchlaufen.
- **Schritt 2 (Dashboard-JS):** JS-Schnittstellen per Node.js auf Syntaxfehler prüfen (`node --check` falls anwendbar).
- **Schritt 3 (Trockentest/Loopback):** Der im T-CAN485-Logger integrierte Loopback-Selbsttest beim Boot stellt sicher, dass der CAN-Controller Senden und Empfangen can, ohne am Fahrzeug angeschlossen zu sein.
- **Schritt 4 (Fahrzeugtest):**
  - Anschluss ans Auto, Zündung an.
  - Beobachtung über serielle Konsole (`pio run --target monitor`).
  - Überprüfung des Web-Dashboards: Werden Lenkwinkel, Bremsdruck und Gang live aktualisiert?
  - Verifizieren, dass keine CAN-Fehler auftreten (`CAN state` bleibt running, keine Fehlerzähler steigen an).
  - Überprüfung der Log-Dateien auf SD-Karte.

---

## 7. Migrations- & Rollback-Strategie

- Das unveränderte Original `main_original.ino` verbleibt im Repository unter `reference/main_original.ino`.
- Vor dem Refactoring wird die funktionierende, monolithische Version von `src/main.cpp` lokal gesichert (z.B. als `src/main.cpp.v1_stable`).
- Jede Phase wird als isolierter Git-Commit (oder lokale Kopie) strukturiert, um jederzeit zu einem stabilen Zwischenstand zurückkehren zu können.
- Sollte der TWAI-Bus instabil werden, kann die UDS-Polling-Schleife über ein globales Flag im Webinterface (`udsPollEnabled = false`) im laufenden Betrieb komplett abgeschaltet werden, sodass der Logger wieder in den reinen Standard-OBD2-Modus zurückfällt.
