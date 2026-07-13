# Projekt-Briefing: VW Golf 8.5 CAN-Bus Datenlogger (Fork von roypeter/esp32-obd2-logger)

## Kontext / Ziel

Ich baue einen eigenständigen, dauerhaft im Auto verbauten OBD2/CAN-Bus-Datenlogger für meinen
**VW Golf 8.5 (Facelift) eTSI Style, EZ 06/2025, MQB Evo Plattform, DSG-Automatik, 48V-Mildhybrid**.

Ziele:
- Live-Dashboard übers Handy (WLAN, im Auto auf einer Halterung)
- Automatisches Logging jeder Fahrt auf SD-Karte, ohne manuellen Trigger nötig
- Langzeit-Auswertung über Wochen/Monate (Verbrauchstrends, Streckenvergleiche, später Fütterung an eine KI zur Anomalie-Suche)
- Kein Abo, keine Cloud, lokal-first, Open Source
- So wenig Löten wie möglich

Kein fertiges Produkt (OBDLink, Freematics etc.) reicht, weil ich vollen Zugriff auf Rohdaten will und das
Projekt langfristig um VAG-spezifische UDS-Werte (Ladedruck real, DSG-Temperatur, Hybrid-Batterie) erweitern
will, was mit fertigen Consumer-Adaptern nicht geht.

## Hardware (bereits bestellt/entschieden)

| Teil | Zweck | Status |
|---|---|---|
| **LilyGO T-CAN485** (ESP32 + SN65HVD231-CAN-Transceiver + microSD-Slot integriert, Schraubklemmen für CAN/RS485/Power, USB-C) | Hauptboard | Bestellt |
| **OBD2-Breakout-Kabel, 16-Pin, Male 30cm** (voller Pinout einzeln auf offene Adern rausgeführt) | Verbindung Auto ↔ Board | Bestellt |
| **DS3231 RTC-Modul** (I2C: VCC/GND/SDA/SCL, batteriegepuffert, ~2ppm Drift) | Persistente Echtzeituhr über Neustarts hinweg | Als nächstes zu bestellen |
| Micro-SD-Karte, USB-C-Kabel | | Noch zu besorgen |

**Bewusst verworfene Alternativen** (falls relevant für Architekturentscheidungen):
- Flipper Zero → kein nativer CAN-Transceiver, ungeeignet
- LilyGO T-2CAN (ESP32-S3, Dual-CAN, CAN-FD) → Overkill, kein SD-Slot, unbestückte GPIO-Header, wir brauchen nur einen Standard-CAN-Bus
- Ale1x/obd-realtime-platform-Architektur → braucht dauerhaft angeschlossenen PC/Raspberry Pi per USB während der Fahrt, nicht eigenständig genug
- VW-eigener eSIM-WLAN-Hotspot fürs Zeit-Sync → braucht kostenpflichtiges Cubic-Telecom-Datenpaket, widerspricht "kein Abo"
- BLE Current Time Service (0x1805) fürs Zeit-Sync vom iPhone → unzuverlässig ohne MFi-Zertifizierung/eigene App laut Recherche

## Pin-Belegung OBD2 (Standard, SAE J1962)

- Pin 6 = CAN-H, Pin 14 = CAN-L, Pin 4/5 = Masse
- Pin 16 = Dauerplus (Klemme 30, standardisiert, IMMER aktiv – NICHT als Stromquelle nutzen, sonst Batterie-Drain-Risiko)
- Pin 1 = Verdachtsfall auf Zündungsplus (Klemme 15) – bei anderen VW-Modellen (z.B. T5) dokumentiert, bei diesem Golf 8.5 **noch mit Multimeter zu verifizieren** (Pin 4/GND vs. Pin 1, Zündung an/aus vergleichen). Falls positiv: direkt als Stromquelle nutzen, dann brauchen wir keine Sleep-Logik. Falls negativ: Fallback ist ein Zündungsplus-Abgriff über eine Add-a-Fuse-Zwischenklemme im Sicherungskasten.
- Für den allerersten Bench-Test: Stromversorgung einfach über USB-C/Powerbank, unabhängig vom Auto.

## Ausgangscode: github.com/roypeter/esp32-obd2-logger

Einzeiliges `main.ino`, Arduino-Framework auf ESP32 (TWAI-Treiber für CAN, nicht ELM327-basiert).
Ursprünglich für einen **2004 Nissan / Tata Nexon 1.2T** gebaut – PID-Liste und Gang-Schätzung sind
darauf kalibriert, nicht auf unser Auto.

### Architektur des Originalcodes (funktioniert, als Ausgangsbasis behalten)
- Dual-Core-Aufteilung: Core 0 = Webserver-Task, Core 1 = Hauptloop (CAN-Polling + Logging)
- TWAI-Treiber direkt über ESP32, Standard-OBD2-Broadcast (ID 0x7DF), Antwort von ECU (ID 0x7E8)
- Request-Response-State-Machine mit 50ms Timeout pro PID
- PID-Liste aufgeteilt in `fast_pids[]` (~4Hz, häufig ändernde Werte: RPM, Speed, MAP, Load, Throttle etc.)
  und `slow_pids[]` (~1Hz, Coolant, IAT, Baro, Fuel Level etc.), Polling im Verhältnis 5:1 (SLOW_POLL_RATIO)
- Auto-Disable: PID wird nach 10 aufeinanderfolgenden Timeouts deaktiviert (`PidStats` Tracking)
- Storage-Abstraktion: SD-Karte bevorzugt, Fallback auf internen Flash (LittleFS) wenn keine SD gefunden
- Session-basiertes Logging: bei jedem Boot neue Datei `/data/s{NNN}.csv`, Session-Counter persistiert
- Mutex (`fsMutex`) zwischen Loop (schreibt CSV) und Webserver (liest zum Download) — verhindert Korruption
- WiFi Access Point ("obd2logger"), Webserver mit Routen:
  - `/` — Live-Dashboard mit ~28 Gauges, Auto-Refresh per JS-Polling auf `/data` (JSON)
  - `/data` — JSON-Endpoint mit allen aktuellen Werten
  - `/download?s=N` — CSV-Download einer Session
  - `/delete?s=N` — Session löschen (aktuell ohne Zugriffsschutz)
  - `/sessions` — Liste aller Sessions mit Pagination, Download/Delete-Links
  - `/log` — Ringpuffer-Debug-Log (letzte 50 Zeilen)
  - `/timeouts` — Tabelle: welche PIDs antworten, welche sind disabled
- Optionales OLED-Display (SSD1306, 128x64, I2C) mit 3 rotierenden Seiten (Live, Durchschnittswerte, Session-Stats)
- Gang-Schätzung über RPM/Speed-Verhältnis, mit `GEAR_RATIO_MIN/MAX[]`-Arrays für 6-Gang-Schaltgetriebe kalibriert

### Bekannte Probleme im Originalcode (aus Code-Review, siehe unten für Fix-Prioritäten)
1. **Kein echter Zeitstempel** — `timestamp = millis()`, nur Millisekunden seit Boot, kein Datum/Uhrzeit
2. **Keine CAN-Bus-Fehlerbehandlung** — Bus-Off-Zustand wird geloggt (`twai_get_status_info`), aber nie behandelt (kein `twai_initiate_recovery()`)
3. **OLED-Init-Fehler nicht global geflaggt** — `updateOled()` läuft auch wenn `oled.begin()` fehlgeschlagen ist
4. **Kein Power-/Sleep-Management** — Firmware läuft durch, solange Strom da ist, keine CAN-Stille-Erkennung
5. **Distanz/Verbrauch nur fürs OLED berechnet**, landet nicht in der CSV
6. **Gang-Schätzung 1:1 auf Nexon-6-Gang-Schaltgetriebe kalibriert** — passt nicht zu unserem DSG
7. **Reagiert nur auf ECU-Antwort-ID 0x7E8**, ignoriert andere IDs (z.B. 0x7E9 für Zusatzsteuergeräte)
8. **`/delete`-Route ohne Zugriffsschutz** — nur clientseitiger JS-Confirm-Dialog

Kompletter Originalcode liegt als Referenz bei: siehe `main_original.ino` im selben Ordner wie dieses Briefing
(falls nicht vorhanden: von github.com/roypeter/esp32-obd2-logger klonen, branch `main`, Pfad `main/main.ino`).

## Was der Fork am Ende können soll

### 1. PID-Discovery statt Trial-and-Error
Standard-OBD2 (SAE J1979) erlaubt es, das Auto direkt zu fragen, welche PIDs unterstützt werden:
PID `0x00` liefert eine Bitmaske für `0x01–0x20`, PID `0x20` für `0x21–0x40`, usw. bis `0xE0`.
Nur 8 Anfragen nötig, um die exakt unterstützte PID-Liste zu kennen — dynamisch statt hardcoded.
→ Discovery-Modus beim ersten Start (oder auf Knopfdruck über Webinterface), Ergebnis als lesbare Liste speichern.

### 2. Zwei-Ebenen-Logging
- **Roh-Ebene**: jeden CAN-Frame (ID + 8 Byte Payload + Timestamp) unverändert auf SD schreiben, ohne Parsen — Basis für spätere Reverse-Engineering-Arbeit mit SavvyCAN (VAG-spezifische UDS-Werte)
- **Decoded-Ebene**: bekannte PIDs weiterhin in benannte CSV-Spalten wie bisher, aber gespeist aus der Discovery-Liste statt Hardcoding

### 3. Zeitstempel: RTC mit Fallback-Logik
```
Wenn RTC (DS3231) gefunden UND Batterie nicht "lost power":
    → echte DateTime nutzen, Format "YYYY-MM-DD HH:MM:SS"
Sonst:
    → Fallback-Counter ab Boot: Format "REL_D{Tag} HH:MM:SS" (Tag 1, Tag 2... seit Boot,
      damit beim Auswerten sofort klar ist: keine echte Zeit vorhanden, nur relativ)
```
- `rtc.lostPower()` prüfen — wenn true, wie "kein RTC" behandeln (verhindert falsche aber selbstbewusste Zeitangabe)
- Web-Route zum Syncen: `POST /api/settime?epoch=<unix_timestamp>` — schreibt Handy-Browserzeit (`Date.now()` per JS) auf die RTC

### 4. Settings-Seite (`/settings`)
- Toggle: "Automatische Aufzeichnung beim Boot" an/aus — persistiert in Config-Datei auf SD (statt Hardcoding)
- Wenn Auto-Aufzeichnung aus: manuelle Start/Stop-Buttons (`POST /api/record/start`, `POST /api/record/stop`)
- RTC-Status-Anzeige ("Verbunden, Batterie OK" / "Nicht gefunden" / "Batterie leer, bitte synchronisieren")
- Button "Uhrzeit jetzt synchronisieren"
- Langfristig: WiFi-SSID/Passwort, aktive PID-Gruppen, Sample-Raten ebenfalls aus Config-Datei statt Hardcoding

### 5. Bluetooth-HFP-Zeitsync (experimentell, zusätzlich zur RTC, nicht als Ersatz)
- ESP32 als Bluetooth-Classic-HFP-Client, einmalige Kopplung mit iPhone (wie ein Auto-Radio)
- Holt beim Boot automatisch die iPhone-Systemzeit (offizielles ESP-IDF-Beispiel: `examples/bluetooth/bluedroid/classic_bt/hfp_hf`)
- **Danach Bluetooth-Controller komplett deaktivieren** (`esp_bt_controller_disable()`) um Strom zu sparen — nicht dauerhaft aktiv lassen
- Mein iPhone verbindet sich NIE automatisch mit dem Golf selbst (CarPlay-Auto-Connect bewusst deaktiviert wegen Akkuverbrauch) → keine Konkurrenz um die HFP-Verbindung zu erwarten
- Bekannter offener Punkt: WiFi+Bluetooth+TWAI gleichzeitig auf einem ESP32 — Coexistence sollte laut Espressif funktionieren, aber im echten Testbetrieb beobachten ob WiFi-Dashboard während BT-Sync ruckelt

### 6. Power-Management
- **Primär**: Board über Zündungsplus-Pin (Kandidat: Pin 1, noch zu verifizieren) speisen → kein Software-Sleep nötig, Board ist einfach aus, wenn Zündung aus ist
- **Falls nur Dauerplus verfügbar** (Fuse-Tap-Fallback): Brownout-/Spannungsabfall-Erkennung über ADC-Pin einbauen, um Dateien/SD-Karte sauber zu schließen bevor der ESP32 stirbt (verhindert korrupte Sessions)

### 7. Code-Refactor
Aktuell alles in einer `.ino`-Datei. Für Wartbarkeit aufteilen in logische Module:
- CAN-Handling (TWAI, Request/Response, PID-Discovery)
- Storage (SD/LittleFS-Abstraktion, Session-Management, Config-Persistenz)
- Webserver (Routen, HTML/JS-Templates)
- RTC/Zeit-Handling
- PID-Tabelle (Definitionen + Parser, später erweiterbar um VAG/UDS)

### 8. Spätere Phase (noch nicht jetzt): VAG-spezifische UDS-Werte
Sobald Roh-CAN-Sniffing läuft und mit SavvyCAN reverse-engineered wurde: echter Ladedruck,
DSG-Temperatur, 48V-Hybrid-Batteriedaten als zusätzliche UDS-Abfragen ergänzen (nicht Teil des
aktuellen Scope, aber Architektur sollte das nicht verbauen).

## Offene Fragen / nächste Schritte für Claude Code

1. Sobald Hardware da ist: T-CAN485-Pinbelegung (Datenblatt/Schaltplan) gegen Original-Code-Pins
   (`RX_PIN 4`, `TX_PIN 5`, `SD_CS 15`) abgleichen und anpassen
2. Multimeter-Test Pin 1 vs. Pin 4 (Zündung an/aus) — Ergebnis bestimmt Power-Strategie
3. Unverändertes Original zuerst zum Laufen bringen (Bench-Test mit Powerbank), dann `/timeouts`
   im echten Auto prüfen — welche der ~30 Original-PIDs antworten überhaupt bei unserem Golf
4. Priorität für Umbau: PID-Discovery → RTC+Fallback → Settings-Seite → Power-Handling → Refactor in Module

## Deployment

- Ziel-Plattform: ESP32 (Standard-Xtensa, nicht S3) — Arduino-Framework, wie im Original
- Toolchain: Arduino IDE oder PlatformIO (Entscheidung offen, PlatformIO vermutlich sinnvoller bei
  Modul-Aufteilung und für saubere Versionsverwaltung)
- Flash-Weg: USB-C direkt vom Rechner aus
- Kein Server/Cloud-Deployment — die "Deployment-Umgebung" ist der ESP32 selbst, das Webinterface
  läuft als lokaler Access Point direkt vom Gerät
- Langfristig denkbar (nicht Teil des aktuellen Scope): Daten zusätzlich per MQTT an bestehendes
  Home Assistant/Raspberry-Pi-Setup weiterreichen für Grafana-artige Langzeit-Trendanalyse
