# OpenOBD — UDS-Diagnose & DID-Ermittlung (VW Golf 8.5 eTSI)

Dieses Dokument fasst den aktuellen Wissensstand zusammen, beschreibt die geplante schreibgeschützte UDS-Erweiterung und liefert eine systematische Anleitung, wie am echten Fahrzeug nach beliebigen weiteren herstellerspezifischen Daten (Data Identifiern - DIDs) gesucht werden kann.

---

## 1. Was wir aktuell haben (Status Quo)

- **Hardware-Setup:** LilyGO T-CAN485 (ESP32-WROOM-32) mit integriertem CAN-Transceiver und microSD-Kartenslot. Die Pins sind verifiziert: **TX=GPIO 27**, **RX=GPIO 26**, **5V-Enable=GPIO 16 (HIGH)**.
- **CAN-Bus-Schnittstelle:** Der VW Golf 8.5 (MQB Evo) kommuniziert OBD2/UDS-Diagnosedaten ausschließlich über **29-Bit-Adressierung (ISO 15765-4) bei 500 kbit/s** mit dem Daten-Padding **0xAA**.
- **Standard-OBD2-Polling:** Der Logger pollt aktiv ~28 Standard-PIDs (Drehzahl, Geschwindigkeit, Moment, Lambda etc.).
- **Zwei-Ebenen-Logging:**
  - `sNNN.csv`: Enthält die dekodierten Standard-OBD2-Werte.
  - `raw_sNNN.csv`: Ein lückenloser, hochpräziser (64-Bit µs) Roh-Mitschnitt aller gesendeten (Tx) und empfangenen (Rx) Frames im SavvyCAN-Format.
- **UDS-Erkenntnisse am OBD-Port:**
  - Das Gateway (`J533`) filtert sämtliche fahrzeuginternen Broadcast-Frames ab. Am OBD2-Stecker ist der Bus passiv nahezu still.
  - **Alle VAG-spezifischen Werte müssen aktiv per UDS Service 0x22 (Read Data by Identifier) abgefragt werden.**
  - **Sperre im Standard-Modus:** Standard-UDS-Anfragen (`0x22`) werden standardmäßig abgelehnt (Antwort `7F 22 11` -> NRC `0x11` *serviceNotSupported*).

---

## 2. Was wir machen wollen (Die UDS-Strategie)

Wir erweitern die Firmware um eine fahrzeugsichere, schreibgeschützte UDS-Polling-Engine und modularisieren das pflegebedürftige Projekt.

### 2.1 Schreibgeschütztes UDS-Session-Management
Um herstellerspezifische Daten lesen zu dürfen, müssen wir eine **erweiterte Diagnosesitzung** öffnen und halten:
1. **Sitzung öffnen (Service 0x10):** Senden von `02 10 03 AA AA AA AA AA` (Extended Session) an die Ziel-ECU.
2. **Sitzung halten (Service 3E):** Senden von `02 3E 80 AA AA AA AA AA` (TesterPresent) alle ~2 Sekunden. Die Subfunktion `0x80` (*Suppress Positive Response*) weist das Steuergerät an, **nicht** auf das Keep-Alive zu antworten. Das spart wertvolle CAN-Bandbreite und hält den Logger extrem leichtgewichtig.
3. **Strikter Lese-Sicherheitsfilter (Read-Only):**
   - Die Engine unterstützt technologisch **ausschließlich** die Services `0x10` (Sitzungssteuerung), `0x3E` (TesterPresent) und `0x22` (Daten lesen).
   - Es werden **niemals** Schreibzugriffe (`0x2E`), Stellgliedtests/Aktuatorentests (`0x30`), Steuergeräte-Codierungen oder Routinen (`0x31`) implementiert oder gesendet. Damit ist eine Fehlfunktion oder Manipulation von Fahrzeugsystemen physikalisch und softwareseitig zu 100 % ausgeschlossen.

### 2.2 Intervallgesteuerter DID-Scheduler
Statt alle Daten stumpf nacheinander abzufragen, bekommt jede DID ein definiertes Intervall:
- **Fast-DIDs (z.B. Lenkwinkel, Bremsdruck):** Abfrage alle 100 ms für flüssige Live-Anzeige und präzises Tracking bei dynamischen Fahrmanövern.
- **Medium-DIDs (z.B. Getriebegang, Drehmomente):** Abfrage alle 200–500 ms.
- **Slow-DIDs (z.B. Mild-Hybrid Ladezustand, Öltemperaturen):** Abfrage alle 2000–5000 ms (da sich diese Werte nur langsam ändern).

---

## 3. Die 5 MQB-Standard-DIDs als Startbasis

Für den Start nutzen wir folgende DIDs, die sich auf MQB-Plattformen bewährt haben:

1. **Lenkwinkel (Steering Wheel Angle):**
   - **ECU:** `0x44` (Lenkhilfe / Steering Assist) -> Request ID: `0x18DA44F1`, Response ID: `0x18DAF144`
   - **DID:** `0x0448`
   - **Parser:** 16-Bit Signed Integer (Grad, negatives Vorzeichen = Lenkung links, positives = rechts).
2. **Bremsdruck (Brake Pressure):**
   - **ECU:** `0x03` (Bremse/ABS) -> Request ID: `0x18DA03F1`, Response ID: `0x18DAF103`
   - **DID:** `0x3F07`
   - **Parser:** 16-Bit Unsigned Integer (bar).
3. **Bremspedalschalter (Brake Light Switch):**
   - **ECU:** `0x03` (Bremse/ABS)
   - **DID:** `0x01F9`
   - **Parser:** 1 Byte Boolean (0 = gelöst, 1 = gedrückt).
4. **Aktueller Gang (Selected Gear DSG):**
   - **ECU:** `0x02` (Getriebe/DSG) -> Request ID: `0x18DA02F1`, Response ID: `0x18DAF102`
   - **DID:** `0x04FE`
   - **Parser:** 1 Byte. Werte: 1-7 = Gang 1-7, 8 = Rückwärtsgang (R), 13 = Neutral (N), 14 = Parken (P).
5. **Mild-Hybrid Batterie Ladezustand (SoC):**
   - **ECU:** `0x01` (Motorsteuergerät) -> Request ID: `0x18DA01F1`, Response ID: `0x18DAF101`
   - **DID:** `0x1164` (Normalized SoC)
   - **Parser:** 1 Byte. Skalierung: `Wert / 2.5` oder direkter Prozentwert (je nach exaktem Motorcode).

---

## 4. Systematische Suche nach weiteren DIDs (Die Detektiv-Anleitung)

Da moderne MQB-Evo-Fahrzeuge Tausende Steuergeräte-interne Parameter verwalten, zeigen wir hier, wie man gezielt nach weiteren DIDs (wie Blinker, Rekuperationsleistung, Elektromotor-Drehmoment etc.) sucht, ohne auf externe Datenbanken angewiesen zu sein.

### 4.1 Der automatisierte UDS-DID-Scanner (Integrierte Software-Suche)
Wir implementieren in der neuen Firmware einen **Hintergrund-DID-Scanner**.
- **Funktionsweise:**
  - Der Scanner öffnet die Extended Session (`10 03`) auf einem vom Benutzer gewählten Steuergerät (z.B. `0x19` Gateway oder `0x01` Motor).
  - Er sendet sequentiell Leseanfragen für DIDs in definierten Suchbereichen (z.B. `0x0100` bis `0x4FFF` für Live-Messwerte, und `0xF100` bis `0xF5FF` für Systeminformationen).
  - Er wertet die Antwort des Steuergeräts aus (siehe Interpretation unten).
  - Gefundene, aktive DIDs werden samt ihrer Antwortlänge und den Rohbytes in einer Datei auf der SD-Karte protokolliert (z.B. `/data/dids_ecu_19.txt`).

### 4.2 Interpretation der UDS-Antworten (Wichtig für die Suche!)
Wenn der Scanner `22 {DID_HI} {DID_LO}` an ein Steuergerät sendet, gibt es vier mögliche Reaktionen:

| Antwort-Muster | Bedeutung | Interpretation / Nächster Schritt |
|---|---|---|
| **`62 {DID_HI} {DID_LO} [Daten...]`** | **Erfolg! (Positivantwort)** | **Die DID existiert und liefert Daten.** Die Rohdatenlänge und Byte-Werte werden auf SD gespeichert. Diese DID ist ein heißer Kandidat! |
| **`7F 22 31`** | NRC `0x31` (*requestOutOfRange*) | **Die DID existiert auf diesem Steuergerät nicht.** Kann übersprungen werden. |
| **`7F 22 12`** | NRC `0x12` (*subFunctionNotSupported*) | **Die DID wird in dieser Sitzung nicht unterstützt.** Meist gleichbedeutend mit "existiert nicht". |
| **`7F 22 33`** | NRC `0x33` (*securityAccessDenied*) | **Die DID existiert, ist aber durch ein Sicherheits-Login geschützt.** (Erfordert Service `0x27` zur Entsperrung. Zeigt uns, dass an dieser Adresse wichtige/geschützte Systemdaten liegen). |
| **`7F 22 22`** | NRC `0x22` (*conditionsNotCorrect*) | **Die DID existiert, kann aber gerade nicht gelesen werden** (z.B. weil der Motor aus ist, das Fahrzeug rollt, oder das Steuergerät in einem unpassenden Zustand ist). |

### 4.3 Wie man die Bedeutung einer gefundenen DID entschlüsselt (Reverse Engineering)
Sobald der Scanner z.B. 15 existierende DIDs auf dem Gateway (`0x19`) oder dem Motorsteuergerät (`0x01`) gefunden hat, wenden wir die **"Harvest-Drive-Methode"** an, um herauszufinden, welches Signal (z.B. Blinker oder Rekuperation) hinter welcher DID steckt:

1. **Gezieltes Logging:** Trage die gefundenen DIDs in die Polling-Tabelle ein, sodass sie während einer Testfahrt mit hoher Frequenz (z.B. alle 100 ms) gelesen und in die CSV-Datei geschrieben werden.
2. **Spezifische Testfahrt durchführen:**
   - **Beispiel Blinker:** 1 Minute stehen, Blinker links an (10s), aus, Blinker rechts an (10s), aus. Warnblinker an (10s).
   - **Beispiel Rekuperation:** Auf freier Strecke beschleunigen, dann abrupt Fuß vom Gas (Segeln/Rekuperation), Bremse leicht antippen, voll mechanisch bremsen.
   - **Beispiel Lenkung:** Im Stand Lenkrad ganz nach links einschlagen, halten, ganz nach rechts einschlagen, halten.
3. **Korrelation und Bit-Analyse per Python (`analysis/analyze.py`):**
   - Nutze das Analyse-Skript mit der Korrelationsfunktion (`--corr`):
     `python3 analyze.py ../results/raw_sNNN.csv --corr ../results/sNNN.csv --signal Speed_kmh` (oder gegen ein anderes bekanntes Signal).
   - Die Korrelation berechnet die Übereinstimmung der DID-Bytes mit den realen Ereignissen. Ein Korrelationskoeffizient von nahe `+1.0` oder `-1.0` verrät sofort das gesuchte Byte!
   - Für Zustände wie Blinker (binäre Signale) nutzt man die Bit-Toggle-Analyse (`--id {ID}`):
     Hier sieht man sofort, welches spezifische Bit im Byte von `0` auf `1` springt, sobald der Blinker aktiviert wird.

---

## 5. Systematische Suchbereiche nach Steuergeräten (Targeting)

Wenn wir gezielt nach den vom Benutzer gewünschten Signalen suchen, steuern wir folgende Steuergeräte mit dem Scanner an:

### 5.1 Blinker, Lichtsignale, Türstatus
- **Steuergerät:** `0x09` (Bordnetzsteuergerät / BCM) -> `18DA09F1` oder `0x19` (Gateway) -> `18DA19F1`.
- **Hintergrund:** Das Gateway spiegelt fast alle Komfort- und Karosseriesignale für die Diagnose. Im Gateway sind diese oft einfacher und ohne restriktive Sicherheits-Logins abrufbar.

### 5.2 Rekuperation & Mild-Hybrid-Drehmomente
- **Steuergerät:** `0x01` (Motorsteuergerät / ECM) -> `18DA01F1` oder `0x19` (Gateway).
- **Hintergrund:** Der Riemenstartergenerator (RSG) des 48V-Mildhybrids speist Energie zurück. Das Motorsteuergerät erfasst dieses "negative Drehmoment" (Rekuperationsmoment in Nm) oder den Generatorstrom in Ampere.
- **Suchbereich:** DIDs im Bereich `0x1000` bis `0x2FFF` auf `0x01` scannen, während der Fahrt loggen und das "Schubbetrieb-Verhalten" mit dem Gaspedal-Verlauf korrelieren.

### 5.3 Ganganzeige (DSG) & Kupplungsdrücke
- **Steuergerät:** `0x02` (Getriebeelektronik) -> `18DA02F1`.
- **Hintergrund:** Liefert neben dem eingelegten Gang auch Getriebeöltemperaturen, Kupplungsdrücke und Drehzahlen der Getriebewellen.
- **Suchbereich:** DIDs im Bereich `0x0100` bis `0x09FF` auf `0x02`.

Mit diesem systematischen Werkzeugkasten ist das OpenOBD-Projekt nicht mehr auf fremde Tabellen angewiesen, sondern kann sich die gesamte Datenwelt des VW Golf 8.5 eTSI vollautomatisch und absolut gefahrlos selbst erschließen!
