# OpenOBD — CAN-Analyse-Kit (KI-neutral)

Dieses Kit macht dich **unabhängig von einem teuren KI-Chat-Kontext**. Es verdichtet
deine großen CAN-Rohlogs lokal (kostenlos) zu kompakten Berichten, die du in **jede**
KI einfügen kannst — Gemini, ChatGPT, Claude, egal.

## Das Grundprinzip (wichtig!)

> **Python rechnet, die KI interpretiert.**

Eine `raw_sNNN.csv` hat bis zu ~1 Mio. Zeilen (30 MB+). Wenn du die einer KI in den
Chat kippst:
- wird es **teuer** (jede Zeile kostet Tokens),
- wird es **langsam**,
- und die KI **rechnet trotzdem falsch** — LLMs können große Zahlentabellen nicht
  zuverlässig aggregieren.

Deshalb: erst `analyze.py` laufen lassen → es kommt ~1 Seite Text raus → **die** gibst
du der KI zum Deuten. Kostet Cent statt Dollar.

## Benutzung

```bash
cd analysis

# Gesamtüberblick eines Logs (ID-Inventar + UDS-Übersicht + Bit-Analyse der Top-IDs)
python3 analyze.py ../results/raw_s030.csv

# Als Markdown-Datei speichern (die lädst du bei der KI hoch / fügst sie ein)
python3 analyze.py ../results/raw_s030.csv --out bericht_s030.md

# Eine einzelne CAN-ID im Detail (jedes Byte: Wertebereich + welche Bits wechseln)
python3 analyze.py ../results/raw_s030.csv --id 18DAF101

# Korrelation: welches Byte korreliert mit einem bekannten Signal?
python3 analyze.py ../results/raw_s030.csv --corr ../results/s030.csv --signal RPM
#   verfügbare Signale = Spaltennamen in der dekodierten sNNN.csv
#   (RPM, Speed_kmh, Coolant_C, MAP_kPa, Load_pct, ...)
```

Abhängigkeiten: `python3`, `pandas`, `numpy` (schon installiert). Falls nicht:
`pip3 install pandas numpy`.

## Was die Ausgabe bedeutet

- **ID-Inventar** — welche CAN-IDs, wie oft, wie schnell, Tx (wir senden) vs Rx
  (Fahrzeug antwortet), wie viele verschiedene Payloads.
- **UDS/Diagnose-Übersicht** — erkennt 29-Bit-Diagnoseframes (`18DA..`), trennt
  Requests / positive Antworten (`62 ...`) / negative Antworten (`7F svc NRC`).
  Genau hier siehst du z.B. `18DAF110 NEG svc 22 NRC 11` = DSG lehnt UDS-Read ab.
- **Bit-Analyse** — pro Byte: min/max + welche Bits sich je ändern (`X`) vs. statisch
  sind (`0`/`1`). So erkennst du Zähler/Checksummen (fast alle Bits wechseln) vs.
  echte Signal-Kandidaten.
- **Korrelation** — Reference-Signal-Methode: `|corr|` nahe 1.0 = das Byte kodiert
  vermutlich dieses Signal. **Achtung:** Am OBD-Port dieses Golf fließen nur
  Diagnose-Antworten (schon dekodiert) → passive Korrelation findet hier wenig Neues.
  Voll nützlich wird sie bei **UDS-DID-Antworten** oder einem direkten CAN-Abgriff.

## Der empfohlene Arbeits-Loop (mit jeder KI)

1. Neue Fahrt loggen → `raw_sNNN.csv` von SD holen, in `results/` legen.
2. `python3 analyze.py ../results/raw_sNNN.csv --out bericht.md`
3. `bericht.md` + die Datei `../HANDOFF.md` (Projekt-Kontext) der KI geben.
4. KI um Deutung / nächsten Schritt / Code-Änderung bitten.
5. Änderung an `src/main.cpp`, neu flashen, wieder fahren. Zurück zu 1.

---

## Einfüge-Text für die KI (Kontext-Primer)

> Kopiere den folgenden Block **einmal** in eine neue KI-Unterhaltung, dann hängst du
> den `analyze.py`-Bericht an. So ist jede KI sofort im Bild.

```
Du hilfst mir bei "OpenOBD", einem ESP32-CAN-Datenlogger (LilyGO T-CAN485) für einen
VW Golf 8.5 eTSI (1.5 TSI, 48V-Mildhybrid, DSG). Firmware in C++/Arduino/PlatformIO.

Feste Fakten (nicht neu debuggen):
- Board-Pins: CAN TX=27, RX=26, Silent-Enable=23 (LOW), 5V-Enable=16 (HIGH).
- Der Golf spricht OBD NUR über 29-Bit-Adressierung, 500 kbit/s, Padding 0xAA.
  Funktionale Anfrage 0x18DB33F1; Antworten 0x18DAF101/102 (Motor), 0x18DAF110 (DSG).
  11-Bit (0x7DF/0x7E0) bekommt KEINE Antwort.
- UDS physisch: Anfrage an STG 0xNN = 0x18DA{NN}F1, Antwort = 0x18DAF1{NN}.
  STG: 0x01/0x02 Motor, 0x10 DSG. Service 0x22 = ReadDataByIdentifier.
- VAG-Werte (48V/DSG/ACM) sind NICHT im Broadcast — nur aktiv per UDS 0x22 lesbar.
- Aktueller Blocker: DSG antwortet auf UDS, aber Service 0x22 ist in der Standard-
  Sitzung gesperrt (NRC 0x11 serviceNotSupported). Nächster Schritt: erweiterte
  Diagnose-Sitzung (10 03) + TesterPresent (3E 00), dann 0x22 lesen, dann DID-Scan.
- Regel: UDS während der Fahrt STRIKT read-only (nur 0x10/0x3E/0x22; nie 0x2E/0x31).

Logformat (SavvyCAN-CSV): Time Stamp(µs),ID(hex),Extended,Dir,Bus,LEN,D1..D8(hex).
Ich schicke dir gleich einen mit analyze.py erzeugten Bericht. Bitte hilf mir, ihn zu
deuten und den nächsten konkreten Schritt (Firmware oder Analyse) vorzuschlagen.
```
