# OBD-II Referenz & Ableitungen – VW Golf 8 1.5 eTSI

> Basierend auf deinem Raw-CAN-Log und den dekodierten Daten.  
> Enthält: 10 neu identifizierte Standard-PIDs + alle daraus ableitbaren physikalischen Werte.

---

## 1. Neu identifizierte Standard-PIDs (Mode 01)

Dein aktuelles Tool dekodiert bereits 32 Parameter. Diese 10 PIDs sind in deinem **Raw-Log** vorhanden, fehlen aber noch in deinem dekodierten CSV.  
*(Hinweis: `A` bis `D` stehen für die Datenbytes D4 bis D7 im CAN-Frame nach `41 PID`).*

| PID (Hex) | PID (Name) | Einheit | Formel (SAE J1979) | Beispiel |
| :--- | :--- | :--- | :--- | :--- |
| **0x03** | Fuel System Status | Status | `Byte A` = Bank 1: <br> 1=OpenLoop, 2=ClosedLoop, 4=OL_Error, 8=CL_Error | `41 03 02` → **Closed Loop** |
| **0x10** | Mass Air Flow (MAF) | g/s | `((A*256)+B) / 100` | `41 10 01 5E` → **3.50 g/s** |
| **0x21** | Distance since MIL on | km | `(A*256)+B` | `41 21 00 0D` → **13 km** |
| **0x30** | Warm-ups since DTC clear | Anzahl | `A` | `41 30 00` → **0** |
| **0x45** | Evap Vapor Pressure | kPa | `((A*256)+B) * 0.0390625` <br> *(Vorzeichen: wenn Wert > 32767, dann -65536)* | `41 45 80 00` → **-0.0 kPa** |
| **0x47** | Absolute Throttle Position B | % | `(A*100)/255` | `41 47 64` → **39.2 %** |
| **0x51** | Fuel Type | Typ | `A` = Code: <br> 1=Benzin, 2=Methanol, ... 8=Elektro | `41 51 01` → **Benzin** |
| **0x53** | Ethanol Fuel % | % | `(A*100)/255` | `41 53 00` → **0 %** |
| **0x55** | Relative Throttle Position | % | `(A*100)/255` | `41 55 32` → **19.6 %** |
| **0x70** | Fuel Rail Pressure (gauge) | kPa | `((A*256)+B) * 0.0390625` | `41 70 01 2C` → **300 kPa** |

---

## 2. Abgeleitete Werte (Formelsammlung)

Basierend auf den 42 PIDs (deine 32 + die 10 neuen) lassen sich folgende Größen berechnen.

> **Allgemeine Annahmen:**  
> `V_h` = 1,5 Liter (Hubraum 1.5 TSI)  
> `n` = Drehzahl (RPM)  
> `M` = Drehmoment (Nm)  
> Kraftstoffdichte ≈ 750 g/L  

---

### 2.1 Leistung & Drehmoment

| Bezeichnung | Formel | Benötigte PIDs |
| :--- | :--- | :--- |
| **Motorleistung (kW)** | \(P_{kW} = \dfrac{n \cdot M}{9550}\) | **RPM** (0x0C), **ActualTorque** (0x63) |
| **Motorleistung (PS)** | \(P_{PS} = P_{kW} \cdot 1,3596\) | s.o. |
| **Spez. Kraftstoffverbrauch (BSFC)** | \(\text{BSFC} = \dfrac{\text{FuelRate [L/h]} \cdot 750}{P_{kW}}\) [g/kWh] | **FuelRate** (0x5E), \(P_{kW}\) |
| **Mittlerer effektiver Druck (pme)** | \(p_{me} = \dfrac{P_{kW} \cdot 1200}{n \cdot 1,5}\) [kPa] | **RPM**, \(P_{kW}\) |

---

### 2.2 Verbrennung & Gemisch

| Bezeichnung | Formel | Benötigte PIDs |
| :--- | :--- | :--- |
| **Luftverhältnis (Lambda)** | \(\lambda = \text{CmdEquivRatio}\) | **CmdEquivRatio** (0x34) |
| **AFR (Luft-Kraftstoff-Verhältnis)** | \(AFR = \lambda \cdot 14,7\) | s.o. |
| **Gemisch-Korrekturfaktor** | \(\text{Korr} = 1 + \dfrac{STFT + LTFT}{100}\) | **STFT** (0x06), **LTFT** (0x07) |
| **Luftdichte im Saugrohr** | \(\rho = \dfrac{MAP \cdot 1000}{287 \cdot (IAT + 273,15)}\) [kg/m³] | **MAP** (0x0B), **IAT** (0x0F) |
| **Volumetrischer Wirkungsgrad** | \(\eta_v = \dfrac{MAF_{kg/s}}{\rho \cdot 0,0015 \cdot \frac{n}{120}}\) [%] | **MAF** (0x10), MAP, IAT, RPM |

---

### 2.3 Turbolader & Ladedruck

| Bezeichnung | Formel | Benötigte PIDs |
| :--- | :--- | :--- |
| **Ladedruckverhältnis** | \(\Pi = \dfrac{MAP}{Baro}\) | **MAP**, **Baro** (0x33) |
| **Überdruck (Boost)** | \(\Delta p = MAP - Baro\) [kPa] | **MAP**, **Baro** |
| **Verdichter-Austrittstemp. (theor.)** | \(T_2 = (IAT+273,15) \cdot \Pi^{0,286}\) [K] | **IAT**, \(\Pi\) |
| **Turbo-Response (Gradient)** | \(\text{Response} = \dfrac{\Delta MAP}{\Delta \text{ThrottlePos}}\) | MAP, **Throttle** (0x11) |

---

### 2.4 Fahrdynamik & Verbrauch

| Bezeichnung | Formel | Benötigte PIDs |
| :--- | :--- | :--- |
| **Momentanverbrauch** | \(\text{Verbr.} = \dfrac{\text{FuelRate [L/h]}}{\text{Speed [km/h]}} \cdot 100\) [L/100km] | **FuelRate**, **Speed** (0x0D) |
| **Beschleunigung** | \(a = \dfrac{v_2 - v_1}{t_2 - t_1}\) [m/s²] | **Speed**, **Timestamp** |
| **Fahrwiderstandsleistung** | \(P_{FW} = P_{kW} - \dfrac{1350 \cdot a \cdot v}{1000}\) [kW] *(m ≈ 1350 kg)* | Leistung, Speed, a |
| **Luftwiderstandskraft** | \(F_{Luft} = 0,5 \cdot 0,28 \cdot 2,2 \cdot \rho_{Luft} \cdot v^2\) [N] *(cw=0,28, A=2,2 m²)* | Speed, Außentemperatur (optional) |

---

### 2.5 Zündung & Klopfregelung

| Bezeichnung | Formel | Benötigte PIDs |
| :--- | :--- | :--- |
| **Zündwinkel-Reserve** | \(\text{Reserve} = 40^\circ - \text{TimingAdv}\) [°] | **TimingAdv** (0x0E) |
| **Klopf-Aktivitätsindex** | \(\text{Klopf} = \text{True, wenn } \text{TimingAdv} < 10^\circ\) | TimingAdv |

---

### 2.6 Bordnetz & Elektrik

| Bezeichnung | Formel | Benötigte PIDs |
| :--- | :--- | :--- |
| **Spannungsstabilität** | \(\Delta U = U_{max} - U_{min}\) (über Zeitfenster) | **ModuleVoltage** (0x42) |

---

### 2.7 Abgas & Katalysator

| Bezeichnung | Formel | Benötigte PIDs |
| :--- | :--- | :--- |
| **Kat-Konversionseffizienz (geschätzt)** | \(\eta_{Kat} = 1 - \dfrac{O2S2\_V}{0,45}\) (vereinfacht) | **O2S2_V** (0x56) |
| **Kat-Aufheizdauer** | \(\Delta t = t_{Kat > 400^\circ C} - t_{Start}\) | **CatalystTemp** (0x3C), Timestamp |

---

### 2.8 Statistik & Korrelation

| Bezeichnung | Formel | Benötigte PIDs |
| :--- | :--- | :--- |
| **Korrelation (Pedal ↔ Torque)** | \(\text{Corr}(AccelPedal, ActualTorque)\) | **AccelPedal** (0x44), **ActualTorque** |
| **Motorbeanspruchung** | \(\text{Stress} = \dfrac{ActualTorque \cdot RPM}{OilTemp}\) (relative Einheit) | ActualTorque, RPM, **OilTemp** (0x5C) |
| **Fahreffizienz-Index** | \(\eta_{Fahrt} = \dfrac{Speed}{\text{Momentanverbrauch}}\) [km/l] | Speed, FuelRate |

---

## 3. Nächste Schritte

1. Übertrage die 10 neuen PIDs in deine Decoder-Tabelle, um sie in deinem Haupt-CSV zu ergänzen.
2. Berechne die abgeleiteten Werte mit Excel/Google Sheets oder deinem bevorzugten Tool.
3. Die spannendsten Felder für deine Analyse sind:
   - `Volumetrischer Wirkungsgrad` (Motorgesundheit)
   - `BSFC` (Effizienz im Teillastbereich)
   - `Boost Pressure` (Turbo-Ladeverhalten)
   - `Fahreffizienz-Index` (Eco-Tracking)
