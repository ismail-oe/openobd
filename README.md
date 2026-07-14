# OpenOBD

A small, rugged **CAN‑bus data logger for the VW Golf 8.5 eTSI** (1.5 TSI mild‑hybrid, DSG),
built on the **LilyGO T‑CAN485** (ESP32 + CAN transceiver + microSD).

It quietly logs your car to the SD card on every drive and broadcasts a few live values
over **Bluetooth LE** so you can glance at them in any BLE scanner app — no cloud, no
account, no app to install, fully local. Meant to stay permanently plugged into the OBD‑II
port and just run.

> Fork of [roypeter/esp32-obd2-logger](https://github.com/roypeter/esp32-obd2-logger),
> heavily rewritten.

---

## What it does

- **Polls standard OBD‑II PIDs** and decodes ~40 values into a clean CSV (`s001.csv`) —
  RPM, speed, coolant, boost, load, torque, lambda, fuel trims, consumption, **gear**
  (from the DSG), **odometer**, trip distance/fuel, and more.
- **Auto‑records on boot.** Plug it in, ignition on, it logs. No buttons, no phone needed.
- **BLE status broadcast** — a heartbeat plus live values, readable in any BLE scanner
  (see [Bluetooth](#bluetooth)). This replaces a web dashboard: far lighter and more robust
  for a permanent install.
- **Bomb‑proof by design:** fixed buffers (no heap growth), single‑writer SD access under a
  mutex, CAN bus‑off recovery, file closed after every write (power‑loss safe), and a
  **software watchdog that auto‑reboots** if anything ever hangs — then keeps logging.
- **Time via NTP** — it briefly joins a known Wi‑Fi at boot to get the real time (with
  automatic daylight‑saving), then turns Wi‑Fi off and runs on BLE only.
- **Dev mode** additionally writes a full raw capture of every CAN frame (SavvyCAN format)
  and can stream frames wirelessly for reverse‑engineering.

The only thing you have to manage is the **SD card filling up** — everything else is meant
to take care of itself.

---

## Hardware

| Part | Notes |
|---|---|
| LilyGO T‑CAN485 (ESP32‑WROOM‑32 + SN65HVD231 + microSD) | the logger |
| microSD, **≤ 32 GB, FAT32** | 32 GB holds years of drives; 64 GB must be re‑formatted to FAT32 (ESP32 can't read exFAT) |
| OBD‑II 16‑pin breakout cable | to tap CAN + power |
| 12 V → 5 V USB‑C car adapter + inline fuse | for the permanent install (never feed 12 V into the board directly) |

### Pin map (in `src/main.cpp`)

| Function | GPIO |
|---|---|
| 5 V boost enable (powers CAN + SD) | 16 |
| CAN TX / RX / silent‑enable | 27 / 26 / 23 |
| SD SCLK / MISO / MOSI / CS | 14 / 2 / 15 / 13 |

### Wiring (OBD‑II → board)

| OBD pin | Wire | To |
|---|---|---|
| 6 | CAN‑H | screw terminal **CANH** |
| 14 | CAN‑L | screw terminal **CANL** |
| 5 | GND (signal) | screw terminal **GND** |
| 1 | ignition + | → 2 A fuse → 12 V→5 V adapter → USB‑C |
| 16 | 12 V constant | **do not connect** |

> The Golf 8.5 speaks OBD only over **29‑bit addressing (ISO 15765‑4) at 500 kbit/s**
> (functional request `0x18DB33F1`). The firmware handles this automatically.

---

## Build & flash

Uses [PlatformIO](https://platformio.org/). Two flash targets — you just pick one:

```bash
# Normal: logs to SD + BLE broadcast (everyday use)
pio run -e t-can485      --target upload --target monitor

# Dev: no dashboard, full raw CAN log + optional wireless stream (reverse-engineering)
pio run -e t-can485-dev  --target upload --target monitor
```

(If `pio` isn't on your PATH, use `python3 -m platformio run ...`.)

---

## Configuration

Everything is set at compile time in **`include/config.h`** — there is **no config file on
the SD card**. Edit it, flash, done.

| Setting | Meaning |
|---|---|
| `CFG_DEV_MODE` | usually set by the flash target (normal vs dev) |
| `CFG_LOG_DECODED` / `CFG_LOG_RAW` | write the clean CSV / the raw capture |
| `CFG_LOG_INTERVAL_MS` | how often the clean CSV is written (default 250 ms) |
| `CFG_WIFI_MODE` | `"join"` = get time from Wi‑Fi/NTP, then Wi‑Fi off · `"off"` = never use Wi‑Fi |
| `CFG_WIFI_1_SSID` … `CFG_WIFI_4_SSID` | up to 4 known networks; it connects to the strongest one available |
| `CFG_TZ` | POSIX timezone with DST, e.g. Germany `"CET-1CEST,M3.5.0,M10.5.0/3"`, Turkey `"<+03>-3"` |
| `CFG_TCP_PORT` / `CFG_MQTT*` | dev‑mode wireless streaming |

> **Tip for road trips:** add your car's built‑in hotspot **and** your phone's hotspot to
> the Wi‑Fi list, so it can always grab the correct time. Without any known network it still
> logs fine, just with a relative timestamp.

> **Wi‑Fi passwords live in `config.h`.** Keep your repo private, or git‑ignore `config.h`,
> before pushing.

---

## Bluetooth

The device advertises as **`OpenOBD`**. Connect with any BLE scanner app
(e.g. **nRF Connect**, **Bluetooth Inspector**) and read these characteristics (each has a
human‑readable name):

| Characteristic | Example | Shows |
|---|---|---|
| **Status** | `HB188 UP189s HEAP141k DROP0` | heartbeat (counts up = alive), uptime, free RAM, dropped frames |
| **System** | `SD3756MB REC1 S18 T:12:44:03` | SD space free, recording, session #, clock |
| **Trip** | `KM42.1 L2.6 NOW5.3` | trip distance, trip litres, instant consumption (L/100) |
| **PIDs** | `41 PIDs: 04 05 0B 0C 0D …` | which OBD PIDs the car reports |
| **Connection** | `verbunden RX108764 TEC0` | whether the car is answering on the CAN bus |

You never *need* the app — it's just for an occasional "is everything running?" glance.
The heartbeat counting up tells you it's alive and logging.

---

## Files on the SD card

Logs are sorted by year/month; clean and raw data are kept separate:

```
/pids.txt  /session.txt        supported PIDs · session counter
/sessions.csv  /lasttime.txt    session index · last known time (RTC substitute)
/uds_scan.txt                   optional UDS scan result
/data/2026/07/s001.csv          clean, decoded logs (+ s001.anchor)
/dev/2026/07/raw_s001.csv       raw CAN capture (dev mode only)
```

To pull data off, just read the SD card on a computer. The clean CSVs open in Excel/pandas;
the raw captures open in [SavvyCAN](https://github.com/collin80/SavvyCAN).

---

## If something looks wrong

If PIDs aren't detected, the time is off, or values look stuck — **restart the device to
re‑run the whole setup** (discovery, NTP time, self‑test):

- **Press the `RST` button** on the T‑CAN485, or cycle the ignition (it's ignition‑powered).
- The `BOOT` button is only for flashing — you don't need it for a normal restart.

The software watchdog also reboots automatically if the firmware ever hangs.

---

## Analysis kit

`analysis/analyze.py` turns a big raw capture into a compact summary (ID inventory, UDS
overview, bit‑flip analysis, reference‑signal correlation) you can feed to any tool or AI —
because an LLM can't crunch a million‑row CSV directly, but it can read a one‑page summary.
See `analysis/README.md`.

---

## Status

Running in the car: ~40 decoded PIDs including gear and odometer, dated SD logs, BLE status,
auto‑reboot watchdog. Possible next steps: multi‑frame UDS for manufacturer‑specific values,
and (a bigger project) tapping the raw vehicle bus with the opendbc database for signals the
OBD port doesn't expose.
