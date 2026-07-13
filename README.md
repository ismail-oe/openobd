# OpenOBD

An open-source CAN-bus data logger for the **VW Golf 8.5 eTSI**. Local-first, completely subscription-free, and open-source.

This tool polls standard OBD2 parameters while simultaneously capturing every raw CAN frame passing through the bus.

*(Forked from [roypeter/esp32-obd2-logger](https://github.com/roypeter/esp32-obd2-logger))*

> **Current Status (v2):** Tested and running stably in the vehicle. Supports 29-bit OBD (~28 PIDs), raw logging, compile-time configuration via `include/config.h`, Wi-Fi joining with NTP time sync, a clean web-based mobile dashboard, gentle UDS scanning, and live developer streaming.

---

# Features

## Core Capabilities

- **Clean OBD2 Polling:** Actively polls standard OBD2 PIDs and saves decoded, ready-to-use values into tidy `s{Session_ID}.csv` files.
- **Raw CAN Capturing:** Records every sent and received CAN frame in SavvyCAN-compatible format (`raw_s{Session_ID}.csv`). Timestamps use 64-bit precision to prevent overflow on long road trips.
- **Live Bus Explorer (`/explore`):** A browser-based view showing every active CAN ID, its frequency (Hz), message counter, and live byte changes. Perfect for seeing what your Golf's network is doing in real time.
- **Auto-Record on Boot:** Automatically starts logging as soon as the ignition turns on. You can pause, resume, or start a **New Session** directly from the dashboard.
- **Fault-Tolerant Storage:** Instantly flushes files to the microSD card to prevent data loss when the ignition is switched off. Automatically falls back to internal flash memory if the SD card is missing or full.
- **PID Discovery (`/pids`):** Queries your Golf using bitmasks to find out exactly which OBD2 PIDs are physically supported, then polls only those. Includes a manual re-scan button if the vehicle's ECUs were still asleep during boot.
- **Modern Cockpit Dashboard:** A dark-themed, mobile-friendly instrument cluster featuring gauges, status cards, history graphs, and computed metrics (e.g. real-time & average L/100km fuel economy, horsepower, and boost pressure).
- **Polite UDS Scan (`/uds`):** Performs read-only diagnostics (Service `0x22`) to identify ECUs and read software numbers safely, without triggering warning lights or initiating intrusive sessions.
- **Wireless Live Streaming:** In developer mode, raw frames are streamed wirelessly over TCP or MQTT (network tasks run on Core 0 so they never interrupt critical CAN bus logging).

---

# Hardware

## Bill of Materials

| Part | Status / Note |
|------|----------------|
| **LilyGO T-CAN485** | ESP32-based board with onboard CAN transceiver and microSD slot |
| **MicroSD Card (≤ 32 GB)** | **Must be formatted as FAT32** (the ESP32 SD library does not support exFAT) |
| **OBD2 16-Pin Breakout Cable** | Open-ended wires for manual pinning |
| **12V-to-5V USB-C Step-Down Converter** | Rugged automotive-grade converter to safely power the board and filter voltage spikes |
| **Wago 221 Connectors** | For quick, solderless wiring |

## Pinout Configuration (LilyGO T-CAN485)

- **5V Boost Enable (Powers CAN & SD):** GPIO 16
- **CAN Transceiver:** TX (GPIO 27) / RX (GPIO 26) / Silent Enable (GPIO 23)
- **microSD Slot:** SCLK (GPIO 14) / MISO (GPIO 2) / MOSI (GPIO 15) / CS (GPIO 13)
- **External DS3231 RTC (Optional):** SDA (GPIO 32) / SCL (GPIO 33)

---

# Wiring Guide

## 1. Connecting the OBD2 Cable to the T-CAN485 Screw Terminals

| OBD2 Pin | Typical Wire Color | Function | Connection on Board |
|----------|--------------------|----------|---------------------|
| **Pin 6** | Green | CAN-High | Screw Terminal **CANH** |
| **Pin 14** | Green-White | CAN-Low | Screw Terminal **CANL** |
| **Pin 5** | Light Blue | Signal Ground | Screw Terminal **GND** |

## 2. Sane Power Strategy (Ignition-Switched Power)

To prevent the logger from slowly draining your car's battery, we do **not** use the constant 12V pin (Pin 16) from the OBD port. Instead, we tap into ignition-switched 12V:

- **OBD2 Pin 1 (Ignition 12V / usually Brown):** Connect via an inline 2A fuse to the red (+) input wire of your 12V-to-5V USB converter.
- **OBD2 Pin 4 (Chassis Ground / usually Orange):** Connect to the black (−) input wire of your 12V-to-5V USB converter.
- **USB Converter Output:** Connect directly to the T-CAN485's USB-C port.

> ⚠️ **The Three Golden Rules**
>
> 1. **Do not swap CAN-High and CAN-Low.** If you do, communication will fail completely.
> 2. **Never feed raw 12V–14.7V vehicle power directly to the board's GPIO pins.** Always use a regulated 5V step-down converter.
> 3. **Individually isolate all 13 unused wires** of your OBD breakout cable using electrical tape or heat shrink tubing before plugging it into the car. Loose, bare wires can cause short circuits.

---

# Compilation & Flashing

The project is built using **PlatformIO** (either via the VS Code extension or the command line).

## 1. Configuration

Open `include/config.h` to set up your Wi-Fi credentials, NTP servers, and default polling intervals.

You don't need to change config files on the SD card—the operating mode is determined by which target environment you flash.

| Target | Environment Name | Dashboard | Raw CAN Logging | Live Streaming |
|--------|-------------------|-----------|-----------------|----------------|
| **Normal** | `t-can485` | ✅ Fully Enabled | Disabled (minimal) | Disabled |
| **Developer** | `t-can485-dev` | Disabled | ✅ Enabled | ✅ Enabled (TCP/MQTT) |

## 2. Flashing the Board

Connect the board to your computer via USB-C and run the appropriate command:

```bash
# Compile and flash the Normal Mode environment
pio run -e t-can485 --target upload --target monitor

# Compile and flash the Developer Mode environment
pio run -e t-can485-dev --target upload --target monitor
```

> **Troubleshooting:** If the upload utility cannot find the board, hold down the physical **BOOT** button on the T-CAN485, tap the **RST** button, release **BOOT**, and try flashing again.

---

# Testing Procedure

## Bench Test (On Your Desk)

1. Insert a FAT32-formatted microSD card into the board.
2. Power the board using a standard USB-C cable.
3. Join the Wi-Fi network **OpenOBD** (Password: `openobd1234`).
4. Open your browser and navigate to `http://192.168.4.1`.
5. The dashboard should load. (All values will display as `--` because the board is not connected to a vehicle CAN bus.)

## Vehicle Test (In Your Car)

1. With the ignition completely off, connect the OBD2 cable's CAN and Ground wires to the board.
2. Turn the ignition on.
3. Power up the board.
4. Connect to the **OpenOBD** Wi-Fi network (or let the board join your configured hotspot if `CFG_WIFI_MODE` is set to `"join"`).
5. Open `http://openobd.local` (or `http://192.168.4.1`).
6. Navigate to `/explore` and verify that raw CAN frames are being received.

---

# SD Card Directory Structure

Data is organized by year and month to keep the filesystem fast and readable.

```text
├── pids.txt             # Discovered OBD2 PIDs
├── sessions.csv         # Index of recorded sessions
├── uds_scan.txt         # Passive UDS scan output
└── data/
    └── 2026/
        └── 07/
            ├── s001.csv
            └── raw_s001.csv
```

> **Timekeeping:** If no internet connection is available during boot, the system restores the last known timestamp from `/lasttime.txt` to approximate folder creation. Once NTP becomes available, the internal clock is corrected automatically.

---

# Roadmap

- [ ] Multi-frame UDS: Flow-control implementation to read VINs and long ECU names.
- [ ] Custom VAG Dashboards: High-voltage 48V systems, DSG oil temperatures, ACT state, energy flow, and additional manufacturer-specific values.
- [ ] Native GVRET Protocol: Direct SavvyCAN compatibility over the network without intermediary Python scripts.
- [ ] Code Refactoring: Split the monolithic firmware into clean modules (CAN, Storage, Web, UDS, PIDs).