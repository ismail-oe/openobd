# OpenOBD

An open-source CAN-bus data logger for the **VW Golf 8.5 eTSI**. Local-first, completely subscription-free, and open-source. 
This tool polls standard OBD2 parameters while simultaneously capturing every raw CAN frame passing through the bus.
*(Forked from [roypeter/esp32-obd2-logger](https://github.com/roypeter/esp32-obd2-logger))*

> **Current Status (v2):** Tested and running stably in the vehicle. Supports 29-bit OBD (~28 PIDs), raw logging, compile-time configuration via `include/config.h`, Wi-Fi joining with NTP time sync, a clean web-based mobile dashboard, gentle UDS scanning, and live developer streaming.

---

## Features

### Core Capabilities
* **Clean OBD2 Polling:** Actively polls standard OBD2 PIDs and saves decoded, ready-to-use values into tidy `s{Session_ID}.csv` files.
* **Raw CAN Capturing:** Records every sent and received CAN frame in SavvyCAN-compatible format (`raw_s{Session_ID}.csv`). Timestamps use 64-bit precision to prevent overflow on long road trips.
* **Live Bus Explorer (`/explore`):** A browser-based view showing every active CAN ID, its frequency (Hz), message counter, and live byte changes. Perfect for seeing what your Golf's network is doing in real time.
* **Auto-Record on Boot:** Automatically starts logging as soon as the ignition turns on. You can pause, resume, or start a "New Session" directly from the dashboard.
* **Fault-Tolerant Storage:** Instantly flushes files to the microSD card to prevent data loss when the ignition is switched off. Automatically falls back to internal flash memory if the SD card is missing or full.
* **PID Discovery (`/pids`):** Queries your Golf using bitmasks to find out exactly which OBD2 PIDs are physically supported, then polls only those. Includes a manual re-scan button if the vehicle's ECUs were still asleep during boot.
* **Modern Cockpit Dashboard:** A dark-themed, mobile-friendly instrument cluster featuring gauges, status cards, history graphs, and computed metrics (e.g., real-time & average L/100km fuel economy, horsepower, and boost pressure).
* **Polite UDS Scan (`/uds`):** Performs read-only diagnostics (Service 0x22) to identify ECUs and read software numbers safely, without triggering warning lights or initiating intrusive sessions.
* **Wireless Live Streaming:** In developer mode, raw frames are streamed wirelessly over TCP or MQTT (network tasks run on Core 0 so they never interrupt critical CAN bus logging).

---

## Hardware

### Bill of Materials
| Part | Status / Note |
|---|---|
| **LilyGO T-CAN485** | ESP32-based board with onboard CAN transceiver and microSD slot |
| **MicroSD Card (≤ 32 GB)** | **Must be formatted as FAT32** (the ESP32 SD library does not support exFAT) |
| **OBD2 16-Pin Breakout Cable** | Open-ended wires for manual pinning |
| **12V-to-5V USB-C Step-Down Converter** | Rugged automotive-grade converter to safely power the board and filter voltage spikes |
| **Wago 221 Connectors** | For quick, solderless wiring |

### Pinout Configuration (LilyGO T-CAN485)
* **5V Boost Enable (Powers CAN & SD):** GPIO 16
* **CAN Transceiver:** TX (GPIO 27) / RX (GPIO 26) / Silent Enable (GPIO 23)
* **microSD Slot:** SCLK (GPIO 14) / MISO (GPIO 2) / MOSI (GPIO 15) / CS (GPIO 13)
* **External DS3231 RTC (Optional):** SDA (GPIO 32) / SCL (GPIO 33)

---

## Wiring Guide

### 1. Connecting the OBD2 Cable to the T-CAN485 Screw Terminals:

| OBD2 Pin | Typical Wire Color | Function | Connection on Board |
| :--- | :--- | :--- | :--- |
| **Pin 6** | Green | CAN-High | Screw Terminal **CANH** |
| **Pin 14** | Green-White | CAN-Low | Screw Terminal **CANL** |
| **Pin 5** | Light Blue | Signal Ground | Screw Terminal **GND** |

### 2. Sane Power Strategy (Ignition-Switched Power)
To prevent the logger from slowly draining your car's battery, we do **not** use the constant 12V pin (Pin 16) from the OBD port. Instead, we tap into ignition-switched 12V:

* **OBD2 Pin 1 (Ignition 12V / usually Brown):** Connect via a inline 2A fuse to the Red (+) input wire of your 12V-to-5V USB converter.
* **OBD2 Pin 4 (Chassis Ground / usually Orange):** Connect to the Black (−) input wire of your 12V-to-5V USB converter.
* **USB Converter Output:** Connect directly to the T-CAN485's USB-C port.

> ⚠️ **The Three Golden Rules:**
> 1. **Do not swap CAN-High and CAN-Low.** If you do, communication will fail completely.
> 2. **Never feed raw 12V-14.7V vehicle power directly to the board's GPIO pins.** Always run it through a regulated 5V step-down converter.
> 3. **Individually isolate all 13 unused wires** of your OBD breakout cable using electrical tape or shrink tubing before plugging it into the car. Loose, bare wires can cause short circuits.

---

## Compilation & Flashing

The project is built using **PlatformIO** (either via the VS Code extension or the command line).

### 1. Configuration
Open `include/config.h` to set up your Wi-Fi credentials, NTP servers, and default polling intervals. 

You don't need to change config files on the SD card—the operating mode is determined by which target environment you flash:

| Target | Environment Name | Dashboard | Raw CAN Logging | Live Streaming |
|---|---|---|---|---|
| **Normal** | `t-can485` | ✅ Fully Enabled | Disabled (minimal) | Disabled |
| **Developer** | `t-can485-dev` | Disabled | ✅ Enabled | ✅ Enabled (TCP/MQTT) |

### 2. Flashing the Board
Connect the board to your computer via USB-C and run the appropriate command:

```bash
# Compile and flash the Normal Mode environment:
pio run -e t-can485 --target upload --target monitor

# Compile and flash the Developer Mode environment:
pio run -e t-can485-dev --target upload --target monitor
Troubleshooting: If the upload utility cannot find the board, hold down the physical BOOT button on the T-CAN485, tap the RST button, let go of the BOOT button, and try flashing again.
Testing Procedure
Bench Test (On your desk)
Insert a FAT32-formatted microSD card into the board.
Power the board using a standard USB-C cable.
On your phone or laptop, join the Wi-Fi network named OpenOBD (Password: openobd1234).
Open your browser and navigate to http://192.168.4.1.
The dashboard should load. (All values will display as -- because the board is not connected to a vehicle's CAN bus).
Vehicle Test (In your car)
With the car's ignition completely off, connect the OBD2 cable's CAN and Ground wires to the board.
Turn the car's ignition on.
Power up the board (using your wired-in USB supply or a portable power bank for testing).
Connect to the OpenOBD network (or let the board join your car's built-in Wi-Fi hotspot if you configured CFG_WIFI_MODE as "join").
Open your browser and visit http://openobd.local (or http://192.168.4.1).
Navigate to /explore to confirm you are actively seeing raw CAN frames flowing across the network.
SD Card Directory Structure
Data is neatly organized in folders by year and month to keep the file system fast and readable:
Plaintext
├── pids.txt             # Discovered OBD2 PIDs supported by the vehicle
├── sessions.csv         # Index list of all recorded sessions
├── uds_scan.txt         # Output of the passive UDS controller scan
└── data/
    └── 2026/
        └── 07/
            ├── s001.csv     # Decoded OBD2 sensor data (speed, RPM, temps, etc.)
            └── raw_s001.csv # Raw CAN log in SavvyCAN format (Dev mode only)
Timekeeping note: If an internet connection isn't available at boot to fetch NTP time, the system reads the last known timestamp from /lasttime.txt to approximate the folder structure. As soon as your phone connects to the dashboard or the device connects to the internet via Wi-Fi, the system clock updates and corrects future folder naming.
Roadmap
[ ] Multi-frame UDS: Flow-control implementation to read vehicle VINs and long ECU names.
[ ] Custom VAG Dashboards: Targeted integration for tracking high-voltage 48v systems, DSG oil temperatures, coasting/cylinder-deactivation (ACT) states, and real-time energy flow.
[ ] Native GVRET Protocol: Support direct network connections to SavvyCAN without needing intermediary Python scripts.
[ ] Code Refactoring: Split monolith code into clean modules (CAN, Storage, Web, UDS, PIDs).