# OpenOBD

An open-source, local-first CAN bus data logger designed for the **VW Golf 8.5 eTSI**. No subscriptions, no cloud dependencies. 

It actively polls standard OBD2 parameters and simultaneously captures raw CAN frames. 
*(Fork of [roypeter/esp32-obd2-logger](https://github.com/roypeter/esp32-obd2-logger))*

> **Current Status (v2):** Tested and running on-vehicle. Supports 29-bit OBD (~28 PIDs), raw logging, compile-time configuration via `include/config.h`, Wi-Fi integration with NTP time sync, a clean web dashboard for mobile browsers, safe UDS scanning, and wireless developer streaming.

---

## Features

### What the Logger Does
* **Clean Data Logging:** Actively polls standard OBD2 PIDs and saves decoded, human-readable values to `s{Session}.csv`.
* **Raw CAN Capturing:** Records every sent and received CAN frame in the SavvyCAN-compatible format (`raw_s{Session}.csv`). Uses 64-bit timestamps to prevent overflows during long road trips.
* **Live CAN Explorer (`/explore`):** View every active CAN ID, its transmission frequency (Hz), and live data bytes directly in your mobile browser.
* **Auto-Record:** Starts logging automatically as soon as the ignition turns on. Use the web dashboard to pause, resume, or start a new recording session.
* **Robust File Handling:** Flushes data to the SD card immediately after every write to prevent data loss when the car is turned off. Automatically falls back to internal flash memory if the SD card is missing.
* **PID Discovery (`/pids`):** Scans your car to find exactly which OBD2 PIDs are supported, optimization-polling only those. 
* **Mobile Cockpit Dashboard:** A dark-themed, mobile-friendly interface featuring gauges, historical charts, and real-time calculated metrics (e.g., instant & average fuel consumption, horsepower estimation, boost pressure).
* **Gentle UDS Scanning (`/uds`):** Requests basic ECU identity information (Service 0x22) safely, without modifying vehicle states or forcing diagnostic sessions.
* **Wireless Streaming:** In Developer Mode, the logger streams raw CAN frames via TCP or MQTT, letting you analyze bus traffic live in SavvyCAN or custom Python tools without pulling the SD card.

---

## Hardware Requirements

### Shopping List
* **LilyGO T-CAN485** (ESP32 development board with an integrated CAN transceiver and microSD slot)
* **MicroSD Card** (32 GB or less, formatted as **FAT32**—the ESP32 SD library does not support exFAT)
* **OBD2 to Open-Ended Cable** (16-pin breakout cable)
* **12V-to-5V USB-C Step-Down Converter** (For permanent installation; isolates and filters noisy vehicle power)
* **Wago 221 Connectors** (For solder-free wiring)

---

## Wiring Diagram

### 1. Connecting the OBD2 Breakout to the T-CAN485 Screw Terminals:

| OBD2 Pin | Wire Color (Typical) | Function | T-CAN485 Terminal |
| :--- | :--- | :--- | :--- |
| **Pin 6** | Green | CAN-High | **CANH** |
| **Pin 14** | Green/White | CAN-Low | **CANL** |
| **Pin 5** | Light Blue | Signal Ground | **GND** |

### 2. Ignition-Switched Power (Permanent Install):
To prevent draining the car's 12V battery while parked, we do **not** power the board from OBD2 Pin 16 (Permanent 12V). Instead, we tap into the ignition wire:

* **OBD2 Pin 1 (Ignition / usually Brown):** Connect through a 2A fuse to the Red wire (+) of the 12V-to-5V step-down converter.
* **OBD2 Pin 4 (Chassis Ground / usually Orange):** Connect to the Black wire (−) of the step-down converter.
* **Converter USB-C Output:** Plugs directly into the USB-C port of the T-CAN485.

> ⚠️ **Safety Rules:**
> 1. Do not swap CAN-High (Green) and CAN-Low (Green/White), or communication will fail.
> 2. Never connect the car's raw 12V–14.7V electrical system directly to the ESP32 pins. Always use the 5V converter.
> 3. Separately insulate all unused wires from the breakout cable with electrical tape to avoid accidental short circuits.

---

## Building & Flashing

The project is built using **PlatformIO** (available as a VS Code extension or via CLI).

### 1. Edit the Configuration
Open `include/config.h` and configure your Wi-Fi credentials and preferences. Choose your build target:

* **Normal Mode (`t-can485`):** Standard web dashboard, optimized logging on the SD card.
* **Developer Mode (`t-can485-dev`):** Disables the heavy dashboard UI to free up system resources, enabling full raw CAN captures and network streaming.

### 2. Upload the Code
Connect the T-CAN485 to your computer via USB-C and run the appropriate terminal command:

```bash
# Flash Normal Mode and open the serial monitor:
pio run -e t-can485 --target upload --target monitor

# Flash Developer Mode and open the serial monitor:
pio run -e t-can485-dev --target upload --target monitor