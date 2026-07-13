#pragma once
// ============================================================================
//  OpenOBD — Konfiguration (wird beim FLASHEN fest eingebaut, NICHT auf SD)
// ----------------------------------------------------------------------------
//  Hier alles einstellen, dann flashen. Keine config.json auf der Karte.
//
//  Zwei fertige Flash-Ziele (siehe platformio.ini):
//    Normal:  python3 -m platformio run -e t-can485      --target upload --target monitor
//    Dev:     python3 -m platformio run -e t-can485-dev  --target upload --target monitor
//  Das Dev-Ziel setzt CFG_DEV_MODE=1 automatisch — du musst dafuer nichts ändern.
// ============================================================================

// --- Betriebsmodus -----------------------------------------------------------
// true  = Dev  : KEIN Dashboard, volles Roh-Log, Dev-Streaming an (Datensammeln)
// false = Normal: Cockpit-Dashboard + schlankes Logging (nur dekodiert)
#ifndef CFG_DEV_MODE
#define CFG_DEV_MODE      false
#endif

// Aus dem Modus abgeleitet — bei Bedarf hier fest überschreiben:
#define CFG_WEB_UI        (!CFG_DEV_MODE)   // Dashboard nur im Normal-Modus
#define CFG_LOG_RAW       (CFG_DEV_MODE)    // Roh-Log (raw_*.csv) v.a. im Dev-Modus
#define CFG_STREAM_TCP    (CFG_DEV_MODE)    // TCP-Roh-Stream v.a. im Dev-Modus

// --- Logging -----------------------------------------------------------------
#define CFG_LOG_DECODED     true            // saubere dekodierte CSV schreiben
#define CFG_LOG_INTERVAL_MS 250             // Schreibrate dekodiert (ms)
#define CFG_AUTO_RECORD     true            // beim Boot sofort aufzeichnen
#define CFG_DISCOVERY       true            // beim Boot PID-Discovery fahren
#define CFG_LED             false            // Status-LED (Platzhalter)

// --- WLAN --------------------------------------------------------------------
// "ap"   = eigener Hotspot (Handy verbindet sich damit, kein Internet am Handy)
// "join" = ins Auto-WLAN einwählen (Handy behält Internet, Zeit per NTP, openobd.local)
#define CFG_WIFI_MODE     "ap"
#define CFG_WIFI_SSID     ""                // nur bei "join": SSID des Auto-Hotspots
#define CFG_WIFI_PASS     ""                // nur bei "join": Passwort
#define CFG_FALLBACK_AP   true             // "join" gescheitert -> eigenen AP aufmachen
#define CFG_AP_SSID       "OpenOBD"
#define CFG_AP_PASS       "openobd1234"     // mind. 8 Zeichen

// --- Zeit --------------------------------------------------------------------
#define CFG_NTP           "pool.ntp.org"
#define CFG_TZ_MIN        60               // lokaler Offset in Minuten (CET=60, CEST=120)

// --- Dev-Streaming (nur relevant im Dev-Modus) -------------------------------
#define CFG_TCP_PORT      3333             // Roh-Frames als SavvyCAN-Zeilen (nc/Python)
#define CFG_MQTT          false            // Roh-Frames zusätzlich per MQTT publizieren
#define CFG_MQTT_HOST     ""               // Broker-IP/Host
#define CFG_MQTT_PORT     1883
#define CFG_MQTT_TOPIC    "openobd/can"
