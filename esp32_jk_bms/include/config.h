/*
 * ============================================================================
 * include/config.h - ESP32 JK-BMS NimBLE User Configuration Header
 * ============================================================================
 */

#ifndef CONFIG_H
#define CONFIG_H



// --- Wi-Fi Settings ---
#define WIFI_SSID           "your_ssid"
#define WIFI_PASSWORD       "your_pass"
#define WIFI_POWER_SAVE     true

// --- Web Server Security ---
#define WWW_USERNAME        "your_admin"
#define WWW_PASSWORD        "your_pass"
#define WEB_SERVER_PORT     8089

// --- Target Jikong (JK) BMS ---
// Set exact MAC address (e.g., "C8:47:8C:12:34:56") or leave empty "" to auto-scan
#define TARGET_BMS_MAC      ""
//#define AUTO_CONNECT_FIRST  true

#define JK_SERVICE_UUID         "0000ffe0-0000-1000-8000-00805f9b34fb"
#define JK_CHARACTERISTIC_UUID  "0000ffe1-0000-1000-8000-00805f9b34fb"
#define MIN_VOLTAGE                   10.0f
#define MAX_VOLTAGE                   100.0f
#define MIN_CURRENT                   1.0f

// Frame Header signatures
#define JK_REQ_HEADER_BYTE0     0xAA
#define JK_REQ_HEADER_BYTE1     0x55
#define JK_REQ_HEADER_BYTE2     0x90
#define JK_REQ_HEADER_BYTE3     0xEB

#define JK_RESP_HEADER_BYTE0    0x55
#define JK_RESP_HEADER_BYTE1    0xAA
#define JK_RESP_HEADER_BYTE2    0xEB
#define JK_RESP_HEADER_BYTE3    0x90

// --- BLE Power & Connection Timing ---
// Lower connection interval = faster updates, higher power
// Higher connection interval = slower updates, ultra-low power
#define BLE_MIN_CONN_INTERVAL_MS   40
#define BLE_MAX_CONN_INTERVAL_MS   80
#define BLE_SLAVE_LATENCY          1
#define BLE_SUPERVISION_TIMEOUT_MS 4000

// --- Polling & Watchdog Intervals ---
#define TELEMETRY_POLL_INTERVAL_MS 1500
#define RECONNECT_RETRY_DELAY_MS   5000
#define BMS_WATCHDOG_TIMEOUT_MS    45000

// --- Hardware Pins ---
#define LED_PIN             2

#endif // CONFIG_H
