# ESP32 Jikong (JK) BMS Monitor & Web Server (NimBLE-Arduino Optimized Edition)

A high-performance ESP32 firmware for wirelessly reading live telemetry, cell voltages, temperatures, and controlling **Jikong (JK) Active Balance BMS** units over **Bluetooth Low Energy (BLE)** with an embedded **AJAX/JSON Web Server** on port `8089`.

> **Note on Localization:**  
> This edition is fully localized into **English** across all code comments, serial terminal logs, web UI menus, and HTML tables (translated from the original Ukrainian project at [vernonet/ESP32_PRJ/esp32_jk_bms](https://github.com/vernonet/ESP32_PRJ/tree/main/esp32_jk_bms) / [taraskinua](https://github.com/taraskinua/JK-BMS-web-server-on-esp-32)).

---

## ⚡️ Key Features

- **Wireless BLE Client**: ESP32 acts as a BLE Client connecting directly to Jikong BMS via UUID `0000ffe0-0000-1000-8000-00805f9b34fb`. No physical wires to the BMS needed!
- **Low-Level CCCD (0x2902) ESP-IDF Workaround**: Directly invokes `esp_ble_gattc_write_char_descr()` with handle `char_handle + 1`, bypassing Jikong's descriptor discovery timeout bug.
- **Lightweight AJAX / JSON Web Server (Port 8089)**: Fast dynamic browser updates without full page reloads, saving precious ESP32 heap RAM.
- **Live Battery Telemetry**:
  - Total Battery Voltage (V), Current (A), and Power (W)
  - State of Charge (SOC %)
  - Individual Cell Voltages (4S to 32S) with millivolt resolution
  - Individual Cell Wire Resistances (Ω)
  - Cell Delta (V) with active balancing indicator
  - Temperatures: Sensor T1, Sensor T2, and Power MOSFETs
- **Interactive Control Switches**:
  - Charge MOSFET Toggle (ON / OFF)
  - Discharge MOSFET Toggle (ON / OFF)
  - Active Balancing Toggle (ON / OFF) with live balancing current (A)
- **Settings Editor via Web Interface**:
  - Undervoltage Protection (UVP) & Recovery
  - Overvoltage Protection (OVP) & Recovery
  - Full Charge & Float Charge Voltages
  - Balance Trigger & Balance Precision Thresholds
  - Max Charge, Discharge & Balance Current Limits
  - Configurable series cell count & total battery capacity
- **BLE Scanner (`/scan`)**: Scan and connect to any nearby Jikong BMS device over the air.
- **OTA Firmware Updates (`/update`)**: Flash new `.bin` firmware binaries directly over WiFi without USB cable. Automatically disconnects BLE prior to flashing to free heap memory.
- **Zero Third-Party Library Dependencies**: Compiles cleanly using only the standard Espressif ESP32 Arduino Core!

---

## 🛠 Hardware Requirements

1. **ESP32 Development Board**:
   - WEMOS D1 Mini32, ESP32 DevKit V1, NodeMCU-32S, or ESP32-WROOM-32.
2. **Jikong (JK) Smart Active Balance BMS**:
   - Any BLE-enabled Jikong BMS (e.g., `JK-B2A24S20P`, `JK-BD6A20S10P`, `JK-B1A8S20P`, `JK-B2A16S20P`, etc.).
3. **Power Supply**:
   - 5V USB power supply for the ESP32.
4. **Status LED**:
   - Built-in LED on `GPIO 2` (blinks while connecting to WiFi; stays solid ON once connected).


---
## 🚀 Quick Start & Installation

### Option 1: PlatformIO (VS Code)  - tested

1. Put the source code into `src/esp32_JK_BMS.cpp`.
2. Use the provided `platformio.ini`:
   ```ini
   [env:wemos_d1_mini32]
   platform = espressif32 @ 6.4.0
   board = wemos_d1_mini32
   framework = arduino
   monitor_speed = 115200
   monitor_filters = time, esp32_exception_decoder
  
   ; Library Dependencies:
   ; Using high-efficiency Apache NimBLE stack instead of heavy Bluedroid
   lib_deps =
      h2zero/NimBLE-Arduino @ ^1.4.2
  
   ; Build Optimization Flags:
   ; 1. Disable unused peripheral/server roles to save Flash & Heap RAM
   ; 2. Enable aggressive compiler size optimization (-Os)
   build_flags =
      -Os
      -D CONFIG_BT_NIMBLE_ROLE_PERIPHERAL_DISABLED
      -D CONFIG_BT_NIMBLE_ROLE_BROADCASTER_DISABLED
      -D CONFIG_BT_NIMBLE_MAX_CONNECTIONS=1
      -D CONFIG_BT_NIMBLE_ATT_PREFERRED_MTU=256
  
   ; Partitions:
   ; min_spiffs gives 1.9MB for application binary, allowing comfortable dual-OTA partitions!
   board_build.partitions = min_spiffs.csv
   ```
3. Run:
   ```bash
   pio run -t upload
   pio device monitor -b 115200
   ```
---


### Option 2: Arduino IDE (v1.8.x or v2.x) - not tested

1. **Install ESP32 Core**:
   - Open Arduino IDE -> *File > Preferences*.
   - In *Additional Board Manager URLs*, add:
     ```
     https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
     ```
   - Go to *Tools > Board > Boards Manager*, search for **esp32** by Espressif Systems and click **Install**.

2. **Prepare the Sketch**:
   - Create a folder named `esp32_jk_bms`.
   - Place `esp32_jk_bms.ino` inside that folder:
     ```text
     esp32_jk_bms/
       └── esp32_jk_bms.ino
     ```

3. **Configure WiFi Credentials**:
   - Open `esp32_jk_bms.ino` and edit lines 22-26:
     ```cpp
     const char *ssid = "Your_WiFi_SSID";
     const char *password = "Your_WiFi_Password";

     const char *www_username = "admin";
     const char *www_password = "admin_password";
     ```

4. **⚠️ CRITICAL: Set Partition Scheme**:
   - Go to: **Tools > Board > "ESP32 Dev Module"** (or "WEMOS D1 MINI ESP32").
   - Go to: **Tools > Partition Scheme > "Minimal SPIFFS (1.9MB APP with OTA/190KB SPIFFS)"** or **"Huge APP (3MB No OTA/1MB SPIFFS)"**.
   > *Note:* If you keep the default 1.2MB partition scheme, the compiler will fail with `sketch too big` because BLE + WiFi + OTA require ~1.35MB flash space.

5. **Upload**:
   - Connect ESP32 via USB and click **Upload** (`Ctrl + U`).
   - Open Serial Monitor at **115200 baud** to see the assigned IP address.



---

## 🌐 Web Server Endpoints

Once connected to your local WiFi, open `http://<ESP32_IP>:8089` in your web browser:

| Route | Method | Description |
| :--- | :--- | :--- |
| `/` | `GET` | Main live dashboard (telemetry, cell voltages, switches) |
| `/data.json` | `GET` | JSON API returning real-time voltages, current, SOC, and cell array |
| `/settings` | `GET` | Chunked HTML form to view & edit BMS parameters |
| `/settings_update`| `POST`| Saves changed parameters into BMS non-volatile registers |
| `/info` | `GET` | Device information (Hardware/Software version, Serial No., Build time) |
| `/scan` | `GET` | Discovers nearby Jikong BLE devices and displays connect buttons |
| `/connect?mac=..`| `GET` | Initiates BLE GATT connection to specific BMS MAC address |
| `/disconnect` | `GET` | Disconnects the BLE Client connection |
| `/charge_on` | `GET` | Enables charging MOSFET switch (Register `0x1D = 1`) |
| `/charge_off`| `GET` | Disables charging MOSFET switch (Register `0x1D = 0`) |
| `/discharge_on`| `GET`| Enables discharging MOSFET switch (Register `0x1E = 1`) |
| `/discharge_off`| `GET`| Disables discharging MOSFET switch (Register `0x1E = 0`) |
| `/balance_on`| `GET` | Enables active balancing (Register `0x1F = 1`) |
| `/balance_off`| `GET` | Disables active balancing (Register `0x1F = 0`) |
| `/update` | `GET/POST`| Web-based OTA firmware upload (.bin) |
| `/reboot` | `GET` | Reboots the ESP32 microcontroller |

---

## 🔬 Jikong BLE Protocol & Register Reference

### Packet Framing
- **Inbound frames (BMS -> ESP32)**:
  - Header: `0x55 0xAA 0xEB 0x90` (4 bytes)
  - Frame Type: `0x01` (Settings), `0x02` (Telemetry ~300 bytes), `0x03` (Device Info)
  - Checksum: 8-bit additive CRC (`sum(data) % 256`)
- **Outbound command frames (ESP32 -> BMS)**:
  - Header: `0xAA 0x55 0x90 0xEB <address> <length> <value_little_endian> ... <crc>`

### Key Command Registers

| Register | Name | Operation | Description |
| :---: | :--- | :---: | :--- |
| `0x95` | Telemetry Poll | Read | Polled every 2000 ms to retrieve live cell and pack data |
| `0x96` | Settings Poll | Read | Polled every 4000 ms until settings frame is received |
| `0x97` | Device Info Poll | Read | Polled every 4000 ms until device info frame is received |
| `0x1D` | Charge Switch | Write | `1` = Charge ON, `0` = Charge OFF |
| `0x1E` | Discharge Switch | Write | `1` = Discharge ON, `0` = Discharge OFF |
| `0x1F` | Balance Switch | Write | `1` = Active Balance ON, `0` = Active Balance OFF |
| `0x0A` | UVP Threshold | Write | Cell undervoltage protection in mV (Value * 1000) |
| `0x0C` | OVP Threshold | Write | Cell overvoltage protection in mV (Value * 1000) |
| `0x27` | Cell Count | Write | Configured series cells connected (e.g. 8, 16, 24) |
| `0x41` | Total Capacity | Write | Nominal battery capacity in mAh (Value * 1000) |

---

## 🧩 The Low-Level CCCD (0x2902) ESP-IDF Fix

Many Arduino sketches fail to receive notifications from Jikong BMS because the standard `pNotifyCharacteristic->registerForNotify()` triggers an internal descriptor search for UUID `0x2902`. Jikong's firmware frequently fails to respond to descriptor queries, resulting in GATT discovery timeouts.

This sketch resolves the issue by writing directly to the descriptor handle via the ESP-IDF driver:
```cpp
// Register callback without scanning descriptors (passing false)
pNotifyCharacteristic->registerForNotify(notifyCallback, false);

// Direct low-level kernel write to characteristic handle + 1 (CCCD on Jikong boards)
uint16_t notify_en_value = 0x0001; 
esp_ble_gattc_write_char_descr(
    pClient->getGattcIf(),
    pClient->getConnId(),
    pNotifyCharacteristic->getHandle() + 1,
    sizeof(notify_en_value),
    (uint8_t *)&notify_en_value,
    ESP_GATT_WRITE_TYPE_RSP,
    ESP_GATT_AUTH_REQ_NONE
);
```
---

# JK-BMS Data Reader

## 🚀 Interactive Web Server (Demo)
To demonstrate how the web interface works without connecting any real hardware, you can launch a live simulator directly in your browser:
👉 [Run Live ESP32 JK-BMS Simulator](https://vernonet.github.io/jk-bms-web-simulator/)

---

## 📜 Credits & License

- Original project and concept by **[taraskinua](https://github.com/taraskinua/JK-BMS-web-server-on-esp-32)**.
- Enhanced and maintained by **[vernonet](https://github.com/vernonet/ESP32_PRJ/tree/main/esp32_jk_bms)**.
- English localization, interactive web studio, and documentation created with Google AI Studio Build.
- Licensed under the **Apache-2.0 License**.
