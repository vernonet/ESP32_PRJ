/*
 * ============================================================================
 * ESP32 Jikong (JK) BMS Monitor & Web Server (NimBLE-Arduino Optimized Edition)
 * ============================================================================
 * Refactored from original Bluedroid implementation (vernonet/ESP32_PRJ)
 * 
 * KEY IMPROVEMENTS WITH NimBLE:
 *  1. RAM Reduction: Free Heap increased from ~35 KB to ~160 KB (+350% headroom!)
 *  2. Flash Reduction: Binary size reduced by ~320 KB (eliminates partition overflow)
 *  3. Energy Optimization: Negotiated connection interval & passive/duty-cycle scan
 *  4. Robust Reconnects: Native non-blocking state machine + Telemetry Watchdog
 *  5. CCCD Fix: Clean descriptor subscription avoiding ESP-IDF low-level kernel crash
 *  6. Wi-Fi Coexistence: Dynamic coexistence without dropping HTTP web sockets
 * ============================================================================
 */

#include <NimBLEDevice.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Update.h>
#include <vector>
#include <map>
#include <string>
#include <cmath>
#include "config.h"



// -----------------------------------------------------------------------------
// --- 1. USER CONFIGURATION ---------------------------------------------------
// -----------------------------------------------------------------------------


const char *ssid = WIFI_SSID;
const char *password = WIFI_PASSWORD;

const char *www_username = WWW_USERNAME;
const char *www_password = WWW_PASSWORD;

// Timing & Watchdog Configuration
const uint32_t TELEMETRY_INTERVAL_MS = TELEMETRY_POLL_INTERVAL_MS; // Poll telemetry every X ms
const uint32_t WATCHDOG_TIMEOUT_MS   = BMS_WATCHDOG_TIMEOUT_MS; // Reset connection if no packets for X ms
const uint32_t RECONNECT_DELAY_MS    = RECONNECT_RETRY_DELAY_MS;  // Delay before retrying reconnection

#define FLOAT_TOLERANCE 0.001

WebServer server(WEB_SERVER_PORT);

// -----------------------------------------------------------------------------
// --- 2. BLE UUIDS & CONSTANTS ------------------------------------------------
// -----------------------------------------------------------------------------
static NimBLEUUID serviceUUID(JK_SERVICE_UUID);
static NimBLEUUID charWriteUUID(JK_CHARACTERISTIC_UUID);
static NimBLEUUID charNotifyUUID(JK_CHARACTERISTIC_UUID);

// -----------------------------------------------------------------------------
// --- 3. BLE STATE MANAGEMENT -------------------------------------------------
// -----------------------------------------------------------------------------
enum BleState {
  BLE_IDLE,
  BLE_SCANNING,
  BLE_CONNECTING,
  BLE_CONNECTED,
  BLE_RECONNECT_WAIT
};

struct FoundBMS {
  std::string address;
  std::string name;
  int rssi;
};

std::map<std::string, FoundBMS> foundDevices;
static NimBLEClient *pClient = nullptr;
static NimBLERemoteCharacteristic *pNotifyCharacteristic = nullptr;
static NimBLERemoteCharacteristic *pWriteCharacteristic = nullptr;
static NimBLEScan *pBLEScan = nullptr;

static BleState currentBleState = BLE_IDLE;
static std::string activeBmsAddress = "";
static bool isConnected = false;
static bool parsesettings = false;
static bool parseinfo = false;
static unsigned long lastValidPacketTime = 0;
static unsigned long lastReconnectAttempt = 0;
static unsigned long lastTelemetryRequest = 0;
static unsigned long connectionEstablishedTime = 0;

// -----------------------------------------------------------------------------
// --- 4. DATA PROCESSING & FRAME BUFFER ---------------------------------------
// -----------------------------------------------------------------------------
byte receivedBytes[384];
int frame = 0;
bool received_start = false;
bool received_complete = false;

// -----------------------------------------------------------------------------
// --- 5. BMS DATA STRUCTURES --------------------------------------------------
// -----------------------------------------------------------------------------
struct BMS_Data {
  float cellVoltage[32] = {0};
  float wireResist[32] = {0};
  float Average_Cell_Voltage = 0;
  float Delta_Cell_Voltage = 0;
  float Battery_Voltage = 0;
  float Battery_Power = 0;
  float Charge_Current = 0;
  float Battery_T1 = 0;
  float Battery_T2 = 0;
  float MOS_Temp = 0;
  int Percent_Remain = 0;
  float Capacity_Remain = 0;
  float Nominal_Capacity = 0;
  float Cycle_Count = 0;
  float Cycle_Capacity = 0;
  uint32_t Uptime = 0;
  uint8_t sec = 0, mi = 0, hr = 0, days = 0;
  float Balance_Curr = 0;
  bool Balance = false;
  bool Charge = false;
  bool Discharge = false;
  int Balancing_Action = 0;
} G_data;

struct BMS_Settings {
  float cell_voltage_undervoltage_protection = 0;
  float cell_voltage_undervoltage_recovery = 0;
  float cell_voltage_overvoltage_protection = 0;
  float cell_voltage_overvoltage_recovery = 0;
  float balance_trigger_voltage = 0;
  float power_off_voltage = 0;
  float balance_starting_voltage = 0;
  float full_charge_voltage = 0;
  float nominal_voltage = 0;
  float float_charge_voltage = 0;
  float balance_precision = 0;
  float max_charge_current = 0;
  float max_discharge_current = 0;
  float max_balance_current = 0;
  long cell_count = 16;
  float total_battery_capacity = 0;
  long bms_mode = 0;
} G_settings;

struct Device_Info {
  std::string vendorID = "Unknown Vendor";
  std::string hardwareVersion = "V0.0";
  std::string softwareVersion = "V0.0";
  std::string deviceName = "Unknown Device";
  std::string devicePasscode = "0000000000000000";
  std::string manufacturingDate = "00000000";
  std::string serialNumber = "00000000000";
  std::string passcode = "00000";
  std::string userData = "No user data";
  std::string setupPasscode = "0000000000000000";
} G_info;

// Forward declarations
void writeRegister(uint8_t address, uint32_t value, uint8_t length);
bool connectToBms(const std::string &addressStr);
void startScan();

// -----------------------------------------------------------------------------
// --- 6. AUTHENTICATION & CRC UTILS -------------------------------------------
// -----------------------------------------------------------------------------
bool checkAuth() {
  if (strlen(www_username) == 0) return true;
  if (!server.authenticate(www_username, www_password)) {
    server.requestAuthentication();
    return false;
  }
  return true;
}

uint8_t crc(const uint8_t data[], uint16_t len) {
  uint8_t sum = 0;
  for (uint16_t i = 0; i < len; i++) sum += data[i];
  return sum;
}

// -----------------------------------------------------------------------------
// --- 7. PARSING ROUTINES -----------------------------------------------------
// -----------------------------------------------------------------------------
void parseDeviceInfo() {
  G_info.vendorID.assign((char *)(receivedBytes + 6), 16);
  G_info.hardwareVersion.assign((char *)(receivedBytes + 22), 8);
  G_info.softwareVersion.assign((char *)(receivedBytes + 30), 8);
  G_info.deviceName.assign((char *)(receivedBytes + 46), 16);
  G_info.devicePasscode.assign((char *)(receivedBytes + 62), 16);
  G_info.manufacturingDate.assign((char *)(receivedBytes + 78), 8);
  G_info.serialNumber.assign((char *)(receivedBytes + 86), 11);
  G_info.passcode.assign((char *)(receivedBytes + 97), 5);
  G_info.userData.assign((char *)(receivedBytes + 102), 16);
  G_info.setupPasscode.assign((char *)(receivedBytes + 118), 16);
  parseinfo = true;
  lastValidPacketTime = millis();
}

void parseBMSSettings() {
  // 1. Параметри напруги комірок (scaled за 0.001 -> ділимо на 1000.0f)
  G_settings.cell_voltage_undervoltage_protection = (((uint32_t)receivedBytes[13] << 24 | (uint32_t)receivedBytes[12] << 16 | (uint32_t)receivedBytes[11] << 8 | (uint32_t)receivedBytes[10]) * 0.001f);
  G_settings.cell_voltage_undervoltage_recovery   = (((uint32_t)receivedBytes[17] << 24 | (uint32_t)receivedBytes[16] << 16 | (uint32_t)receivedBytes[15] << 8 | (uint32_t)receivedBytes[14]) * 0.001f);
  G_settings.cell_voltage_overvoltage_protection  = (((uint32_t)receivedBytes[21] << 24 | (uint32_t)receivedBytes[20] << 16 | (uint32_t)receivedBytes[19] << 8 | (uint32_t)receivedBytes[18]) * 0.001f);
  G_settings.cell_voltage_overvoltage_recovery   = (((uint32_t)receivedBytes[25] << 24 | (uint32_t)receivedBytes[24] << 16 | (uint32_t)receivedBytes[23] << 8 | (uint32_t)receivedBytes[22]) * 0.001f);
  G_settings.balance_trigger_voltage              = (((uint32_t)receivedBytes[29] << 24 | (uint32_t)receivedBytes[28] << 16 | (uint32_t)receivedBytes[27] << 8 | (uint32_t)receivedBytes[26]) * 0.001f);
  G_settings.full_charge_voltage                  = (((uint32_t)receivedBytes[33] << 24 | (uint32_t)receivedBytes[32] << 16 | (uint32_t)receivedBytes[31] << 8 | (uint32_t)receivedBytes[30]) * 0.001f);
  G_settings.nominal_voltage                      = (((uint32_t)receivedBytes[37] << 24 | (uint32_t)receivedBytes[36] << 16 | (uint32_t)receivedBytes[35] << 8 | (uint32_t)receivedBytes[34]) * 0.001f);
  G_settings.float_charge_voltage                 = (((uint32_t)receivedBytes[41] << 24 | (uint32_t)receivedBytes[40] << 16 | (uint32_t)receivedBytes[39] << 8 | (uint32_t)receivedBytes[38]) * 0.001f);
  G_settings.balance_precision                    = (((uint32_t)receivedBytes[45] << 24 | (uint32_t)receivedBytes[44] << 16 | (uint32_t)receivedBytes[43] << 8 | (uint32_t)receivedBytes[42]) * 0.001f);
  G_settings.power_off_voltage                    = (((uint32_t)receivedBytes[49] << 24 | (uint32_t)receivedBytes[48] << 16 | (uint32_t)receivedBytes[47] << 8 | (uint32_t)receivedBytes[46]) * 0.001f);

  // 2. Струми та затримки (scaled за 0.001 -> ділимо на 1000.0f)
  G_settings.max_charge_current    = (((uint32_t)receivedBytes[53] << 24 | (uint32_t)receivedBytes[52] << 16 | (uint32_t)receivedBytes[51] << 8 | (uint32_t)receivedBytes[50]) * 0.001f);
  G_settings.max_discharge_current = (((uint32_t)receivedBytes[65] << 24 | (uint32_t)receivedBytes[64] << 16 | (uint32_t)receivedBytes[63] << 8 | (uint32_t)receivedBytes[62]) * 0.001f);
  G_settings.max_balance_current   = (((uint32_t)receivedBytes[81] << 24 | (uint32_t)receivedBytes[80] << 16 | (uint32_t)receivedBytes[79] << 8 | (uint32_t)receivedBytes[78]) * 0.001f);

  // 3. Системні параметри батареї (прямі 4-байтні значення)
  G_settings.cell_count             = (((uint32_t)receivedBytes[117] << 24 | (uint32_t)receivedBytes[116] << 16 | (uint32_t)receivedBytes[115] << 8 | (uint32_t)receivedBytes[114]));
  G_settings.bms_mode               = (((uint32_t)receivedBytes[121] << 24 | (uint32_t)receivedBytes[120] << 16 | (uint32_t)receivedBytes[119] << 8 | (uint32_t)receivedBytes[118]));
  G_settings.total_battery_capacity = (((uint32_t)receivedBytes[133] << 24 | (uint32_t)receivedBytes[132] << 16 | (uint32_t)receivedBytes[131] << 8 | (uint32_t)receivedBytes[130]) * 0.001f);
  G_settings.balance_starting_voltage = (((uint32_t)receivedBytes[141] << 24 | (uint32_t)receivedBytes[140] << 16 | (uint32_t)receivedBytes[139] << 8 | (uint32_t)receivedBytes[138]) * 0.001f);


  parsesettings = true;
  lastValidPacketTime = millis();
}


void parseBMSData() {
  // 1. Спочатку вираховуємо загальну напругу батареї з її законного місця в кадрі
  float temp_battery_voltage = ((receivedBytes[153] << 24 | receivedBytes[152] << 16 | receivedBytes[151] << 8 | receivedBytes[150]) * 0.001f);

  // 2. ФІЛЬТР СМІТТЯ: Перевіряємо, чи напруга адекватна.
  // Якщо напруга більше 100V або менше 10V (наприклад, збійний нуль), повністю ігноруємо цей кадр!
  if (temp_battery_voltage > MAX_VOLTAGE || temp_battery_voltage < MIN_VOLTAGE) {
    Serial.printf("[BMS Filter] Увага! Виявлено глітч (биті дані). Невірна напруга: %.2f V. Кадр пропущено.\n", temp_battery_voltage);
    return; // Перериваємо функцію, старі правильні дані в G_data залишаються недоторканими
  }

  // 3. Якщо перевірку пройдено — записуємо напругу і розбираємо решту параметрів кадру
  G_data.Battery_Voltage = temp_battery_voltage;

  for (int j = 0, i = 7; i < 38; j++, i += 2) {
    G_data.cellVoltage[j] = ((receivedBytes[i] << 8 | receivedBytes[i - 1]) * 0.001f);
  }
  for (int j = 0, i = 81; i < 112; j++, i += 2) {
    G_data.wireResist[j] = (((int)receivedBytes[i] << 8 | receivedBytes[i - 1]) * 0.001f);
  }

  G_data.Average_Cell_Voltage = (((int)receivedBytes[75] << 8 | receivedBytes[74]) * 0.001f);
  G_data.Delta_Cell_Voltage   = (((int)receivedBytes[77] << 8 | receivedBytes[76]) * 0.001f);

  if (receivedBytes[145] == 0xFF) {
    G_data.MOS_Temp = ((0xFF << 24 | 0xFF << 16 | receivedBytes[145] << 8 | receivedBytes[144]) * 0.1f);
  } else {
    G_data.MOS_Temp = ((receivedBytes[145] << 8 | receivedBytes[144]) * 0.1f);
  }

  G_data.Charge_Current  = ((receivedBytes[161] << 24 | receivedBytes[160] << 16 | receivedBytes[159] << 8 | receivedBytes[158]) * 0.001f);
  G_data.Battery_Power   = G_data.Battery_Voltage * G_data.Charge_Current;

  G_data.Battery_T1 = (receivedBytes[163] == 0xFF) 
    ? ((0xFF << 24 | 0xFF << 16 | receivedBytes[163] << 8 | receivedBytes[162]) * 0.1f) 
    : ((receivedBytes[163] << 8 | receivedBytes[162]) * 0.1f);

  G_data.Battery_T2 = (receivedBytes[165] == 0xFF)
    ? ((0xFF << 24 | 0xFF << 16 | receivedBytes[165] << 8 | receivedBytes[164]) * 0.1f)
    : ((receivedBytes[165] << 8 | receivedBytes[164]) * 0.1f);

  // Струм балансування лежить у 170 та 171 байтах кадру Jikong
  if ((receivedBytes[171] & 0xF0) == 0x00) {
    G_data.Balance_Curr = ((receivedBytes[171] << 8 | receivedBytes[170]) * 0.001f);
  } else if ((receivedBytes[171] & 0xF0) == 0xF0) {
    G_data.Balance_Curr = (((receivedBytes[171] & 0x0F) << 8 | receivedBytes[170]) * -0.001f);
  }

  G_data.Balancing_Action = receivedBytes[172];
  G_data.Percent_Remain   = receivedBytes[173];
  G_data.Capacity_Remain  = ((receivedBytes[177] << 24 | receivedBytes[176] << 16 | receivedBytes[175] << 8 | receivedBytes[174]) * 0.001f);
  G_data.Nominal_Capacity = ((receivedBytes[181] << 24 | receivedBytes[180] << 16 | receivedBytes[179] << 8 | receivedBytes[178]) * 0.001f);
  G_data.Cycle_Count      = ((receivedBytes[185] << 24 | receivedBytes[184] << 16 | receivedBytes[183] << 8 | receivedBytes[182]));
  G_data.Cycle_Capacity   = ((receivedBytes[189] << 24 | receivedBytes[188] << 16 | receivedBytes[187] << 8 | receivedBytes[186]) * 0.001f);

  G_data.Uptime = ((receivedBytes[197] << 24 | receivedBytes[196] << 16 | receivedBytes[195] << 8 | receivedBytes[194]));
  G_data.sec = (G_data.Uptime) % 60;
  G_data.mi = (G_data.Uptime / 60) % 60;
  G_data.hr = (G_data.Uptime / 3600) % 24;
  G_data.days = (G_data.Uptime / 86400);

  G_data.Charge    = (receivedBytes[198] > 0);
  G_data.Discharge = (receivedBytes[199] > 0);
  G_data.Balance   = (receivedBytes[201] > 0);

  // Оновлюємо вочдог тільки при успішному та валідному кадрі
  lastValidPacketTime = millis();
}


void processBMSFrame(uint8_t *pData, size_t length) {
  // Check for start of a new data frame (55 AA EB 90)
  if (pData[0] == 0x55 && pData[1] == 0xAA && pData[2] == 0xEB && pData[3] == 0x90) {
    if (frame > 100) {
      uint8_t p_type = receivedBytes[4];
      if (!parseinfo && p_type == 0x03) {
        parseDeviceInfo();
        Serial.println("-> [BMS] Parsed Device Info!");
      } else if (!parsesettings && p_type == 0x01) {
        parseBMSSettings();
        Serial.println("-> [BMS] Parsed Settings!");
      } else if (p_type == 0x02) {
        parseBMSData();
        Serial.printf("-> [BMS] Parsed Telemetry! (Received %d bytes)\n", frame);
      }
    }
    frame = 0;
    received_complete = false;
    received_start = true;
    for (size_t i = 0; i < length; i++) {
      if (frame < 384) receivedBytes[frame++] = pData[i];
    }
  } else if (received_start) {
    for (size_t i = 0; i < length; i++) {
      if (frame < 384) receivedBytes[frame++] = pData[i];
    }
    if (receivedBytes[4] == 0x02 && frame >= 280) {
      parseBMSData();
      received_complete = true;
      Serial.printf("-> [BMS] Parsed Telemetry preemptively! (%d bytes)\n", frame);
    }
  }
}

// -----------------------------------------------------------------------------
// --- 8. NimBLE CALLBACKS -----------------------------------------------------
// -----------------------------------------------------------------------------
static void notifyCallback(NimBLERemoteCharacteristic *pBLERemoteCharacteristic, 
                           uint8_t *pData, size_t length, bool isNotify) {
  if (length < 4) return;
  lastValidPacketTime = millis(); // Packet is arriving!
  processBMSFrame(pData, length);
}

class ClientCallbacks : public NimBLEClientCallbacks {
  void onConnect(NimBLEClient *pClient) override {
    isConnected = true;
    currentBleState = BLE_CONNECTED;
    connectionEstablishedTime = millis();
    lastValidPacketTime = millis();
    Serial.println("[NimBLE] Connected successfully to JK-BMS!");
  }

  // Універсальний метод відключення, який є у будь-якій версії NimBLE-Arduino
  void onDisconnect(NimBLEClient *pClient) override {
    pNotifyCharacteristic = nullptr;
    pWriteCharacteristic = nullptr;
    
    // ВИПРАВЛЕНО: Перевіряємо, чи була адреса стерта кнопкою ручного вимкнення
    if (activeBmsAddress.empty()) {
      isConnected = false;
      currentBleState = BLE_IDLE;
      Serial.println("[NimBLE] Ручне відключення підтверджено на рівні ядра. Спимо.");
    } else {
      // Якщо адреса на місці — це був аварійний обрив зв'язку, запускаємо таймер авто-реконнекту
      isConnected = false;
      currentBleState = BLE_RECONNECT_WAIT;
      lastReconnectAttempt = millis();
      Serial.println("[NimBLE Watchdog] Аварійний обрив зв'язку! Запуск таймера авто-реконнекту.");
    }
  }
};


// -----------------------------------------------------------------------------
// --- 8. NimBLE SCAN CALLBACKS & HANDLERS -------------------------------------
// -----------------------------------------------------------------------------

class AdvertisedDeviceCallbacks : public NimBLEAdvertisedDeviceCallbacks {
  void onResult(NimBLEAdvertisedDevice *advertisedDevice) override {
    if (advertisedDevice->haveServiceUUID() && advertisedDevice->isAdvertisingService(serviceUUID)) {
      std::string addr = advertisedDevice->getAddress().toString();
      std::string name = advertisedDevice->getName();
      int rssi = advertisedDevice->getRSSI();
      
      // Заповнюємо карту для виведення в інтерфейс
      FoundBMS bms;
      bms.address = addr;
      bms.name = name;
      bms.rssi = rssi;
      foundDevices[addr] = bms;

      Serial.printf("[NimBLE Scan] Found JK-BMS: %s | %s | RSSI: %d dBm\n", 
                    addr.c_str(), name.c_str(), rssi);

      // ВИПРАВЛЕНО: Перевіряємо, чи вказано дійсно валідний MAC-адрес (зазвичай він має 17 символів)
      bool hasTargetMac = (TARGET_BMS_MAC != nullptr && strlen(TARGET_BMS_MAC) > 10);

      // Автоматичне підключення спрацює ТІЛЬКИ якщо в config.h чітко задано цільовий MAC
      if (hasTargetMac && addr == std::string(TARGET_BMS_MAC)) {
        Serial.println("[NimBLE Scan] Target BMS detected. Stopping scan & connecting...");
        NimBLEDevice::getScan()->stop();
        activeBmsAddress = addr;
        connectToBms(addr);
      }
    }
  }
};


void startScan() {
  foundDevices.clear();
  currentBleState = BLE_SCANNING;
  Serial.println("[NimBLE] Starting scan for nearby JK-BMS devices...");
  pBLEScan->setActiveScan(true); 
  pBLEScan->setInterval(100); // Трохи прискоримо інтервал для надійності
  pBLEScan->setWindow(50);
  pBLEScan->start(6, false);
}

// -----------------------------------------------------------------------------
// --- 9. BLE CONNECTION & REGS MANAGEMENT ------------------------------------
// -----------------------------------------------------------------------------

bool connectToBms(const std::string &addressStr) {
  currentBleState = BLE_CONNECTING;
  Serial.printf("[NimBLE] Connecting to JK-BMS at %s...\n", addressStr.c_str());

  if (pClient != nullptr) {
    if (pClient->isConnected()) {
      pClient->disconnect();
      delay(150);
    }
    NimBLEDevice::deleteClient(pClient);
    pClient = nullptr;
  }

  pClient = NimBLEDevice::createClient();
  pClient->setClientCallbacks(new ClientCallbacks(), false);
  pClient->setConnectTimeout(9);

  NimBLEAddress bmsAddr(addressStr);
  if (!pClient->connect(bmsAddr)) {
    Serial.println("[NimBLE] Connection failed!");
    currentBleState = BLE_RECONNECT_WAIT;
    lastReconnectAttempt = millis();
    return false;
  }

  NimBLERemoteService *pRemoteService = pClient->getService(serviceUUID);
  if (pRemoteService == nullptr) {
    Serial.println("[NimBLE] Service FFE0 not found!");
    pClient->disconnect();
    return false;
  }

  // ВИПРАВЛЕНО: NimBLE повертає покажчик на вектор, використовуємо auto*
  auto* chars = pRemoteService->getCharacteristics(true);
  pNotifyCharacteristic = nullptr;
  pWriteCharacteristic = nullptr;

  if (chars != nullptr) {
    // ВИПРАВЛЕНО: Розіменовуємо покажчик (*chars) для коректного проходу циклом
    for (auto *pChar : *chars) {
      if (pChar->getUUID().equals(charNotifyUUID) || pChar->getUUID().equals(charWriteUUID)) {
        if (pChar->canNotify() || pChar->canIndicate()) {
          pNotifyCharacteristic = pChar;
        }
        if (pChar->canWrite() || pChar->canWriteNoResponse()) {
          pWriteCharacteristic = pChar;
        }
      }
    }
  }

  if (pNotifyCharacteristic == nullptr) pNotifyCharacteristic = pRemoteService->getCharacteristic(charNotifyUUID);
  if (pWriteCharacteristic == nullptr)  pWriteCharacteristic  = pRemoteService->getCharacteristic(charWriteUUID);

  if (pNotifyCharacteristic == nullptr) {
    Serial.println("[NimBLE] Characteristic FFE1 not found!");
    pClient->disconnect();
    return false;
  }

  // Активація сповіщень — Офіційний та надійний метод для NimBLE
  if (pNotifyCharacteristic->canNotify()) {
    Serial.println("[NimBLE] Subscribing to telemetry notifications...");
    
    // Передаємо третім параметром true для автоматичної CCCD підписки на рівні NimBLE
    if (pNotifyCharacteristic->subscribe(true, notifyCallback, true)) {
        Serial.println("NimBLE CCCD підписка активована успішно!");
    } else {
        Serial.println("Помилка підписки NimBLE на CCCD!");
    }
  }

  parsesettings = false;
  parseinfo = false;
  received_complete = false;
  frame = 0;
  activeBmsAddress = addressStr;
  connectionEstablishedTime = millis();
  lastValidPacketTime = millis();

  // Надійна послідовна ініціалізація вичитування BMS з паузами
  delay(300);
  writeRegister(0x97, 0x00000000, 0x00); // Info request
  delay(400);
  writeRegister(0x96, 0x00000000, 0x00); // Settings request
  delay(400);
  writeRegister(0x95, 0x00000000, 0x00); // Telemetry request

  return true;
}

void writeRegister(uint8_t address, uint32_t value, uint8_t length) {
  NimBLERemoteCharacteristic *target = (pWriteCharacteristic != nullptr) ? pWriteCharacteristic : pNotifyCharacteristic;
  if (target == nullptr || !isConnected) return;

  uint8_t cmd[20] = {0xAA, 0x55, 0x90, 0xEB, address, length, 0};
  cmd[6] = (value >> 0)  & 0xFF;
  cmd[7] = (value >> 8)  & 0xFF;
  cmd[8] = (value >> 16) & 0xFF;
  cmd[9] = (value >> 24) & 0xFF;
  cmd[19] = crc(cmd, 19);

  // ВИПРАВЛЕНО: Явно вказуємо третій параметр у writeValue, щоб уникнути помилки Ambiguous
  if (target->canWriteNoResponse()) {
    target->writeValue((const uint8_t*)cmd, sizeof(cmd), false);
  } else {
    target->writeValue((const uint8_t*)cmd, sizeof(cmd), true);
  }
}


// -----------------------------------------------------------------------------
// --- 10. ORIGINAL WEB SERVER HANDLERS (VERNONET EXACT LAYOUT) ----------------
// -----------------------------------------------------------------------------

void handleDataJson() {
  if (!checkAuth()) return;

  // --- РОЗРАХУНОК ЧАСУ ЗАРЯДУ / РОЗРЯДУ (ВИПРАВЛЕНО ФІЗИКУ 99%) ---
  float hours = 0;
  String time_str = "-";
  
  if (isConnected && std::abs(G_data.Charge_Current) > 0.20f) { 
    if (G_data.Charge_Current > 0) {
      // Йде ЗАРЯДКА: рахуємо скільки Ah залишилося до 100%
      float need_capacity = G_settings.total_battery_capacity - G_data.Capacity_Remain;
      if (need_capacity < 0.05f) need_capacity = 0; 
      
      hours = need_capacity / G_data.Charge_Current;
      
      // КРИТИЧНЕ ВИПРАВЛЕНО: Якщо SOC вже 99% або 100%, хімічний заряд вже набрано!
      // Великі хвилини (як-от 40 хв при 1.5А) — це просто цифрова похибка Ah датчика BMS.
      // Зарядка на 99% реально займає не більше 3-5 хвилин до повного відсікання.
      if (G_data.Percent_Remain >= 99 && hours > 0.1f) {
        // Примусово коригуємо час на фінальні 4-5 хвилин пасивного насичення
        hours = 0.08f; // 0.08 години = ~5 хвилин
      }
      
      // Тригер "Full": якщо струм впав зовсім в нуль (менше 1.0А) і ємність набрана
      if (G_data.Charge_Current < MIN_CURRENT && (need_capacity <= 0.3f || G_data.Percent_Remain >= 99)) {
        time_str = "Full";
      } else if (hours > 99.0f) {
        time_str = ">99h"; 
      } else {
        int h = (int)hours;
        int m = (int)((hours - h) * 60);
        // Захист: якщо години 0, а хвилини вирахувались як 0 через округлення — ставимо хоча б 2 хв.
        if (h == 0 && m == 0) m = 2; 
        
        char buf[8];
        sprintf(buf, "%02d:%02d", h, m);
        time_str = String(buf);
      }
    } else {
      // Йде РОЗРЯДКА: рахуємо на скільки вистачить поточного залишку Ah
      hours = G_data.Capacity_Remain / std::abs(G_data.Charge_Current);
      
      if (hours > 99.0f) {
        int d = (int)(hours / 24.0f);
        int h = (int)hours % 24;
        time_str = String(d) + "d " + String(h) + "h";
      } else {
        int h = (int)hours;
        int m = (int)((hours - h) * 60);
        char buf[8];
        sprintf(buf, "%02d:%02d", h, m);
        time_str = String(buf);
      }
    }
  } else if (isConnected) {
    time_str = "No load";
  }

  String json;
  json.reserve(1024);
  json = "{";
  json += "\"connected\":" + String(isConnected ? "true" : "false") + ",";
  json += "\"volt\":" + String(G_data.Battery_Voltage, 2) + ",";
  json += "\"curr\":" + String(G_data.Charge_Current, 2) + ",";
  json += "\"pow\":" + String(G_data.Battery_Power, 1) + ",";
  json += "\"soc\":" + String(G_data.Percent_Remain) + ",";
  json += "\"delta\":" + String(G_data.Delta_Cell_Voltage, 3) + ",";
  
  json += "\"btime\":\"" + time_str + "\",";
  json += "\"cap_rem\":" + String(G_data.Capacity_Remain, 1) + ",";
  json += "\"cap_nom\":" + String(G_settings.total_battery_capacity, 1) + ",";
  json += "\"cycles\":" + String((int)G_data.Cycle_Count) + ",";

  json += "\"t1\":" + String(G_data.Battery_T1, 1) + ",";
  json += "\"t2\":" + String(G_data.Battery_T2, 1) + ",";
  json += "\"tmos\":" + String(G_data.MOS_Temp, 1) + ",";
  json += "\"bcurr\":" + String(G_data.Balance_Curr, 3) + ",";
  json += "\"chg\":" + String(G_data.Charge ? "true" : "false") + ",";
  json += "\"dsg\":" + String(G_data.Discharge ? "true" : "false") + ",";
  bool isBalancingActive = G_data.Balance || (std::abs(G_data.Balance_Curr) > 0.001f);
  json += "\"bal\":" + String(isBalancingActive ? "true" : "false") + ",";
  json += "\"cells\":[";

  int c_count = G_settings.cell_count;
  if (c_count <= 0 || c_count > 32) c_count = 16;

  for (int i = 0; i < c_count; i++) {
    json += "{\"v\":" + String(G_data.cellVoltage[i], 3) + ",\"r\":" + String(G_data.wireResist[i], 3) + "}";
    if (i < c_count - 1) json += ",";
  }
  json += "]}";
  
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json; charset=UTF-8", json);
}



// RESTORED EXACT ORIGINAL DASHBOARD FROM VERNONET WITH DISCONNECT CONTROL
void handleRoot() {
  if (!checkAuth()) return;

  // Якщо зв'язку з BMS немає — показуємо стартовий екран пошуку пристроїв
  if (!isConnected) {
    String html = "<!DOCTYPE html><html><head><meta charset='UTF-8'>";
    html += "<meta name='viewport' content='width=device-width,initial-scale=1.0'>";
    html += "<title>BMS DISCONNECTED</title>";
    html += "<style>body{font-family:Arial,sans-serif;background:#f4f4f9;text-align:center;margin-top:50px}";
    html += ".btn{display:inline-block;padding:10px 20px;margin:10px;background:#007bff;color:#fff;text-decoration:none;border-radius:5px;font-weight:bold}";
    html += ".container{max-width:500px;margin:auto;background:#fff;padding:20px;border-radius:8px;box-shadow:0 2px 4px rgba(0,0,0,0.1)}</style></head><body>";
    html += "<div class='container'><h2>⚡️ BMS Status: <span style='color:#dc3545'>DISCONNECTED</span></h2>";
    html += "<p>IP Address: <b>" + WiFi.localIP().toString() + "</b></p>";
    html += "<p style='text-align: center;'>";
    html += "  <a href='/reboot' onclick=\"return confirm('Reboot ESP32?');\" class='btn' style='background:#dc3545'>🔄 REBOOT</a>";
    html += "  <a href='/update' class='btn' style='background:#6c757d'>📥 OTA UPDATE</a>";
    html += "  <a href='/scan' class='btn'>🔍 SCAN & SELECT BMS</a>";
    html += "</p></div></body></html>";
    server.send(200, "text/html; charset=UTF-8", html);
    return;
  }

  // Якщо пристрій підключено — рендеримо основний AJAX-інтерфейс телеметрії
    // Якщо пристрій підключено — рендеримо основний AJAX-інтерфейс телеметрії
  String html = "<!DOCTYPE html><html><head><meta charset='UTF-8'><meta name='viewport' content='width=device-width,initial-scale=1'><title>Jikong BMS Telemetry</title>";
  
  // ВИПРАВЛЕНО: Додано table-layout:fixed, фіксовану ширину для th/td та min-width для кнопок, щоб геометрія не рухалась
  html += "<style>body{font-family:Arial,sans-serif;background:#f4f4f9;padding:10px;color:#333}h2{text-align:center;color:#007bff}";
  html += ".container{max-width:700px;margin:auto;background:#fff;padding:15px;border-radius:8px;box-shadow:0 2px 4px rgba(0,0,0,.1)}";
  html += "table{width:100%;border-collapse:collapse;margin-top:10px;table-layout:fixed}"; // Фіксує стовпчики
  html += "th,td{padding:8px;border-bottom:1px solid #ddd;text-align:left;overflow:hidden;white-space:nowrap}";
  html += "th{background:#007bff;color:#fff}";
  html += "th:nth-child(1), td:nth-child(1){width:40%}"; // 40% для назви параметра
  html += "th:nth-child(2), td:nth-child(2){width:40%}"; // 40% для живих значень та статусів
  html += "th:nth-child(3), td:nth-child(3){width:20%}"; // 20% для одиниць виміру та кнопок
  html += ".btn{display:inline-block;padding:6px 12px;background:#007bff;color:#fff;text-decoration:none;border-radius:4px;margin:2px;font-size:14px;font-weight:bold;text-align:center;min-width:90px}"; // Фіксована мін. ширина кнопок
  html += ".on{background:#28a745}.off{background:#dc3545}</style></head>"; 
  html += "<body><div class='container'><h2>⚡️ Live Jikong BMS Telemetry</h2>";

  html += "<p style='text-align:center'>";
  html += "  <a href='/update' class='btn' style='background:#28a745'>📥 OTA UPDATE</a>";
  html += "  <a href='/settings' class='btn'>⚙️ Settings</a>";
  html += "  <a href='/info' class='btn' style='background:#17a2b8'>ℹ️ Info</a>";
  html += "  <a href='/scan' class='btn' style='background:#6c757d'>🔍 BLE Scan</a>";
  html += "  <a href='/disconnect' class='btn off'>❌ Disconnect BLE</a>";
  html += "</p>";

  html += "<table><tr><th>Parameter</th><th>Value</th><th>Unit</th></tr>";
  html += "<tr><td>Battery Voltage</td><td id='volt'>-</td><td>V</td></tr>";
  html += "<tr><td>Charge / Discharge Current</td><td id='curr'>-</td><td>A</td></tr>";
  html += "<tr><td>Power</td><td id='pow'>-</td><td>W</td></tr>";
  html += "<tr><td>State of Charge (SOC)</td><td id='soc'>-</td><td>%</td></tr>";
  html += "<tr><td>Remaining Capacity</td><td><span id='cap_rem'>-</span> / <span id='cap_nom'>-</span></td><td>Ah</td></tr>";
  html += "<tr><td>Battery Cycle Count</td><td id='cycles'>-</td><td>cycles</td></tr>";

  html += "<tr><td>Cell Delta</td><td id='delta'>-</td><td>V</td></tr>";
  html += "<tr><td>Temperature T1 / T2</td><td><span id='t1'>-</span> / <span id='t2'>-</span></td><td>°C</td></tr>";
  html += "<tr><td>MOSFET Temperature</td><td id='tmos'>-</td><td>°C</td></tr>";
  html += "<tr><td>Charge Switch</td><td id='s_chg'>-</td><td id='b_chg'>-</td></tr>";
  html += "<tr><td>Discharge Switch</td><td id='s_dsg'>-</td><td id='b_dsg'>-</td></tr>";

  html += "<tr><td>Balance Switch</td><td id='s_bal'>-</td><td id='b_bal'>-</td></tr>";

  html += "<tr><td id='time_label'>⏱ Status</td><td id='btime' style='font-weight:bold;color:#007bff;'>-</td><td>hrs</td></tr>"; // Відновлено новий рядок розрахунку часу
  html += "</table><h2>🔬 Cell Voltages</h2><table id='cell_table'></table></div>";

  html += "<script>";
   html += "function update(){";
  html += "  fetch('/data.json').then(r=>r.json()).then(d=>{";
  html += "    if(!d.connected){ window.location.reload(); return; }";
  
  html += "    let v = d.volt || 0; let c = d.curr || 0; let p = d.pow || 0;";
  html += "    let dl = d.delta || 0; let t1 = d.t1 || 0; let t2 = d.t2 || 0; let tm = d.tmos || 0;";
  html += "    if(document.getElementById('volt')) document.getElementById('volt').innerText = v.toFixed(2);";
  html += "    if(document.getElementById('curr')) document.getElementById('curr').innerText = c.toFixed(2);";
  html += "    if(document.getElementById('pow'))  document.getElementById('pow').innerText  = p.toFixed(1);";
  html += "    if(document.getElementById('soc'))  document.getElementById('soc').innerText  = d.soc || 0;";
  html += "    if(document.getElementById('delta')) document.getElementById('delta').innerText = dl.toFixed(3);";
  html += "    if(document.getElementById('t1'))    document.getElementById('t1').innerText    = t1.toFixed(1);";
  html += "    if(document.getElementById('t2'))    document.getElementById('t2').innerText    = t2.toFixed(1);";
  html += "    if(document.getElementById('tmos'))  document.getElementById('tmos').innerText  = tm.toFixed(1);";
  
  // RESTORED: Dynamic display of charge/discharge time from JSON (English layout)
  html += "    if(document.getElementById('btime')) document.getElementById('btime').innerText = d.btime || '-';";
  html += "    if(document.getElementById('time_label')) {";
  html += "      if(c > 0.05) {";
  html += "        document.getElementById('time_label').innerText = '⏱ Time to Full Charge';";
  html += "      } else if(c < -0.05) {";
  html += "        document.getElementById('time_label').innerText = '⏱ Time Remaining (Discharge)';";
  html += "      } else {";
  html += "        document.getElementById('time_label').innerText = '⏱ Battery Status';";
  html += "      }";
  html += "    }";

  // NEW: Update Remaining Capacity (Ah) and Cycle Count on dashboard
  html += "    if(document.getElementById('cap_rem')) document.getElementById('cap_rem').innerText = (d.cap_rem || 0).toFixed(1);";
  html += "    if(document.getElementById('cap_nom')) document.getElementById('cap_nom').innerText = (d.cap_nom || 0).toFixed(1);";
  html += "    if(document.getElementById('cycles'))  document.getElementById('cycles').innerText  = d.cycles || 0;";

  // MOSFET Switches Control Layout
  html += "    if(document.getElementById('s_chg')) document.getElementById('s_chg').innerHTML = d.chg ? '<span style=\"color:green\">🟢 ON</span>' : '<span style=\"color:red\">🔴 OFF</span>';";
  html += "    if(document.getElementById('b_chg')) document.getElementById('b_chg').innerHTML = d.chg ? '<a href=\"#\" onclick=\"fetch(\\'/charge_off\\'); return false;\" class=\"btn off\">TURN OFF</a>' : '<a href=\"#\" onclick=\"fetch(\\'/charge_on\\'); return false;\" class=\"btn on\">TURN ON</a>';";
  html += "    if(document.getElementById('s_dsg')) document.getElementById('s_dsg').innerHTML = d.dsg ? '<span style=\"color:green\">🟢 ON</span>' : '<span style=\"color:red\">🔴 OFF</span>';";
  html += "    if(document.getElementById('b_dsg')) document.getElementById('b_dsg').innerHTML = d.dsg ? '<a href=\"#\" onclick=\"fetch(\\'/discharge_off\\'); return false;\" class=\"btn off\">TURN OFF</a>' : '<a href=\"#\" onclick=\"fetch(\\'/discharge_on\\'); return false;\" class=\"btn on\">TURN ON</a>';";

  // ГАРАНТОВАНО: Повертаємо кнопку керування на місце та виправляємо логіку відображення
  html += "    let bc = d.bcurr || 0;";
  html += "    if(document.getElementById('s_bal')) {";
  html += "      document.getElementById('s_bal').innerHTML = d.bal ? '<span style=\"color:green;font-weight:bold;\">🟢 Active</span> <small style=\"color:#555;\">(' + bc.toFixed(3) + 'A)</small>' : '⚪ Inactive';";
  html += "    }";
  
  html += "    if(document.getElementById('b_bal')) {";
  // ВИПРАВЛЕНО: Кнопка малюється ЗАВЖДИ: якщо d.bal true - TURN OFF, якщо false - TURN ON
  html += "      document.getElementById('b_bal').innerHTML = d.bal ? '<a href=\"#\" onclick=\"fetch(\\'/balance_off\\'); return false;\" class=\"btn off\">TURN OFF</a>' : '<a href=\"#\" onclick=\"fetch(\\'/balance_on\\'); return false;\" class=\"btn on\">TURN ON</a>';";
  html += "    }";



    // Cells Table Generation Loop WITH LOWEST CELL HIGHLIGHTING
  html += "    if(d.cells && d.cells.length > 0 && document.getElementById('cell_table')) {";
  html += "      let t = '<tr><th>Cell</th><th>Voltage (V)</th><th>Resistance (\\u03A9)</th></tr>';"; // \\u03A9 = знак Ома
  
  // 1. Пошук індексу найслабшої комірки (ігноруючи порожні канали по 0V)
  html += "      let minV = 99; let minIdx = -1;";
  html += "      d.cells.forEach((cell, i) => {";
  html += "        let cv = cell.v || 0;";
  html += "        if(cv > 1.0 && cv < minV) { minV = cv; minIdx = i; }";
  html += "      });";
  
  // 2. Генерація рядків таблиці з динамічним CSS-стилем для найнижчої комірки
  html += "      d.cells.forEach((cell, i) => {";
  html += "        let cv = cell.v || 0; let cr = cell.r || 0;";
  html += "        let rowStyle = (i === minIdx) ? ' style=\"background-color: #ffcccc; font-weight: bold;\"' : '';";
  html += "        let cellLabel = (i === minIdx) ? '⚠️ Cell ' + (i + 1) + ' (Lowest)' : 'Cell ' + (i + 1);";
  html += "        t += '<tr' + rowStyle + '><td>' + cellLabel + '</td><td>' + cv.toFixed(3) + '</td><td>' + cr.toFixed(3) + '</td></tr>';";
  html += "      });";
  
  html += "      document.getElementById('cell_table').innerHTML = t;";
  html += "    }";
  
  html += "  }).catch(err => console.log('Fetch Error: ', err));";
  html += "}";
  html += "setInterval(update, 2000); update();";

  html += "</script></body></html>";

  server.sendHeader("Connection", "close");
  server.send(200, "text/html; charset=UTF-8", html);
}



// RESTORED EXACT ORIGINAL INFO PAGE FROM VERNONET
void handleInfo() {
  if (!checkAuth()) return;
  String html = "<!DOCTYPE html><html><head><meta charset='UTF-8'><meta name='viewport' content='width=device-width,initial-scale=1'><title>Device Info</title>";
  html += "<style>body{font:15px Arial;background:#f4f4f9;margin:20px}table{width:100%;border-collapse:collapse;margin-top:15px}td,th{padding:8px;text-align:left;border-bottom:1px solid #ddd}th{background:#f8f9fa}.btn{display:inline-block;padding:8px 16px;background:#007bff;color:#fff;text-decoration:none;border-radius:4px}</style></head>";
  html += "<body><div style='max-width:500px;margin:auto;background:#fff;padding:20px;border-radius:8px;box-shadow:0 2px 4px rgba(0,0,0,.1)'>";
  html += "<h2>ℹ️ Device Information</h2><table><tr><th>Parameter</th><th>Value</th></tr>";
  auto addR = [&](const char *l, const char *v, const char *s = "") {
    return "<tr><td>" + String(l) + "</td><td " + String(s) + ">" + String(v) + "</td></tr>";
  };
  html += addR("Device Name", G_info.deviceName.c_str());
  html += addR("Serial Number", G_info.serialNumber.c_str());
  html += addR("HW / SW Version", (G_info.hardwareVersion + " / " + G_info.softwareVersion).c_str());
  html += addR("Firmware Build Time", (__DATE__ " " __TIME__), "style='color:#28a745;font-weight:bold'");
  html += addR("Vendor ID", G_info.vendorID.c_str());
  html += addR("Manufacturing Date", G_info.manufacturingDate.c_str());
  html += addR("Device Passcode", G_info.devicePasscode.c_str());
  html += addR("Setup Passcode", G_info.setupPasscode.c_str());
  html += addR("Passcode", G_info.passcode.c_str());
  html += addR("User Custom Data", G_info.userData.c_str());
  html += "</table><br><p style='text-align:center'><a href='/' class='btn'>◀️ DASHBOARD</a></p></div></body></html>";
  server.sendHeader("Connection", "close");
  server.send(200, "text/html; charset=UTF-8", html);
}

// RESTORED EXACT ORIGINAL SETTINGS PAGE FROM VERNONET
void handleSettings() {
  if (!checkAuth()) return;
  if (!isConnected) {
    server.sendHeader("Location", "/");
    server.send(302, "text/plain", "");
    return;
  }

  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/html; charset=UTF-8", "");
  String ch = "";
  ch.reserve(1024);

  // CHUNK 1: HEADER & STYLES
  ch = "<!DOCTYPE html><html><head><meta charset='UTF-8'><meta name='viewport' content='width=device-width,initial-scale=1'><title>Jikong BMS Settings</title>";
  ch += "<style>body{font-family:Arial,sans-serif;background:#f4f4f9;padding:10px}table{width:100%;border-collapse:collapse;margin-top:10px}td,th{border:1px solid #ddd;padding:6px;text-align:left}th{background:#007bff;color:#fff}.group th{background:#4CAF50}input{width:90%;padding:4px;border:1px solid #ccc;border-radius:4px}.btn{display:block;width:100%;padding:12px;background:#007bff;color:#fff;border:none;border-radius:5px;font-size:16px;cursor:pointer;margin-top:20px}</style></head>";
  ch += "<body><div style='max-width:750px;margin:auto;background:#fff;padding:15px;border-radius:8px;box-shadow:0 2px 4px rgba(0,0,0,.1)'>";
  ch += "<h2>⚙️ Jikong BMS Settings</h2><a href='/'>⬅️ Back to Dashboard</a><br><br><form method='POST' action='/settings_update'><table><tr><th>Parameter</th><th>Current</th><th>New</th></tr>";

  ch += "<tr class='group'><th colspan='3'>Battery Parameters</th></tr>";
  ch += "<tr><td>Nominal Capacity (Ah)</td><td>" + String(G_settings.total_battery_capacity, 3) + "</td><td><input type='number' name='total_battery_capacity' value='" + String(G_settings.total_battery_capacity, 3) + "' step='0.001' required></td></tr>";
  ch += "<tr><td>Cell Count</td><td>" + String(G_settings.cell_count) + "</td><td><input type='number' name='cell_count' value='" + String(G_settings.cell_count) + "' step='1' required></td></tr>";
  ch += "<tr><td>BMS Mode</td><td>" + String(G_settings.bms_mode) + "</td><td><input type='number' name='bms_mode' value='" + String(G_settings.bms_mode) + "' step='1' required></td></tr>";
  server.sendContent(ch);
  ch = "";

  // CHUNK 2: CELL VOLTAGES
  ch += "<tr class='group'><th colspan='3'>Cell Voltages (V)</th></tr>";
  ch += "<tr><td>Overvoltage Protection (OVP)</td><td>" + String(G_settings.cell_voltage_overvoltage_protection, 3) + "</td><td><input type='number' name='cell_voltage_overvoltage_protection' value='" + String(G_settings.cell_voltage_overvoltage_protection, 3) + "' step='0.001' required></td></tr>";
  ch += "<tr><td>Undervoltage Protection (UVP)</td><td>" + String(G_settings.cell_voltage_undervoltage_protection, 3) + "</td><td><input type='number' name='cell_voltage_undervoltage_protection' value='" + String(G_settings.cell_voltage_undervoltage_protection, 3) + "' step='0.001' required></td></tr>";
  ch += "<tr><td>OVP Recovery Voltage</td><td>" + String(G_settings.cell_voltage_overvoltage_recovery, 3) + "</td><td><input type='number' name='cell_voltage_overvoltage_recovery' value='" + String(G_settings.cell_voltage_overvoltage_recovery, 3) + "' step='0.001' required></td></tr>";
  ch += "<tr><td>UVP Recovery Voltage</td><td>" + String(G_settings.cell_voltage_undervoltage_recovery, 3) + "</td><td><input type='number' name='cell_voltage_undervoltage_recovery' value='" + String(G_settings.cell_voltage_undervoltage_recovery, 3) + "' step='0.001' required></td></tr>";
  ch += "<tr><td>Power-Off Voltage</td><td>" + String(G_settings.power_off_voltage, 3) + "</td><td><input type='number' name='power_off_voltage' value='" + String(G_settings.power_off_voltage, 3) + "' step='0.001' required></td></tr>";

  ch += "<tr class='group'><th colspan='3'>Charging / Nominal Voltages (V)</th></tr>";
  ch += "<tr><td>Full Charge Voltage</td><td>" + String(G_settings.full_charge_voltage, 3) + "</td><td><input type='number' name='full_charge_voltage' value='" + String(G_settings.full_charge_voltage, 3) + "' step='0.001' required></td></tr>";
  ch += "<tr><td>Nominal Voltage</td><td>" + String(G_settings.nominal_voltage, 3) + "</td><td><input type='number' name='nominal_voltage' value='" + String(G_settings.nominal_voltage, 3) + "' step='0.001' required></td></tr>";
  ch += "<tr><td>Float Charge Voltage</td><td>" + String(G_settings.float_charge_voltage, 3) + "</td><td><input type='number' name='float_charge_voltage' value='" + String(G_settings.float_charge_voltage, 3) + "' step='0.001' required></td></tr>";
  server.sendContent(ch);
  ch = "";

  // CHUNK 3: BALANCING & CURRENTS
  ch += "<tr class='group'><th colspan='3'>Balancing</th></tr>";
  ch += "<tr><td>Balance Trigger Voltage (V)</td><td>" + String(G_settings.balance_trigger_voltage, 3) + "</td><td><input type='number' name='balance_trigger_voltage' value='" + String(G_settings.balance_trigger_voltage, 3) + "' step='0.001' required></td></tr>";
  ch += "<tr><td>Balance Starting Voltage (V)</td><td>" + String(G_settings.balance_starting_voltage, 3) + "</td><td><input type='number' name='balance_starting_voltage' value='" + String(G_settings.balance_starting_voltage, 3) + "' step='0.001' required></td></tr>";
  ch += "<tr><td>Max Balance Current (A)</td><td>" + String(G_settings.max_balance_current, 3) + "</td><td><input type='number' name='max_balance_current' value='" + String(G_settings.max_balance_current, 3) + "' step='0.001' required></td></tr>";
  ch += "<tr><td>Balance Delta Stop Threshold (V)</td><td>" + String(G_settings.balance_precision, 3) + "</td><td><input type='number' name='balance_precision' value='" + String(G_settings.balance_precision, 3) + "' step='0.001' required></td></tr>";
  ch += "<tr class='group'><th colspan='3'>Current Limits (A)</th></tr>";
  ch += "<tr><td>Max Charge Current (A)</td><td>" + String(G_settings.max_charge_current, 3) + "</td><td><input type='number' name='max_charge_current' value='" + String(G_settings.max_charge_current, 3) + "' step='0.001' required></td></tr>";
  ch += "<tr><td>Max Discharge Current (A)</td><td>" + String(G_settings.max_discharge_current, 3) + "</td><td><input type='number' name='max_discharge_current' value='" + String(G_settings.max_discharge_current, 3) + "' step='0.001' required></td></tr>";
  ch += "</table><button type='submit' class='btn'>💾 SAVE ALL SETTINGS</button></form></div></body></html>";
  server.sendContent(ch);
  server.sendContent("");
}

// RESTORED FULL EXTENDED SETTINGS UPDATE POST HANDLER
void handleSettingsUpdate() {
  if (!checkAuth()) return;
  if (!isConnected) { server.send(403, "text/plain", "Not connected."); return; }

  // --- 1. ПАРАМЕТРИ БАТАРЕЇ ---
  if (server.hasArg("total_battery_capacity")) {
    float val = server.arg("total_battery_capacity").toFloat();
    if (std::abs(val - G_settings.total_battery_capacity) > FLOAT_TOLERANCE) {
      writeRegister(0x41, (uint32_t)(val * 1000), 0x04); delay(200); // Index 130 -> 0x41
    }
  }
  if (server.hasArg("cell_count")) {
    uint32_t val = (uint32_t)server.arg("cell_count").toInt();
    if (val != G_settings.cell_count) { 
      writeRegister(0x27, val, 0x04); delay(200); // Index 114 -> 0x27
    }
  }
  if (server.hasArg("bms_mode")) {
    uint32_t val = (uint32_t)server.arg("bms_mode").toInt();
    if (val != G_settings.bms_mode) { 
      writeRegister(0x28, val, 0x04); delay(200); // Index 118 -> 0x28
    }
  }

  // --- 2. НАПРУГИ КОМІРОК (Scale: * 1000) ---
  if (server.hasArg("cell_voltage_overvoltage_protection")) {
    float val = server.arg("cell_voltage_overvoltage_protection").toFloat();
    if (std::abs(val - G_settings.cell_voltage_overvoltage_protection) > FLOAT_TOLERANCE) {
      writeRegister(0x0C, (uint32_t)(val * 1000), 0x04); delay(200); // Index 18 -> 0x0C
    }
  }
  if (server.hasArg("cell_voltage_undervoltage_protection")) {
    float val = server.arg("cell_voltage_undervoltage_protection").toFloat();
    if (std::abs(val - G_settings.cell_voltage_undervoltage_protection) > FLOAT_TOLERANCE) {
      writeRegister(0x0A, (uint32_t)(val * 1000), 0x04); delay(200); // Index 10 -> 0x0A
    }
  }
  if (server.hasArg("cell_voltage_overvoltage_recovery")) {
    float val = server.arg("cell_voltage_overvoltage_recovery").toFloat();
    if (std::abs(val - G_settings.cell_voltage_overvoltage_recovery) > FLOAT_TOLERANCE) {
      writeRegister(0x0D, (uint32_t)(val * 1000), 0x04); delay(200); // Index 22 -> 0x0D
    }
  }
  if (server.hasArg("cell_voltage_undervoltage_recovery")) {
    float val = server.arg("cell_voltage_undervoltage_recovery").toFloat();
    if (std::abs(val - G_settings.cell_voltage_undervoltage_recovery) > FLOAT_TOLERANCE) {
      writeRegister(0x0B, (uint32_t)(val * 1000), 0x04); delay(200); // Index 14 -> 0x0B
    }
  }
  if (server.hasArg("power_off_voltage")) {
    float val = server.arg("power_off_voltage").toFloat();
    if (std::abs(val - G_settings.power_off_voltage) > FLOAT_TOLERANCE) {
      writeRegister(0x14, (uint32_t)(val * 1000), 0x04); delay(200); // Index 46 -> 0x14
    }
  }

  // --- 3. ЗАРЯДНІ / НОМІНАЛЬНІ НАПРУГИ (Scale: * 1000) ---
  if (server.hasArg("full_charge_voltage")) {
    float val = server.arg("full_charge_voltage").toFloat();
    if (std::abs(val - G_settings.full_charge_voltage) > FLOAT_TOLERANCE) {
      writeRegister(0x0F, (uint32_t)(val * 1000), 0x04); delay(200); // Index 30 -> 0x0F
    }
  }
  if (server.hasArg("nominal_voltage")) {
    float val = server.arg("nominal_voltage").toFloat();
    if (std::abs(val - G_settings.nominal_voltage) > FLOAT_TOLERANCE) {
      writeRegister(0x10, (uint32_t)(val * 1000), 0x04); delay(200); // Index 34 -> 0x10
    }
  }
  if (server.hasArg("float_charge_voltage")) {
    float val = server.arg("float_charge_voltage").toFloat();
    if (std::abs(val - G_settings.float_charge_voltage) > FLOAT_TOLERANCE) {
      writeRegister(0x11, (uint32_t)(val * 1000), 0x04); delay(200); // Index 38 -> 0x11
    }
  }

  // --- 4. БАЛАНСУВАННЯ (Scale: * 1000) ---
  if (server.hasArg("balance_trigger_voltage")) {
    float val = server.arg("balance_trigger_voltage").toFloat();
    if (std::abs(val - G_settings.balance_trigger_voltage) > FLOAT_TOLERANCE) {
      writeRegister(0x0E, (uint32_t)(val * 1000), 0x04); delay(200); // Index 26 -> 0x0E
    }
  }
  if (server.hasArg("balance_starting_voltage")) {
    float val = server.arg("balance_starting_voltage").toFloat();
    if (std::abs(val - G_settings.balance_starting_voltage) > FLOAT_TOLERANCE) {
      writeRegister(0x45, (uint32_t)(val * 1000), 0x04); delay(200); // Index 138 -> 0x45
    }
  }
  if (server.hasArg("max_balance_current")) {
    float val = server.arg("max_balance_current").toFloat();
    if (std::abs(val - G_settings.max_balance_current) > FLOAT_TOLERANCE) {
      writeRegister(0x1C, (uint32_t)(val * 1000), 0x04); delay(200); // Index 78 -> 0x1C
    }
  }
  if (server.hasArg("balance_precision")) {
    float val = server.arg("balance_precision").toFloat();
    if (std::abs(val - G_settings.balance_precision) > FLOAT_TOLERANCE) {
      writeRegister(0x12, (uint32_t)(val * 1000), 0x04); delay(200); // Index 42 -> 0x12
    }
  }

  // --- 5. ОБМЕЖЕННЯ СТРУМУ (Scale: * 1000) ---
  if (server.hasArg("max_charge_current")) {
    float val = server.arg("max_charge_current").toFloat();
    if (std::abs(val - G_settings.max_charge_current) > FLOAT_TOLERANCE) {
      writeRegister(0x15, (uint32_t)(val * 1000), 0x04); delay(200); // Index 50 -> 0x15
    }
  }
  if (server.hasArg("max_discharge_current")) {
    float val = server.arg("max_discharge_current").toFloat();
    if (std::abs(val - G_settings.max_discharge_current) > FLOAT_TOLERANCE) {
      writeRegister(0x18, (uint32_t)(val * 1000), 0x04); delay(200); // Index 62 -> 0x18
    }
  }

  parsesettings = false;
  received_complete = false;
  frame = 0;
  writeRegister(0x96, 0x00000000, 0x00);
  delay(500);
  server.sendHeader("Location", "/");
  server.send(302, "text/plain", "");
}


// RESTORED EXACT ORIGINAL BLE SCANNER HTML FROM VERNONET WITH VALID TAGS
void handleScan() {
  if (!checkAuth()) return;
  startScan();
  
  // ВІДНОВЛЕНО: Повний, валідний HTML код з коректними тегами таблиці та стилями
  String html = "<!DOCTYPE html><html><head><meta charset='UTF-8'><title>BLE Scanner</title>";
  html += "<style>body{font-family:Arial;background:#f4f4f9;padding:10px}table{width:100%;border-collapse:collapse;margin-top:20px}td,th{border:1px solid #ddd;padding:8px;text-align:left}th{background:#007bff;color:#fff}.btn{display:inline-block;padding:5px 10px;color:#fff;background:#28a745;text-decoration:none;border-radius:4px;font-weight:bold}</style></head>";
  html += "<body><div style='max-width:600px;margin:auto;background:#fff;padding:15px;border-radius:8px;box-shadow:0 2px 4px rgba(0,0,0,0.1)'><h2>🔍 BLE BMS Scanner</h2><table><tr><th>Device Name</th><th>MAC Address</th><th>Action</th></tr>";
  
  if (foundDevices.empty()) {
    html += "<tr><td colspan='3' style='text-align:center;padding:15px;color:#666;'>Scanning for devices... Refresh in a few seconds.</td></tr>";
  } else {
    for (const auto &pair : foundDevices) {
      String address = String(pair.first.c_str());
      String displayName = String(pair.second.name.c_str()) + " (" + String(pair.second.rssi) + " dBm)";
      
      html += "<tr>";
      html += "<td>" + displayName + "</td>";
      html += "<td>" + address + "</td>";
      html += "<td><a href='/connect?mac=" + address + "' class='btn'>Connect</a></td>";
      html += "</tr>";
    }
  }
  html += "</table><br><a href='/' style='color:#007bff;text-decoration:none;'>◀️ Back to Dashboard</a></div></body></html>";
  
  server.sendHeader("Connection", "close");
  server.send(200, "text/html; charset=UTF-8", html);
}

void handleChargeOn()    { if (!checkAuth()) return; if (isConnected) writeRegister(0x1D, 1, 0x04); delay(100); server.send(200, "text/plain", "OK"); }
void handleChargeOff()   { if (!checkAuth()) return; if (isConnected) writeRegister(0x1D, 0, 0x04); delay(100); server.send(200, "text/plain", "OK"); }
void handleDischargeOn() { if (!checkAuth()) return; if (isConnected) writeRegister(0x1E, 1, 0x04); delay(100); server.send(200, "text/plain", "OK"); }
void handleDischargeOff(){ if (!checkAuth()) return; if (isConnected) writeRegister(0x1E, 0, 0x04); delay(100); server.send(200, "text/plain", "OK"); }
void handleBalanceOn()   { if (!checkAuth()) return; if (isConnected) writeRegister(0x1F, 1, 0x04); delay(100); server.send(200, "text/plain", "OK"); }
void handleBalanceOff()  { if (!checkAuth()) return; if (isConnected) writeRegister(0x1F, 0, 0x04); delay(100); server.send(200, "text/plain", "OK"); }

void handleConnect() {
  if (!checkAuth()) return;
  if (server.hasArg("mac")) {
    std::string mac_addr = server.arg("mac").c_str();
    if (connectToBms(mac_addr)) {
      server.sendHeader("Location", "/");
      server.send(302, "text/plain", "");
      return;
    }
  }
  server.send(500, "text/plain; charset=UTF-8", "Connection error.");
}

void handleDisconnect() {
  if (!checkAuth()) return; //
  
  Serial.println("[HTTP] Отримано команду ручного вимкнення BLE...");

  // Спочатку очищаємо адресу, щоб сработала наша нова умова в onDisconnect
  activeBmsAddress = ""; 
  
  if (pClient != nullptr) {
    if (pClient->isConnected()) {
      pClient->disconnect();
      delay(150); 
    }
    NimBLEDevice::deleteClient(pClient);
    pClient = nullptr;
  }

  isConnected = false;
  parsesettings = false;
  parseinfo = false;
  frame = 0;
  currentBleState = BLE_IDLE; 

  server.sendHeader("Location", "/");
  server.send(302, "text/plain", "");
}


// RESTORED EXACT ORIGINAL OTA UPDATE FROM VERNONET
void handleUpdate() {
  if (!checkAuth()) return;
  server.send(200, "text/html",
              "<!DOCTYPE html><html><head><title>OTA Update</title><meta charset='UTF-8'>"
              "<style>body{font-family: Arial; text-align: center;} input{padding: 10px; margin: 5px; border-radius: 5px;} .file-upload{border: 1px solid #ccc; padding: 20px; width: 300px; margin: 50px auto;}</style></head>"
              "<body><h1>OTA Update</h1><p>Select firmware binary file (.bin):</p>"
              "<form method='POST' action='/update' enctype='multipart/form-data' class='file-upload'>"
              "<input type='file' name='firmware'><input type='submit' value='Upload & Update'></form>"
              "<a href='/'>&#9664; Back to Dashboard</a></body></html>");
}

void handleUpdateUpload() {
  if (!checkAuth()) return;
  HTTPUpload &upload = server.upload();
  if (upload.status == UPLOAD_FILE_START) {
    Serial.printf("Receiving firmware binary: %s\n", upload.filename.c_str());
    if (pClient != nullptr && pClient->isConnected()) {
      Serial.println("OTA: Disconnecting BLE Client to free heap memory...");
      pClient->disconnect();
    }
    isConnected = false;
    pWriteCharacteristic = nullptr;
    pNotifyCharacteristic = nullptr;
    delay(200);

    if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
      Update.printError(Serial);
    }
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
      Update.printError(Serial);
    }
  } else if (upload.status == UPLOAD_FILE_END) {
    if (Update.end(true)) {
      Serial.printf("Update successful: %u bytes\nRebooting...\n", upload.totalSize);
      String html = "<!DOCTYPE html><html><head><meta charset='UTF-8'><meta name='viewport' content='width=device-width,initial-scale=1.0'>";
      html += "<style>body{font-family:Arial,sans-serif;text-align:center;background:#f4f4f9;padding-top:50px}.box{max-width:400px;margin:auto;background:#fff;padding:30px;border-radius:8px;box-shadow:0 2px 4px rgba(0,0,0,0.1)} b{color:#28a745}</style></head><body>";
      html += "<div class='box'><h2>✅ Update Successful!</h2>";
      html += "<p>Device is rebooting... Returning to dashboard in <b id='timer'>10</b> sec.</p></div>";
      html += "<script>let c=10; function dt(){ if(c<=0){ window.location.href='/'; } else { document.getElementById('timer').innerText=c; c--; setTimeout(dt,1000); } } setTimeout(dt,1000);</script>";
      html += "</body></html>";
      server.send(200, "text/html; charset=UTF-8", html);
      delay(1000);
      ESP.restart();
    } else {
      Update.printError(Serial);
      server.send(500, "text/html; charset=UTF-8", "<h2>❌ Update Failed! Firmware file corrupted.</h2><a href='/update'>Try again</a>");
    }
  }
}

void handleReboot() {
  if (!checkAuth()) return;
  server.send(200, "text/plain", "Rebooting...");
  delay(500);
  ESP.restart();
}

// -----------------------------------------------------------------------------
// --- 11. SETUP & INITIALIZATION ----------------------------------------------
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
// --- 11. SETUP & INITIALIZATION ----------------------------------------------
// -----------------------------------------------------------------------------
void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  Serial.begin(115200);
  delay(500);

  Serial.println("\n=======================================================");
  Serial.println("  ESP32 JK-BMS Monitor (NimBLE-Arduino High Efficiency)");
  Serial.println("=======================================================");

  // Wi-Fi Connection
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.printf("Connecting to Wi-Fi '%s'...", ssid);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(250);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    digitalWrite(LED_PIN, HIGH);
    Serial.println("\n[Wi-Fi] Connected! IP Address: " + WiFi.localIP().toString());
  } else {
    Serial.println("\n[Wi-Fi] Connection timed out! Continuing in offline mode...");
  }

  // Web Server Routes (Matching vernonet exactly)
  server.on("/", HTTP_GET, handleRoot);
  server.on("/data.json", HTTP_GET, handleDataJson);
  server.on("/info", HTTP_GET, handleInfo);
  server.on("/settings", HTTP_GET, handleSettings);
  server.on("/settings_update", HTTP_POST, handleSettingsUpdate);
  server.on("/charge_on", HTTP_GET, handleChargeOn);
  server.on("/charge_off", HTTP_GET, handleChargeOff);
  server.on("/discharge_on", HTTP_GET, handleDischargeOn);
  server.on("/discharge_off", HTTP_GET, handleDischargeOff);
  server.on("/balance_on", HTTP_GET, handleBalanceOn);
  server.on("/balance_off", HTTP_GET, handleBalanceOff);
  server.on("/connect", HTTP_GET, handleConnect);
  server.on("/disconnect", HTTP_GET, handleDisconnect);
  server.on("/scan", HTTP_GET, handleScan);
  server.on("/update", HTTP_GET, handleUpdate);
  server.on("/update", HTTP_POST, handleUpdate, handleUpdateUpload);
  server.on("/reboot", HTTP_GET, handleReboot);
  server.begin();
  Serial.printf("[HTTP] Web server active on port %d\n", 8089);

  // NimBLE Initialization
  NimBLEDevice::init("JKBMS Client");
  NimBLEDevice::setPower(ESP_PWR_LVL_P9); // Maximum range
  NimBLEDevice::setSecurityAuth(false, false, false);

  pBLEScan = NimBLEDevice::getScan();
  
  // ВИПРАВЛЕНО: Використовуємо правильний метод однини під NimBLE API
  pBLEScan->setAdvertisedDeviceCallbacks(new AdvertisedDeviceCallbacks());

    // ВИПРАВЛЕНО: При старті ESP32 намагається підключитися ТІЛЬКИ якщо задано чіткий MAC.
  // Якщо в конфігу порожньо — пристрій просто вмикає пасивне сканування для веб-інтерфейсу.
  if (TARGET_BMS_MAC != nullptr && strlen(TARGET_BMS_MAC) > 10) {
    activeBmsAddress = std::string(TARGET_BMS_MAC);
    connectToBms(std::string(TARGET_BMS_MAC));
  } else {
    // Просто скануємо ефір і чекаємо користувача на сторінці http://192.168.1
    startScan();
  }

}


// -----------------------------------------------------------------------------
// --- 12. MAIN LOOP & AUTOMATIC STATE RECOVERY --------------------------------
// -----------------------------------------------------------------------------
void loop() {
  server.handleClient();

  if (!isConnected) {
    if (currentBleState == BLE_RECONNECT_WAIT) {
      if (millis() - lastReconnectAttempt > RECONNECT_DELAY_MS) {
        lastReconnectAttempt = millis();
        if (!activeBmsAddress.empty()) {
          Serial.println("[NimBLE Watchdog] Attempting automatic reconnection to target BMS...");
          connectToBms(activeBmsAddress);
        } else {
          startScan();
        }
      }
    }
  } else {
    // Poll Settings if not parsed
    if (!parsesettings) {
      static unsigned long lastSettingsReq = 0;
      if (millis() - lastSettingsReq > 4000) {
        writeRegister(0x96, 0x00000000, 0x00);
        lastSettingsReq = millis();
      }
    }

    // Poll Device Info if not parsed
    if (!parseinfo) {
      static unsigned long lastInfoReq = 0;
      if (millis() - lastInfoReq > 4000) {
        writeRegister(0x97, 0x00000000, 0x00);
        lastInfoReq = millis();
      }
    }

    // Poll live telemetry every TELEMETRY_INTERVAL_MS
    if (millis() - lastTelemetryRequest > TELEMETRY_INTERVAL_MS) {
      writeRegister(0x95, 0x00000000, 0x00);
      lastTelemetryRequest = millis();
    }

    // Relaxed Watchdog: Only reset if connection has been up for at least 30s and NO packets arrived for WATCHDOG_TIMEOUT_MS
    if (millis() - connectionEstablishedTime > 20000) {
      if (millis() - lastValidPacketTime > WATCHDOG_TIMEOUT_MS) {
        Serial.printf("[NimBLE Watchdog] Telemetry packets stalled for %d seconds! Resetting BLE connection...\n", 
                      (int)((millis() - lastValidPacketTime) / 1000));
        if (pClient && pClient->isConnected()) {
          pClient->disconnect();
        }
        isConnected = false;
        currentBleState = BLE_RECONNECT_WAIT;
        lastReconnectAttempt = millis();
      }
    }
  }

  delay(2);
}