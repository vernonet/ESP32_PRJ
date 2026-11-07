
/*
 * Скетч ESP32 для читання даних з Jikong (JK) BMS по BLE (Bluetooth Low Energy)
 * ESP32 виступає в ролі BLE Client.
 * ОПТИМІЗОВАНО: Переведено на AJAX/JSON для живого оновлення даних та економії RAM.
 */

#include "BLEDevice.h"
#include <WiFi.h>
#include <WebServer.h>
#include <Update.h>
#include <vector>
#include <map>
#include <string>
#include <cmath>

#define LED_PIN 2

// -----------------------------------------------------------------------------
// --- 2. WIFI & WEB SERVER CONFIGURATION --------------------------------------
// -----------------------------------------------------------------------------
const char *ssid = "your_ssid";
const char *password = "your_pass";

const char *www_username = "your_admin";
const char *www_password = "your_pass";

WebServer server(8089);

// -----------------------------------------------------------------------------
// --- 3. BLE UUIDS & CONSTANTS ------------------------------------------------
// -----------------------------------------------------------------------------
static BLEUUID serviceUUID("0000ffe0-0000-1000-8000-00805f9b34fb");
static BLEUUID charWriteUUID("0000ffe1-0000-1000-8000-00805f9b34fb");
static BLEUUID charNotifyUUID("0000ffe1-0000-1000-8000-00805f9b34fb");
#define CCCD_UUID ((uint16_t)0x2902)
#define FLOAT_TOLERANCE 0.001

// -----------------------------------------------------------------------------
// --- 4. BLE GLOBAL STATE & DEVICE MANAGEMENT ---------------------------------
// -----------------------------------------------------------------------------
struct FoundBMS
{
  std::string address;
  std::string name;
  BLEAdvertisedDevice *pAdvertisedDevice;
};

std::map<std::string, FoundBMS> foundDevices;

static BLEClient *pClient = nullptr;
static BLEAdvertisedDevice *pBmsDevice = nullptr;
static BLERemoteCharacteristic *pWriteCharacteristic = nullptr;
static BLERemoteCharacteristic *pNotifyCharacteristic = nullptr;
static BLEScan *pBLEScan = nullptr;

static bool deviceFound = false;
static bool isConnected = false;
static bool parsesettings = false;
static bool parseinfo = false;

// -----------------------------------------------------------------------------
// --- 5. DATA PROCESSING & FRAME STATE ----------------------------------------
// -----------------------------------------------------------------------------
byte receivedBytes[384];
int frame = 0;
bool received_start = false;
bool received_complete = false;

// -----------------------------------------------------------------------------
// --- 6. BMS DATA STRUCTURES --------------------------------------------------
// -----------------------------------------------------------------------------
struct BMS_Data
{
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
  uint32_t Uptime;
  uint8_t sec, mi, hr, days;
  float Balance_Curr = 0;
  bool Balance = false;
  bool Charge = false;
  bool Discharge = false;
  int Balancing_Action = 0;
} G_data;

struct BMS_Settings
{
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
  long charge_overcurrent_protection_delay = 0;
  long charge_overcurrent_protection_recovery_time = 0;
  long discharge_overcurrent_protection_delay = 0;
  long discharge_overcurrent_protection_recovery_time = 0;
  long short_circuit_protection_recovery_time = 0;
  long short_circuit_protection_delay = 0;
  float charge_overtemperature_protection = 0;
  float charge_overtemperature_protection_recovery = 0;
  float discharge_overtemperature_protection = 0;
  float discharge_overtemperature_protection_recovery = 0;
  float charge_undertemperature_protection = 0;
  float charge_undertemperature_protection_recovery = 0;
  float power_tube_overtemperature_protection = 0;
  float power_tube_overtemperature_protection_recovery = 0;
  float fan_on_temperature = 0;
  float fan_off_temperature = 0;
  long cell_count = 0;
  float total_battery_capacity = 0;
  long bms_mode = 0;
} G_settings;

struct Device_Info
{
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

bool checkAuth()
{
  if (!server.authenticate(www_username, www_password))
  {
    server.requestAuthentication();
    return false;
  }
  return true;
}

uint8_t crc(const uint8_t data[], uint16_t len)
{
  uint8_t crc = 0;
  for (uint16_t i = 0; i < len; i++)
    crc += data[i];
  return crc;
}

void writeRegister(uint8_t address, uint32_t value, uint8_t length)
{
  if (pNotifyCharacteristic == nullptr)
    return;
  uint8_t frame[20] = {0xAA, 0x55, 0x90, 0xEB, address, length};
  frame[6] = value >> 0;
  frame[7] = value >> 8;
  frame[8] = value >> 16;
  frame[9] = value >> 24;
  frame[19] = crc(frame, 19);
  pNotifyCharacteristic->writeValue((uint8_t *)frame, (size_t)sizeof(frame));
}

void parseBMSData()
{
  for (int j = 0, i = 7; i < 38; j++, i += 2)
  {
    G_data.cellVoltage[j] = ((receivedBytes[i] << 8 | receivedBytes[i - 1]) * 0.001);
  }
  for (int j = 0, i = 81; i < 112; j++, i += 2)
  {
    G_data.wireResist[j] = (((int)receivedBytes[i] << 8 | receivedBytes[i - 1]) * 0.001);
  }
  G_data.Average_Cell_Voltage = (((int)receivedBytes[75] << 8 | receivedBytes[74]) * 0.001);
  G_data.Delta_Cell_Voltage = (((int)receivedBytes[77] << 8 | receivedBytes[76]) * 0.001);

  if (receivedBytes[145] == 0xFF)
  {
    G_data.MOS_Temp = ((0xFF << 24 | 0xFF << 16 | receivedBytes[145] << 8 | receivedBytes[144]) * 0.1);
  }
  else
  {
    G_data.MOS_Temp = ((receivedBytes[145] << 8 | receivedBytes[144]) * 0.1);
  }

  G_data.Battery_Voltage = ((receivedBytes[153] << 24 | receivedBytes[152] << 16 | receivedBytes[151] << 8 | receivedBytes[150]) * 0.001);
  G_data.Charge_Current = ((receivedBytes[161] << 24 | receivedBytes[160] << 16 | receivedBytes[159] << 8 | receivedBytes[158]) * 0.001);
  G_data.Battery_Power = G_data.Battery_Voltage * G_data.Charge_Current;

  if (receivedBytes[163] == 0xFF)
  {
    G_data.Battery_T1 = ((0xFF << 24 | 0xFF << 16 | receivedBytes[163] << 8 | receivedBytes[162]) * 0.1);
  }
  else
  {
    G_data.Battery_T1 = ((receivedBytes[163] << 8 | receivedBytes[162]) * 0.1);
  }
  if (receivedBytes[165] == 0xFF)
  {
    G_data.Battery_T2 = ((0xFF << 24 | 0xFF << 16 | receivedBytes[165] << 8 | receivedBytes[164]) * 0.1);
  }
  else
  {
    G_data.Battery_T2 = ((receivedBytes[165] << 8 | receivedBytes[164]) * 0.1);
  }

  if ((receivedBytes[171] & 0xF0) == 0x0)
  {
    G_data.Balance_Curr = ((receivedBytes[171] << 8 | receivedBytes[170]) * 0.001);
  }
  else if ((receivedBytes[171] & 0xF0) == 0xF0)
  {
    G_data.Balance_Curr = (((receivedBytes[171] & 0x0F) << 8 | receivedBytes[170]) * -0.001);
  }

  G_data.Balancing_Action = receivedBytes[172];
  G_data.Percent_Remain = (receivedBytes[173]);
  G_data.Capacity_Remain = ((receivedBytes[177] << 24 | receivedBytes[176] << 16 | receivedBytes[175] << 8 | receivedBytes[174]) * 0.001);
  G_data.Nominal_Capacity = ((receivedBytes[181] << 24 | receivedBytes[180] << 16 | receivedBytes[179] << 8 | receivedBytes[178]) * 0.001);
  G_data.Cycle_Count = ((receivedBytes[185] << 24 | receivedBytes[184] << 16 | receivedBytes[183] << 8 | receivedBytes[182]));
  G_data.Cycle_Capacity = ((receivedBytes[189] << 24 | receivedBytes[188] << 16 | receivedBytes[187] << 8 | receivedBytes[186]) * 0.001);

  G_data.Uptime = receivedBytes[196] << 16 | receivedBytes[195] << 8 | receivedBytes[194];
  G_data.sec = G_data.Uptime % 60;
  G_data.Uptime /= 60;
  G_data.mi = G_data.Uptime % 60;
  G_data.Uptime /= 60;
  G_data.hr = G_data.Uptime % 24;
  G_data.days = G_data.Uptime / 24;

  G_data.Charge = (receivedBytes[198] > 0);
  G_data.Discharge = (receivedBytes[199] > 0);
  G_data.Balance = (receivedBytes[201] > 0);
}

void parseBMSSettings()
{
  G_settings.cell_voltage_undervoltage_protection = (((uint32_t)receivedBytes[13] << 24 | (uint32_t)receivedBytes[12] << 16 | (uint32_t)receivedBytes[11] << 8 | (uint32_t)receivedBytes[10]) * 0.001);
  G_settings.cell_voltage_undervoltage_recovery = (((uint32_t)receivedBytes[17] << 24 | (uint32_t)receivedBytes[16] << 16 | (uint32_t)receivedBytes[15] << 8 | (uint32_t)receivedBytes[14]) * 0.001);
  G_settings.cell_voltage_overvoltage_protection = (((uint32_t)receivedBytes[21] << 24 | (uint32_t)receivedBytes[20] << 16 | (uint32_t)receivedBytes[19] << 8 | (uint32_t)receivedBytes[18]) * 0.001);
  G_settings.cell_voltage_overvoltage_recovery = (((uint32_t)receivedBytes[25] << 24 | (uint32_t)receivedBytes[24] << 16 | (uint32_t)receivedBytes[23] << 8 | (uint32_t)receivedBytes[22]) * 0.001);
  G_settings.balance_trigger_voltage = (((uint32_t)receivedBytes[29] << 24 | (uint32_t)receivedBytes[28] << 16 | (uint32_t)receivedBytes[27] << 8 | (uint32_t)receivedBytes[26]) * 0.001);
  G_settings.full_charge_voltage = (((uint32_t)receivedBytes[33] << 24 | (uint32_t)receivedBytes[32] << 16 | (uint32_t)receivedBytes[31] << 8 | (uint32_t)receivedBytes[30]) * 0.001);
  G_settings.nominal_voltage = (((uint32_t)receivedBytes[37] << 24 | (uint32_t)receivedBytes[36] << 16 | (uint32_t)receivedBytes[35] << 8 | (uint32_t)receivedBytes[34]) * 0.001);
  G_settings.float_charge_voltage = (((uint32_t)receivedBytes[41] << 24 | (uint32_t)receivedBytes[40] << 16 | (uint32_t)receivedBytes[39] << 8 | (uint32_t)receivedBytes[38]) * 0.001);
  G_settings.balance_precision = (((uint32_t)receivedBytes[45] << 24 | (uint32_t)receivedBytes[44] << 16 | (uint32_t)receivedBytes[43] << 8 | (uint32_t)receivedBytes[42]) * 0.001);
  G_settings.power_off_voltage = (((uint32_t)receivedBytes[49] << 24 | (uint32_t)receivedBytes[48] << 16 | (uint32_t)receivedBytes[47] << 8 | (uint32_t)receivedBytes[46]) * 0.001);

  G_settings.max_charge_current = (((uint32_t)receivedBytes[53] << 24 | (uint32_t)receivedBytes[52] << 16 | (uint32_t)receivedBytes[51] << 8 | (uint32_t)receivedBytes[50]) * 0.001);
  G_settings.charge_overcurrent_protection_delay = (((uint32_t)receivedBytes[57] << 24 | (uint32_t)receivedBytes[56] << 16 | (uint32_t)receivedBytes[55] << 8 | (uint32_t)receivedBytes[54]));
  G_settings.charge_overcurrent_protection_recovery_time = (((uint32_t)receivedBytes[61] << 24 | (uint32_t)receivedBytes[60] << 16 | (uint32_t)receivedBytes[59] << 8 | (uint32_t)receivedBytes[58]));
  G_settings.max_discharge_current = (((uint32_t)receivedBytes[65] << 24 | (uint32_t)receivedBytes[64] << 16 | (uint32_t)receivedBytes[63] << 8 | (uint32_t)receivedBytes[62]) * 0.001);
  G_settings.discharge_overcurrent_protection_delay = (((uint32_t)receivedBytes[69] << 24 | (uint32_t)receivedBytes[68] << 16 | (uint32_t)receivedBytes[67] << 8 | (uint32_t)receivedBytes[66]));
  G_settings.discharge_overcurrent_protection_recovery_time = (((uint32_t)receivedBytes[73] << 24 | (uint32_t)receivedBytes[72] << 16 | (uint32_t)receivedBytes[71] << 8 | (uint32_t)receivedBytes[70]));
  G_settings.short_circuit_protection_recovery_time = (((uint32_t)receivedBytes[77] << 24 | (uint32_t)receivedBytes[76] << 16 | (uint32_t)receivedBytes[75] << 8 | (uint32_t)receivedBytes[74]));
  G_settings.max_balance_current = (((uint32_t)receivedBytes[81] << 24 | (uint32_t)receivedBytes[80] << 16 | (uint32_t)receivedBytes[79] << 8 | (uint32_t)receivedBytes[78]) * 0.001);

  G_settings.charge_overtemperature_protection = (int16_t)((receivedBytes[83] << 8) | receivedBytes[82]) * 0.1;
  G_settings.charge_overtemperature_protection_recovery = (int16_t)((receivedBytes[87] << 8) | receivedBytes[86]) * 0.1;
  G_settings.discharge_overtemperature_protection = (int16_t)((receivedBytes[91] << 8) | receivedBytes[90]) * 0.1;
  G_settings.discharge_overtemperature_protection_recovery = (int16_t)((receivedBytes[95] << 8) | receivedBytes[94]) * 0.1;
  G_settings.charge_undertemperature_protection = (int16_t)((receivedBytes[99] << 8) | receivedBytes[98]) * 0.1;
  G_settings.charge_undertemperature_protection_recovery = (int16_t)((receivedBytes[103] << 8) | receivedBytes[102]) * 0.1;
  G_settings.power_tube_overtemperature_protection = (int16_t)((receivedBytes[107] << 8) | receivedBytes[106]) * 0.1;
  G_settings.power_tube_overtemperature_protection_recovery = (int16_t)((receivedBytes[111] << 8) | receivedBytes[110]) * 0.1;

  G_settings.cell_count = (((uint32_t)receivedBytes[117] << 24 | (uint32_t)receivedBytes[116] << 16 | (uint32_t)receivedBytes[115] << 8 | (uint32_t)receivedBytes[114]));
  G_settings.bms_mode = (((uint32_t)receivedBytes[121] << 24 | (uint32_t)receivedBytes[120] << 16 | (uint32_t)receivedBytes[119] << 8 | (uint32_t)receivedBytes[118]));
  G_settings.fan_on_temperature = (((uint32_t)receivedBytes[125] << 24 | (uint32_t)receivedBytes[124] << 16 | (uint32_t)receivedBytes[123] << 8 | (uint32_t)receivedBytes[122]) * 0.1);
  G_settings.fan_off_temperature = (((uint32_t)receivedBytes[129] << 24 | (uint32_t)receivedBytes[128] << 16 | (uint32_t)receivedBytes[127] << 8 | (uint32_t)receivedBytes[126]) * 0.1);
  G_settings.total_battery_capacity = (((uint32_t)receivedBytes[133] << 24 | (uint32_t)receivedBytes[132] << 16 | (uint32_t)receivedBytes[131] << 8 | (uint32_t)receivedBytes[130]) * 0.001);
  G_settings.short_circuit_protection_delay = (((uint32_t)receivedBytes[137] << 24 | (uint32_t)receivedBytes[136] << 16 | (uint32_t)receivedBytes[135] << 8 | (uint32_t)receivedBytes[134]) * 1);
  G_settings.balance_starting_voltage = (((uint32_t)receivedBytes[141] << 24 | (uint32_t)receivedBytes[140] << 16 | (uint32_t)receivedBytes[139] << 8 | (uint32_t)receivedBytes[138]) * 0.001);

  parsesettings = true;
}

void parseDeviceInfo()
{
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
}

void processBMSFrame(uint8_t* pData, size_t length) {
  // Перевірка на початок нового кадру даних (AA 55 EB 90)
  if (pData[0] == 0x55 && pData[1] == 0xAA && pData[2] == 0xEB && pData[3] == 0x90) {
    
    // Якщо у буфері вже лежать дані попереднього кадру і їх достатньо для парсингу
    if (frame > 100) {
      uint8_t p_type = receivedBytes[4];
      if (!parseinfo && p_type == 0x03) {
        parseDeviceInfo();
        Serial.println("-> Розпарсено Info!");
      } else if (!parsesettings && p_type == 0x01) {
        parseBMSSettings();
        Serial.println("-> Розпарсено Settings!");
      } else if (p_type == 0x02) {
        parseBMSData();
        Serial.printf("-> Розпарсено Telemetry! (Отримано %d байт)\n", frame);
      }
    }
    
    // Скидаємо лічильник для накопичення нового кадру
    frame = 0;
    received_complete = false;
    received_start = true;

    for (size_t i = 0; i < length; i++) {
      if (frame < 384) receivedBytes[frame++] = pData[i];
    }
  } else if (received_start) {
    // Накопичуємо шматочки одного й того самого кадру в буфер
    for (size_t i = 0; i < length; i++) {
      if (frame < 384) { receivedBytes[frame++] = pData[i]; }
    }
    
    // Оскільки пакети телеметрії Jikong довгі (близько 300 байт),
    // якщо ми вже набрали повний пакет, можемо розпарсити його превентивно, не чекаючи наступного заголовка
    if (receivedBytes[4] == 0x02 && frame >= 280) {
      parseBMSData();
      received_complete = true;
    }
  }
}




static void notifyCallback(BLERemoteCharacteristic *pBLERemoteCharacteristic, uint8_t *pData, size_t length, bool isNotify)
{
  if (length < 10)
    return;
  processBMSFrame(pData, length);
}

class MyClientCallback : public BLEClientCallbacks
{
  void onConnect(BLEClient *pClient) { isConnected = true; }
  void onDisconnect(BLEClient *pClient)
  {
    isConnected = false;
    deviceFound = false;
    pBmsDevice = nullptr;
    pWriteCharacteristic = nullptr;
    pNotifyCharacteristic = nullptr;
    Serial.println("Disconnected from BMS.");
  }
};

class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks
{
  void onResult(BLEAdvertisedDevice advertisedDevice)
  {
    if (advertisedDevice.getName().length() > 0 && advertisedDevice.haveServiceUUID() && advertisedDevice.getServiceUUID().equals(serviceUUID))
    {
      std::string deviceAddress = advertisedDevice.getAddress().toString();
      std::string deviceName = advertisedDevice.getName();
      if (foundDevices.find(deviceAddress) == foundDevices.end())
      {
        FoundBMS bms;
        bms.address = deviceAddress;
        bms.name = deviceName;
        bms.pAdvertisedDevice = new BLEAdvertisedDevice(advertisedDevice);
        foundDevices[deviceAddress] = bms;
      }
    }
  }
};

void bleScanTask()
{
  for (auto &pair : foundDevices)
  {
    delete pair.second.pAdvertisedDevice;
  }
  foundDevices.clear();
  pBLEScan->start(5, false);
}

bool connectToServer(BLEAddress address)
{
  if (pClient != nullptr)
  {
    if (pClient->isConnected())
      pClient->disconnect();
    delete pClient;
    pClient = nullptr;
  }
  pClient = BLEDevice::createClient();
  pClient->setClientCallbacks(new MyClientCallback());
  if (!pClient->connect(address))
    return false;

  BLERemoteService *pRemoteService = pClient->getService(serviceUUID);
  if (pRemoteService == nullptr)
  {
    pClient->disconnect();
    return false;
  }
  pWriteCharacteristic = pRemoteService->getCharacteristic(charWriteUUID);
  if (pWriteCharacteristic == nullptr)
  {
    pClient->disconnect();
    return false;
  }
  pNotifyCharacteristic = pRemoteService->getCharacteristic(charNotifyUUID);
  if (pNotifyCharacteristic == nullptr)
  {
    pClient->disconnect();
    return false;
  }

    // Включаємо сповіщення (Notify) — оптимізовано під Jikong BMS
  // Включаємо сповіщення (Notify) — НИЗЬКОРІВНЕВИЙ ОБХІД БАГУ ДЛЯ JIKONG BMS
  if (pNotifyCharacteristic->canNotify()) {
    Serial.println("Реєстрація обробника сповіщень...");
    pNotifyCharacteristic->registerForNotify(notifyCallback, false); // false, щоб не запускати дефектний стандартний сканер дескрипторів

    Serial.println("Примусова низькорівнева підписка на CCCD (0x2902)...");
    
    // Отримуємо внутрішні системні ідентифікатори BLE-стеку ESP32
    uint16_t char_handle = pNotifyCharacteristic->getHandle();
    uint16_t gattc_if = pClient->getGattcIf();
    uint16_t conn_id = pClient->getConnId();
    
    // Складаємо сирий пакет активації сповіщень (Little-Endian: 0x0001)
    uint16_t notify_en_value = 0x0001; 

    // Викликаємо пряму функцію ядра ESP-IDF для запису в дескриптор (Handle характеристики + 1)
    esp_err_t err  = esp_ble_gattc_write_char_descr(
        gattc_if,
        conn_id,
        char_handle + 1, // Хандл дескриптора CCCD у плат Jikong завжди дорівнює Handle характеристики + 1
        sizeof(notify_en_value),
        (uint8_t *)&notify_en_value,
        ESP_GATT_WRITE_TYPE_RSP,
        ESP_GATT_AUTH_REQ_NONE
    );

    if (err == ESP_OK) {
        Serial.println("Підписка на рівні ядра завершена успішно!");
    } else {
        Serial.printf("Помилка низькорівневого запису: 0x%X\n", err);
    }
  }


  parsesettings = false;
  parseinfo = false;
  received_complete = false;
  frame = 0;

  writeRegister(0x97, 0x00000000, 0x00);
  delay(300);
  writeRegister(0x96, 0x00000000, 0x00);
  delay(300);
  writeRegister(0x95, 0x00000000, 0x00);
  delay(400);
  return true;
}

void handleDataJson() {
  if (!checkAuth()) return;

  String json;
  json.reserve(1024);

  json = "{";
  json += "\"connected\":" + String(isConnected ? "true" : "false") + ",";
  json += "\"volt\":" + String(G_data.Battery_Voltage, 2) + ",";
  json += "\"curr\":" + String(G_data.Charge_Current, 2) + ",";
  json += "\"pow\":" + String(G_data.Battery_Power, 1) + ",";
  json += "\"soc\":" + String(G_data.Percent_Remain) + ",";
  json += "\"delta\":" + String(G_data.Delta_Cell_Voltage, 3) + ",";
  json += "\"t1\":" + String(G_data.Battery_T1, 1) + ",";
  json += "\"t2\":" + String(G_data.Battery_T2, 1) + ",";
  json += "\"tmos\":" + String(G_data.MOS_Temp, 1) + ",";
  json += "\"bcurr\":" + String(G_data.Balance_Curr, 3) + ",";
  json += "\"chg\":" + String(G_data.Charge ? "true" : "false") + ",";
  json += "\"dsg\":" + String(G_data.Discharge ? "true" : "false") + ",";
  json += "\"bal\":" + String(G_data.Balance ? "true" : "false") + ",";

  json += "\"cells\":[";
  
  int c_count = G_settings.cell_count;
  if (c_count <= 0 || c_count > 32) {
    c_count = 16; 
  }
  
  for (int i = 0; i < c_count; i++) {
    json += "{\"v\":" + String(G_data.cellVoltage[i], 3) + ",\"r\":" + String(G_data.wireResist[i], 3) + "}";
    if (i < (c_count - 1)) {
      json += ",";
    }
  }
  json += "]}";

  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json; charset=UTF-8", json);
}



void handleRoot() {
  if (!checkAuth()) return;

  if (!isConnected) {
    String html = "<!DOCTYPE html><html><head><meta charset='UTF-8'><title>BMS ВІДКЛЮЧЕНО</title>";
    html += "<style>body{font-family:Arial;background:#f4f4f9;text-align:center;margin-top:50px}.btn{display:inline-block;padding:10px 20px;margin:10px;background:#007bff;color:#fff;text-decoration:none;border-radius:5px}</style></head>";
    html += "<body><h2>⚡️ Статус BMS: ВІДКЛЮЧЕНО</h2><p>IP: " + WiFi.localIP().toString() + "</p>";
    
    // ДОДАНО: Кнопка OTA на екран відключеного стану (як в оригіналі)
    html += "<p style='text-align: center;'>";
    html += "<a href='/reboot' onclick=\"return confirm('Перезавантажити ESP32?');\" class='btn' style='background:#dc3545'>🔄 REBOOT</a>";
    html += "<a href='/update' class='btn' style='background:#6c757d'>📥 OTA UPDATE</a>";
    html += "<a href='/scan' class='btn'>🔍 СКАНУВАТИ ТА ВИБРАТИ BMS</a></p>";
    html += "</body></html>";
    server.send(200, "text/html; charset=UTF-8", html);
    return;
  }

  String html = "<!DOCTYPE html><html><head><meta charset='UTF-8'><meta name='viewport' content='width=device-width,initial-scale=1.0'><title>Дані Jikong BMS</title>";
  html += "<style>body{font-family:Arial,sans-serif;background:#f4f4f9;margin:0;padding:10px}td,th{border:1px solid #ddd;padding:8px;text-align:left}th{background:#007bff;color:#fff}";
  html += ".container{max-width:700px;margin:auto;background:#fff;padding:15px;border-radius:8px;box-shadow:0 2px 4px rgba(0,0,0,.1)}table{width:100%;border-collapse:collapse;margin-top:10px}";
  html += ".btn{display:inline-block;padding:6px 12px;background:#007bff;color:#fff;text-decoration:none;border-radius:4px;margin:2px;font-size:14px}.on{background:#28a745}.off{background:#dc3545}</style></head>";
  html += "<body><div class='container'><h2>⚡️ Живі дані Jikong BMS</h2>";
  
  // ВІДНОВЛЕНО ТА ПОКРАЩЕНО: Красиве текстове меню з кнопкою OTA на головному екрані
  html += "<p style='text-align:center'>";
  html += "  <a href='/update' class='btn' style='background:#28a745'>📥 OTA UPDATE</a>";
  html += "  <a href='/settings' class='btn'>⚙️ Налаштування</a>";
  html += "  <a href='/info' class='btn' style='background:#17a2b8'>ℹ️ Інфо</a>";
  html += "  <a href='/disconnect' class='btn off'>❌ Вимкнути BLE</a>";
  html += "</p>";
  
  html += "<table><tr><th>Параметр</th><th>Значення</th><th>Од.</th></tr>";
  html += "<tr><td>Напруга батареї</td><td id='volt'>-</td><td>V</td></tr>";
  html += "<tr><td>Струм заряду/розряду</td><td id='curr'>-</td><td>A</td></tr>";
  html += "<tr><td>Потужність</td><td id='pow'>-</td><td>W</td></tr>";
  html += "<tr><td>Залишок заряду (SOC)</td><td id='soc'>-</td><td>%</td></tr>";
  html += "<tr><td>Дельта комірок</td><td id='delta'>-</td><td>V</td></tr>";
  html += "<tr><td>Температура T1 / T2</td><td><span id='t1'>-</span> / <span id='t2'>-</span></td><td>°C</td></tr>";
  html += "<tr><td>Температура MOS</td><td id='tmos'>-</td><td>°C</td></tr>";
  html += "<tr><td>Заряд</td><td id='s_chg'>-</td><td id='b_chg'>-</td></tr>";
  html += "<tr><td>Розряд</td><td id='s_dsg'>-</td><td id='b_dsg'>-</td></tr>";
  html += "<tr><td>Баланс</td><td id='s_bal'>-</td><td id='b_bal'>-</td></tr>";
  html += "</table><h2>🔬 Напруги комірок</h2><table id='cell_table'></table></div>";

  html += "<script>";
  html += "function update(){";
  html += "  fetch('/data.json').then(r=>r.json()).then(d=>{";
  html += "    if(!d.connected){window.location.reload();return;}";
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
  
  html += "    if(document.getElementById('s_chg')) document.getElementById('s_chg').innerHTML = d.chg ? '<span style=\"color:green\">🟢 УВІМК</span>' : '<span style=\"color:red\">🔴 ВИМК</span>';";
  html += "    if(document.getElementById('b_chg')) document.getElementById('b_chg').innerHTML = d.chg ? '<a href=\"#\" onclick=\"fetch(\\'/charge_off\\'); return false;\" class=\"btn off\">ВИМК</a>' : '<a href=\"#\" onclick=\"fetch(\\'/charge_on\\'); return false;\" class=\"btn on\">УВІМК</a>';";
  html += "    if(document.getElementById('s_dsg')) document.getElementById('s_dsg').innerHTML = d.dsg ? '<span style=\"color:green\">🟢 УВІМК</span>' : '<span style=\"color:red\">🔴 ВИМК</span>';";
  html += "    if(document.getElementById('b_dsg')) document.getElementById('b_dsg').innerHTML = d.dsg ? '<a href=\"#\" onclick=\"fetch(\\'/discharge_off\\'); return false;\" class=\"btn off\">ВИМК</a>' : '<a href=\"#\" onclick=\"fetch(\\'/discharge_on\\'); return false;\" class=\"btn on\">УВІМК</a>';";
  
  html += "    let bc = d.bcurr || 0;";
  html += "    if(document.getElementById('s_bal')) {";
  html += "      if(d.bal) {";
  html += "        document.getElementById('s_bal').innerHTML = '<span style=\"color:green\">🟢 Активний</span> (' + bc.toFixed(3) + ' A) <a href=\"#\" onclick=\"fetch(\\'/balance_off\\'); return false;\" class=\"btn off\">ВИМК</a>';";
  html += "      } else {";
  html += "        document.getElementById('s_bal').innerHTML = '⚪ Ні <a href=\"#\" onclick=\"fetch(\\'/balance_on\\'); return false;\" class=\"btn on\">УВІМК</a>';";
  html += "      }";
  html += "    }";
  
  html += "    if(d.cells && d.cells.length > 0 && document.getElementById('cell_table')) {";
  html += "      let t = '<tr><th>Комірка</th><th>Напруга (V)</th><th>Опір (Ом)</th></tr>';";
  html += "      d.cells.forEach((cell, i) => {";
  html += "        let cv = cell.v || 0; let cr = cell.r || 0;";
  html += "        t += '<tr><td>' + (i + 1) + '</td><td>' + cv.toFixed(3) + '</td><td>' + cr.toFixed(3) + '</td></tr>';";
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


void handleReboot()
{
  if (!checkAuth())
    return;
  server.send(200, "text/plain", "Rebooting...");
  delay(500);
  ESP.restart();
}

void handleInfo()
{
  if (!checkAuth())
    return;
  String html = "<!DOCTYPE html><html><head><meta charset='UTF-8'><meta name='viewport' content='width=device-width,initial-scale=1'><title>Device Info</title>";
  html += "<style>body{font:15px Arial;background:#f4f4f9;margin:20px}table{width:100%;border-collapse:collapse;margin-top:15px}td,th{padding:8px;text-align:left;border-bottom:1px solid #ddd}th{background:#f8f9fa}.btn{display:inline-block;padding:8px 16px;background:#007bff;color:#fff;text-decoration:none;border-radius:4px}</style></head>";
  html += "<body><div style='max-width:500px;margin:auto;background:#fff;padding:20px;border-radius:8px;box-shadow:0 2px 4px rgba(0,0,0,.1)'>";
  html += "<h2>ℹ️ Інформація</h2><table><tr><th>Параметр</th><th>Значення</th></tr>";

  auto addR = [&](const char *l, const char *v, const char *s = "")
  {
    return "<tr><td>" + String(l) + "</td><td " + String(s) + ">" + String(v) + "</td></tr>";
  };
  html += addR("Пристрій", G_info.deviceName.c_str());
  html += addR("Серійний №", G_info.serialNumber.c_str());
  html += addR("Версія HW/SW", (G_info.hardwareVersion + " / " + G_info.softwareVersion).c_str());
  html += addR("Компіляція", (__DATE__ " " __TIME__), "style='color:#28a745;font-weight:bold'");
  html += addR("Vendor ID", G_info.vendorID.c_str());
  html += addR("Дата виготовлення", G_info.manufacturingDate.c_str());
  html += addR("Device Passcode", G_info.devicePasscode.c_str());
  html += addR("Setup Passcode", G_info.setupPasscode.c_str());
  html += addR("Passcode", G_info.passcode.c_str());
  html += addR("Користувач", G_info.userData.c_str());
  html += "</table><br><p style='text-align:center'><a href='/' class='btn'>◀️ ГОЛОВНА</a></p></div></body></html>";

  server.sendHeader("Connection", "close");
  server.send(200, "text/html; charset=UTF-8", html);
}

void handleSettings() {
  if (!checkAuth()) return;
  if (!isConnected) { 
    server.sendHeader("Location", "/"); 
    server.send(302, "text/plain", ""); 
    return; 
  }
  
  // Повідомляємо браузеру про передачу даних частинами
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/html; charset=UTF-8", "");

  String ch = "";
  ch.reserve(1024);

  // --- ЧАНК 1: ШАПКА ТА СТИЛІ ---
  ch = "<!DOCTYPE html><html><head><meta charset='UTF-8'><meta name='viewport' content='width=device-width,initial-scale=1'><title>Налаштування Jikong BMS</title>";
  ch += "<style>body{font-family:Arial,sans-serif;background:#f4f4f9;padding:10px}table{width:100%;border-collapse:collapse;margin-top:10px}td,th{border:1px solid #ddd;padding:6px;text-align:left}th{background:#007bff;color:#fff}.group th{background:#4CAF50}input{width:90%;padding:4px;border:1px solid #ccc;border-radius:4px}.btn{display:block;width:100%;padding:12px;background:#007bff;color:#fff;border:none;border-radius:5px;font-size:16px;cursor:pointer;margin-top:20px}</style></head>";
  ch += "<body><div style='max-width:750px;margin:auto;background:#fff;padding:15px;border-radius:8px;box-shadow:0 2px 4px rgba(0,0,0,.1)'>";
  ch += "<h2>⚙️ Налаштування Jikong BMS</h2><a href='/'>⬅️ На головну</a><br><br><form method='POST' action='/settings_update'><table><tr><th>Параметр</th><th>Поточне</th><th>Нове</th></tr>";
  
  ch += "<tr class='group'><th colspan='3'>Параметри батареї</th></tr>";
  ch += "<tr><td>Номінальна ємність (Ah)</td><td>" + String(G_settings.total_battery_capacity, 3) + "</td><td><input type='number' name='total_battery_capacity' value='" + String(G_settings.total_battery_capacity, 3) + "' step='0.001' required></td></tr>";
  ch += "<tr><td>Кількість комірок</td><td>" + String(G_settings.cell_count) + "</td><td><input type='number' name='cell_count' value='" + String(G_settings.cell_count) + "' step='1' required></td></tr>";
  ch += "<tr><td>Режим BMS</td><td>" + String(G_settings.bms_mode) + "</td><td><input type='number' name='bms_mode' value='" + String(G_settings.bms_mode) + "' step='1' required></td></tr>";
  
  server.sendContent(ch); 
  ch = ""; // Очищаємо буфер перед наступною частиною

  // --- ЧАНК 2: НАПРУГИ КОМІРОК ---
  ch += "<tr class='group'><th colspan='3'>Напруги комірок (V)</th></tr>";
  ch += "<tr><td>Захист від перенапруги (ОВР)</td><td>" + String(G_settings.cell_voltage_overvoltage_protection, 3) + "</td><td><input type='number' name='cell_voltage_overvoltage_protection' value='" + String(G_settings.cell_voltage_overvoltage_protection, 3) + "' step='0.001' required></td></tr>";
  ch += "<tr><td>Захист від низької напруги (УВР)</td><td>" + String(G_settings.cell_voltage_undervoltage_protection, 3) + "</td><td><input type='number' name='cell_voltage_undervoltage_protection' value='" + String(G_settings.cell_voltage_undervoltage_protection, 3) + "' step='0.001' required></td></tr>";
  ch += "<tr><td>Відновлення ОВР</td><td>" + String(G_settings.cell_voltage_overvoltage_recovery, 3) + "</td><td><input type='number' name='cell_voltage_overvoltage_recovery' value='" + String(G_settings.cell_voltage_overvoltage_recovery, 3) + "' step='0.001' required></td></tr>";
  ch += "<tr><td>Відновлення УВР</td><td>" + String(G_settings.cell_voltage_undervoltage_recovery, 3) + "</td><td><input type='number' name='cell_voltage_undervoltage_recovery' value='" + String(G_settings.cell_voltage_undervoltage_recovery, 3) + "' step='0.001' required></td></tr>";
  ch += "<tr><td>Напруга вимкнення (Power Off)</td><td>" + String(G_settings.power_off_voltage, 3) + "</td><td><input type='number' name='power_off_voltage' value='" + String(G_settings.power_off_voltage, 3) + "' step='0.001' required></td></tr>";
  
  ch += "<tr class='group'><th colspan='3'>Зарядні / Номінальні напруги (V)</th></tr>";
  ch += "<tr><td>Повна напруга заряду</td><td>" + String(G_settings.full_charge_voltage, 3) + "</td><td><input type='number' name='full_charge_voltage' value='" + String(G_settings.full_charge_voltage, 3) + "' step='0.001' required></td></tr>";
  ch += "<tr><td>Номінальна напруга</td><td>" + String(G_settings.nominal_voltage, 3) + "</td><td><input type='number' name='nominal_voltage' value='" + String(G_settings.nominal_voltage, 3) + "' step='0.001' required></td></tr>";
  ch += "<tr><td>Підтримуюча напруга заряду</td><td>" + String(G_settings.float_charge_voltage, 3) + "</td><td><input type='number' name='float_charge_voltage' value='" + String(G_settings.float_charge_voltage, 3) + "' step='0.001' required></td></tr>";

  server.sendContent(ch); 
  ch = "";

  // --- ЧАНК 3: БАЛАНСУВАННЯ ТА СТРУМИ ---
  ch += "<tr class='group'><th colspan='3'>Балансування</th></tr>";
  ch += "<tr><td>Напруга запуску балансування (V)</td><td>" + String(G_settings.balance_trigger_voltage, 3) + "</td><td><input type='number' name='balance_trigger_voltage' value='" + String(G_settings.balance_trigger_voltage, 3) + "' step='0.001' required></td></tr>";
  ch += "<tr><td>Мінімальна напруга запуску (V)</td><td>" + String(G_settings.balance_starting_voltage, 3) + "</td><td><input type='number' name='balance_starting_voltage' value='" + String(G_settings.balance_starting_voltage, 3) + "' step='0.001' required></td></tr>";
  ch += "<tr><td>Макс. струм балансування (A)</td><td>" + String(G_settings.max_balance_current, 3) + "</td><td><input type='number' name='max_balance_current' value='" + String(G_settings.max_balance_current, 3) + "' step='0.001' required></td></tr>";
  ch += "<tr><td>Дельта напруги для зупинки (V)</td><td>" + String(G_settings.balance_precision, 3) + "</td><td><input type='number' name='balance_precision' value='" + String(G_settings.balance_precision, 3) + "' step='0.001' required></td></tr>";

  ch += "<tr class='group'><th colspan='3'>СТРУМИ (А)</th></tr>";
  ch += "<tr><td>Макс. струм заряду (A)</td><td>" + String(G_settings.max_charge_current, 3) + "</td><td><input type='number' name='max_charge_current' value='" + String(G_settings.max_charge_current, 3) + "' step='0.001' required></td></tr>";
  ch += "<tr><td>Макс. струм розряду (A)</td><td>" + String(G_settings.max_discharge_current, 3) + "</td><td><input type='number' name='max_discharge_current' value='" + String(G_settings.max_discharge_current, 3) + "' step='0.001' required></td></tr>";

  ch += "</table><button type='submit' class='btn'>💾 ЗБЕРЕГТИ ВСІ НАЛАШТУВАННЯ</button></form></div></body></html>";
  
  server.sendContent(ch);
  server.sendContent(""); // Завершуємо передачу даних чанками
}


void handleScan()
{
  if (!checkAuth())
    return;
  bleScanTask();
  String html = "<!DOCTYPE html><html><head><meta charset='UTF-8'><title>Сканування</title><style>body{font-family:Arial;background:#f4f4f9;padding:10px}table{width:100%;border-collapse:collapse}td,th{border:1px solid #ddd;padding:8px}th{background:#007bff;color:#fff}.btn{padding:5px 10px;color:#fff;background:#28a745;text-decoration:none;border-radius:4px}</style></head>";
  html += "<body><div style='max-width:600px;margin:auto;background:#fff;padding:15px'><h2>🔍 Сканування BMS</h2><table><tr><th>Ім'я пристрою</th><th>MAC</th><th>Дія</th></tr>";
  if (foundDevices.empty())
  {
    html += "<tr><td colspan='3'>Пристроїв не знайдено. Оновіть сторінку.</td></tr>";
  }
  else
  {
    for (const auto &pair : foundDevices)
    {
      html += "<tr><td>" + String(pair.second.name.c_str()) + "</td><td>" + String(pair.first.c_str()) + "</td><td><a href='/connect?mac=" + String(pair.first.c_str()) + "' class='btn'>Підключитись</a></td></tr>";
    }
  }
  html += "</table><br><a href='/'>◀️ На головну</a></div></body></html>";
  server.sendHeader("Connection", "close");
  server.send(200, "text/html; charset=UTF-8", html);
}

void handleSelectDevice() { handleScan(); }

void handleConnect()
{
  if (!checkAuth())
    return;
  if (server.hasArg("mac"))
  {
    std::string mac_addr = server.arg("mac").c_str();
    auto it = foundDevices.find(mac_addr);
    if (it != foundDevices.end())
    {
      pBmsDevice = it->second.pAdvertisedDevice;
      deviceFound = true;
      if (connectToServer(pBmsDevice->getAddress()))
      {
        server.sendHeader("Location", "/");
        server.send(302, "text/plain", "");
        return;
      }
    }
  }
  server.send(500, "text/plain; charset=UTF-8", "Помилка підключення.");
}

void handleDisconnect()
{
  if (!checkAuth())
    return;
  if (pClient != nullptr && pClient->isConnected())
    pClient->disconnect();
  server.sendHeader("Location", "/");
  server.send(302, "text/plain", "");
}

void handleUpdate() {
  if (!checkAuth()) return;
  server.send(200, "text/html",
              "<!DOCTYPE html><html><head><title>OTA Update</title><meta charset='UTF-8'>"
              "<style>body{font-family: Arial; text-align: center;} input{padding: 10px; margin: 5px; border-radius: 5px;} .file-upload{border: 1px solid #ccc; padding: 20px; width: 300px; margin: 50px auto;}</style></head>"
              "<body><h1>OTA Update</h1><p>Оберіть файл прошивки (.bin):</p>"
              "<form method='POST' action='/update' enctype='multipart/form-data' class='file-upload'>"
              "<input type='file' name='firmware'><input type='submit' value='Оновити'></form>"
              "<a href='/'>&#9664; На головну</a></body></html>");
}


void handleUpdateUpload() {
  if (!checkAuth()) return; //
  HTTPUpload& upload = server.upload(); //
  
  if (upload.status == UPLOAD_FILE_START) { //
    Serial.printf("Отримання файлу прошивки: %s\n", upload.filename.c_str()); //
    
    // --- ДОДАНО: Розриваємо зв'язок з BMS для вивільнення пам'яті (RAM) та стабільного OTA по WiFi ---
    if (pClient != nullptr && pClient->isConnected()) {
      Serial.println("OTA: Відключаємо BLE Client для очищення пам'яті...");
      pClient->disconnect();
    }
    isConnected = false;
    pWriteCharacteristic = nullptr;
    pNotifyCharacteristic = nullptr;
    delay(300); // Коротка пауза для повного вивантаження BLE-стеку з Heap RAM
    
    if (!Update.begin(UPDATE_SIZE_UNKNOWN)) { //
      Update.printError(Serial); //
    }
  } 
  else if (upload.status == UPLOAD_FILE_WRITE) { //
    if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) { //
      Update.printError(Serial); //
    }
  } 
  else if (upload.status == UPLOAD_FILE_END) { //
    if (Update.end(true)) { //
      Serial.printf("Оновлення успішне: %u байт\nПерезавантаження...\n", upload.totalSize); //
      
      // ВИПРАВЛЕНО: Віддаємо красиву сторінку з таймером замість сирого тексту, щоб точно не було крякозябр
      String html = "<!DOCTYPE html><html><head><meta charset='UTF-8'><meta name='viewport' content='width=device-width,initial-scale=1.0'>";
      html += "<style>body{font-family:Arial,sans-serif;text-align:center;background:#f4f4f9;padding-top:50px}.box{max-width:400px;margin:auto;background:#fff;padding:30px;border-radius:8px;box-shadow:0 2px 4px rgba(0,0,0,0.1)} b{color:#28a745}</style></head><body>";
      html += "<div class='box'><h2>✅ Оновлення успішне!</h2>";
      html += "<p>Пристрій перезавантажується... Повернення на головну через <b id='timer'>10</b> сек.</p></div>";
      html += "<script>let c=10; function dt(){ if(c<=0){ window.location.href='/'; } else { document.getElementById('timer').innerText=c; c--; setTimeout(dt,1000); } } setTimeout(dt,1000);</script>";
      html += "</body></html>";
      
      server.send(200, "text/html; charset=UTF-8", html);
      
      delay(1000); // Даємо 1 секунду на повну передачу сторінки у браузер перед рестартом
      ESP.restart(); //
    } else {
      Update.printError(Serial); //
      
      // ВИПРАВЛЕНО: Гарне сповіщення про помилку в UTF-8
      server.send(500, "text/html; charset=UTF-8", "<h2>❌ Помилка оновлення! Файл пошкоджений.</h2><a href='/update'>Спробувати знову</a>");
    }
  }
}




void handleSettingsUpdate() {
  if (!checkAuth()) return;
  if (!isConnected) { server.send(403, "text/plain", "Not connected."); return; }

  if (server.hasArg("cell_voltage_undervoltage_protection")) {
    float val = server.arg("cell_voltage_undervoltage_protection").toFloat();
    if (std::abs(val - G_settings.cell_voltage_undervoltage_protection) > FLOAT_TOLERANCE) {
      writeRegister(0x0A, (uint32_t)(val * 1000), 0x04); delay(200);
    }
  }
  if (server.hasArg("cell_voltage_overvoltage_protection")) {
    float val = server.arg("cell_voltage_overvoltage_protection").toFloat();
    if (std::abs(val - G_settings.cell_voltage_overvoltage_protection) > FLOAT_TOLERANCE) {
      writeRegister(0x0C, (uint32_t)(val * 1000), 0x04); delay(200);
    }
  }
  if (server.hasArg("total_battery_capacity")) {
    float val = server.arg("total_battery_capacity").toFloat();
    if (std::abs(val - G_settings.total_battery_capacity) > FLOAT_TOLERANCE) {
      writeRegister(0x41, (uint32_t)(val * 1000), 0x04); delay(200);
    }
  }
  if (server.hasArg("cell_count")) {
    // ВИПРАВЛЕНО: Змінено друкарську помилку з uint33_t на uint32_t
    uint32_t val = (uint32_t)server.arg("cell_count").toInt();
    if (val != G_settings.cell_count) { writeRegister(0x27, val, 0x04); delay(200); }
  }

  parsesettings = false;
  received_complete = false;
  frame = 0;
  writeRegister(0x96, 0x00000000, 0x00);
  delay(500);
  server.sendHeader("Location", "/");
  server.send(302, "text/plain", "");
}


void handleChargeOn()
{
  if (isConnected)
    writeRegister(0x1D, 1, 0x04);
  delay(100);
  server.send(200, "text/plain", "OK");
}
void handleChargeOff()
{
  if (isConnected)
    writeRegister(0x1D, 0, 0x04);
  delay(100);
  server.send(200, "text/plain", "OK");
}
void handleDischargeOn()
{
  if (isConnected)
    writeRegister(0x1E, 1, 0x04);
  delay(100);
  server.send(200, "text/plain", "OK");
}
void handleDischargeOff()
{
  if (isConnected)
    writeRegister(0x1E, 0, 0x04);
  delay(100);
  server.send(200, "text/plain", "OK");
}
void handleBalanceOn()
{
  if (isConnected)
    writeRegister(0x1F, 1, 0x04);
  delay(100);
  server.send(200, "text/plain", "OK");
}
void handleBalanceOff()
{
  if (isConnected)
    writeRegister(0x1F, 0, 0x04);
  delay(100);
  server.send(200, "text/plain", "OK");
}

void init_wifi()
{
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(250);
  }
  digitalWrite(LED_PIN, HIGH);
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
}
void setup()
{
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(115200);
  init_wifi();
  server.on("/", handleRoot);
  server.on("/data.json", handleDataJson);
  server.on("/info", handleInfo);
  server.on("/settings", handleSettings);
  server.on("/charge_on", handleChargeOn);
  server.on("/charge_off", handleChargeOff);
  server.on("/discharge_on", handleDischargeOn);
  server.on("/discharge_off", handleDischargeOff);
  server.on("/balance_on", handleBalanceOn);
  server.on("/balance_off", handleBalanceOff);
  server.on("/connect", HTTP_GET, handleConnect);
  server.on("/update", HTTP_GET, handleUpdate);
  server.on("/update", HTTP_POST, handleUpdate, handleUpdateUpload);
  server.on("/settings_update", HTTP_POST, handleSettingsUpdate);
  server.on("/scan", HTTP_GET, handleScan);
  server.on("/select_device", HTTP_GET, handleSelectDevice);
  server.on("/disconnect", HTTP_GET, handleDisconnect);
  server.on("/reboot", HTTP_GET, handleReboot);
  server.begin();
  BLEDevice::init("JKBMS service");
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);
  bleScanTask();
}

unsigned long lastTelemetryRequest = 0;
void loop()
{
  server.handleClient();
  
  if (isConnected && pNotifyCharacteristic != nullptr)
  {
    if (!parsesettings)
    {
      static unsigned long lastSettingsReq = 0;
      if (millis() - lastSettingsReq > 4000)
      {
        writeRegister(0x96, 0x00000000, 0x00);
        lastSettingsReq = millis();
      }
    }
    if (!parseinfo)
    {
      static unsigned long lastInfoReq = 0;
      if (millis() - lastInfoReq > 4000)
      {
        writeRegister(0x97, 0x00000000, 0x00);
        lastInfoReq = millis();
      }
    }
    if (millis() - lastTelemetryRequest > 2000)
    {
      writeRegister(0x95, 0x00000000, 0x00);
      lastTelemetryRequest = millis();
    }
  }
  delay(2); // Безпечна мікро-пауза для стабільності ядра
}
