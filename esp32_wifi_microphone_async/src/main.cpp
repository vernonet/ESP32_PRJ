#if !(defined(ESP32) )
  #error This code is intended to run on the ESP32 platform! Please check your Tools->Board setting.
#endif
#include <esp_wifi.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <esp_arduino_version.h>
#define USE_AVAILABLE_PAGES      true
#define USE_ESP_WIFIMANAGER_NTP  false
#define USE_CLOUDFLARE_NTP       false
#include "I2SMEMSSampler.h"
#include <ESPmDNS.h>
#include <NTPClient.h>
#include <driver/i2s.h>
#include "main.h"
#include <Update.h>
#include "soc/timer_group_struct.h"
#include "soc/timer_group_reg.h"
#include <ArduinoJson.h> 
#include <ESPAsyncDNSServer.h>
#include <AsyncTCP.h>
#include <ESPAsync_WiFiManager.h>               //https://github.com/khoih-prog/ESPAsync_WiFiManager
//#include <ESPAsync_WiFiManager.hpp> 
//#include <ESPAsync_WiFiManager-Impl.h>        //https://github.com/khoih-prog/ESPAsync_WiFiManager#howto-fix-multiple-definitions-linker-error
#include "AudioTools.h"
#include "AudioTools/Buffers.h"
#include "AudioTools/AudioStreams.h"
#if ( USING_ESP32_S2 || USING_ESP32_C3 )
 #include "driver/temp_sensor.h"
#endif
//#if !( USING_ESP32_S2 || USING_ESP32_C3 )
   AsyncDNSServer dnsServer;
//#endif 
#include "esp32/rom/rtc.h"
#if ( USING_ESP32_S2 )
  #if ARDUINO_USB_CDC_ON_BOOT
   #define HWSerial Serial   //usb serial
  #else
   #define HWSerial Serial1  //uart serial
  #endif
#else
 #define HWSerial Serial
#endif

#include "FS.h"
#include <LittleFS.h>
FS* filesystem =      &LittleFS;
#define FileFS        LittleFS
#define FS_Name       "LittleFS"
const char* CONFIG_FILE = "/ConfigSW.json";



extern "C" {
uint8_t temprature_sens_read();
}

// I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S
// SPH0645, INMP441 MEMS MICROPHONE
//insert in vlc  "http://wifi-mic.local:8080/rec.wav" or "http://<user>:<pass>@wifi-mic.local:8080/rec.wav" or "http://ip:8080/rec.wav"   ip - ip address of mic
//for ota update  http://wifi-mic.local:8080/upd_frm 
//for info        http://wifi-mic.local:8080/info

//#define NO_WIFI                  //testing the microphone using the "serial_audio.exe" program.

#define SAMPLE_BUFFER_SIZE         512    //Length of one buffer, in 32-bit words.  //512 //300  
#define BUF_CNT                    16      // Number of buffers in the I2S circular buffer   
#define SAMPLE_RATE                22050  //22050 //33000  //16000
#define BITS_PER_SAMPLE            (16)   // 16, maybe 24  
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(2, 0, 4)
  #define I2S_MIC_CHANNEL            I2S_CHANNEL_FMT_ONLY_RIGHT // most microphones will probably default to left channel but you may need to tie the L/R pin low
#else
  #define I2S_MIC_CHANNEL            I2S_CHANNEL_FMT_ONLY_LEFT
#endif  
#if (DEV_VARIANT == 0)
  //esp32a1s
  #define I2S_MIC_SERIAL_CLOCK       GPIO_NUM_18
  #define I2S_MIC_LEFT_RIGHT_CLOCK   GPIO_NUM_5
  #define I2S_MIC_SERIAL_DATA        GPIO_NUM_23
  #define LED_ON                     LOW
  #define LED_OFF                    HIGH
  #define PIN_LED                    GPIO_NUM_22
  const int TRIGGER_PIN3 = GPIO_NUM_13;      // short  contact to ground to enter config portal
#elif (DEV_VARIANT == 1)
  //esp32dev
  #define I2S_MIC_SERIAL_CLOCK       GPIO_NUM_32
  #define I2S_MIC_LEFT_RIGHT_CLOCK   GPIO_NUM_25
  #define I2S_MIC_SERIAL_DATA        GPIO_NUM_33
  #define LED_ON                     HIGH
  #define LED_OFF                    LOW
  #define PIN_LED                    GPIO_NUM_22
  const int TRIGGER_PIN3 = GPIO_NUM_2;      // short  contact to ground to enter config portal
#elif (DEV_VARIANT == 2)
  //esp32s2
  #define I2S_MIC_SERIAL_CLOCK       GPIO_NUM_12
  #define I2S_MIC_LEFT_RIGHT_CLOCK   GPIO_NUM_13
  #define I2S_MIC_SERIAL_DATA        GPIO_NUM_14
  #define LED_ON                     HIGH
  #define LED_OFF                    LOW
  #define PIN_LED                    GPIO_NUM_15
  const int TRIGGER_PIN3 = GPIO_NUM_21;      // short  contact to ground to enter config portal  
#else 
 #warning "DEV should equal 0 or 1 or 2"  
#endif


#define SERVER_PORT                "8080"
#define BAUDRATE                   (BAUDRATE_)
#define SIGNAL_GAIN                (0)// 0 - max gain, 2 - no gain
#define REC_TIME                   (6000) //sec
#define NUM_CPY                    ((SAMPLE_RATE * BITS_PER_SAMPLE / 8 * REC_TIME)/SAMPLE_BUFFER_SIZE)//
#define USER_OTA                   "admin"
#define PASS_OTA                   "admin"
#define PASS_CONF_PORTAL           "testtest"
#define LOG_SIZE                   (0x4000)
#define HOST_NME                   "wifi-mic"

#define SEND_BLOCK_SIZE_MAX        (2048)
#define SEND_BLOCK_SIZE_MIN        (2048)  //1024

#define WIFI_CONNECT_TIMEOUT      30000L
#define WHILE_LOOP_DELAY          200L
#define WHILE_LOOP_STEPS          (WIFI_CONNECT_TIMEOUT / ( 3 * WHILE_LOOP_DELAY ))

// Use USE_DHCP_IP == true for dynamic DHCP IP, false to use static IP which you have to change accordingly to your network
#if (defined(USE_STATIC_IP_CONFIG_IN_CP) && !USE_STATIC_IP_CONFIG_IN_CP)
  // Force DHCP to be true
  #if defined(USE_DHCP_IP)
    #undef USE_DHCP_IP
  #endif
  #define USE_DHCP_IP     true
#else
  // You can select DHCP or Static IP here
  #define USE_DHCP_IP     true
#endif


#define reverse_bytes(value)        ((value & 0x000000FFU) << 24 | (value & 0x0000FF00U) << 8 |(value & 0x00FF0000U) >> 8 | (value & 0xFF000000U) >> 24)
#define reverse_halfword(value)     ((value & 0x00FFU) << 24 |  (value & 0xFF000000U) >> 24)
#define reverse_sample16(value)     (((value >> 8)&0xFF) | ((value << 8)&0xFF00))

#define param_info(str, nme, value)   str +=F("<tr><td>");\
                                      str +=F(nme);\
                                      str +=F("</td><td>");\
                                      str += value;\
                                      str += F("</td></tr>");

#ifdef ARDUINO_ESP32_RELEASE_1_0_6 
  #define ESP_getChipId()   (ESP.getChipId())
#endif 

const i2s_port_t I2S_PORT = I2S_NUM_0;
uint8_t signal_gain = SIGNAL_GAIN;
volatile uint32_t wait_reset = 0;
uint32_t temp_buf_f[44/4]; //for header
String ssid = "wifi_mic_ap";
const char* password = PASS_CONF_PORTAL;     //password for config portal
// SSID and PW for your Router
String Router_SSID;
String Router_Pass;
volatile bool con_flag = false, start_rec = false, starting = true;
volatile unsigned int tme = 0;
uint32_t send_count;
WiFi_STA_IPConfig  WM_STA_IPconfig_;
const char compile_date[] = __DATE__ " " __TIME__;
bool authenticate=false;
char temp_[6];
char date_[11];
String log_page;
__NOINIT_ATTR char log_str[100];
uint32_t free_mem8 = 0, free_mem32 = 0;

using namespace audio_tools;
RingBufferStream wav_stream(SAMPLE_BUFFER_SIZE*96);
//RingBuffer<uint8_t> wav_stream(SAMPLE_BUFFER_SIZE*64); 
int16_t samples[SAMPLE_BUFFER_SIZE*(BITS_PER_SAMPLE>>3)*2];
bool stream_active = false;
bool blk_sze_changed = false;
uint32_t aviable;
extern bool client_connected;

AsyncClient *client_ = nullptr;
AsyncWebServer * server;
AsyncWebServer * webServer_async;
TaskHandle_t i2sMemsToBuffTaskHandle;
SemaphoreHandle_t mutex_wav_stream;

//flag to use from web update to reboot the ESP
bool shouldReboot = false;

// Define NTP Client to get time
const long utcOffsetInSeconds = 2 * 60 * 60; //+2;
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", utcOffsetInSeconds);
char m_login[12], m_pass[12], m_port[8];

void launchWeb(int webtype);
void i2sMemsToBuffTask(void *param);
void i2sMemsToUartTask(void *param);
void handleNotFound(AsyncWebServerRequest *request);
void handle_rec_wav(AsyncWebServerRequest *request); 
void handle_update(AsyncWebServerRequest *request);
void handle_upload(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final);
void handle_upd_frm(AsyncWebServerRequest *request);
void handle_log(AsyncWebServerRequest *request);
void handle_info(AsyncWebServerRequest *request);
//void handle_scan(AsyncWebServerRequest *request);
//void handle_restart(AsyncWebServerRequest *request);
//void onWsEvent(AsyncWebSocket * server, AsyncWebSocketClient * client, AwsEventType type, void * arg, uint8_t *data, size_t len);
bool writeConfigFile(void);
bool readConfigFile(void);
bool mount_fs(void);
const char * reset_reson_str(esp_reset_reason_t val);
String ESP32GetResetReason(uint32_t cpu_no);
#if ( USING_ESP32_S2 || USING_ESP32_C3 )
 void initTempSensor(void);
#endif 


I2SSampler *i2sSampler = NULL;
i2s_config_t i2s_config = {
  .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX), // Receive, not transfer
  .sample_rate = SAMPLE_RATE,
  .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT, // could only get it to work with 32bits
  .channel_format = I2S_MIC_CHANNEL, // although the SEL config should be left, it seems to transmit on right
#ifdef ARDUINO_ESP32_RELEASE_1_0_6  
  .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
#else  // 2.00.....
  .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_STAND_I2S),
#endif
  .intr_alloc_flags = (ESP_INTR_FLAG_LEVEL1 | ESP_INTR_FLAG_SHARED),        // Interrupt level 1
  .dma_buf_count = BUF_CNT,                        // number of buffers
  .dma_buf_len = SAMPLE_BUFFER_SIZE,               // samples per buffer
 // .use_apll = true,
  .tx_desc_auto_clear = true
};
// The pin config as per the setup
i2s_pin_config_t i2s_mic_pins = {
  .bck_io_num = I2S_MIC_SERIAL_CLOCK,
  .ws_io_num = I2S_MIC_LEFT_RIGHT_CLOCK,
  .data_out_num = I2S_PIN_NO_CHANGE,
  .data_in_num = I2S_MIC_SERIAL_DATA
};




void setup(void) {

  log_page.reserve(LOG_SIZE);
#if (BITS_PER_SAMPLE != 16 && BITS_PER_SAMPLE != 24)
  #error "This BITS_PER_SAMPLE not suported!!!!"
#endif  

  pinMode(TRIGGER_PIN3, INPUT_PULLUP);   //to enter config portal
  pinMode(PIN_LED, OUTPUT);  
  digitalWrite(PIN_LED, LED_OFF);   

#ifdef NO_WIFI
  WiFi.mode(WIFI_OFF);
#endif
  delay(500);
  #if ( USING_ESP32_S2 )
   initTempSensor();
  #endif
  HWSerial.begin(BAUDRATE);
  HWSerial.print("\r\nStarting...\r\ncompile date: ");
  HWSerial.println(compile_date);
  if (strncmp(log_str, "Last client", 11) == 0) // If the last client caused a restart  ESP32.
    HWSerial.println(log_str);
  if (!mount_fs() || !readConfigFile())
  {
    HWSerial.println(F("Failed to read ConfigFile, using default values"));
    strcpy(m_login, USER_OTA);
    strcpy(m_pass, PASS_OTA);
    strcpy(m_port, SERVER_PORT);
  }
#ifndef NO_WIFI
   unsigned long startedAt = millis();
   webServer_async = new AsyncWebServer(80);
// #if ( USING_ESP32_S2 || USING_ESP32_C3 )
//  ESPAsync_WiFiManager ESPAsync_wifiManager(webServer_async, NULL, "wifi-mic");
// #else
 ESPAsync_WiFiManager ESPAsync_wifiManager(webServer_async, &dnsServer, "wifi-mic");
// #endif  
  ESPAsync_wifiManager.setMinimumSignalQuality(-1);
  // Set static IP, Gateway, Subnetmask, DNS1 and DNS2. New in v1.0.5
  // ESP_wifiManager.setSTAStaticIPConfig(IPAddress(192,168,2,114), IPAddress(192,168,2,1), IPAddress(255,255,255,0), IPAddress(192,168,2,1), IPAddress(8,8,8,8));
  // Have to create a new function to store in EEPROM/SPIFFS for this purpose
  Router_SSID = ESPAsync_wifiManager.WiFi_SSID();
  Router_Pass = ESPAsync_wifiManager.WiFi_Pass();  
  //ESP_wifiManager.getSTAStaticIPConfig(WM_STA_IPconfig_);
  //Remove this line if you do not want to see WiFi password printed
  HWSerial.println("Stored: SSID = " + Router_SSID + ", Pass = " + Router_Pass);
  // SSID to uppercase
  ssid.toUpperCase();
  if (Router_SSID == "")
  {
    HWSerial.println("We haven't got any access point credentials, so get them now");
    HWSerial.println("It is necessary to wait a bit until the WIFI-MIC access point appears");
    HWSerial.println("Starting configuration portal: AP SSID = " + ssid + ", Pass = " + password );
    digitalWrite(PIN_LED, LED_ON); // Turn led on as we are in configuration mode.
    
    ESPAsync_WMParameter p_m_login("mic_login", "mic Login", USER_OTA, 12);  
    ESPAsync_WMParameter p_m_pass("mic_pass", "mic password", PASS_OTA, 12);
    ESPAsync_WMParameter p_m_port("mic_port", "server port", SERVER_PORT, 8);

    ESPAsync_wifiManager.addParameter(&p_m_login);
    ESPAsync_wifiManager.addParameter(&p_m_pass);
    ESPAsync_wifiManager.addParameter(&p_m_port);

    //it starts an access point
    //and goes into a blocking loop awaiting configuration
    if (!ESPAsync_wifiManager.startConfigPortal((const char *) ssid.c_str(), password))
      HWSerial.println("Not connected to WiFi but continuing anyway.");
    else {
      HWSerial.println("WiFi connected...:)");
    }

    strcpy(m_login, p_m_login.getValue());
    strcpy(m_pass, p_m_pass.getValue());
    strcpy(m_port, p_m_port.getValue());
    // Writing JSON config file to flash for next boot
    writeConfigFile();
    delete webServer_async;
  }

  digitalWrite(PIN_LED, LED_OFF); // Turn led off as we are not in configuration mode.

  startedAt = millis();

  while ( (WiFi.status() != WL_CONNECTED) && (millis() - startedAt < WIFI_CONNECT_TIMEOUT ) )
  { 
    IPAddress ip_255(255,255,255,255);   
    WiFi.disconnect(); // probalbly not needed
    delay(1000);
    WiFi.mode(WIFI_STA);
    //WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE);
    //WiFi.hostname("wifi-mic");
    String hostname = "wifi-mic";
    WiFi.setHostname(hostname.c_str());
    //WiFi.persistent (false);
    // We start by connecting to a WiFi network

    HWSerial.print("Connecting to ");
    HWSerial.println(Router_SSID);

    WiFi.begin(Router_SSID.c_str(), Router_Pass.c_str());

    int i = 0;
    while ((!WiFi.status() || WiFi.status() >= WL_DISCONNECTED) && i++ < WHILE_LOOP_STEPS)
    {
      delay(WHILE_LOOP_DELAY);
    }

    if (WiFi.localIP() == ip_255 && WiFi.status() == WL_CONNECTED) {//v.2.00.. get always 255.255.255.255 as IP address from WiFi.localIP()
      WiFi.disconnect();
      delay(1000);
      WiFi.mode(WIFI_STA);
      WiFi.setHostname(hostname.c_str());
      WiFi.config(IPAddress(0, 0, 0, 0), IPAddress(192, 168, 1, 1), IPAddress(255, 255, 255, 0), IPAddress(192, 168, 1, 1));
      WiFi.begin(Router_SSID.c_str(), Router_Pass.c_str()); 
      i = 0;
       while ((!WiFi.status() || WiFi.status() >= WL_DISCONNECTED) && i++ < WHILE_LOOP_STEPS)
      {
        delay(WHILE_LOOP_DELAY);
      }
    }
  }

  HWSerial.print("After waiting ");
  HWSerial.print((millis() - startedAt) / 1000);
  HWSerial.print(" secs more in setup(), connection result is ");

  if (WiFi.status() == WL_CONNECTED)
  {
    HWSerial.print("connected. Local IP: ");
    HWSerial.println(WiFi.localIP());
    con_flag = true;
    timeClient.begin();
    // GMT +1 = 3600
    // GMT +8 = 28800
    // GMT -1 = -3600
    // GMT 0 = 0
    timeClient.setTimeOffset(3600 * 3); //for GMT+3
    timeClient.update();
    launchWeb(0);
    digitalWrite(PIN_LED, LED_ON);
    delay(500);
    digitalWrite(PIN_LED, LED_OFF);
  }
  else
    HWSerial.println(ESPAsync_wifiManager.getStatus(WiFi.status()));

#endif
}

//loop  loop  loop  loop  loop  loop  loop  loop  loop  loop  loop  loop  loop  loop  loop  loop  loop  loop  loop
void loop(void) {
  int32_t value;
  char buff[9];

#ifndef NO_WIFI

  if (con_flag && !stream_active) {
    //server.handleClient();
    if (tme == 0)
    {
      if (!(timeClient.isTimeSet()))
      {
        timeClient.update();
      }
      unsigned long epochTime = timeClient.getEpochTime();
      // Get a time structure
      struct tm *ptm = gmtime((time_t *)&epochTime);
      int monthDay = ptm->tm_mday;
      int currentMonth = ptm->tm_mon + 1;
      int currentYear = ptm->tm_year + 1900;
      sprintf(date_, "%02d/%02d/%04d", monthDay, currentMonth, currentYear);
      // get internal temp of ESP32
      #if !( USING_ESP32_S2 || USING_ESP32_C3 )
       uint8_t temp_farenheit = temprature_sens_read();
       double temp = (temp_farenheit - 32) / 1.8;
      #else
       //uint8_t temp_farenheit = 104;
       float temp = 0;
       temp_sensor_read_celsius(&temp); 
      #endif
      // convert farenheit to celcius
      
      memset(temp_, 0x30, sizeof temp_);
      sprintf(temp_, "%.4lg", temp);
      // // Print complete date:
      // char str[18];
      // sprintf(str, "%02d/%02d/%04d %02d:%02d  itemp°C:%.4lg\r\n", monthDay,currentMonth,currentYear,timeClient.getHours(),timeClient.getMinutes(),temp);
      // HWSerial.printf("%s", str);    
      //if (WiFi.status() != WL_CONNECTED) {
      //  HWSerial.println("NO WIFI, try to connect");
      //   WiFi.mode(WIFI_STA);
      // // WiFi.hostname("wifi-mic");
      //  WiFi.setHostname("wifi-mic");
      //  //WiFi.persistent (true);
      //  //WiFi.setOutputPower(0);
      //  WiFi.begin(Router_SSID.c_str(), Router_Pass.c_str());
      //}
    }
    if (starting){
        char strr[30];
        sprintf(strr, "...starting %s %02d:%02d\r\n", date_, timeClient.getHours(), timeClient.getMinutes());
        log_page = String("<div class='text'><pre>") + String(strr);
        //log_page += String("ESP reset reason - ") + String(reset_reson_str(esp_reset_reason())) + String("\n");
        log_page += String("ESP reset reason rtc - ") + ESP32GetResetReason(0) + String("\n");
        if (strncmp(log_str, "Last client", 11) == 0)  //If the last client caused a restart  ESP32.
          log_page += String(log_str);
        starting = false;
        mutex_wav_stream  = xSemaphoreCreateMutex();
        free_mem8  = heap_caps_get_free_size(MALLOC_CAP_8BIT);
        free_mem32 = heap_caps_get_free_size(MALLOC_CAP_32BIT);
      }
    tme++;
    if (tme > 6000)
    {
        tme = 0;
        if (WiFi.status() != WL_CONNECTED)
        {
          HWSerial.println("NO WIFI, try to connect");
          WiFi.mode(WIFI_STA);
          // WiFi.hostname(HOST_NME);
          WiFi.setHostname(HOST_NME);
          // WiFi.persistent (true);
          // WiFi.setOutputPower(0);
          WiFi.begin(Router_SSID.c_str(), Router_Pass.c_str());
        }
        else if (WiFi.status() == WL_CONNECTED)
        {
          if (!con_flag) {  //if at startup it was not possible to connect to the wifi
          con_flag = true;
          timeClient.begin();
          timeClient.setTimeOffset(3600 * 2); // for GMT+2
          timeClient.update();
          launchWeb(0);
          }
        }
    }
    // if (tme > 6000) {
    //    if (timeClient.isTimeSet()) tme = 1;
    //       else tme = 0;
    // }
  }

  // is configuration portal requested?
  if (digitalRead(TRIGGER_PIN3) == LOW)
  {
    HWSerial.println("\nConfiguration portal requested.");
    digitalWrite(PIN_LED, LED_ON); // turn the LED on by making the voltage LOW to tell us we are in configuration mode.

    //Local intialization. Once its business is done, there is no need to keep it around
    // #if ( USING_ESP32_S2 || USING_ESP32_C3 )
    //   ESPAsync_WiFiManager ESPAsync_wifiManager(webServer_async, NULL, "wifi-mic-ap");
    // #else
       ESPAsync_WiFiManager ESPAsync_wifiManager(webServer_async, &dnsServer, "wifi-mic-ap");;
    // #endif 


    //Check if there is stored WiFi router/password credentials.
    //If not found, device will remain in configuration mode until switched off via webserver.
    HWSerial.print("Opening configuration portal. ");
    Router_SSID = ESPAsync_wifiManager.WiFi_SSID();
    if (Router_SSID != "")
    {
      ESPAsync_wifiManager.setConfigPortalTimeout(60); //If no access point name has been previously entered disable timeout.
      HWSerial.println("Got stored Credentials. Timeout 60s");
    }
    else
      HWSerial.println("No stored Credentials. No timeout");
    
    ESPAsync_WMParameter p_m_login("mic_login", "mic Login", USER_OTA, 12);  
    ESPAsync_WMParameter p_m_pass("mic_pass", "mic password", PASS_OTA, 12);
    ESPAsync_WMParameter p_m_port("mic_port", "server port", SERVER_PORT, 8);

    ESPAsync_wifiManager.addParameter(&p_m_login);
    ESPAsync_wifiManager.addParameter(&p_m_pass);
    ESPAsync_wifiManager.addParameter(&p_m_port);
    //it starts an access point
    //and goes into a blocking loop awaiting configuration
    if (!ESPAsync_wifiManager.startConfigPortal((const char *) ssid.c_str(), password))
    {
      HWSerial.println("Not connected to WiFi but continuing anyway.");
    }
    else
    {
      //if you get here you have connected to the WiFi
      HWSerial.println("connected...yeey :):):)");
    }

    strcpy(m_login, p_m_login.getValue());
    strcpy(m_pass, p_m_pass.getValue());
    strcpy(m_port, p_m_port.getValue());
    // Writing JSON config file to flash for next boot
    writeConfigFile();

    digitalWrite(PIN_LED, LED_OFF); // Turn led off as we are not in configuration mode.
    delete webServer_async;
    ESP.restart(); ///////////////////////
  }

  // if (stream_active && client_connected)
  // {
  //   // for watchdog timer
  //   // TIMERG0.wdt_wprotect = TIMG_WDT_WKEY_VALUE;
  //   // TIMERG0.wdt_feed = 1;
  //   // TIMERG0.wdt_wprotect = 0;
  //   //digitalWrite(PIN_LED, !digitalRead(PIN_LED)); 
  // }

  if (stream_active && client_->disconnected())
  {
    stream_active = false;
    vTaskSuspend(i2sMemsToBuffTaskHandle);
    HWSerial.print(" <---> client disconected\r\n");
    char strr[18] = "--/--/---- 00:00";
    if (timeClient.isTimeSet())
    {
      sprintf(strr, "%02d:%02d", timeClient.getHours(), timeClient.getMinutes());
      sprintf((char *)log_str + strlen(log_str) - 2, " <---> %02d:%02d\r\n", timeClient.getHours(), timeClient.getMinutes());
    }
    log_page += String(strr) + String(" client disconected ") + String("\n");
    i2s_stop(I2S_PORT);
    client_ = nullptr;
    digitalWrite(PIN_LED, LED_OFF);
  }
   vTaskDelay(10 / portTICK_PERIOD_MS);
#else

  start_rec = true;
  i2sSampler = new I2SMEMSSampler(I2S_PORT, i2s_mic_pins, i2s_config, false);
  // start sampling from i2s device
  i2sSampler->start();
  // set up the i2s sample writer task
  TaskHandle_t i2sMemsToUartTaskHandle;
  xTaskCreatePinnedToCore(i2sMemsToUartTask, "I2S Writer Task", 4096, i2sSampler, 1, &i2sMemsToUartTaskHandle, 1);
  while (true) {};

#endif

}

void createWebServer(int webtype)
{
  if ( webtype == 1 ) {  //softAP
    //

  } else if (webtype == 0) {  //are configured
    server = new AsyncWebServer(atoi(m_port)); 
    if (MDNS.begin("wifi-mic")) {
      HWSerial.println("MDNS responder started");
    }
    else {
      HWSerial.println("Error setting up MDNS responder!");
    }
    MDNS.addService("http", "tcp", atoi(m_port));  //orig 80
    //MDNS.addService("osc", "udp", 4500);
    //MDNS.addService("telnet", "tcp", 23);// Telnet server RemoteDebug

  //ws.onEvent(onWsEvent);
  //server->addHandler(&ws);
  //server->addHandler(&events);  

  server->onNotFound(handleNotFound);
  server->on("/upd_frm", HTTP_GET, handle_upd_frm);//for update frm http://wifi-mic.local:8080/upd_frm     admin:admin
  /*handling uploading firmware file */
  server->on("/update", HTTP_POST, handle_update, handle_upload);
  server->on("/info", HTTP_GET, handle_info);
  server->on("/rec.wav", HTTP_GET, handle_rec_wav);
  server->on("/log", HTTP_GET, handle_log);
  //server->on("/scan", HTTP_GET,handle_scan);
  //server->on("/restart", HTTP_GET,handle_restart);
  }
}


void launchWeb(int webtype) {
  HWSerial.println("");
  HWSerial.println("WiFi connected");
  HWSerial.print("Local IP: ");
  HWSerial.println(WiFi.localIP());
  HWSerial.print("SoftAP IP: ");
  HWSerial.println(WiFi.softAPIP());
  createWebServer(webtype);
  // Start the server
  server->begin();
  HWSerial.println("Server started");
}


void handleNotFound(AsyncWebServerRequest *request) {
  request->send(404, "text/plain", "404: Not found"); // Send HTTP status 404 (Not Found) when there's no handler for the URI in the request
}

void handle_rec_wav(AsyncWebServerRequest *request) {
  if (strlen(USER_OTA) > 0)
  {
    authenticate = true;
  }
  if (authenticate && !request->authenticate(m_login, m_pass))
    return request->requestAuthentication();

  client_ = request->client();
  if (client_)
  {
    //client_connected = true;
    wav_stream.reset();
    stream_active = true;
    HWSerial.print("New client conected - IP ");
    HWSerial.print(client_->remoteIP());
    HWSerial.printf(" PORT %d", client_->remotePort());
    sprintf(log_str, "Last client IP %s PORT %d ", client_->remoteIP().toString().c_str(), client_->remotePort());
    //HWSerial.println(log_str);
    char strr[18] = "--/--/---- 00:00";
    if (timeClient.isTimeSet())
    {
      sprintf(strr, "%s %02d:%02d", date_, timeClient.getHours(), timeClient.getMinutes());
      sprintf((char*)log_str + strlen(log_str), "%s %02d:%02d\r\n", date_, timeClient.getHours(), timeClient.getMinutes());
    }
    if (log_page.length() >= (LOG_SIZE - 100))
      log_page = "";
    // if (log_page. length() == 0) log_page = F("<div class='text'><pre>");
    // if (log_page.endsWith("</pre></div>")) log_page.remove(log_page.length()-12, 12);
    if (!(log_page.startsWith("<div class='text'><pre>")))
      log_page = String("<div class='text'><pre>") + log_page;
    log_page.replace("</pre></div>", ""); // delete old "</pre></div>"
    if (!start_rec)
    { //sizeof tcp buff to log
      log_page += String("TCP_buff_size ") + String(client_->space()) + String(" bytes\n");
    }
    log_page += String(strr) + String(" client conected IP ") + client_->remoteIP().toString() + String(" <---> ");
    if (!start_rec)
    { // if first connections
      start_rec = true;
      i2sSampler = new I2SMEMSSampler(I2S_NUM_0, i2s_mic_pins, i2s_config, false);
      // start sampling from i2s device
      i2sSampler->start();
      delay(10);
      // set up the i2s sample writer task
      #if !( USING_ESP32_S2 || USING_ESP32_C3 )
        xTaskCreatePinnedToCore(i2sMemsToBuffTask, "I2S Writer Task", 4096, i2sSampler, 1, &i2sMemsToBuffTaskHandle, 1);
      #else
        xTaskCreatePinnedToCore(i2sMemsToBuffTask, "I2S Writer Task", 4096, i2sSampler, 1, &i2sMemsToBuffTaskHandle, 0);
      #endif
    }
    // if any other connections
    else
    {
      i2s_start(I2S_PORT);
      delay(10);
      vTaskResume(i2sMemsToBuffTaskHandle);
    }

    send_count = 0;
    blk_sze_changed = false;
    aviable = 0;

    memcpy(&temp_buf_f[0], &test_wav[0], 44);
    temp_buf_f[4 / 4] = (SAMPLE_BUFFER_SIZE)*NUM_CPY + 36; // long size
    temp_buf_f[24 / 4] = SAMPLE_RATE;
    temp_buf_f[28 / 4] = SAMPLE_RATE * (BITS_PER_SAMPLE / 8);
    temp_buf_f[32 / 4] = (uint32_t)(BITS_PER_SAMPLE >> 3) + ((BITS_PER_SAMPLE << 16) & 0xFF0000);
    temp_buf_f[40 / 4] = (SAMPLE_BUFFER_SIZE)*NUM_CPY; // long chunkSize

    free_mem8 = heap_caps_get_free_size(MALLOC_CAP_8BIT);
    free_mem32 = heap_caps_get_free_size(MALLOC_CAP_32BIT);

    //HWSerial.println("New response ");
    //HWSerial.printf("new client -> 0x%x\r\n", request->client());

    // beginChunkedResponse
    AsyncWebServerResponse *response = request->beginResponse("audio/x-wav", 100, [](uint8_t *buffer, size_t maxLen, size_t index) -> size_t {
    
        uint32_t sze, aviable_;
        uint32_t space = client_->space(), client_mss = client_->getMss();
        uint16_t block_size;

        if (xSemaphoreTake(mutex_wav_stream, 3 / portTICK_PERIOD_MS) != pdTRUE)
        {
          HWSerial.println(" RESPONSE_TRY_AGAIN - mutex busy");
          return RESPONSE_TRY_AGAIN;
        }

        aviable = wav_stream.available();
        if (aviable < 4096 || space <= 4096)
        {
          //////aviable = wav_stream.available();
          //HWSerial.printf("RESPONSE_TRY_AGAIN  aviable %d\r\n", aviable);
          xSemaphoreGive(mutex_wav_stream);
          return RESPONSE_TRY_AGAIN;
        }
        else
        {
          if (!index)
          {
            memcpy(buffer, &temp_buf_f[0], 44);
            sze = 44;
          }
          else
          {
            if (!blk_sze_changed)
            {
              block_size = (space < client_mss * 4) ? SEND_BLOCK_SIZE_MIN : SEND_BLOCK_SIZE_MAX;
              if (block_size < SEND_BLOCK_SIZE_MAX)
                blk_sze_changed = true;
            }
            else
              block_size = SEND_BLOCK_SIZE_MIN;

            
            //HWSerial.printf("index %d aviable %d bytes heap %d block_size %d mss %d space %d\r\n", index, aviable, heap_caps_get_free_size(MALLOC_CAP_8BIT), block_size, client_mss, space);
            sze = wav_stream.readBytes(buffer, block_size);
            //////aviable -= sze;
          }
          xSemaphoreGive(mutex_wav_stream);
        }
     
     return sze; });

    response->setContentLength(SAMPLE_BUFFER_SIZE * (BITS_PER_SAMPLE / 8) * NUM_CPY + 44);
    request->send(response);
  }
}

void handle_upd_frm(AsyncWebServerRequest *request)
{ 
    if (strlen(m_login) > 0) {
      authenticate = true;
    }

    if(authenticate && !request->authenticate(m_login, m_pass))
        return request->requestAuthentication();   

    request->send(200, "text/html", serverIndex);
}

void handle_update(AsyncWebServerRequest *request)
{
    shouldReboot = !Update.hasError();
    AsyncWebServerResponse *response = request->beginResponse(200, "text/plain", shouldReboot?"OK":"FAIL");
    response->addHeader("Connection", "close");
    request->send(response);
    delay(100);
    ESP.restart();
}
  
void handle_upload(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final){
    if(!index){
      HWSerial.printf("Update Start: %s\n", filename.c_str());
      //Update.begin(true);
      if(!Update.begin(UPDATE_SIZE_UNKNOWN)){
        Update.printError(HWSerial);
      }
    }
    if(!Update.hasError()){
      if(Update.write(data, len) != len){
        Update.printError(HWSerial);
      }
    }
    if(final){
      if(Update.end(true)){
        HWSerial.printf("Update Success: %uB\n", index+len);
      } else {
        Update.printError(HWSerial);
      }
    }
  
}

 void handle_info(AsyncWebServerRequest *request) 
{
    if (strlen(m_login) > 0) {
      authenticate = true;
     }
    if(authenticate && !request->authenticate(m_login, m_pass))
        return request->requestAuthentication();   

    
    // http://wifi-mic.local:8080/info
    String page = F("<html><head><meta http-equiv='content-type' content='text/html; charset=utf-8'></head><body>\n");
    page += F(WM_HTTP_STYLE);
    page += F("</head><body><div class='container'><div style='text-align:left;display:inline-block;min-width:260px;'>");
    page += F("<h2>wifi-mic information</h2>");
    page += F("<fieldset>");
    page += F("<h3>Device Data</h3>");
    page += F("<table class=\"table\">");
    page += F("<thead><tr><th>Name</th><th>Value</th></tr></thead><tbody>");

    param_info(page, "ESP32 core version    ", String(ESP_ARDUINO_VERSION_MAJOR) + "." + String(ESP_ARDUINO_VERSION_MINOR) + "." + String(ESP_ARDUINO_VERSION_PATCH)); 
    param_info(page, "Chip Model    ", String(ESP.getChipModel()) + " Rev" + String(ESP.getChipRevision())); 
    param_info(page, "Compil. date of  FW ", String(compile_date));    
    param_info(page, "Flash Chip Size ", (ESP.getFlashChipSize()/1024) + String(" kbytes"));
    param_info(page, "Access Point IP ", (WiFi.softAPIP().toString()));   
    param_info(page, "Access Point MAC", WiFi.softAPmacAddress()); 
    param_info(page, "SSID", Router_SSID);   
    param_info(page, "Station IP", WiFi.localIP().toString());  
    param_info(page, "Station MAC", WiFi.macAddress());  
    param_info(page, "Core temp", String(temp_) + String(" °C"));  
    param_info(page, "CpuFreq", String(ESP.getCpuFreqMHz()) + String(" MHz"));  
    param_info(page, "Free mem8/32bit", String(free_mem8/1024) + "/" + String(free_mem32/1024) + String(" Kbytes"));  
    //param_info(page, "Free mem8/32bit", free_mem32/1024 + String(" Kbytes"));
   
    page += F("</tbody></table>");
    page += F("</fieldset>");
    page += F("<a href='/log'>View log("); 
    page += String(log_page.length());
    page += F(" bytes)</a>");
    page += FPSTR(WM_HTTP_END);

    AsyncWebServerResponse *response = request->beginResponse(200, "text/html", (const char *) page.c_str());
    response->addHeader("Connection", "close");
    request->send(response);
}

void handle_log(AsyncWebServerRequest *request) 
{
    if (strlen(m_login) > 0) {
      authenticate = true;
     }
   
    if(authenticate && !request->authenticate(m_login, m_pass))
        return request->requestAuthentication();   

    log_page += F("</pre></div>"); 
    AsyncWebServerResponse *response = request->beginResponse(200, "text/html", (const char *) log_page.c_str());
    response->addHeader("Connection", "close");
    request->send(response);
}

void handle_scan(AsyncWebServerRequest *request)
{
  if (strlen(m_login) > 0)
  {
    authenticate = true;
  }

  if (authenticate && !request->authenticate(m_login, m_pass))
    return request->requestAuthentication();

  {
    String json = "[";
    int n = WiFi.scanComplete();
    if (n == -2)
    {
      WiFi.scanNetworks(true, true);
    }
    else if (n)
    {
      for (int i = 0; i < n; ++i)
      {
        if (i)
          json += ",";
        json += "{";
        json += "\"rssi\":" + String(WiFi.RSSI(i));
        json += ",\"ssid\":\"" + WiFi.SSID(i) + "\"";
        json += ",\"bssid\":\"" + WiFi.BSSIDstr(i) + "\"";
        json += ",\"channel\":" + String(WiFi.channel(i));
        json += ",\"secure\":" + String(WiFi.encryptionType(i));
        //json += ",\"hidden\":" + String(WiFi.isHidden(i) ? "true" : "false");
        json += "}";
      }
      WiFi.scanDelete();
      if (WiFi.scanComplete() == -2)
      {
        WiFi.scanNetworks(true);
      }
    }
    json += "]";
    request->send(200, "application/json", json);
    json = String();
  }
}

void handle_restart(AsyncWebServerRequest *request)
{
  if (strlen(m_login) > 0)
  {
    authenticate = true;
  }
  if (authenticate && !request->authenticate(m_login, m_pass))
    return request->requestAuthentication();

  request->send(200, "text", "ESP32 restarting...");
  delay(100);
  ESP.restart();
}

// Task to write samples into stream
void i2sMemsToBuffTask(void *param)
{
  I2SSampler *sampler = (I2SSampler *)param;
  uint32_t sze;
  int samples_read;

  while (true)
  {
    //  TIMERG0.wdt_wprotect = TIMG_WDT_WKEY_VALUE;
    //  TIMERG0.wdt_feed = 1;
    //  TIMERG0.wdt_wprotect = 0;

    samples_read = sampler->read(samples, SAMPLE_BUFFER_SIZE * 4, BITS_PER_SAMPLE);
    while (xSemaphoreTake(mutex_wav_stream, portMAX_DELAY) != pdTRUE)
    {
      vTaskDelay(5 / portTICK_PERIOD_MS);
    }
    // HWSerial.print("Start read samples ->");
    if (samples_read) {
      wav_stream.write((uint8_t *)samples, samples_read * (BITS_PER_SAMPLE / 8));
    }  
    // HWSerial.println(" Stop read samples");
    xSemaphoreGive(mutex_wav_stream);
    digitalWrite(PIN_LED, !digitalRead(PIN_LED)); // indicate process
    //delay ms 120 for SAMPLE_RATE 16000, 90 for SAMPLE_RATE 22050, 59 for SAMPLE_RATE 33000
    vTaskDelay(((SAMPLE_BUFFER_SIZE*4*1000)/SAMPLE_RATE - 3) / portTICK_PERIOD_MS);  
  }
}

// Task to send samples to uart
void i2sMemsToUartTask(void *param)
{
  I2SSampler *sampler = (I2SSampler *)param;

  while (true)
  {
     int samples_read = sampler->read(samples, SAMPLE_BUFFER_SIZE, BITS_PER_SAMPLE);
     HWSerial.write((char *)samples,  samples_read * (BITS_PER_SAMPLE/8));
     digitalWrite(PIN_LED, !digitalRead(PIN_LED)); 
  }
}

bool writeConfigFile()
{
  HWSerial.println("Saving config file");

#if (ARDUINOJSON_VERSION_MAJOR >= 6)
  DynamicJsonDocument json(1024);
#else
  DynamicJsonBuffer jsonBuffer;
  JsonObject& json = jsonBuffer.createObject();
#endif

  // JSONify local configuration parameters
  json["mic_login"] = m_login;
  json["mic_pass"]  = m_pass;
  json["mic_port"]  = m_port;
 

  // Open file for writing
  File f = FileFS.open(CONFIG_FILE, "w");

  if (!f)
  {
    HWSerial.println("Failed to open config file for writing");
    return false;
  }

#if (ARDUINOJSON_VERSION_MAJOR >= 6)
  serializeJsonPretty(json, HWSerial);
  // Write data to file and close it
  serializeJson(json, f);
#else
  json.prettyPrintTo(HWSerial);
  // Write data to file and close it
  json.printTo(f);
#endif

  f.close();

  HWSerial.println("\nConfig file was successfully saved");
  return true;
}

bool readConfigFile(void)
{
  // this opens the config file in read-mode
  File f = FileFS.open(CONFIG_FILE, "r");

  if (!f)
  {
    HWSerial.println("Configuration file not found");
    return false;
  }
  else
  {
    // we could open the file
    size_t size = f.size();
    // Allocate a buffer to store contents of the file.
    std::unique_ptr<char[]> buf(new char[size + 1]);

    // Read and store file contents in buf
    f.readBytes(buf.get(), size);
    // Closing file
    f.close();
    // Using dynamic JSON buffer which is not the recommended memory model, but anyway
    // See https://github.com/bblanchon/ArduinoJson/wiki/Memory%20model

#if (ARDUINOJSON_VERSION_MAJOR >= 6)
    DynamicJsonDocument json(1024);
    auto deserializeError = deserializeJson(json, buf.get());
    if ( deserializeError )
    {
      HWSerial.println("JSON parseObject() failed");
      return false;
    }
    serializeJson(json, HWSerial);
#else
    DynamicJsonBuffer jsonBuffer;
    // Parse JSON string
    JsonObject& json = jsonBuffer.parseObject(buf.get());
    // Test if parsing succeeds.
    if (!json.success())
    {
      HWSerial.println("JSON parseObject() failed");
      return false;
    }
    json.printTo(HWSerial);
#endif

    // Parse all config file parameters, override
    // local config variables with parsed values
    if (json.containsKey("mic_login"))
    {
      strcpy(m_login, json["mic_login"]);
    }

    if (json.containsKey("mic_pass"))
    {
      strcpy(m_pass, json["mic_pass"]);
    }

    if (json.containsKey("mic_port"))
    {
      strcpy(m_port, json["mic_port"]);
    }
  }
  HWSerial.println("\nConfig file was successfully parsed");
  return true;
}


//return true - ok, false - error
bool mount_fs(void) {

  if (!FileFS.begin(true)) {


    HWSerial.println(F("LittleFS failed! Already tried formatting."));
  
    if (!FileFS.begin())
    {     
      // prevents debug info from the library to hide err message.
      delay(100);
      
      HWSerial.println(F("LittleFS failed!. Please use SPIFFS or EEPROM. Stay forever"));

      return false;
    }
    
  }
   else {
     HWSerial.println(F("LittleFS mount OK!"));
     return true;
   }
  return true;
}

const char * reset_reson_str(esp_reset_reason_t val)
{

    switch (val)
    {
    case ESP_RST_UNKNOWN:   return "UNKNOWN";
    case ESP_RST_POWERON:   return "POWERON";
    case ESP_RST_EXT:       return "EXT";
    case ESP_RST_SW:        return "SW";
    case ESP_RST_PANIC:     return "PANIC";
    case ESP_RST_INT_WDT:   return "INT_WDT";
    case ESP_RST_TASK_WDT:  return "TASK_WDT";
    case ESP_RST_WDT:       return "WDT";
    case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
    case ESP_RST_BROWNOUT:  return "BROWNOUT";
    case ESP_RST_SDIO:      return "SDIO";
    default:
        return "Unknown esp_reset_reason_t";
    }
}

String ESP32GetResetReason(uint32_t cpu_no) {
	
  switch (rtc_get_reset_reason(cpu_no)) {
    case POWERON_RESET          : return F("Vbat power on reset");                              // 1
    case SW_RESET               : return F("Software reset digital core");                      // 3
    case OWDT_RESET             : return F("Legacy watch dog reset digital core");              // 4
    case DEEPSLEEP_RESET        : return F("Deep Sleep reset digital core");                    // 5
    case SDIO_RESET             : return F("Reset by SLC module, reset digital core");          // 6
    case TG0WDT_SYS_RESET       : return F("Timer Group0 Watch dog reset digital core");        // 7
    case TG1WDT_SYS_RESET       : return F("Timer Group1 Watch dog reset digital core");        // 8
    case RTCWDT_SYS_RESET       : return F("RTC Watch dog Reset digital core");                 // 9
    case INTRUSION_RESET        : return F("Instrusion tested to reset CPU");                   // 10
    case TGWDT_CPU_RESET        : return F("Time Group reset CPU");                             // 11
    case SW_CPU_RESET           : return F("Software reset CPU");                               // 12
    case RTCWDT_CPU_RESET       : return F("RTC Watch dog Reset CPU");                          // 13
    case EXT_CPU_RESET          : return F("or APP CPU, reseted by PRO CPU");                   // 14
    case RTCWDT_BROWN_OUT_RESET : return F("Reset when the vdd voltage is not stable");         // 15
    case RTCWDT_RTC_RESET       : return F("RTC Watch dog reset digital core and rtc module");  // 16            
  }
  return F("NO_MEAN");                                                                          // 0 and undefined
}

#if (USING_ESP32_S2 || USING_ESP32_C3)
void initTempSensor(void)
{
  temp_sensor_config_t temp_sensor = TSENS_CONFIG_DEFAULT();
  temp_sensor.dac_offset = TSENS_DAC_L2; // TSENS_DAC_L2 is default   L4(-40℃ ~ 20℃), L2(-10℃ ~ 80℃) L1(20℃ ~ 100℃) L0(50℃ ~ 125℃)
  temp_sensor_set_config(temp_sensor);
  temp_sensor_start();
}
#endif
