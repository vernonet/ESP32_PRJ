#if !(defined(ESP32) )
  #error This code is intended to run on the ESP32 platform! Please check your Tools->Board setting.
#endif
#include <esp_wifi.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <esp_arduino_version.h>
#define USE_AVAILABLE_PAGES     true
#include "I2SMEMSSampler.h"
#include <ESPmDNS.h>
#include <NTPClient.h>
#include <driver/i2s.h>
#include "main.h"
#include <Update.h>
#include "soc/timer_group_struct.h"
#include "soc/timer_group_reg.h"
#include <ArduinoJson.h> 
#include <AsyncTCP.h>
#include <ESPAsync_WiFiManager.h>              //https://github.com/khoih-prog/ESPAsync_WiFiManager
#if !( USING_ESP32_S2 || USING_ESP32_C3 )
  DNSServer dnsServer;
#endif 
//#include "sha_parallel_engine.h"

#include "FS.h"
#include <LittleFS.h>
FS* filesystem =      &LittleFS;
#define FileFS        LittleFS
#define FS_Name       "LittleFS"
const char* CONFIG_FILE = "/ConfigSW.json";

#include "AudioTools.h"
#include "AudioCodecs/CodecAACFDK.h"
#include "AudioTools/AudioStreams.h"


extern "C" {
uint8_t temprature_sens_read();
}

// I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S
// SPH0645, INMP441 MEMS MICROPHONE
//insert in vlc  "http://wifi-mic.local:8080/rec.aac" or "http://<user>:<pass>@wifi-mic.local:8080/rec.aac" or "http://ip:8080/rec.aac"   ip - ip address of mic
//for ota update  http://wifi-mic.local:8080/upd_frm 
//for info        http://wifi-mic.local:8080/info


#define SAMPLE_BUFFER_SIZE         512    //Length of one buffer, in 32-bit words.  //512 //300  
#define BUF_CNT                    8      // Number of buffers in the I2S circular buffer   
#define SAMPLE_RATE                22050  //22050 //33000  //16000
#define BITS_PER_SAMPLE            (16)   // only 16   
#define I2S_MIC_CHANNEL            I2S_CHANNEL_FMT_ONLY_LEFT // most microphones will probably default to left channel but you may need to tie the L/R pin low
#define I2S_MIC_SERIAL_CLOCK       GPIO_NUM_18
#define I2S_MIC_LEFT_RIGHT_CLOCK   GPIO_NUM_5
#define I2S_MIC_SERIAL_DATA        GPIO_NUM_23
#define SERVER_PORT                "8080"
#define BAUDRATE                  (BAUDRATE_)
#define SIGNAL_GAIN                (0)// 0 - max gain, 2 - no gain
#define REC_TIME                   (6000) //sec
#define NUM_CPY                    ((SAMPLE_RATE * BITS_PER_SAMPLE / 8 * REC_TIME)/SAMPLE_BUFFER_SIZE)//
#define USER_OTA                   "admin"
#define PASS_OTA                   "admin"
#define PASS_CONF_PORTAL           "testtest"
#define LOG_SIZE                   (0x10000)


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

#define LED_ON                     LOW
#define LED_OFF                    HIGH
#define PIN_LED                    GPIO_NUM_22         // 
#ifdef ARDUINO_ESP32_RELEASE_1_0_6 
  #define ESP_getChipId()   (ESP.getChipId())
#endif 

const i2s_port_t I2S_PORT = I2S_NUM_0;
uint8_t signal_gain = SIGNAL_GAIN;
volatile uint32_t wait_reset = 0;
String ssid = "wifi_mic_ap";
const char* password = PASS_CONF_PORTAL;     //password for config portal
// SSID and PW for your Router
String Router_SSID;
String Router_Pass;
const int TRIGGER_PIN3 = GPIO_NUM_13;      // short  contact to ground to enter config portal
volatile bool con_flag = false, start_rec = false, starting = true;
volatile unsigned int tme = 0;
uint32_t send_count;
WiFi_STA_IPConfig  WM_STA_IPconfig_;
const char compile_date[] = __DATE__ " " __TIME__;
bool authenticate=false;
char temp_[6];
char date_[11];
String log_page;
uint32_t free_mem8 = 0, free_mem32 = 0;

AsyncClient *client_ = nullptr;
AsyncWebServer * server;
AsyncWebServer webServer_async(80);
//AsyncEventSource events("/events");
//AsyncWebSocket ws("/ws");
//AsyncWebServerRequest *request_;
//AsyncWebServerResponse *response_;
static const char *STREAM_CONTENT_TYPE = "audio/aac";
extern bool client_connected;
bool stream_active = false;

//flag to use from web update to reboot the ESP
bool shouldReboot = false;

// Define NTP Client to get time
const long utcOffsetInSeconds = 2 * 60 * 60; //+2;
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", utcOffsetInSeconds);
const int SAMPLE_SIZE = SAMPLE_BUFFER_SIZE;
int16_t *samples;
uint8_t tmp_buf[1024];
//TaskHandle_t SendToClientTaskHandle;
TaskHandle_t AAC_processTaskHandle;
extern UINT  alloc_mem;
char m_login[12], m_pass[12], m_port[8];

void launchWeb(int webtype);
void i2sMemsToClientTask(void *param);
void i2sMemsToUartTask(void *param);
//void SendToClientTask(void *param);
//void AAC_processTask(void *param);
void AAC_process(void);
void configWiFi(WiFi_STA_IPConfig in_WM_STA_IPconfig);
void handleNotFound(AsyncWebServerRequest *request);
void handle_rec_aac(AsyncWebServerRequest *request); 
void handle_update(AsyncWebServerRequest *request);
void handle_upload(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final);
void handle_upd_frm(AsyncWebServerRequest *request);
void handle_log(AsyncWebServerRequest *request);
void handle_info(AsyncWebServerRequest *request);
//void onWsEvent(AsyncWebSocket * server, AsyncWebSocketClient * client, AwsEventType type, void * arg, uint8_t *data, size_t len);
void logMemory();
static void fill_mem_seed(int seed, void *mem, int len, int block_size);
static bool check_mem_seed(int seed, void *mem, int len, int block_size);
bool writeConfigFile(void);
bool readConfigFile(void);
bool mount_fs(void);

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
  .use_apll = true,
  .tx_desc_auto_clear = true
};
// The pin config as per the setup
i2s_pin_config_t i2s_mic_pins = {
  .bck_io_num = I2S_MIC_SERIAL_CLOCK,
  .ws_io_num = I2S_MIC_LEFT_RIGHT_CLOCK,
  .data_out_num = I2S_PIN_NO_CHANGE,
  .data_in_num = I2S_MIC_SERIAL_DATA
};

using namespace audio_tools;
RingBufferStream aac_stream(SAMPLE_BUFFER_SIZE*32);
//AACEncoderFDK aac(aac_stream);
AACEncoderFDK *aac;
AudioBaseInfo info;

class AsyncAudioStreamResponse : public AsyncAbstractResponse {
   private:
    size_t _index;
    bool first = true;

   public:
    AsyncAudioStreamResponse() {
        _callback = nullptr;
        _code = 200;
        _contentLength = 0;
        _contentType = STREAM_CONTENT_TYPE;
        _sendContentLength = false;
        _chunked = true;
        _index = 0;
    }
    ~AsyncAudioStreamResponse() {
    }
    bool _sourceValid() const {
        return true;
    }
    virtual size_t _fillBuffer(uint8_t *buf, size_t maxLen) override {
        size_t ret = _content(buf, maxLen, _index);
        if (ret != RESPONSE_TRY_AGAIN) {
            _index += ret;
        }
        return ret;
    }
    size_t _content(uint8_t *buffer, size_t maxLen, size_t index) {

      if (aac_stream.available() <= 2048)
      {
        return RESPONSE_TRY_AGAIN;
      }
      maxLen = 1400;
      if (first)
      { //first packet
        first = false;
        // send header
        size_t hlen = sprintf((char *)buffer, STREAM_CONTENT_TYPE, 100);
        buffer += hlen;
        // send frame
        hlen = maxLen - hlen;
        // Serial.printf("first aviable %d bytes\n\r", aac_stream.available());
        aac_stream.readBytes(buffer, hlen);
        return maxLen;
      }
      else
      { //any other packet
        //Serial.printf("aviable %d bytes\n\r", aac_stream.available());
        uint32_t cnt = aac_stream.readBytes(buffer, maxLen);
        return cnt;
      }
    }
};

void setup(void) {

  log_page.reserve(LOG_SIZE);
#if (BITS_PER_SAMPLE != 16)
  #error "This BITS_PER_SAMPLE not suported!!!!"
#endif  

  pinMode(TRIGGER_PIN3, INPUT_PULLUP);   //to enter config portal
  pinMode(PIN_LED, OUTPUT);              //

  delay(500);
  Serial.begin(BAUDRATE);
  
  Serial.printf("Total heap: %d KBytes\n\r",  heap_caps_get_total_size(MALLOC_CAP_8BIT)/1024);
  Serial.printf("Free heap: %d KBytes\n\r", heap_caps_get_free_size(MALLOC_CAP_8BIT)/1024);
  Serial.printf("Total PSRAM: %d KBytes\n\r", ESP.getPsramSize()/1024);
  Serial.printf("Free PSRAM: %d KBytes\n\r", ESP.getFreePsram()/1024);

  byte* psdRamBuffer = (byte*)ps_malloc(0x10000*16*3);  //test 3Mbytes
  fill_mem_seed(0xaaaa, psdRamBuffer, sizeof psdRamBuffer, 0x1000);
  if (check_mem_seed(0xaaaa, psdRamBuffer, sizeof psdRamBuffer, 0x1000)) {
     Serial.println("PSRAM test - OK!\n\r");
  }
  else Serial.println("PSRAM test - ERROR!\n\r");
  free(psdRamBuffer);

  Serial.print("\nStarting...\n\rcompile date: ");
  Serial.println(compile_date);
  samples = (int16_t *)malloc((BITS_PER_SAMPLE/8) * SAMPLE_SIZE*2);
  if (!samples) Serial.println("Failed to allocate memory for samples");

  if (!mount_fs() || !readConfigFile())
  {
    Serial.println(F("Failed to read ConfigFile, using default values"));
    strcpy(m_login, USER_OTA);
    strcpy(m_pass, PASS_OTA);
    strcpy(m_port, SERVER_PORT);
  }

  if (!(heap_caps_check_integrity_all(true))) 
     Serial.println("HEAP check integrity fall");
   
  unsigned long startedAt = millis();
#if ( USING_ESP32_S2 || USING_ESP32_C3 )
 ESPAsync_WiFiManager ESPAsync_wifiManager(&webServer_async, NULL, "wifi-mic");
#else
 ESPAsync_WiFiManager ESPAsync_wifiManager(&webServer_async, &dnsServer, "wifi-mic-ap");
#endif  
  ESPAsync_wifiManager.setMinimumSignalQuality(-1);
  // Set static IP, Gateway, Subnetmask, DNS1 and DNS2. New in v1.0.5
  // ESP_wifiManager.setSTAStaticIPConfig(IPAddress(192,168,2,114), IPAddress(192,168,2,1), IPAddress(255,255,255,0), IPAddress(192,168,2,1), IPAddress(8,8,8,8));
  // Have to create a new function to store in EEPROM/SPIFFS for this purpose
  Router_SSID = ESPAsync_wifiManager.WiFi_SSID();
  Router_Pass = ESPAsync_wifiManager.WiFi_Pass();  
  //ESP_wifiManager.getSTAStaticIPConfig(WM_STA_IPconfig_);
  //Remove this line if you do not want to see WiFi password printed
  Serial.println("Stored: SSID = " + Router_SSID + ", Pass = " + Router_Pass);
  // SSID to uppercase
  ssid.toUpperCase();
  if (Router_SSID == "")
  {
    Serial.println("We haven't got any access point credentials, so get them now");
    Serial.println("It is necessary to wait a bit until the WIFI-MIC access point appears");
    Serial.println("Starting configuration portal: AP SSID = " + ssid + ", Pass = " + password );
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
      Serial.println("Not connected to WiFi but continuing anyway.");
    else {
      Serial.println("WiFi connected...:)");
    }

    strcpy(m_login, p_m_login.getValue());
    strcpy(m_pass, p_m_pass.getValue());
    strcpy(m_port, p_m_port.getValue());
    // Writing JSON config file to flash for next boot
    writeConfigFile();
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

    Serial.print("Connecting to ");
    Serial.println(Router_SSID);

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
      //configWiFi(WM_STA_IPconfig_);
      WiFi.config(IPAddress(0, 0, 0, 0), IPAddress(192, 168, 1, 1), IPAddress(255, 255, 255, 0), IPAddress(192, 168, 1, 1));
      WiFi.begin(Router_SSID.c_str(), Router_Pass.c_str()); 
      i = 0;
       while ((!WiFi.status() || WiFi.status() >= WL_DISCONNECTED) && i++ < WHILE_LOOP_STEPS)
      {
        delay(WHILE_LOOP_DELAY);
      }
    }
  }

  Serial.print("After waiting ");
  Serial.print((millis() - startedAt) / 1000);
  Serial.print(" secs more in setup(), connection result is ");

  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.print("connected. Local IP: ");
    Serial.println(WiFi.localIP());
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
    Serial.println(ESPAsync_wifiManager.getStatus(WiFi.status()));
}

//loop  loop  loop  loop  loop  loop  loop  loop  loop  loop  loop  loop  loop  loop  loop  loop  loop  loop  loop
void loop(void) {
  int32_t value;
  char buff[9]; 

  if (con_flag && !stream_active) {
    //server->handleClient();
    if (tme < 1) {
      timeClient.update();
      unsigned long epochTime = timeClient.getEpochTime();
       //Get a time structure
      struct tm *ptm = gmtime ((time_t *)&epochTime); 
      int monthDay = ptm->tm_mday;
      int currentMonth = ptm->tm_mon+1;
      int currentYear = ptm->tm_year+1900;
      //get internal temp of ESP32
      uint8_t temp_farenheit= temprature_sens_read();
      //convert farenheit to celcius
      double temp = ( temp_farenheit - 32 ) / 1.8;
      memset(temp_, 0x30, sizeof temp_);
      sprintf(temp_, "%.4lg",temp);
      sprintf(date_, "%02d/%02d/%04d", monthDay,currentMonth,currentYear);
      if (starting){
        char strr[30];
        sprintf(strr, "...starting %s %02d:%02d\n", date_, timeClient.getHours(), timeClient.getMinutes());
        log_page = String(strr); 
        starting = false;
        free_mem8  = heap_caps_get_free_size(MALLOC_CAP_8BIT);
        free_mem32 = heap_caps_get_free_size(MALLOC_CAP_32BIT);
      }
      //Print complete date:
      char str[18];
      sprintf(str, "%02d/%02d/%04d %02d:%02d  itemp°C:%.4lg\n\r", monthDay,currentMonth,currentYear,timeClient.getHours(),timeClient.getMinutes(),temp);
      Serial.printf("%s", str);    
      if (WiFi.status() != WL_CONNECTED) {
        Serial.println("NO WIFI, try to connect");
        WiFi.mode(WIFI_STA);
       // WiFi.hostname("wifi-mic");
        WiFi.setHostname("wifi-mic");
        //WiFi.persistent (true);
        //WiFi.setOutputPower(0);
        WiFi.begin(Router_SSID.c_str(), Router_Pass.c_str());
      }
    }
    tme++;
    if (tme > 3000000) tme = 0;
  }

  // is configuration portal requested?
  if (digitalRead(TRIGGER_PIN3) == LOW)
  {
    Serial.println("\nConfiguration portal requested.");
    digitalWrite(PIN_LED, LED_ON); // turn the LED on by making the voltage LOW to tell us we are in configuration mode.

    //Local intialization. Once its business is done, there is no need to keep it around
     ESPAsync_WiFiManager ESPAsync_wifiManager(&webServer_async, &dnsServer, "wifi-mic-ap");;


    //Check if there is stored WiFi router/password credentials.
    //If not found, device will remain in configuration mode until switched off via webserver.
    Serial.print("Opening configuration portal. ");
    Router_SSID = ESPAsync_wifiManager.WiFi_SSID();
    if (Router_SSID != "")
    {
      ESPAsync_wifiManager.setConfigPortalTimeout(60); //If no access point name has been previously entered disable timeout.
      Serial.println("Got stored Credentials. Timeout 60s");
    }
    else
      Serial.println("No stored Credentials. No timeout");
    
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
      Serial.println("Not connected to WiFi but continuing anyway.");
    }
    else
    {
      //if you get here you have connected to the WiFi
      Serial.println("connected...yeey :):):)");
    }

    strcpy(m_login, p_m_login.getValue());
    strcpy(m_pass, p_m_pass.getValue());
    strcpy(m_port, p_m_port.getValue());
    // Writing JSON config file to flash for next boot
    writeConfigFile();

    digitalWrite(PIN_LED, LED_OFF); // Turn led off as we are not in configuration mode.
  }
  else if (!(con_flag)) {
    wait_reset++;
    delay(100);
    if (wait_reset >= 1800) ESP.restart();
  }

  if (stream_active && client_connected)
  {
    // for watchdog timer
    TIMERG0.wdt_wprotect = TIMG_WDT_WKEY_VALUE;
    TIMERG0.wdt_feed = 1;
    TIMERG0.wdt_wprotect = 0;
    AAC_process();
    digitalWrite(PIN_LED, !digitalRead(PIN_LED)); 
    yield();
  }
    else if (stream_active && !client_connected)
    {
      stream_active = false;
      Serial.print("client disconected\n\r");
      char strr[18];
      sprintf(strr, "%02d:%02d", timeClient.getHours(), timeClient.getMinutes());
      log_page += String(strr) + String(" client disconected ") + String("\n");
      i2s_stop(I2S_PORT);
      digitalWrite(PIN_LED, LED_OFF);
      aac->end();
      delete(aac);
      aac_stream.flush();
      // clear the counter of allocated memory for the aac encoder
      alloc_mem = 0;
   }
}

void createWebServer(int webtype)
{
  if ( webtype == 1 ) {  //softAP
    //

  } else if (webtype == 0) {  //are configured
    server = new AsyncWebServer(atoi(m_port));
    if (MDNS.begin("wifi-mic")) {
      Serial.println("MDNS responder started");
    }
    else {
      Serial.println("Error setting up MDNS responder!");
    }
    MDNS.addService("http", "tcp", atoi(m_port));  //orig 80
    //MDNS.addService("osc", "udp", 4500);
    //MDNS.addService("telnet", "tcp", 23);// Telnet server RemoteDebug

  //ws.onEvent(onWsEvent);
  //server->addHandler(&ws);
  // server->addHandler(&events);  

  server->onNotFound(handleNotFound);
  /*return index page which is stored in serverIndex */
  server->on("/upd_frm", HTTP_GET, handle_upd_frm);//for update frm http://wifi-mic.local:8080/upd_frm     admin:admin
  //server->on("/serverIndex", handle_serverIndex);
  /*handling uploading firmware file */
  server->on("/update", HTTP_POST, handle_update, handle_upload);
  server->on("/info", HTTP_GET, handle_info);
  server->on("/rec.aac", HTTP_GET, handle_rec_aac);
  server->on("/log", HTTP_GET, handle_log);
  }
}


void launchWeb(int webtype) {
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.print("Local IP: ");
  Serial.println(WiFi.localIP());
  Serial.print("SoftAP IP: ");
  Serial.println(WiFi.softAPIP());
  createWebServer(webtype);
  // Start the server
  //server = new AsyncWebServer(atoi(m_port));
  server->begin();
  Serial.println("Server started");
}



void handleNotFound(AsyncWebServerRequest *request) {
  request->send(404, "text/plain", "404: Not found"); // Send HTTP status 404 (Not Found) when there's no handler for the URI in the request
}

void handle_rec_aac(AsyncWebServerRequest *request) {
  // //for watchdog timer
  // TIMERG0.wdt_wprotect = TIMG_WDT_WKEY_VALUE;
  // TIMERG0.wdt_feed = 1;
  // TIMERG0.wdt_wprotect = 0;

  if (strlen(m_login) > 0) {
      authenticate = true;
     }
  if(authenticate && !request->authenticate(m_login, m_pass))
        return request->requestAuthentication();   

  client_ = request->client();
  if (client_) {
    client_connected = true;
    stream_active = true;
    info.channels = 1;
    info.sample_rate = SAMPLE_RATE;
    info.bits_per_sample = BITS_PER_SAMPLE;
    aac = new AACEncoderFDK(aac_stream);
    //aac->setOutputStream(aac_stream);
    aac->setOutputBufferSize(10240);
    aac->begin(info);

    Serial.print("New client conected - IP ");
    Serial.print(client_->remoteIP());
    Serial.printf(" PORT %d\n\r", client_->remotePort());
    char strr[18];
    sprintf(strr, "%s %02d:%02d", date_, timeClient.getHours(), timeClient.getMinutes());
    if (log_page. length() >= (LOG_SIZE-100)) log_page = "";
    //if (log_page. length() == 0) log_page = F("<div class='text'><pre>");
    //if (log_page.endsWith("</pre></div>")) log_page.remove(log_page.length()-12, 12);
    if (!(log_page.startsWith("<div class='text'><pre>"))) log_page =  String("<div class='text'><pre>") + log_page;
    log_page.replace("</pre></div>", "");   //delete old "</pre></div>"
    log_page += String(strr) +  String(" client conected IP ") + client_->remoteIP().toString() +  String(" <---> ");
  }
  if (!start_rec)  {  //if first connections
    start_rec = true;
    i2sSampler = new I2SMEMSSampler(I2S_NUM_0, i2s_mic_pins, i2s_config, false);
    // start sampling from i2s device
    i2sSampler->start();
  }
   //if any other connections
   else i2s_start(I2S_PORT);

#if (1)
   AsyncWebServerResponse *response = request->beginChunkedResponse("audio/aac", [](uint8_t *buffer, size_t maxLen, size_t index) -> size_t {
    
    if (aac_stream.available() <= 2048)
        return RESPONSE_TRY_AGAIN;
    maxLen = 1400;   
    //Serial.printf("index %d aviable %d bytes\n\r", index, aac_stream.available());
    return aac_stream.readBytes(buffer, maxLen);
  });
   //response->addHeader("Server","ESP Async Web Server");
   request->send(response);

#else
   AsyncAudioStreamResponse *response = new AsyncAudioStreamResponse();
  //https://gist.github.com/me-no-dev/d34fba51a8f059ac559bf62002e61aa3
    if (!response) {
        request->send(501);
        return;
    }
    request->send(response);
#endif  
  
   free_mem8 =  heap_caps_get_free_size(MALLOC_CAP_8BIT);
   free_mem32 = heap_caps_get_free_size(MALLOC_CAP_32BIT);
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
      Serial.printf("Update Start: %s\n", filename.c_str());
      //Update.begin(true);
      if(!Update.begin(UPDATE_SIZE_UNKNOWN)){
        Update.printError(Serial);
      }
    }
    if(!Update.hasError()){
      if(Update.write(data, len) != len){
        Update.printError(Serial);
      }
    }
    if(final){
      if(Update.end(true)){
        Serial.printf("Update Success: %uB\n", index+len);
      } else {
        Update.printError(Serial);
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

void configWiFi(WiFi_STA_IPConfig in_WM_STA_IPconfig)
{
  #if USE_CONFIGURABLE_DNS  
    // Set static IP, Gateway, Subnetmask, DNS1 and DNS2. New in v1.0.5
    WiFi.config(in_WM_STA_IPconfig._sta_static_ip, in_WM_STA_IPconfig._sta_static_gw, in_WM_STA_IPconfig._sta_static_sn, in_WM_STA_IPconfig._sta_static_dns1, in_WM_STA_IPconfig._sta_static_dns2);  
  #else
    // Set static IP, Gateway, Subnetmask, Use auto DNS1 and DNS2.
    WiFi.config(in_WM_STA_IPconfig._sta_static_ip, in_WM_STA_IPconfig._sta_static_gw, in_WM_STA_IPconfig._sta_static_sn);
  #endif 
}

void AAC_process(void)
{
  send_count = 0;
  int samples_read;
  uint32_t readed_bytes = 0;

      samples_read = i2sSampler->read(samples, SAMPLE_SIZE*2, BITS_PER_SAMPLE);
      readed_bytes += samples_read * (BITS_PER_SAMPLE/8);
      //Serial.printf("samples_read = %d \n\r", samples_read);
      if (samples_read) aac->write((uint8_t*)samples, samples_read * (BITS_PER_SAMPLE/8));      
}

void logMemory() {
  log_d("Used PSRAM: %d", ESP.getPsramSize() - ESP.getFreePsram());
}

//Fills the memory in 32-bit words for speed.
static void fill_mem_seed(int seed, void *mem, int len, int block_size)
{
    uint32_t *p = (uint32_t *)mem;
    unsigned int rseed = seed ^ 0xa5a5a5a5;
    for (int i = 0; i < len/block_size; i++) {
        rseed = i ^ rseed;
        for (int ii = 0; ii < block_size/4; ii++) {
          *p++ = rand_r(&rseed);
        }
    }
}

//Check the memory filled by fill_mem_seed. Returns true if the data matches the data
//that fill_mem_seed wrote (when given the same seed).
//Returns true if there's a match, false when the region differs from what should be there.
static bool check_mem_seed(int seed, void *mem, int len, int block_size)
{
    uint32_t *p = (uint32_t *)mem;
    unsigned int rseed = seed ^ 0xa5a5a5a5;
    for (int i = 0; i < len/block_size; i++) {
        rseed = i ^ rseed;
        for (int ii = 0; ii < block_size/4; ii++) {
          uint32_t ex = rand_r(&rseed);
          if (ex != *p) {
             Serial.printf("check_mem_seed: %x has 0x%08x expected 0x%08x\n", i*block_size+((char*)p-(char*)mem), *p, ex);
             return false;
          }
          p++;
        }
    }
    return true;
}

bool writeConfigFile()
{
  Serial.println("Saving config file");

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
    Serial.println("Failed to open config file for writing");
    return false;
  }

#if (ARDUINOJSON_VERSION_MAJOR >= 6)
  serializeJsonPretty(json, Serial);
  // Write data to file and close it
  serializeJson(json, f);
#else
  json.prettyPrintTo(Serial);
  // Write data to file and close it
  json.printTo(f);
#endif

  f.close();

  Serial.println("\nConfig file was successfully saved");
  return true;
}

bool readConfigFile(void)
{
  // this opens the config file in read-mode
  File f = FileFS.open(CONFIG_FILE, "r");

  if (!f)
  {
    Serial.println("Configuration file not found");
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
      Serial.println("JSON parseObject() failed");
      return false;
    }
    serializeJson(json, Serial);
#else
    DynamicJsonBuffer jsonBuffer;
    // Parse JSON string
    JsonObject& json = jsonBuffer.parseObject(buf.get());
    // Test if parsing succeeds.
    if (!json.success())
    {
      Serial.println("JSON parseObject() failed");
      return false;
    }
    json.printTo(Serial);
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
  Serial.println("\nConfig file was successfully parsed");
  return true;
}


//return true - ok, false - error
bool mount_fs(void) {

  if (!FileFS.begin(true)) {
    Serial.println(F("LittleFS failed! Already tried formatting."));
  
    if (!FileFS.begin())
    {     
      // prevents debug info from the library to hide err message.
      delay(100);
      Serial.println(F("LittleFS failed!. Please use SPIFFS or EEPROM. Stay forever"));
      return false;
    }
    
  }
   else {
     Serial.println(F("LittleFS mount OK!"));
     return true;
   }
  return true;
}

