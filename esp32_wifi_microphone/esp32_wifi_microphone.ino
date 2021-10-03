#if defined(ESP8266)
  #include <ESP8266WiFi.h>  //ESP8266 Core WiFi Library 
  #include <DNSServer.h>
  #include <ESP8266WebServer.h>
#else      //for esp32    
  #include <esp_wifi.h>
  #include <WiFi.h>
  #include <WiFiClient.h>
#endif
#include <core_version.h>
#define USE_AVAILABLE_PAGES     true
#include <ESP_WiFiManager.h>
#include "I2SMEMSSampler.h"
#include <ESPmDNS.h>
#include <NTPClient.h>
#include <driver/i2s.h>
#include "sample_wav.h"   //test wav  file
#include <Update.h>


extern "C" {
uint8_t temprature_sens_read();
}

// I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S
// SPH0645, INMP441 MEMS MICROPHONE
//insert in vlc  "http:\\wifi-mic.local:8080\rec.wav"  or "http:\\ip:8080\rec.wav"   ip - ip address of mic
//for ota update  http://wifi-mic.local:8080/upd_frm     admin:admin
//for info http://wifi-mic.local:8080/info

//#define NO_WIFI                  //testing the microphone using the "serial_audio.exe" program.

#define SAMPLE_BUFFER_SIZE         600    //Length of one buffer, in 32-bit words.  //512 //300  
#define BUF_CNT                    8      // Number of buffers in the I2S circular buffer   
#define SAMPLE_RATE                22050  //22050 //33000  //16000
#define BITS_PER_SAMPLE            (16)  // 16 or 24  
#define I2S_MIC_CHANNEL            I2S_CHANNEL_FMT_ONLY_LEFT // most microphones will probably default to left channel but you may need to tie the L/R pin low
#define I2S_MIC_SERIAL_CLOCK       GPIO_NUM_32
#define I2S_MIC_LEFT_RIGHT_CLOCK   GPIO_NUM_25
#define I2S_MIC_SERIAL_DATA        GPIO_NUM_33
#define SERVER_PORT                8080
#ifdef  NO_WIFI
 #define BAUDRATE                  921600//921600//921600     //baud one of 300, 600, 1200, 2400, 4800, 9600, 19200, 31250, 38400, 57600, 74880, 115200, 230400, 256000, 460800, 921600, 1843200, 3686400
#else
 #define BAUDRATE                  115200
#endif
#define SIGNAL_GAIN                (0)// 0 - max gain, 2 - no gain
#define REC_TIME                   (6000) //sec
#define NUM_CPY                    ((SAMPLE_RATE * BITS_PER_SAMPLE / 8 * REC_TIME)/SAMPLE_BUFFER_SIZE)//
#define USER_OTA                   "admin"
#define PASS_OTA                   "admin"
#define LOG_SIZE                   (100000)


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


#define reverse_bytes(val)        ((val & 0x000000FFU) << 24 | (val & 0x0000FF00U) << 8 |(val & 0x00FF0000U) >> 8 | (val & 0xFF000000U) >> 24)
#define reverse_halfword(val)     ((val & 0x00FFU) << 24 |  (val & 0xFF000000U) >> 24)
#define reverse_sample16(val)     (((val >> 8)&0xFF) | ((val << 8)&0xFF00))

#define param_info(str, nme, z)   str +=F("<tr><td>");\
                                  str +=F(nme);\
                                  str +=F("</td><td>");\
                                  str += z;\
                                  str += F("</td></tr>");

#define LED_ON                     HIGH
#define LED_OFF                    LOW
#define PIN_LED                    GPIO_NUM_22         // 
#ifdef ARDUINO_ESP32_RELEASE_1_0_6 
  #define ESP_getChipId()   (ESP.getChipId())
#endif

#ifndef ARDUINO_ESP32_RELEASE_1_0_6
//  #error Recomended Arduino core for the ESP32 v1.06, v.2.00.. get always 255.255.255.255 as IP address from WiFi.localIP()
#endif  

const i2s_port_t I2S_PORT = I2S_NUM_0;
uint8_t signal_gain = SIGNAL_GAIN;
volatile uint32_t wait_reset = 0;
uint8_t temp_buf_f[128] __attribute__((aligned(4))); //for header
String ssid = "wifi_mic_ap";
const char* password = "testtest";
// SSID and PW for your Router
String Router_SSID;
String Router_Pass;
const int TRIGGER_PIN3 = GPIO_NUM_2;      // short  contact to ground to enter config portal
volatile bool con_flag = false, start_rec = false;
volatile unsigned int tme = 0;
uint32_t send_count;
WiFi_STA_IPConfig  WM_STA_IPconfig_;
const char compile_date[] = __DATE__ " " __TIME__;
bool authenticate=false;
char temp_[5];
char date_[10];
String log_page;

WebServer server(SERVER_PORT);
WiFiClient client_;
// Define NTP Client to get time
const long utcOffsetInSeconds = 2 * 60 * 60; //+2;
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", utcOffsetInSeconds);

void launchWeb(int webtype);
void i2sMemsToClientTask(void *param);
void configWiFi(WiFi_STA_IPConfig in_WM_STA_IPconfig);
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
  // .tx_desc_auto_clear = true,
  .use_apll = true
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
  pinMode(PIN_LED, OUTPUT);              //

#ifdef NO_WIFI
  WiFi.mode(WIFI_OFF);
#endif
  delay(500);
  Serial.begin(BAUDRATE);
  Serial.print("\nStarting...\n\rcompile date: ");
  Serial.println(compile_date);
#ifndef NO_WIFI
  unsigned long startedAt = millis();
  ESP_WiFiManager ESP_wifiManager("wifi-mic");
  ESP_wifiManager.setMinimumSignalQuality(-1);
  // Set static IP, Gateway, Subnetmask, DNS1 and DNS2. New in v1.0.5
  // ESP_wifiManager.setSTAStaticIPConfig(IPAddress(192,168,2,114), IPAddress(192,168,2,1), IPAddress(255,255,255,0), IPAddress(192,168,2,1), IPAddress(8,8,8,8));
  // Have to create a new function to store in EEPROM/SPIFFS for this purpose
  Router_SSID = ESP_wifiManager.WiFi_SSID();
  Router_Pass = ESP_wifiManager.WiFi_Pass();  
  //ESP_wifiManager.getSTAStaticIPConfig(WM_STA_IPconfig_);
  //Remove this line if you do not want to see WiFi password printed
  Serial.println("Stored: SSID = " + Router_SSID + ", Pass = " + Router_Pass);
  // SSID to uppercase
  ssid.toUpperCase();
  if (Router_SSID == "")
  {
    Serial.println("We haven't got any access point credentials, so get them now");
    digitalWrite(PIN_LED, LED_ON); // Turn led on as we are in configuration mode.

    //it starts an access point
    //and goes into a blocking loop awaiting configuration
    if (!ESP_wifiManager.startConfigPortal((const char *) ssid.c_str(), password))
      Serial.println("Not connected to WiFi but continuing anyway.");
    else {
      Serial.println("WiFi connected...:)");
      //launchWeb(0);
    }
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
    Serial.println(ESP_wifiManager.getStatus(WiFi.status()));

#endif
}

//loop  loop  loop  loop  loop  loop  loop  loop  loop  loop  loop  loop  loop  loop  loop  loop  loop  loop  loop
void loop(void) {
  int32_t value;
  char buff[9];

#ifndef NO_WIFI

  if (con_flag) {
    server.handleClient();
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
    if (tme > 100000) tme = 0;
  }

  // is configuration portal requested?
  if (digitalRead(TRIGGER_PIN3) == LOW)
  {
    Serial.println("\nConfiguration portal requested.");
    digitalWrite(PIN_LED, LED_ON); // turn the LED on by making the voltage LOW to tell us we are in configuration mode.

    //Local intialization. Once its business is done, there is no need to keep it around
    ESP_WiFiManager ESP_wifiManager;

    //Check if there is stored WiFi router/password credentials.
    //If not found, device will remain in configuration mode until switched off via webserver.
    Serial.print("Opening configuration portal. ");
    Router_SSID = ESP_wifiManager.WiFi_SSID();
    if (Router_SSID != "")
    {
      ESP_wifiManager.setConfigPortalTimeout(60); //If no access point name has been previously entered disable timeout.
      Serial.println("Got stored Credentials. Timeout 60s");
    }
    else
      Serial.println("No stored Credentials. No timeout");

    //it starts an access point
    //and goes into a blocking loop awaiting configuration
    if (!ESP_wifiManager.startConfigPortal((const char *) ssid.c_str(), password))
    {
      Serial.println("Not connected to WiFi but continuing anyway.");
    }
    else
    {
      //if you get here you have connected to the WiFi
      Serial.println("connected...yeey :):):)");
    }

    digitalWrite(PIN_LED, LED_OFF); // Turn led off as we are not in configuration mode.
  }
  else if (!(con_flag)) {
    wait_reset++;
    delay(100);
    if (wait_reset >= 1800) ESP.restart();
  }
#else

  start_rec = true;
  i2sSampler = new I2SMEMSSampler(i2s_mic_pins, false);
  // set up the i2s sample writer task
  TaskHandle_t i2sMemsToUartTaskHandle;
  xTaskCreatePinnedToCore(i2sMemsToUartTask, "I2S Writer Task", 4096, i2sSampler, 1, &i2sMemsToUartTaskHandle, 1);
  // start sampling from i2s device
  i2sSampler->start(I2S_PORT, i2s_config, BITS_PER_SAMPLE, BITS_PER_SAMPLE, SAMPLE_BUFFER_SIZE * (BITS_PER_SAMPLE >> 3), i2sMemsToUartTaskHandle); //32768
  while (true) {};

#endif

}

void createWebServer(int webtype)
{
  if ( webtype == 1 ) {  //softAP
    //

  } else if (webtype == 0) {  //are configured

    if (MDNS.begin("wifi-mic")) {
      Serial.println("MDNS responder started");
    }
    else {
      Serial.println("Error setting up MDNS responder!");
    }
    MDNS.addService("http", "tcp", 8080);  //orig 80
    //MDNS.addService("osc", "udp", 4500);
    //MDNS.addService("telnet", "tcp", 23);// Telnet server RemoteDebug

  server.onNotFound(handleNotFound);
  /*return index page which is stored in serverIndex */
  server.on("/upd_frm", HTTP_GET, handle_upd_frm);//for update frm http://wifi-mic.local:8080/upd_frm     admin:admin
  //server.on("/serverIndex", handle_serverIndex);
  /*handling uploading firmware file */
  server.on("/update", HTTP_POST, handle_update, handle_upload);
  server.on("/info", HTTP_GET, handle_info);
  server.on("/rec.wav", HTTP_GET, handle_rec_wav);
  server.on("/log", HTTP_GET, handle_log);
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
  server.begin();
  Serial.println("Server started");
}



void handleNotFound() {
  server.send(404, "text/plain", "404: Not found"); // Send HTTP status 404 (Not Found) when there's no handler for the URI in the request
}

void handle_rec_wav() {

  client_ = server.client();
  if (client_) {
    Serial.print("New client conected - IP ");
    Serial.print(client_.remoteIP());
    Serial.printf(" PORT %d\n\r", client_.remotePort());
    char strr[18];
    sprintf(strr, "%s %02d:%02d", date_, timeClient.getHours(), timeClient.getMinutes());
    if (log_page. length() >= 100000) log_page = "";
    //if (log_page. length() == 0) log_page = F("<div class='text'><pre>");
    //if (log_page.endsWith("</pre></div>")) log_page.remove(log_page.length()-12, 12);
    if (!(log_page.startsWith("<div class='text'><pre>"))) log_page =  String("<div class='text'><pre>") + log_page;
    log_page.replace("</pre></div>", "");   //delete old "</pre></div>"
    log_page += String(strr) +  String(" client conected IP ") + client_.remoteIP().toString() +  String(" <---> ");
  }
  if (!start_rec)  {  //if first connections
    start_rec = true;
    i2sSampler = new I2SMEMSSampler(i2s_mic_pins, false);
    // set up the i2s sample writer task
    TaskHandle_t i2sMemsToClientTaskHandle;
    xTaskCreatePinnedToCore(i2sMemsToClientTask, "I2S Writer Task", 4096, i2sSampler, 1, &i2sMemsToClientTaskHandle, 1);
    // start sampling from i2s device
    i2sSampler->start(I2S_PORT, i2s_config, BITS_PER_SAMPLE, SAMPLE_BUFFER_SIZE * (BITS_PER_SAMPLE >> 3), i2sMemsToClientTaskHandle); //32768  //1200   1800
  }
   //if any other connections
   else i2s_start(I2S_PORT);

  server.setContentLength(SAMPLE_BUFFER_SIZE * (BITS_PER_SAMPLE / 8)*NUM_CPY + 44);

  memcpy(&temp_buf_f[0], &test_wav[0],  44);
  *(uint32_t *)&temp_buf_f[4]  = (SAMPLE_BUFFER_SIZE) * NUM_CPY + 36; //long size
  *(uint32_t *)&temp_buf_f[24]  = SAMPLE_RATE;
  *(uint32_t *)&temp_buf_f[28]  = SAMPLE_RATE * (BITS_PER_SAMPLE / 8);
  *(uint32_t *)&temp_buf_f[32]  = (uint32_t)(BITS_PER_SAMPLE >> 3) + ((BITS_PER_SAMPLE << 16) & 0xFF0000);
  *(uint32_t *)&temp_buf_f[40] = (SAMPLE_BUFFER_SIZE) * NUM_CPY;    //long chunkSize

  server.send_P(200, "audio/x-wav", (const char*)&temp_buf_f[0], 44);

  send_count = 0;

  while (true) {
    if (!client_ || send_count >= (SAMPLE_BUFFER_SIZE * NUM_CPY)) {
      Serial.print("client disconected\n\r");
      char strr[18];
      sprintf(strr, "%02d:%02d", timeClient.getHours(),timeClient.getMinutes());
      log_page += String(strr) +  String(" client disconected ") +  String("\n");
      //ESP.restart();
      i2s_stop(I2S_PORT);
      digitalWrite(PIN_LED, LED_OFF);
      break;
    }
  }
}


void handle_upd_frm()
{ 
    if (strlen(USER_OTA) > 0) {
      authenticate = true;
    }
    if (authenticate && !server.authenticate(USER_OTA, PASS_OTA)) {
        return server.requestAuthentication();
      }
    server.sendHeader("Connection", "close");
    //server.send(200, "text/html", loginIndex);
    server.send(200, "text/html", serverIndex);
    //String temp__ = String(temp_);
    //server.send(200, "text/html", "<div id='tmp'>core temp : 0" + temp__ + "°C" + "</div>");
}

//void handle_serverIndex() 
//{
//    server.sendHeader("Connection", "close");
//    server.send(200, "text/html", serverIndex);
//}

void handle_upload()
 {
    HTTPUpload& upload = server.upload();
    if (upload.status == UPLOAD_FILE_START) {
      Serial.printf("Update: %s\n", upload.filename.c_str());
      if (!Update.begin(UPDATE_SIZE_UNKNOWN)) { //start with max available size
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_WRITE) {
      /* flashing firmware to ESP*/
      if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_END) {
      if (Update.end(true)) { //true to set the size to the current progress
        Serial.printf("Update Success: %u\nRebooting...\n", upload.totalSize);
      } else {
        Update.printError(Serial);
      }
    }
}

void handle_update()
{   
    if (strlen(USER_OTA) > 0) {
      authenticate = true;
    }
    if (authenticate && !server.authenticate(USER_OTA, PASS_OTA)) {
        return server.requestAuthentication();
      }
    server.sendHeader("Connection", "close");
    server.send(200, "text/plain", (Update.hasError()) ? "UPDATE FAIL" : "UPDATE OK");
    ESP.restart();
 }

 void handle_info() 
{
    if (strlen(USER_OTA) > 0) {
      authenticate = true;
     }
    if (authenticate && !server.authenticate(USER_OTA, PASS_OTA)) {
        return server.requestAuthentication();
     }
    server.sendHeader("Connection", "close");
    
    // http://wifi-mic.local:8080/info
    String page = F("<html><head><meta http-equiv='content-type' content='text/html; charset=utf-8'></head><body>\n");
    page += F(WM_HTTP_STYLE);
    page += F("</head><body><div class='container'><div style='text-align:left;display:inline-block;min-width:260px;'>");
    page += F("<h2>wifi-mic information</h2>");
    page += F("<fieldset>");
    page += F("<h3>Device Data</h3>");
    page += F("<table class=\"table\">");
    page += F("<thead><tr><th>Name</th><th>Value</th></tr></thead><tbody>");

    param_info(page, "ESP32 core version    ", String(ARDUINO_ESP32_RELEASE)); 
    param_info(page, "Compil. date of  FW ", String(compile_date));    
    param_info(page, "Flash Chip Size ", (ESP.getFlashChipSize()/1024) + String(" kbytes"));
    param_info(page, "Access Point IP ", (WiFi.softAPIP().toString()));   
    param_info(page, "Access Point MAC", WiFi.softAPmacAddress()); 
    param_info(page, "SSID", Router_SSID);   
    param_info(page, "Station IP", WiFi.localIP().toString());  
    param_info(page, "Station MAC", WiFi.macAddress());  
    param_info(page, "Core temp", String(temp_) + String(" °C"));  
    param_info(page, "CpuFreq", String(ESP.getCpuFreqMHz()) + String(" MHz"));  
   
    page += F("</tbody></table>");
    page += F("</fieldset>");
    page += F("<a href='/log'>View log</a>");
    page += FPSTR(WM_HTTP_END);
    
    server.send(200, "text/html", (const char *) page.c_str());
}

void handle_log() 
{
    if (strlen(USER_OTA) > 0) {
      authenticate = true;
     }
    if (authenticate && !server.authenticate(USER_OTA, PASS_OTA)) {
        return server.requestAuthentication();
     }
    log_page += F("</pre></div>"); 
    server.sendHeader("Connection", "close");
    server.send(200, "text/html", (const char *)log_page.c_str());
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


// Task to send samples to wifi clients
void i2sMemsToClientTask(void *param)
{
  I2SSampler *sampler = (I2SSampler *)param;
  const TickType_t xMaxBlockTime = pdMS_TO_TICKS(100);

  while (true)
  {
    // wait for some samples to save
    uint32_t ulNotificationValue = ulTaskNotifyTake(pdTRUE, xMaxBlockTime);
    if (ulNotificationValue > 0)
    {
      if (send_count < (SAMPLE_BUFFER_SIZE * NUM_CPY)) {
        uint32_t sze;
        if ((SAMPLE_BUFFER_SIZE * NUM_CPY - send_count) < sampler->getBufferSizeInBytes()) sze = SAMPLE_BUFFER_SIZE * NUM_CPY - send_count;
        else sze = sampler->getBufferSizeInBytes();
        //Serial.printf(" send content_p\n\r");
        server.sendContent_P((char *)sampler->getCapturedAudioBuffer(),  sze);
        send_count += sampler->getBufferSizeInBytes();
        digitalWrite(PIN_LED, !digitalRead(PIN_LED));     //indicate process
      }
    }
  }
}

// Task to send samples to uart
void i2sMemsToUartTask(void *param)
{
  uint32_t temp;
  int16_t  temp_S16;
  I2SSampler *sampler = (I2SSampler *)param;
  const TickType_t xMaxBlockTime = pdMS_TO_TICKS(100);

  while (true)
  {
    // wait for some samples to save
    uint32_t ulNotificationValue = ulTaskNotifyTake(pdTRUE, xMaxBlockTime);
    if (ulNotificationValue > 0)
    {
      Serial.write((char *)sampler->getCapturedAudioBuffer(), sampler->getBufferSizeInBytes());
    }
  }
}
