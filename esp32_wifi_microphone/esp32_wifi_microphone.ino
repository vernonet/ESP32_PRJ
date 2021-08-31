#if !(defined(ESP32) )
  #error This code is intended to run on the ESP32 platform! Please check your Tools->Board setting.
#endif
#include <ESPAsync_WiFiManager.h>              //https://github.com/khoih-prog/ESPAsync_WiFiManager
#if !( USING_ESP32_S2 || USING_ESP32_C3 )
  DNSServer dnsServer;
#endif 
#include <esp_wifi.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include "I2SMEMSSampler.h"
#include <ESPmDNS.h>
#include <EEPROM.h>
#include <NTPClient.h>
#include <driver/i2s.h>
#include "sample_wav.h"   //test wav  file



// I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S I2S
// SPH0645, INMP441 MEMS MICROPHONE
//insert in vlc  "http://wifi-mic.local:8080/rec.wav"  or "http://ip:8080/rec.wav"   ip - ip address of mic

//#define NO_WIFI                  //testing the microphone using the "serial_audio.exe" program.

#define SAMPLE_BUFFER_SIZE         (600)    //Length of one buffer, in 32-bit words.  //512 //300  
#define BUF_CNT                    8      // Number of buffers in the I2S circular buffer   
#define SAMPLE_RATE                22050  //22050 //33000  //16000
#define BITS_PER_SAMPLE            (16)  // 16 or 24  
#define I2S_MIC_CHANNEL            I2S_CHANNEL_FMT_ONLY_LEFT // most microphones will probably default to left channel but you may need to tie the L/R pin low
#define I2S_MIC_SERIAL_CLOCK       GPIO_NUM_32
#define I2S_MIC_LEFT_RIGHT_CLOCK   GPIO_NUM_25
#define I2S_MIC_SERIAL_DATA        GPIO_NUM_33
#define SERVER_PORT                8080
#define BAUDRATE                   115200//921600//921600    //500000 //baud one of 300, 600, 1200, 2400, 4800, 9600, 19200, 31250, 38400, 57600, 74880, 115200, 230400, 256000, 460800, 921600, 1843200, 3686400
#define SIGNAL_GAIN                (0)// 0 - max gain, 2 - no gain
#define REC_TIME                   (600) //sec
#define NUM_CPY                    ((SAMPLE_RATE * BITS_PER_SAMPLE / 8 * REC_TIME)/SAMPLE_BUFFER_SIZE)//


#define WIFI_CONNECT_TIMEOUT      30000L
#define WHILE_LOOP_DELAY          200L
#define WHILE_LOOP_STEPS          (WIFI_CONNECT_TIMEOUT / ( 3 * WHILE_LOOP_DELAY ))

#define reverse_bytes(val)        ((val & 0x000000FFU) << 24 | (val & 0x0000FF00U) << 8 |(val & 0x00FF0000U) >> 8 | (val & 0xFF000000U) >> 24)
#define reverse_halfword(val)     ((val & 0x00FFU) << 24 |  (val & 0xFF000000U) >> 24)
#define reverse_sample16(val)     (((val >> 8)&0xFF) | ((val << 8)&0xFF00))

#define LED_ON            LOW
#define LED_OFF           HIGH
#define PIN_LED           2         // 
//#define ESP_getChipId()   (ESP.getChipId())

const i2s_port_t I2S_PORT = I2S_NUM_0;
uint8_t signal_gain = SIGNAL_GAIN;
volatile uint32_t wait_reset = 0;
uint8_t temp_buf[44] __attribute__((aligned(4))); //for header
uint8_t *temp_buf_p = temp_buf;
uint32_t content_buf_sze;
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
String ssid = "wifi_mic_ap";
const char* password = "testtest";
// SSID and PW for your Router
String Router_SSID;
String Router_Pass;
const int TRIGGER_PIN3 = GPIO_NUM_2;      // short  contact to ground to enter config portal
volatile bool con_flag = false, start_rec = false,  sample_gotov = false;
volatile unsigned int tme = 0;
uint32_t send_count;



// Define NTP Client to get time
const long utcOffsetInSeconds = 2 * 60 * 60; //+2;
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", utcOffsetInSeconds);

AsyncClient *client_;
AsyncWebServer server(SERVER_PORT);
AsyncWebServer webServer_async(80);
AsyncWebServerRequest *request_;
AsyncWebServerResponse *response_;
uint8_t * cur_audio_buf_ptr;

void launchWeb(int webtype);
void i2sMemsToClientTask(void *param);
void handleNotFound(AsyncWebServerRequest *request);


I2SSampler *i2sSampler = NULL;
i2s_config_t i2s_config = {
  .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX), // Receive, not transfer
  .sample_rate = SAMPLE_RATE,
  .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT, // could only get it to work with 32bits
  .channel_format = I2S_MIC_CHANNEL, // although the SEL config should be left, it seems to transmit on right
  .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
  //.communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_STAND_I2S),
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

#if (BITS_PER_SAMPLE != 16 && BITS_PER_SAMPLE != 24)
  #error "This BITS_PER_SAMPLE not suported!!!!"
#endif  

  pinMode(TRIGGER_PIN3, INPUT_PULLUP);   //to enter config portal

#ifdef NO_WIFI
  WiFi.mode(WIFI_OFF);
#endif
  delay(500);
  Serial.begin(BAUDRATE);
  Serial.println("\nStarting");
#ifndef NO_WIFI
  unsigned long startedAt = millis();
  //ESPAsync_WiFiManager ESPAsync_wifiManager("wifi-mic");
//  #if ( USING_ESP32_S2 || USING_ESP32_C3 )
//  ESPAsync_WiFiManager ESPAsync_wifiManager(&webServer_async, NULL, "wifi-mic");
//#else
//  ESPAsync_WiFiManager ESPAsync_wifiManager(&webServer_async, &dnsServer, "wifi-mic-ap");
//#endif  
//  ESPAsync_wifiManager.setMinimumSignalQuality(-1);
  // Set static IP, Gateway, Subnetmask, DNS1 and DNS2. New in v1.0.5
  // ESPAsync_wifiManager.setSTAStaticIPConfig(IPAddress(192,168,2,114), IPAddress(192,168,2,1), IPAddress(255,255,255,0), IPAddress(192,168,2,1), IPAddress(8,8,8,8));
  // Have to create a new function to store in EEPROM/SPIFFS for this purpose
//  Router_SSID = ESPAsync_wifiManager.WiFi_SSID();
//  Router_Pass = ESPAsync_wifiManager.WiFi_Pass();
  //Remove this line if you do not want to see WiFi password printed
//  Serial.println("Stored: SSID = " + Router_SSID + ", Pass = " + Router_Pass);
  // SSID to uppercase
///  ssid.toUpperCase();
//  if (Router_SSID == "")
//  {
//    Serial.println("We haven't got any access point credentials, so get them now");
//    //digitalWrite(PIN_LED, LED_ON); // Turn led on as we are in configuration mode.
//
//    //it starts an access point
//    //and goes into a blocking loop awaiting configuration
//    if (!ESPAsync_wifiManager.startConfigPortal((const char *) ssid.c_str(), password))
//      Serial.println("Not connected to WiFi but continuing anyway.");
//    else {
//      Serial.println("WiFi connected...:)");
//      //launchWeb(0);
//    }
//  }

  //digitalWrite(PIN_LED, LED_OFF); // Turn led off as we are not in configuration mode.

  startedAt = millis();

//  while ( (WiFi.status() != WL_CONNECTED) && (millis() - startedAt < WIFI_CONNECT_TIMEOUT ) )
//  {
//   // WiFi.mode(WIFI_STA);
//    WiFi.hostname("wifi-mic");
//    //WiFi.persistent (true);
//    // We start by connecting to a WiFi network
//
//    Serial.print("Connecting to ");
//    Serial.println(Router_SSID);
//
//    WiFi.begin(Router_SSID.c_str(), Router_Pass.c_str());
//
//    int i = 0;
//    while ((!WiFi.status() || WiFi.status() >= WL_DISCONNECTED) && i++ < WHILE_LOOP_STEPS)
//    {
//      delay(WHILE_LOOP_DELAY);
//    }
//  }
//  WiFi.disconnect(true);
//  ~ESPAsync_WiFiManager();
  WiFi.mode(WIFI_STA);
  WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE);
  WiFi.setHostname("wifi-mic");
  //WiFi.begin("PRJ_Frojo_2G", "motorola908");
  WiFi.begin("Tenda_BCF280", "motorola908");
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi..");
  }

  Serial.print("After waiting ");
  Serial.print((millis() - startedAt) / 1000);
  Serial.print(" secs more in setup(), connection result is ");

  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.print("connected. Local IP: ");
    Serial.println(WiFi.localIP());
    con_flag = true;
//    timeClient.begin();
//    // GMT +1 = 3600
//    // GMT +8 = 28800
//    // GMT -1 = -3600
//    // GMT 0 = 0
//    timeClient.setTimeOffset(3600 * 3); //for GMT+3
//    timeClient.update();
    launchWeb(0);
  }
//  else
//    Serial.println(ESPAsync_wifiManager.getStatus(WiFi.status()));

#endif
}

//loop  loop  loop  loop  loop  loop  loop  loop  loop  loop  loop  loop  loop  loop  loop  loop  loop  loop  loop
void loop(void) {
  int32_t value;
  char buff[9];

#ifndef NO_WIFI

//  if (con_flag) {
//    //server.handleClient();
//    if (tme < 1) {
//      Serial.print(daysOfTheWeek[timeClient.getDay()]);
//      Serial.print(", ");
//      Serial.printf("%02d", timeClient.getHours());
//      Serial.print(":");
//      Serial.printf("%02d", timeClient.getMinutes());
//      Serial.print("\n\r");
//      timeClient.update();
//      if (WiFi.status() != WL_CONNECTED) {
//        Serial.println("NO WIFI, try to connect");
//        WiFi.mode(WIFI_STA);
//        WiFi.hostname("wifi-mic");
//        WiFi.persistent (true);
//        //WiFi.setOutputPower(0);
//        WiFi.begin(Router_SSID.c_str(), Router_Pass.c_str());
//      }
//    }
//    tme++;
//    if (tme > 1000000) tme = 0;
//  }

  // is configuration portal requested?
  if (digitalRead(TRIGGER_PIN3) == LOW)
  {
    Serial.println("\nConfiguration portal requested.");
    //digitalWrite(PIN_LED, LED_ON); // turn the LED on by making the voltage LOW to tell us we are in configuration mode.

    //Local intialization. Once its business is done, there is no need to keep it around
    ESPAsync_WiFiManager ESPAsync_wifiManager(&webServer_async, &dnsServer, "wifi-mic-ap");

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

    //it starts an access point
    //and goes into a blocking loop awaiting configuration
    if (!ESPAsync_wifiManager.startConfigPortal((const char *) ssid.c_str(), password))
    {
      Serial.println("Not connected to WiFi but continuing anyway.");
    }
    else
    {
      //if you get here you have connected to the WiFi
      Serial.println("connected...yeey :)");
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
  i2sSampler->start(I2S_PORT, i2s_config, BITS_PER_SAMPLE, BITS_PER_SAMPLE, (SAMPLE_BUFFER_SIZE * (BITS_PER_SAMPLE >> 3))*4, i2sMemsToUartTaskHandle); //32768
  while (true) {};

#endif

}

void createWebServer(int webtype)
{
  if ( webtype == 1 ) {  //softAP
    //

  } else if (webtype == 0) {  //are configured

    

//    if (MDNS.begin("wifi-mic")) {
//      Serial.println("MDNS responder started");
//    }
//    else {
//      Serial.println("Error setting up MDNS responder!");
//    }
//    MDNS.addService("http", "tcp", 8080);  //orig 80
//    MDNS.addService("osc", "udp", 4500);

   server.onNotFound(handleNotFound);
   server.on("/rec.wav", HTTP_GET, handle_rec_wav); 

    if (!start_rec)  {  //if first connections
    
    i2sSampler = new I2SMEMSSampler(i2s_mic_pins, false);
    // set up the i2s sample writer task
    TaskHandle_t i2sMemsToClientTaskHandle;
    xTaskCreatePinnedToCore(i2sMemsToClientTask, "I2S Writer Task", 4096, i2sSampler, 1, &i2sMemsToClientTaskHandle, 1);
    // start sampling from i2s device
    i2sSampler->start(I2S_PORT, i2s_config, BITS_PER_SAMPLE, (SAMPLE_BUFFER_SIZE * (BITS_PER_SAMPLE >> 3))*10, i2sMemsToClientTaskHandle); //32768  //1200   1800
  }
   //if any other connections
   else i2s_start(I2S_PORT);
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



void handleNotFound(AsyncWebServerRequest *request) {
  request->send(404, "text/plain", "404: Not found");
}

void handle_rec_wav(AsyncWebServerRequest *request) {

  client_ = request->client();
  if (client_) {
    Serial.print(" New client conected - IP ");
    Serial.print(client_->remoteIP());
    Serial.printf(" PORT %d\n\r", client_->remotePort());
  }
//  if (!start_rec)  {  //if first connections
//    start_rec = true;
//    i2sSampler = new I2SMEMSSampler(i2s_mic_pins, false);
//    // set up the i2s sample writer task
//    TaskHandle_t i2sMemsToClientTaskHandle;
//    xTaskCreatePinnedToCore(i2sMemsToClientTask, "I2S Writer Task", 4096, i2sSampler, 1, &i2sMemsToClientTaskHandle, 1);
//    // start sampling from i2s device
//    i2sSampler->start(I2S_PORT, i2s_config, BITS_PER_SAMPLE, (SAMPLE_BUFFER_SIZE * (BITS_PER_SAMPLE >> 3))*4, i2sMemsToClientTaskHandle); //32768  //1200   1800
//  }
//   //if any other connections
//   else i2s_start(I2S_PORT);

   //request->setContentLength(SAMPLE_BUFFER_SIZE * (BITS_PER_SAMPLE / 8)*NUM_CPY + 44);

   content_buf_sze = (SAMPLE_BUFFER_SIZE * (BITS_PER_SAMPLE >> 3))*10;

  memcpy(&temp_buf[0], &test_wav[0],  44);
  *(uint32_t *)&temp_buf[4]  = (SAMPLE_BUFFER_SIZE) * NUM_CPY + 36; //long size
  *(uint32_t *)&temp_buf[24]  = SAMPLE_RATE;
  *(uint32_t *)&temp_buf[28]  = SAMPLE_RATE * (BITS_PER_SAMPLE / 8);
  *(uint32_t *)&temp_buf[32]  = (uint32_t)(BITS_PER_SAMPLE >> 3) + ((BITS_PER_SAMPLE << 16) & 0xFF0000);
  *(uint32_t *)&temp_buf[40] = (SAMPLE_BUFFER_SIZE) * NUM_CPY;    //long chunkSize

//  request->send_P(200, "audio/x-wav",(const uint8_t *)test_wav, sizeof test_wav);

//  sample_gotov = true;
//  while(!sample_gotov) {};
  start_rec = true;
  send_count = 0;
  AsyncWebServerResponse *response = request->beginResponse_P(200, "audio/x-wav", (const uint8_t *)temp_buf, 1000);
  response->setContentLength((SAMPLE_BUFFER_SIZE*(BITS_PER_SAMPLE / 8)*NUM_CPY + 44));
  request->send(response);

  send_count = 0;
  //while(1){}

    

//  while (true) {
//    if (!client_ || send_count >= (SAMPLE_BUFFER_SIZE * NUM_CPY)) {
//      Serial.print(" client disconected\n\r");
//      //ESP.restart();
//      i2s_stop(I2S_PORT);
//      break;
//    }
//  }
}



// Task to send samples
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
        sample_gotov = true;
         if (start_rec)  {
          Serial.printf("                                               send content_p\n\r");
        //request_->send_P(200, "audio/x-wav", (const uint8_t *)sampler->getCapturedAudioBuffer(), sampler->getBufferSizeInBytes() );
        cur_audio_buf_ptr = (uint8_t *)sampler->getCapturedAudioBuffer();
        sample_gotov = true;
        send_count += sampler->getBufferSizeInBytes();
        }
      }
    }
  }
}

// Task to send samples
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
      //server.sendContent_P((char *)sampler->getCapturedAudioBuffer(), sampler->getBufferSizeInBytes());
    }
  }
}

//server.on("/sinewave", HTTP_GET, [](AsyncWebServerRequest * request)  {
//    AsyncWebServerResponse* response = request->beginChunkedResponse(
//      "audio/wave",
//      [](uint8_t* buffer, size_t maxLen, size_t index){
//        size_t len;
//        // first chunk does not have the full maxLen available
//        // http-headers(?) Send these plus the WAVE header first.
//        // after that just the audio data
//        if(index == 0){
//          uint8_t* pcm_bytes = static_cast<uint8_t*>(static_cast<void*>(&header_pcm));
//          for (size_t i = 0; i < sizeof(header_pcm); i++){
//              buffer[i] = pcm_bytes[i];
//          }
//          len = (1064 - maxLen) + sizeof(header_pcm);
//          return len;
//        }
//        else{
//          //maxLen = 2048;
//          size_t len = buffer_size;
//          if (!audio_buffer.isEmpty()){
//            for (size_t i = 0; i < len; i++){
//              buffer[i] = audio_buffer.shift();
//            }
//          }
//          else{
//            while(audio_buffer.isEmpty()){
//              Serial.println("Buffer Empty!");
//            }       
//          }
//         return len;
//        }
//      }
//    );
//    response->addHeader("Cache-Control", "no-cache, no-store, must-revalidate");
//    response->addHeader("Pragma", "no-cache");
//    response->addHeader("Expires", "-1");
//    request->send(response);
//  });

//AsyncWebServerResponse *response = request->beginChunkedResponse("text/plain", [](uint8_t *buffer, size_t maxLen, size_t index) -> size_t {
//  //Write up to "maxLen" bytes into "buffer" and return the amount written.
//  //index equals the amount of bytes that have been already sent
//  //You will be asked for more data until 0 is returned
//  //Keep in mind that you can not delay or yield waiting for more data!
//  return mySource.read(buffer, maxLen);
//});
//response->addHeader("Server","ESP Async Web Server");
//request->send(response);
