# wifi microphone with AAC encoding on <a href="https://docs.ai-thinker.com/en/esp32-audio-kit" rel="nofollow">ESP32 audio Kit(ESP32-A1S)</a>.

  In project uses an  I2S microphone SPH0645(INMP441), ESP32 audio Kit (cost ~12$), the onboard microphone can be used, but the sound quality is poor. 
  Since the encoder uses a lot of memory, the use of PSRAM is necessary (included on the board).   
  To play audio insert in vlc  "http:\\\wifi-mic.local:8080\rec.aac" or "http:\\\ip_of_mic:8080\rec.aac".
  Initially, you need to connect to the access point "wifi-mic" and configure the access settings.
  You can find the ip address from the UART terminal, or from the program on your smartphone - "bonjour browser".
  The microphone works much better than a microphone based on ESP8266.  
  Used source code <a href="https://github.com/atomic14/esp32_audio" rel="nofollow">esp32_audio</a>, <a href="https://github.com/khoih-prog/ESP_WiFiManager"    rel="nofollow">ESP_WiFiManager</a>, <a href="https://github.com/pschatzmann/arduino-audio-tools" rel="nofollow">Arduino Audio Tools</a>, <a href="https://github.com/pschatzmann/arduino-fdk-aac" rel="nofollow">Arduino AAC Encoding and Decoding Library</a>.  
  The project was created in  PlatformIO IDE with VS Code (framework arduino), project can also be compiled in arduino ide.   
  To be able to resize the stack, you need to edit file - "C: \ Users \ <user> \ .platformio \ packages \ framework-arduinoespressif32 \ tools \ sdk \ include \ config \ sdkconfig.h" :  
  replace 
  ```
  #define CONFIG_ARDUINO_LOOP_STACK_SIZE 8192  
  ```
  to 
  ```
  #ifndef CONFIG_ARDUINO_LOOP_STACK_SIZE  
   #define CONFIG_ARDUINO_LOOP_STACK_SIZE 8192  
  #endif  
  ``` 
  
  Tested on Arduino ESP32 release 2.0.0, 2.0.1.
  Tested sample rate 16000Hz, 22500Hz.
  Bit-Depth: 16. (encoder supported only 16 bit)
  
  Microphone connection: 
  - I2S_MIC_SERIAL_CLOCK      (SCK)      GPIO_NUM_18
  - I2S_MIC_LEFT_RIGHT_CLOCK  (WS)       GPIO_NUM_5
  - I2S_MIC_SERIAL_DATA       (SD)       GPIO_NUM_23
  - L/R pin of mic connect to ground.
  
  ![Image alt](https://docs.ai-thinker.com/_media/esp32/esp32-audio-kit2.png)

# License

  This software is provided under the  <a href="http://unlicense.org/" rel="nofollow">UNLICENSE</a>

