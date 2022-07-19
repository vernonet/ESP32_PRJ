# wifi microphone on esp32

  In project uses an  I2S microphone SPH0645(INMP441), ESP-WROOM-32 or ESP32-A1S.
  To play audio insert in vlc  "http:\\\wifi-mic.local:8080\rec.wav"  or "http:\\\ip_of_mic:8080\rec.wav".
  Initially, you need to connect to the access point "wifi-mic" and configure the access settings.
  You can find the ip address from the UART terminal, or from the program on your smartphone - "bonjour browser".
  The microphone works much better than a microphone based on ESP8266.
  The project was created in  PlatformIO IDE with VS Code (framework arduino), project can also be compiled in arduino ide.
  Used source code <a href="https://github.com/atomic14/esp32_audio" rel="nofollow">esp32_audio</a>, , <a href="https://github.com/khoih-prog/ESPAsync_WiFiManager"    rel="nofollow">ESPAsync_WiFiManager</a>, <a href="https://github.com/me-no-dev/ESPAsyncWebServer" rel="nofollow">ESPAsyncWebServer</a>, <a href="https://github.com/me-no-dev/AsyncTCP" rel="nofollow">AsyncTCP</a>, <a href="https://github.com/pschatzmann/arduino-audio-tools" rel="nofollow">Arduino Audio Tools</a>.
  Tested on Arduino ESP32 release 1.0.6 ???, 2.0.0 - 2.0.3....
  Tested sample rate 22500Hz.
  Bit-Depth: 16.
  
  Microphone connection: 
  - I2S_MIC_SERIAL_CLOCK      (SCK)      GPIO_NUM_32
  - I2S_MIC_LEFT_RIGHT_CLOCK  (WS)       GPIO_NUM_25
  - I2S_MIC_SERIAL_DATA       (SD)       GPIO_NUM_33
  - L/R pin of mic connect to ground.
  
  serial_audio.exe (<a href="https://github.com/vernonet/serial_audio" rel="nofollow">serial_audio</a>)- program for wifi_microphone,  it allows you to testing  microphone through a serial port (uncomment  "//#define NO_WIFI").
  
  The following libraries need to be installed:
  - <a href="https://github.com/khoih-prog/ESP_WiFiManager" rel="nofollow">ESP_WiFiManager</a>
  - <a href="https://github.com/arduino-libraries/NTPClient" rel="nofollow">NTPClient</a>
  - <a href="https://github.com/bblanchon/ArduinoJson" rel="nofollow">ArduinoJson</a>  

  I rebuilt the arduino libraries, and put them in a separate folder, these files need to be replaced.
  Rebuild utility - <a href="https://github.com/espressif/esp32-arduino-lib-builder" rel="nofollow">esp32-arduino-lib-builder</a>  
  When listening to the microphone, noise may appear (poor quality of the Internet connection?), this problem has not yet been resolved.



# License

  This software is provided under the  <a href="http://unlicense.org/" rel="nofollow">UNLICENSE</a>

