# wifi microphone on esp32

  In project uses an  I2S microphone SPH0645(INMP441), ESP-WROOM-32.
  To play audio insert in vlc  "http:\\\wifi-mic.local:8080\rec.wav"  or "http:\\\ip_of_mic:8080\rec.wav".
  Initially, you need to connect to the access point "wifi-mic" and configure the access settings.
  You can find the ip address from the UART terminal, or from the program on your smartphone - "bonjour browser".
  The microphone works much better than a microphone based on ESP8266.
  Used source code <a href="https://github.com/atomic14/esp32_audio" rel="nofollow">esp32_audio</a>, <a href="https://github.com/khoih-prog/ESP_WiFiManager" rel="nofollow">ESP_WiFiManager</a>.
  Tested on Arduino ESP32 release 1.0.6, 2.0.0, 2.0.1.
  Tested sample rate 16000Hz, 22500Hz.
  Bit-Depth: 24 or 16.
  
  Microphone connection: 
  - I2S_MIC_SERIAL_CLOCK      (SCK)      GPIO_NUM_32
  - I2S_MIC_LEFT_RIGHT_CLOCK  (WS)       GPIO_NUM_25
  - I2S_MIC_SERIAL_DATA       (SD)       GPIO_NUM_33
  - L/R pin of mic connect to ground.
  
  serial_audio.exe - program for wifi_microphone,  it allows you to testing  microphone through a serial port (uncomment  "//#define NO_WIFI").
  



# License

  This software is provided under the  <a href="http://unlicense.org/" rel="nofollow">UNLICENSE</a>

