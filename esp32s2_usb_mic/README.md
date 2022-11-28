# usb microphone on esp32-s2

  In project uses an  I2S microphone INMP441, WEMOS S2 mini.
  The project was created in  PlatformIO IDE with VS Code (framework arduino).
  Used source code <a href="https://github.com/atomic14/esp32_audio" rel="nofollow">esp32_audio</a>, , <a href="https://github.com/hathach/tinyusb"    rel="nofollow">tinyusb</a>.
  Tested on Arduino ESP32 release 2.0.5.  
  Tested sample rate 48000Hz.
  Bit-Depth: 16.
  
  Microphone connection: 
  - I2S_MIC_SERIAL_CLOCK      (SCK)   -   GPIO_NUM_12
  - I2S_MIC_LEFT_RIGHT_CLOCK  (WS)    -   GPIO_NUM_13
  - I2S_MIC_SERIAL_DATA       (SD)    -   GPIO_NUM_14
  - L/R pin of mic connect to ground.
  
  UART connection (for debug):
  - TX   -   GPIO_NUM_17
  - RX   -   GPIO_NUM_18  (not used)
  
  
  ...While it works partially (transfer of sound data is only once), use "CFG_TUSB_DEBUG=1 or 0".
  With "CFG_TUSB_DEBUG=2" you will not be able to record the sound.
  Works only on windows 10-11 (probably linux), older versions of windows need a driver with support for audio class 2.0.


# License

  This software is provided under the  <a href="http://unlicense.org/" rel="nofollow">UNLICENSE</a>
