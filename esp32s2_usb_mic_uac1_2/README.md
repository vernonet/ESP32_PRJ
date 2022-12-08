# usb microphone on esp32-s2 UAC1 UAC2

  This project is similar to <a href="https://github.com/vernonet/ESP32_PRJ/tree/main/esp32s2_usb_mic" rel="nofollow">esp32s2_usb_mic</a>
  but it supports two audio modes - USB Audio Class 1.0 and 2.0 (also known as UAC1 and UAC2). At startup, the device will try to connect in mode UAC2,
  if it fails, after 5 seconds will switch to mode UAC1 (for older versions windows).
  
  In project uses an  I2S microphone INMP441, <a href="https://www.wemos.cc/en/latest/s2/s2_mini.html" rel="nofollow">WEMOS S2 mini</a>.
  The project was created in  PlatformIO IDE with VS Code (framework arduino).
  Used source code <a href="https://github.com/atomic14/esp32_audio" rel="nofollow">esp32_audio</a>, <a href="https://github.com/hathach/tinyusb"    rel="nofollow">tinyusb</a>.
  Tested on Arduino ESP32 release 2.0.5.  
  Tested sample rate 16000, 32000, 44100, 48000 Hz.
  Bit-Depth: 16.
  
  Microphone connection: 
  - I2S_MIC_SERIAL_CLOCK      (SCK)   -   GPIO_NUM_12
  - I2S_MIC_LEFT_RIGHT_CLOCK  (WS)    -   GPIO_NUM_13
  - I2S_MIC_SERIAL_DATA       (SD)    -   GPIO_NUM_14
  - L/R pin of mic connect to ground.
  
  UART connection (for debug):
  - TX   -   GPIO_NUM_17
  - RX   -   GPIO_NUM_18  (not used)
  
  
  ... Use "CFG_TUSB_DEBUG=1 or 0".
  With "CFG_TUSB_DEBUG=2" you will not be able to record the sound.
  Works on windows XP, 7-11 ,Linux (tested on Linux Mint), older versions of windows need a driver with support for audio class 2.0.


# License

  This software is provided under the  <a href="http://unlicense.org/" rel="nofollow">UNLICENSE</a>

