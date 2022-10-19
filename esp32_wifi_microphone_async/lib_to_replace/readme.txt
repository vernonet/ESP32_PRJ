Changes in sdkconfig:

# CONFIG_SPIRAM_TRY_ALLOCATE_WIFI_LWIP is not set
CONFIG_LWIP_TCP_TMR_INTERVAL=200
CONFIG_LWIP_TCP_SND_BUF_DEFAULT=17232

CONFIG_LWIP_TCP_WND_DEFAULT=17232
CONFIG_TCP_SND_BUF_DEFAULT=17232

CONFIG_TCP_WND_DEFAULT=17232

Paste files here -> path - "C:\Users\<USER>\.platformio\packages\framework-arduinoespressif32\tools\sdk\esp32\lib"
.....
In new versions "esp32-arduino-lib-builder" need edit file - "esp32-arduino-lib-builder\esp-idf\components\lwip\Kconfig" to build library.
