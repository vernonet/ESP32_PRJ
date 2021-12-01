/****************************************************************************************************************************
  ESP_WiFiManager_Debug.h
  For ESP8266 / ESP32 boards

  ESP_WiFiManager is a library for the ESP8266/Arduino platform
  (https://github.com/esp8266/Arduino) to enable easy
  configuration and reconfiguration of WiFi credentials using a Captive Portal
  inspired by:
  http://www.esp8266.com/viewtopic.php?f=29&t=2520
  https://github.com/chriscook8/esp-arduino-apboot
  https://github.com/esp8266/Arduino/blob/master/libraries/DNSServer/examples/CaptivePortalAdvanced/

  Modified from Tzapu https://github.com/tzapu/WiFiManager
  and from Ken Taylor https://github.com/kentaylor

  Built by Khoi Hoang https://github.com/khoih-prog/ESP_WiFiManager
  Licensed under MIT license
  Version: 1.7.7

  Version Modified By   Date      Comments
  ------- -----------  ---------- -----------
  1.0.0   K Hoang      07/10/2019 Initial coding
  1.0.1   K Hoang      13/12/2019 Fix bug. Add features. Add support for ESP32
  1.0.2   K Hoang      19/12/2019 Fix bug thatkeeps ConfigPortal in endless loop if Portal/Router SSID or Password is NULL.
  1.0.3   K Hoang      05/01/2020 Option not displaying AvailablePages in Info page. Enhance README.md. Modify examples
  1.0.4   K Hoang      07/01/2020 Add RFC952 setHostname feature.
  1.0.5   K Hoang      15/01/2020 Add configurable DNS feature. Thanks to @Amorphous of https://community.blynk.cc
  1.0.6   K Hoang      03/02/2020 Add support for ArduinoJson version 6.0.0+ ( tested with v6.14.1 )
  1.0.7   K Hoang      13/04/2020 Reduce start time, fix SPIFFS bug in examples, update README.md
  1.0.8   K Hoang      10/06/2020 Fix STAstaticIP issue. Restructure code. Add LittleFS support for ESP8266 core 2.7.1+
  1.0.9   K Hoang      29/07/2020 Fix ESP32 STAstaticIP bug. Permit changing from DHCP <-> static IP using Config Portal.
                                  Add, enhance examples (fix MDNS for ESP32)
  1.0.10  K Hoang      08/08/2020 Add more features to Config Portal. Use random WiFi AP channel to avoid conflict.
  1.0.11  K Hoang      17/08/2020 Add CORS feature. Fix bug in softAP, autoConnect, resetSettings.
  1.1.0   K Hoang      28/08/2020 Add MultiWiFi feature to autoconnect to best WiFi at runtime
  1.1.1   K Hoang      30/08/2020 Add setCORSHeader function to allow flexible CORS. Fix typo and minor improvement.
  1.1.2   K Hoang      17/08/2020 Fix bug. Add example.
  1.2.0   K Hoang      09/10/2020 Restore cpp code besides Impl.h code to use if linker error. Fix bug.
  1.3.0   K Hoang      04/12/2020 Add LittleFS support to ESP32 using LITTLEFS Library
  1.4.1   K Hoang      22/12/2020 Fix staticIP not saved. Add functions. Add complex examples. Sync with ESPAsync_WiFiManager
  1.4.2   K Hoang      14/01/2021 Fix examples' bug not using saved WiFi Credentials after losing all WiFi connections.
  1.4.3   K Hoang      23/01/2021 Fix examples' bug not saving Static IP in certain cases.
  1.5.0   K Hoang      12/02/2021 Add support to new ESP32-S2
  1.5.1   K Hoang      26/03/2021 Fix compiler error if setting Compiler Warnings to All. Retest with esp32 core v1.0.6
  1.5.2   K Hoang      08/04/2021 Fix example misleading messages.
  1.5.3   K Hoang      13/04/2021 Add dnsServer error message.
  1.6.0   K Hoang      20/04/2021 Add support to new ESP32-C3 using SPIFFS or EEPROM
  1.6.1   K Hoang      25/04/2021 Fix MultiWiFi bug. Fix captive-portal bug if CP AP address is not default 192.168.4.1
  1.7.0   K Hoang      06/05/2021 Set _timezoneName. Add support to new ESP32-S2 (METRO_ESP32S2, FUNHOUSE_ESP32S2, etc.)
  1.7.1   K Hoang      08/05/2021 Fix Json bug. Fix timezoneName not displayed in Info page.
  1.7.2   K Hoang      08/05/2021 Fix warnings with ESP8266 core v3.0.0
  1.7.3   K Hoang      29/07/2021 Fix MultiWiFi connection issue with ESP32 core v2.0.0-rc1+
  1.7.4   K Hoang      13/08/2021 Add WiFi scanning of hidden SSIDs
  1.7.5   K Hoang      10/10/2021 Update `platform.ini` and `library.json`
  1.7.6   K Hoang      26/11/2021 Auto detect ESP32 core and use either built-in LittleFS or LITTLEFS library
  1.7.7   K Hoang      26/11/2021 Fix compile error for ESP32 core v1.0.5-
 *****************************************************************************************************************************/

#pragma once

#ifndef ESP_WiFiManager_Debug_H
#define ESP_WiFiManager_Debug_H

#ifdef WIFIMGR_DEBUG_PORT
  #define WM_DBG_PORT      WIFIMGR_DEBUG_PORT
#else
  #define WM_DBG_PORT      Serial
#endif

// Change _WIFIMGR_LOGLEVEL_ to set tracing and logging verbosity
// 0: DISABLED: no logging
// 1: ERROR: errors
// 2: WARN: errors and warnings
// 3: INFO: errors, warnings and informational (default)
// 4: DEBUG: errors, warnings, informational and debug

#ifndef _WIFIMGR_LOGLEVEL_
  #define _WIFIMGR_LOGLEVEL_       0
#endif

const char WM_MARK[] = "[WM] ";
const char WM_SP[]   = " ";

#define WM_PRINT        WM_DBG_PORT.print
#define WM_PRINTLN      WM_DBG_PORT.println

#define WM_PRINT_MARK   WM_PRINT(WM_MARK)
#define WM_PRINT_SP     WM_PRINT(WM_SP)

////////////////////////////////////////////////////

#define LOGERROR(x)         if(_WIFIMGR_LOGLEVEL_>0) { WM_PRINT_MARK; WM_PRINTLN(x); }
#define LOGERROR0(x)        if(_WIFIMGR_LOGLEVEL_>0) { WM_PRINT(x); }
#define LOGERROR1(x,y)      if(_WIFIMGR_LOGLEVEL_>0) { WM_PRINT_MARK; WM_PRINT(x); WM_PRINT_SP; WM_PRINTLN(y); }
#define LOGERROR2(x,y,z)    if(_WIFIMGR_LOGLEVEL_>0) { WM_PRINT_MARK; WM_PRINT(x); WM_PRINT_SP; WM_PRINT(y); WM_PRINT_SP; WM_PRINTLN(z); }
#define LOGERROR3(x,y,z,w)  if(_WIFIMGR_LOGLEVEL_>0) { WM_PRINT_MARK; WM_PRINT(x); WM_PRINT_SP; WM_PRINT(y); WM_PRINT_SP; WM_PRINT(z); WM_PRINT_SP; WM_PRINTLN(w); }

////////////////////////////////////////////////////

#define LOGWARN(x)          if(_WIFIMGR_LOGLEVEL_>1) { WM_PRINT_MARK; WM_PRINTLN(x); }
#define LOGWARN0(x)         if(_WIFIMGR_LOGLEVEL_>1) { WM_PRINT(x); }
#define LOGWARN1(x,y)       if(_WIFIMGR_LOGLEVEL_>1) { WM_PRINT_MARK; WM_PRINT(x); WM_PRINT_SP; WM_PRINTLN(y); }
#define LOGWARN2(x,y,z)     if(_WIFIMGR_LOGLEVEL_>1) { WM_PRINT_MARK; WM_PRINT(x); WM_PRINT_SP; WM_PRINT(y); WM_PRINT_SP; WM_PRINTLN(z); }
#define LOGWARN3(x,y,z,w)   if(_WIFIMGR_LOGLEVEL_>1) { WM_PRINT_MARK; WM_PRINT(x); WM_PRINT_SP; WM_PRINT(y); WM_PRINT_SP; WM_PRINT(z); WM_PRINT_SP; WM_PRINTLN(w); }

////////////////////////////////////////////////////

#define LOGINFO(x)          if(_WIFIMGR_LOGLEVEL_>2) { WM_PRINT_MARK; WM_PRINTLN(x); }
#define LOGINFO0(x)         if(_WIFIMGR_LOGLEVEL_>2) { WM_PRINT(x); }
#define LOGINFO1(x,y)       if(_WIFIMGR_LOGLEVEL_>2) { WM_PRINT_MARK; WM_PRINT(x); WM_PRINT_SP; WM_PRINTLN(y); }
#define LOGINFO2(x,y,z)     if(_WIFIMGR_LOGLEVEL_>2) { WM_PRINT_MARK; WM_PRINT(x); WM_PRINT_SP; WM_PRINT(y); WM_PRINT_SP; WM_PRINTLN(z); }
#define LOGINFO3(x,y,z,w)   if(_WIFIMGR_LOGLEVEL_>2) { WM_PRINT_MARK; WM_PRINT(x); WM_PRINT_SP; WM_PRINT(y); WM_PRINT_SP; WM_PRINT(z); WM_PRINT_SP; WM_PRINTLN(w); }

////////////////////////////////////////////////////

#define LOGDEBUG(x)         if(_WIFIMGR_LOGLEVEL_>3) { WM_PRINT_MARK; WM_PRINTLN(x); }
#define LOGDEBUG0(x)        if(_WIFIMGR_LOGLEVEL_>3) { WM_PRINT(x); }
#define LOGDEBUG1(x,y)      if(_WIFIMGR_LOGLEVEL_>3) { WM_PRINT_MARK; WM_PRINT(x); WM_PRINT_SP; WM_PRINTLN(y); }
#define LOGDEBUG2(x,y,z)    if(_WIFIMGR_LOGLEVEL_>3) { WM_PRINT_MARK; WM_PRINT(x); WM_PRINT_SP; WM_PRINT(y); WM_PRINT_SP; WM_PRINTLN(z); }
#define LOGDEBUG3(x,y,z,w)  if(_WIFIMGR_LOGLEVEL_>3) { WM_PRINT_MARK; WM_PRINT(x); WM_PRINT_SP; WM_PRINT(y); WM_PRINT_SP; WM_PRINT(z); WM_PRINT_SP; WM_PRINTLN(w); }

////////////////////////////////////////////////////

#endif    //ESP_WiFiManager_Debug_H
