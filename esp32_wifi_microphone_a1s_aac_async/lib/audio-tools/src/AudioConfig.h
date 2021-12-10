/**
 * @author Phil Schatzmann
 * @brief AutioTools Configuration
 * @copyright GPLv3
 * 
 */
#pragma once
#include "Arduino.h"
#include <string.h>
#include <stdint.h>

#define AUDIOTOOLS_VERSION "0.6.1"

/**
 * ------------------------------------------------------------------------- 
 * @brief Logging
 * Logging Configuration in Arduino -> set USE_AUDIO_LOGGING to false if you want to deactivate Logging.
 * When using cmake you can set -DUSE_AUDIO_LOGGING=false
 * You can also change the LOG_LEVEL and LOG_STREAM here.
 * However it is recommended to do it in your Sketch e.g with AudioLogger::instance().begin(Serial,AudioLogger::Warning);
 */
 
#define USE_AUDIO_LOGGING true
#define PRINTF_BUFFER_SIZE 100
#define LOG_LEVEL AudioLogger::Warning
#define LOG_STREAM Serial
#define LOG_METHOD __PRETTY_FUNCTION__

/**
 * ------------------------------------------------------------------------- 
 * @brief Common Default Settings that can usually be changed in the API
 */

#define DEFAULT_BUFFER_SIZE (2048)
#define DEFAULT_SAMPLE_RATE 22050
#define DEFAULT_CHANNELS 2
#define DEFAULT_BITS_PER_SAMPLE 16
#define I2S_DEFAULT_PORT 0
#define I2S_BUFFER_SIZE 512
#define I2S_BUFFER_COUNT 8
#define A2DP_BUFFER_SIZE 512
#define A2DP_BUFFER_COUNT 20
#define CODEC_DELAY_MS 10
#define COPY_DELAY_ON_NODATA 10
/**
 * ------------------------------------------------------------------------- 
 * @brief PWM
 */
#define PWM_BUFFER_SIZE 512
#define PWM_BUFFERS 10
#define PWM_FREQUENCY 60000


/**
 * ------------------------------------------------------------------------- 
 * @brief Activate decoders - only after installing them !
 */

//#define USE_HELIX
//#define USE_FDK
//#define USE_LAME
//#define USE_MAD


/**
 * ------------------------------------------------------------------------- 
 * @brief Activate Other Tools
 */

#define USE_STK
//#define USE_SDFAT
//#define USE_DELTASIGMA 


/**
 * ------------------------------------------------------------------------- 
 * @brief Platform specific Settings
 */

#ifdef ESP32
#include "esp32-hal-log.h"
#define USE_A2DP
#define USE_PWM
#define USE_URL_ARDUINO
#define USE_ESP8266_AUDIO
#define USE_I2S
#define USE_AUDIO_SERVER

#define PWM_FREQENCY 30000
#define PWM_START_PIN 12
#define PIN_I2S_BCK 14
#define PIN_I2S_WS 15
#define PIN_I2S_DATA_IN 32
#define PIN_I2S_DATA_OUT -1
#define I2S_USE_APLL true  
// Default Setting: The mute pin can be switched off by setting it to -1. Or you could drive the LED by assigning LED_BUILTIN
#define PIN_I2S_MUTE 23
#define SOFT_MUTE_VALUE LOW  
#define PIN_CS SS
#define PIN_ADC1 34 
#define PIN_ADC2 35

// URLStream
#define URL_STREAM_CORE 0
#define URL_STREAM_PRIORITY 2
#define URL_STREAM_BUFFER_COUNT 10
#define STACK_SIZE 20000

// Default LED
#ifndef LED_BUILTIN
# define LED_BUILTIN 22 // pin number is specific to your esp32 board
#endif

// support for old idf releases
#if ESP_IDF_VERSION_MAJOR < 4 && !defined(I2S_COMM_FORMAT_STAND_I2S)
# define I2S_COMM_FORMAT_STAND_I2S (I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_LSB)
# define I2S_COMM_FORMAT_STAND_MSB (I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB)
#endif

#endif

//----------------
#ifdef ESP8266
#define USE_URL_ARDUINO
#define USE_ESP8266_AUDIO
#define USE_I2S
#define USE_PWM
#define USE_SDFAT
#define USE_AUDIO_SERVER

#define PWM_START_PIN 12
#define PIN_I2S_BCK -1
#define PIN_I2S_WS -1
#define PIN_I2S_DATA_IN -1
#define PIN_I2S_DATA_OUT -1
#define I2S_USE_APLL false  
#define PIN_I2S_MUTE 23
#define SOFT_MUTE_VALUE LOW  
#define PIN_CS SS
#endif

//----------------
#ifdef ARDUINO_ARDUINO_NANO33BLE
#define USE_I2S
#define USE_PWM

#define PWM_START_PIN 6
#define PIN_I2S_BCK 2
#define PIN_I2S_WS 1
#define PIN_I2S_DATA_IN 3
#define PIN_I2S_DATA_OUT 3
#define PIN_I2S_MUTE 4
#define SOFT_MUTE_VALUE LOW  
#define PIN_CS SS
#endif


//----------------
#ifdef ARDUINO_ARCH_RP2040
#define USE_ESP8266_AUDIO
#define USE_I2S
#define USE_PWM

#define PWM_START_PIN 6
#define PIN_I2S_BCK 1
#define PIN_I2S_WS PIN_I2S_BCK+1
#define PIN_I2S_DATA_IN 3
#define PIN_I2S_DATA_OUT 3
#define PIN_I2S_MUTE 4
#define SOFT_MUTE_VALUE LOW  
#define PIN_CS PIN_SPI0_SS
#endif

//----------------
#ifdef __AVR__
#define USE_PWM

#define PWM_START_PIN 6
#define PIN_CS CS
#endif

//----------------
#ifdef ARDUINO_ARCH_STM32F4
#define STM32
#endif

#ifdef STM32
#define USE_I2S
#define USE_PWM

#define PWM_START_PIN 6
#define PIN_I2S_BCK 1
#define PIN_I2S_WS PIN_I2S_BCK+1
#define PIN_I2S_DATA_IN 3
#define PIN_I2S_DATA_OUT 3
#define PIN_I2S_MUTE 4
#define SOFT_MUTE_VALUE LOW  
#define PIN_CS 10
#endif


//----------------
#if defined(__linux__) || defined(_WIN32) || defined(__APPLE__)
#define USE_URL_ARDUINO
#define IS_DESKTOP
#endif


#ifdef IS_DESKTOP
#define USE_URL_ARDUINO
#endif