#pragma once

#ifdef ESP8266
#include "AudioTimer/AudioTimerDef.h"
#include "Ticker.h"

namespace audio_tools {

typedef void (* repeating_timer_callback_t )(void* obj);

class TimerAlarmRepeatingESP8266;
TimerAlarmRepeatingESP8266 *self;

/**
 * @brief Repeating Timer functions for repeated execution: Plaease use the typedef TimerAlarmRepeating
 * 
 * @author Phil Schatzmann
 * @copyright GPLv3
 * 
 */
class TimerAlarmRepeatingESP8266 : public TimerAlarmRepeatingDef {
    public:
    
        TimerAlarmRepeatingESP8266(){
            self = this;
        }

        ~TimerAlarmRepeatingESP8266(){
            end();
        }

        /**
         * We can not do any I2C calls in the interrupt handler so we need to do this in a separate task
         */
        static void complexHandler(void *param) {
        }

        /**
         * Starts the alarm timer
         */
        bool begin(repeating_timer_callback_t callback_f, uint32_t time, TimeUnit unit = MS) override {
            uint32_t timeUs;

            // we determine the time in microseconds
            switch(unit){
                case MS:
                    timeUs = time / 1000;
                    break;
                case US:
                    timeUs = time;
                    break;
            }

            ticker.attach<void*>(timeUs / 1000000, callback_f, this);
            
            return true;
        }

        // ends the timer and if necessary the task
        bool end() override {
            ticker.detach();
            return true;
        }

    protected:
      void (*current_timer_callback)();    
      Ticker ticker; 
};

typedef  TimerAlarmRepeatingESP8266 TimerAlarmRepeating;


}



#endif