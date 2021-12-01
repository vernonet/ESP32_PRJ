#pragma once

#if defined(__arm__)  && __has_include("mbed.h")
#include "AudioTimer/AudioTimerDef.h"
#include "mbed.h"

namespace audio_tools {

class TimerAlarmRepeatingMBED;
TimerAlarmRepeatingMBED *timerAlarmRepeating = nullptr;
typedef void (* repeating_timer_callback_t )(void* obj);

/**
 * @brief Repeating Timer functions for repeated execution: Plaease use the typedef TimerAlarmRepeating
 * 
 * @author Phil Schatzmann
 * @copyright GPLv3
 * 
 */
class TimerAlarmRepeatingMBED : public TimerAlarmRepeatingDef {
    public:
    
        TimerAlarmRepeatingMBED(){
            timerAlarmRepeating = this;
        }

        ~TimerAlarmRepeatingMBED(){
            end();
        }


        /**
         * Starts the alarm timer
         */
        bool begin(repeating_timer_callback_t callback_f, uint32_t time, TimeUnit unit = MS) override {
            callback = callback_f;

            // we determine the time in microseconds
            switch(unit){
                case MS:
                    ticker.attach(tickerCallback, time * 1000);
                    break;
                case US:
                    ticker.attach(tickerCallback, time);
                    break;
            }
            return true;
        }

        // ends the timer and if necessary the task
        bool end(){
            ticker.detach();
            return true;
        }

    protected:
        mbed::Ticker ticker;
        repeating_timer_callback_t callback;

        static void tickerCallback(){
            timerAlarmRepeating->callback(timerAlarmRepeating);
        }

};

typedef  TimerAlarmRepeatingMBED TimerAlarmRepeating;


}



#endif