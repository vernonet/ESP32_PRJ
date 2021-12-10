#pragma once

#ifdef ARDUINO_ARCH_RP2040
#include "AudioTimer/AudioTimerDef.h"
#include "hardware/timer.h"
#include "pico/time.h"
#include <time.h>
#include <functional>


namespace audio_tools {

typedef void (* my_repeating_timer_callback_t )(void* obj);
//typedef bool (* repeating_timer_callback_bool_t )(void* obj);

/**
 * @brief Repeating Timer functions for repeated execution: Plaease use the typedef TimerAlarmRepeating
 * 
 * @author Phil Schatzmann
 * @copyright GPLv3
 * 
 */
class TimerAlarmRepeatingRP2040 : public TimerAlarmRepeatingDef{
    public:
    
        TimerAlarmRepeatingRP2040(){
            alarm_pool_init_default();
            ap = alarm_pool_get_default();
        }

        ~TimerAlarmRepeatingRP2040(){
            end();
        }

        /**
         * Starts the alarm timer
         */
        bool begin(const my_repeating_timer_callback_t callback_f, uint32_t time, TimeUnit unit = MS) override {
            bool result = false;
            this->instanceCallback = callback_f;

            // we determine the time in microseconds
            switch(unit){
                case MS:
                    result = alarm_pool_add_repeating_timer_ms(ap, time, staticCallback, this,  &timer);
                    break;
                case US:
                    result = alarm_pool_add_repeating_timer_us(ap, time, staticCallback, this, &timer);
                    break;
            }
            
            return result;
        }

        static bool staticCallback(repeating_timer *ptr)  {
            TimerAlarmRepeatingRP2040 *self = (TimerAlarmRepeatingRP2040 *)ptr->user_data; 
            self->instanceCallback(self);
            return true;
        }

        // ends the timer and if necessary the task
        bool end(){
            return cancel_repeating_timer(&timer);
        }

    protected:
        alarm_pool_t *ap = nullptr;
        repeating_timer_t timer;
        my_repeating_timer_callback_t instanceCallback;
};

typedef  TimerAlarmRepeatingRP2040 TimerAlarmRepeating;


}



#endif