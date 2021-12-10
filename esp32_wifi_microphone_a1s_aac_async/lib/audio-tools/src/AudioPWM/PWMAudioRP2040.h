
#pragma once
#if defined(ARDUINO_ARCH_RP2040) && !__has_include("mbed.h") 
#include "AudioPWM/PWMAudioBase.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "pico/time.h"

namespace audio_tools {

// forwrd declaratioin of callback
class PWMAudioStreamPico;
typedef PWMAudioStreamPico PWMAudioStream;
bool defaultPWMAudioOutputCallbackPico(repeating_timer* ptr);

/**
 * @brief Rasperry Pico Channel to pin assignments
 * @author Phil Schatzmann
 * @copyright GPLv3
 */
struct PicoChannelOut {
    int gpio = -1;
    int audioChannel;
    uint slice; // pico pwm slice
    uint channel; // pico pwm channel
};


/**
 * @brief Audio output for the Rasperry Pico to PWM pins.
   The Raspberry Pi Pico has 8 PWM blocks/slices(1-8) and each PWM block provides up to two PWM outputs(A-B). 
 * @author Phil Schatzmann
 * @copyright GPLv3

 */

class PWMAudioStreamPico : public PWMAudioStreamBase {
    friend bool defaultPWMAudioOutputCallbackPico(repeating_timer* ptr);

    public:

        PWMAudioStreamPico(){
            LOGD("PWMAudioStreamPico");
        }

        /// Ends the output -> resets the timer and the pins
        void end(){
	 		LOGD(LOG_METHOD);
            if (!cancel_repeating_timer(&timer)){
                LOGE("cancel_repeating_timer failed")
            }
            for(auto pin : pins) {
                if (pin.gpio!=-1){
                    pwm_set_enabled(pin.slice, false);
                }
            } 
        }

    protected:
        Vector<PicoChannelOut> pins;      
        repeating_timer_t timer;


        // setup pwm config and all pins
        void setupPWM(){
	 		LOGD(LOG_METHOD);
            pwm_config cfg = setupPWMConfig();
            
            // initialize empty pins
            PicoChannelOut empty;
            pins.resize(audio_config.channels, empty);

            // setup pin values
            for (int j=0;j< audio_config.channels;j++) {
                int gpio = audio_config.start_pin + j;
                int channel = j;
                if (audio_config.pins!=nullptr){
                    gpio = audio_config.pins[j];
                }
                LOGD("-> defining pin %d",gpio);

                pins[channel].slice = pwm_gpio_to_slice_num(gpio);
                pins[channel].channel = pwm_gpio_to_channel(gpio);
                pins[channel].audioChannel = j;
                pins[channel].gpio = gpio;

                setupPWMPin(cfg, pins[channel]);
            }
        }

        // defines the pwm_config which will be used to drive the pins
        pwm_config setupPWMConfig() {
	 		LOGD(LOG_METHOD);
            // setup pwm frequency
            pwm_config pico_pwm_config = pwm_get_default_config();
            int wrap_value = maxOutputValue(); // amplitude of square wave (pwm values -amplitude to amplitude) for one byte
            float pwmClockDivider = static_cast<float>(clock_get_hz(clk_sys)) / (audio_config.pwm_frequency * wrap_value);
            LOGI("->clock speed is %f", static_cast<float>(clock_get_hz(clk_sys)));
            LOGI("->divider is %f", pwmClockDivider);
            pwm_config_set_clkdiv(&pico_pwm_config, pwmClockDivider);
            pwm_config_set_clkdiv_mode(&pico_pwm_config, PWM_DIV_FREE_RUNNING);
            //pwm_config_set_phase_correct(&pico_pwm_config, false);
            pwm_config_set_wrap (&pico_pwm_config, wrap_value);
            return pico_pwm_config;
        } 

        // set up pwm 
        void setupPWMPin(pwm_config &cfg, PicoChannelOut &pinInfo){
	 		LOGD("%s for gpio %d",LOG_METHOD, pinInfo.gpio);
            // setup pwm pin  
            int gpio = pinInfo.gpio;
            gpio_set_function(gpio, GPIO_FUNC_PWM);
            pinInfo.slice = pwm_gpio_to_slice_num(gpio);
            pinInfo.channel = pwm_gpio_to_channel(gpio);
            pwm_init(pinInfo.slice, &cfg, true);

            // set initial output value 
            pwm_set_chan_level(pinInfo.slice, pinInfo.channel, 0); 
        }

        void setupTimer(){
	 		LOGD(LOG_METHOD);
            // setup timer
            uint32_t time = 1000000UL / audio_config.sample_rate;
            LOGI("->Timer value %ld us", time);

            if (!add_repeating_timer_us(-time, &defaultPWMAudioOutputCallbackPico, this, &timer)){
                LOGE("Error: alarm_pool_add_repeating_timer_us failed; no alarm slots available");
            }
        }

        /// The pico supports max 16 pwm pins
        virtual int maxChannels() {
            return 16;
        };

        /// Max pwm output value
        virtual int maxOutputValue(){
            return 255;
        }
        
        /// write a pwm value to the indicated channel. The values are between 0 and 255
        void pwmWrite(int audioChannel, int value){
            pwm_set_chan_level(pins[audioChannel].slice, pins[audioChannel].channel, value);
        }

};


// timed output executed at the sampleRate
bool defaultPWMAudioOutputCallbackPico(repeating_timer* ptr) {
    PWMAudioStreamPico *self = (PWMAudioStreamPico*)  ptr->user_data;
    if (self!=nullptr){
        self->playNextFrame();
    }
    return true;
}

} // Namespace


#endif

