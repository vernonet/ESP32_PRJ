#pragma once

#include <freertos/FreeRTOS.h>
#include <driver/i2s.h>

/**
 * Base Class for both the ADC and I2S sampler
 **/
class I2SSampler
{
protected:
    i2s_port_t m_i2sPort = I2S_NUM_0;
    i2s_config_t m_i2s_config;
    virtual void configureI2S() = 0;
    virtual void unConfigureI2S(){};
    virtual void processI2SData(void *samples, size_t count){
        // nothing to do for the default case
    };

public:
    I2SSampler(i2s_port_t i2sPort, const i2s_config_t &i2sConfig);
    void start();
    virtual int read(int16_t *samples, int count, uint8_t bps) = 0;
    void stop();
    int sample_rate()
    {
        return m_i2s_config.sample_rate;
    }
    esp_err_t set_sample_rate(uint32_t rate) {
        return i2s_set_sample_rates(m_i2sPort, rate);
    }
};