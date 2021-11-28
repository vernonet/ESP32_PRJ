#include <Arduino.h>
#include "driver/i2s.h"
#include "soc/i2s_reg.h"
#include <algorithm>
#include "I2SMEMSSampler.h"

extern uint8_t signal_gain;



I2SMEMSSampler::I2SMEMSSampler(
    i2s_port_t i2s_port,
    i2s_pin_config_t &i2s_pins,
    i2s_config_t i2s_config,
    bool fixSPH0645) : I2SSampler(i2s_port, i2s_config)
{
    m_i2sPins = i2s_pins;
    m_fixSPH0645 = fixSPH0645;
}

void I2SMEMSSampler::configureI2S()
{
    if (m_fixSPH0645)
    {
        // FIXES for SPH0645
        REG_SET_BIT(I2S_TIMING_REG(m_i2sPort), BIT(9));
        REG_SET_BIT(I2S_CONF_REG(m_i2sPort), I2S_RX_MSB_SHIFT);
    }

    i2s_set_pin(m_i2sPort, &m_i2sPins);
}

int I2SMEMSSampler::read(int16_t *samples, int count, uint8_t bps)
{
    int32_t raw_samples[256], tmp_samples;
    int sample_index = 0;
    while (count > 0)
    {
        size_t bytes_read = 0;
        i2s_read(m_i2sPort, (void **)raw_samples, sizeof(int32_t) * std::min(count, 256), &bytes_read, portMAX_DELAY);
        int samples_read = bytes_read / sizeof(int32_t);
        for (int i = 0; i < samples_read; i++)
        {
            if (bps  == 16) samples[sample_index] = ((raw_samples[i] & 0xFFFFFFF0) >> 13) >>signal_gain;
              else if (bps  == 24) {
                  tmp_samples = (raw_samples[i]  << 3)>>signal_gain;
                  uint8_t * d_buff = (uint8_t *)&samples[0] + sample_index*3;
                  *d_buff = (uint8_t)((tmp_samples>>8)&0xFF); 
                  d_buff++;
                  *d_buff = (uint8_t)((tmp_samples>>16)&0xFF);
                  d_buff++;
                  *d_buff = (uint8_t)((tmp_samples>>24)&0xFF);  
              }
            sample_index++;
            count--;
        }
    }
    return sample_index;
}

   



