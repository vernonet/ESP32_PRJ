
#include <Arduino.h>
#include "I2SSampler.h"
#include "driver/i2s.h"

void I2SSampler::addSample(int32_t sample, uint8_t bps)
{
	if (bps == 16) {
      // add the sample to the current audio buffer
      m_currentAudioBuffer[m_audioBufferPos] = (int16_t)sample;
	}
	  else if (bps == 24){
  		 uint8_t * d_buff = (uint8_t *)&m_currentAudioBuffer[0] + m_audioBufferPos*3;
       //Serial.printf("  0x%08x", sample);
  		*d_buff = (uint8_t)((sample>>8)&0xFF); 
       //Serial.printf("  0x%02x", *d_buff);
       d_buff++;
      *d_buff = (uint8_t)((sample>>16)&0xFF);
       // Serial.printf("  0x%02x", *d_buff); 
       d_buff++;
      *d_buff = (uint8_t)((sample>>24)&0xFF);  
       // Serial.printf("  0x%02x\n\r", *d_buff);
	  }
	  
    m_audioBufferPos++;
    // have we filled the buffer with data?
    if (m_audioBufferPos == m_bufferSizeInSamples)
    {
        // swap to the other buffer
        std::swap(m_currentAudioBuffer, m_capturedAudioBuffer);
        // reset the buffer position
        m_audioBufferPos = 0;
        // tell the writer task to save the data
        xTaskNotify(m_writerTaskHandle, 1, eIncrement);
    }
}

void i2sReaderTask(void *param)
{
    I2SSampler *sampler = (I2SSampler *)param;
    while (true)
    {
        // wait for some data to arrive on the queue
        i2s_event_t evt;
        if (xQueueReceive(sampler->m_i2sQueue, &evt, portMAX_DELAY) == pdPASS)
        {
            if (evt.type == I2S_EVENT_RX_DONE)
            {
                size_t bytesRead = 0;
                do
                {
                    // read data from the I2S peripheral
                    uint8_t i2sData[1200];
                    // read from i2s
                    i2s_read(sampler->getI2SPort(), i2sData, 1200, &bytesRead, 10);  //3072
                    // process the raw data
                    sampler->processI2SData(i2sData, bytesRead, sampler->m_bps);
                } while (bytesRead > 0);
            }
        }
    }
}

//bps -  bit per sample in out stream
void I2SSampler::start(i2s_port_t i2sPort, i2s_config_t &i2sConfig, uint8_t bps, int32_t bufferSizeInBytes, TaskHandle_t writerTaskHandle)
{
    m_i2sPort = i2sPort;
    m_writerTaskHandle = writerTaskHandle;
    m_bufferSizeInSamples = bufferSizeInBytes / (bps/8);
    m_bufferSizeInBytes = bufferSizeInBytes;
    m_audioBuffer1 = (int16_t *)malloc(bufferSizeInBytes);
    m_audioBuffer2 = (int16_t *)malloc(bufferSizeInBytes);
	  m_bps = bps;
	

    m_currentAudioBuffer = m_audioBuffer1;
    m_capturedAudioBuffer = m_audioBuffer2;

    m_writerTaskHandle = writerTaskHandle;
    //install and start i2s driver
    i2s_driver_install(m_i2sPort, &i2sConfig, 4, &m_i2sQueue);
    // set up the I2S configuration from the subclass
    configureI2S();
    // start a task to read samples from the ADC
    TaskHandle_t readerTaskHandle;
    xTaskCreatePinnedToCore(i2sReaderTask, "i2s Reader Task", 4096, this, 1, &readerTaskHandle, 0);
}
