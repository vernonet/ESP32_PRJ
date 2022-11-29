//#include "pins_arduino.h"
#include <Arduino.h>
#include <stdio.h>
#include "esp_heap_caps.h"
//#include "USB.h"
#include "tusb.h"
#include "board.h"
#include "WiFi.h"
#include "I2SMEMSSampler.h"
#include "tusb_config.h"
#include "usb_descriptors.h"

#include "esp_rom_gpio.h"
#include "hal/gpio_ll.h"
#include "hal/usb_hal.h"
#include "soc/usb_periph.h"

#include "esp32-hal-uart.h"
#include "driver/uart.h"

// #include "common/tusb_debug.h"
// #include "tusb_config.h"



#ifndef RX1
#if CONFIG_IDF_TARGET_ESP32S2
 #define RX1 18
#elif CONFIG_IDF_TARGET_ESP32C3
 #define RX1 18
#elif CONFIG_IDF_TARGET_ESP32S3
 #define RX1 15
#endif
#endif

#ifndef TX1
#if CONFIG_IDF_TARGET_ESP32S2
#define TX1 17
#elif CONFIG_IDF_TARGET_ESP32C3
#define TX1 19
#elif CONFIG_IDF_TARGET_ESP32S3
#define TX1 16
#endif
#endif

#define NO_WIFI
#define USBD_STACK_SIZE            4096


#define LED_ON                     HIGH
#define LED_OFF                    LOW
#define PIN_LED                    GPIO_NUM_15

#define BAUDRATE                   (BAUDRATE_)

#define SAMPLE_BUFFER_SIZE         512    //Length of one buffer, in 32-bit words.  //512 //300  
#define BUF_CNT                    8      // Number of buffers in the I2S circular buffer   
#ifndef AUDIO_SAMPLE_RATE
 #define AUDIO_SAMPLE_RATE         48000
#endif
#define BITS_PER_SAMPLE            (16)  
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(2, 0, 4)
 #define I2S_MIC_CHANNEL I2S_CHANNEL_FMT_ONLY_RIGHT // most microphones will probably default to left channel but you may need to tie the L/R pin low
#else
 #define I2S_MIC_CHANNEL I2S_CHANNEL_FMT_ONLY_LEFT
#endif
#define SIGNAL_GAIN                (0)// 0 - max gain, 2 - no gain

#define I2S_MIC_SERIAL_CLOCK GPIO_NUM_12
#define I2S_MIC_LEFT_RIGHT_CLOCK GPIO_NUM_13
#define I2S_MIC_SERIAL_DATA GPIO_NUM_14



#define PRINTF_BUF_SZE           (256)

int16_t samples[SAMPLE_BUFFER_SIZE*(BITS_PER_SAMPLE>>3)*2];

const i2s_port_t I2S_PORT = I2S_NUM_0;
uint8_t signal_gain;

uint32_t free_mem8 = 0, free_mem32 = 0;

I2SSampler *i2sSampler = NULL;
i2s_config_t i2s_config = {
  .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX), // Receive, not transfer
  .sample_rate = AUDIO_SAMPLE_RATE,
  .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT, // could only get it to work with 32bits
  .channel_format = I2S_MIC_CHANNEL, // although the SEL config should be left, it seems to transmit on right
#ifdef ARDUINO_ESP32_RELEASE_1_0_6  
  .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
#else  // >=2.00.....
  .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_STAND_I2S),
#endif
  .intr_alloc_flags = (ESP_INTR_FLAG_LEVEL1 | ESP_INTR_FLAG_SHARED),        // Interrupt level 1
  .dma_buf_count = BUF_CNT,                        // number of buffers
  .dma_buf_len = SAMPLE_BUFFER_SIZE,               // samples per buffer
 // .use_apll = true,
  .tx_desc_auto_clear = true
};
// The pin config as per the setup
i2s_pin_config_t i2s_mic_pins = {
  .bck_io_num = I2S_MIC_SERIAL_CLOCK,
  .ws_io_num = I2S_MIC_LEFT_RIGHT_CLOCK,
  .data_out_num = I2S_PIN_NO_CHANGE,
  .data_in_num = I2S_MIC_SERIAL_DATA
};

uart_t* UART;

/* Blink pattern
 * - 250 ms  : device not mounted
 * - 1000 ms : device mounted
 * - 2500 ms : device is suspended
 */
enum  {
  BLINK_STREAMING   = 25,
  BLINK_NOT_MOUNTED = 250,
  BLINK_MOUNTED     = 1000,
  BLINK_SUSPENDED   = 2500,
};

static uint32_t blink_interval_ms = BLINK_NOT_MOUNTED;

// Audio controls
// Current states
bool mute[CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX + 1]; 				          // +1 for master channel 0
uint16_t volume[CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX + 1]; 					// +1 for master channel 0
uint32_t sampFreq;
uint8_t clkValid;
uint16_t startVal = 0;
uint8_t  wr_cnt = 0;

// Range states
audio_control_range_2_n_t(1) volumeRng[CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX+1]; 			// Volume range state
audio_control_range_4_n_t(1) sampleFreqRng; 						// Sample frequency range state


void led_blinking_task(void);
void audio_task(void);

void i2sMemsToUsbTask(void *param);
static void usbEventCallback(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);

int uart_printf(const char *c, ...);
static void fill_mem_seed(int seed, void *mem, int len, int block_size);
static bool check_mem_seed(int seed, void *mem, int len, int block_size);


void setup() {

  pinMode(PIN_LED, OUTPUT);  
  
  //init uart
  UART =  uartBegin(UART_NUM_1, BAUDRATE, SERIAL_8N1, RX1, TX1, 256, 0, false, 112);
  //uartSetHwFlowCtrlMode(UART, UART_HW_FLOWCTRL_CTS_RTS, 64);

  //testing PSRAM
  byte *psdRamBuffer = (byte *)ps_malloc(0x10000 * 16 * 2); // test 1Mbytes
  fill_mem_seed(0xaaaa, psdRamBuffer, sizeof psdRamBuffer, 0x1000);
  if (check_mem_seed(0xaaaa, psdRamBuffer, sizeof psdRamBuffer, 0x1000))
  {
    uart_printf("PSRAM test - OK!\r\n");
  }
  else
  {
    uart_printf("PSRAM test - ERROR!\r\n");
  }
  uartFlush(UART);
  free(psdRamBuffer);
  delay(100);

  WiFi.mode(WIFI_OFF);
 
  // Init values
  sampFreq = AUDIO_SAMPLE_RATE;
  clkValid = 1;

  sampleFreqRng.wNumSubRanges = 1;
  sampleFreqRng.subrange[0].bMin = AUDIO_SAMPLE_RATE;
  sampleFreqRng.subrange[0].bMax = AUDIO_SAMPLE_RATE;
  sampleFreqRng.subrange[0].bRes = 0;

  volume[0] =  300; volume[1] =  300;                      
  signal_gain = 3 - volume[0]/100; 

  i2sSampler = new I2SMEMSSampler(I2S_PORT, i2s_mic_pins, i2s_config, false);
  //start sampling from i2s device
  i2sSampler->start();
  
  //free_mem8  = heap_caps_get_free_size(MALLOC_CAP_8BIT);
  //free_mem32 = heap_caps_get_free_size(MALLOC_CAP_32BIT);
 
  // init device stack on configured roothub port
  tud_init(BOARD_TUD_RHPORT);
}

void loop()
{
    tud_task(); // tinyusb device task
    led_blinking_task();
    audio_task();
    //vTaskDelay(1 / portTICK_PERIOD_MS);
}



//--------------------------------------------------------------------+
// Device callbacks
//--------------------------------------------------------------------+

// Invoked when device is mounted
void tud_mount_cb(void)
{
  blink_interval_ms = BLINK_MOUNTED;
}

// Invoked when device is unmounted
void tud_umount_cb(void)
{
  blink_interval_ms = BLINK_NOT_MOUNTED;
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en)
{
  (void) remote_wakeup_en;
  blink_interval_ms = BLINK_SUSPENDED;
}

// Invoked when usb bus is resumed
void tud_resume_cb(void)
{
  blink_interval_ms = BLINK_MOUNTED;
}

//--------------------------------------------------------------------+
// AUDIO Task
//--------------------------------------------------------------------+

void audio_task(void)
{
  // int samples_read;
  // // Yet to be filled - e.g. put meas data into TX FIFOs etc.
  // samples_read = i2sSampler->read(samples, SAMPLE_BUFFER_SIZE, BITS_PER_SAMPLE);
  asm("nop");
}

//--------------------------------------------------------------------+
// Application Callback API Implementations
//--------------------------------------------------------------------+

// Invoked when audio class specific set request received for an EP
bool tud_audio_set_req_ep_cb(uint8_t rhport, tusb_control_request_t const * p_request, uint8_t *pBuff)
{
  (void) rhport;
  (void) pBuff;

  // We do not support any set range requests here, only current value requests
  TU_VERIFY(p_request->bRequest == AUDIO_CS_REQ_CUR);

  // Page 91 in UAC2 specification
  uint8_t channelNum = TU_U16_LOW(p_request->wValue);
  uint8_t ctrlSel = TU_U16_HIGH(p_request->wValue);
  uint8_t ep = TU_U16_LOW(p_request->wIndex);

  (void) channelNum; (void) ctrlSel; (void) ep;

  uart_printf( "\r\n  channelNum -> %x ctrlSel -> %x ep -> %x (not implemented)", channelNum, ctrlSel, ep);

  return false; 	// Yet not implemented
}

// Invoked when audio class specific set request received for an interface
bool tud_audio_set_req_itf_cb(uint8_t rhport, tusb_control_request_t const * p_request, uint8_t *pBuff)
{
  (void) rhport;
  (void) pBuff;

  // We do not support any set range requests here, only current value requests
  TU_VERIFY(p_request->bRequest == AUDIO_CS_REQ_CUR);

  // Page 91 in UAC2 specification
  uint8_t channelNum = TU_U16_LOW(p_request->wValue);
  uint8_t ctrlSel = TU_U16_HIGH(p_request->wValue);
  uint8_t itf = TU_U16_LOW(p_request->wIndex);

  (void) channelNum; (void) ctrlSel; (void) itf;

  uart_printf("  channelNum -> %x itf ->%x ctrlSel ->  %x (not implemented)", channelNum, itf, ctrlSel);

  return false; 	// Yet not implemented
}

// Invoked when audio class specific set request received for an entity
bool tud_audio_set_req_entity_cb(uint8_t rhport, tusb_control_request_t const * p_request, uint8_t *pBuff)
{
  (void) rhport;

  // Page 91 in UAC2 specification
  uint8_t channelNum = TU_U16_LOW(p_request->wValue);
  uint8_t ctrlSel = TU_U16_HIGH(p_request->wValue);
  uint8_t itf = TU_U16_LOW(p_request->wIndex);
  uint8_t entityID = TU_U16_HIGH(p_request->wIndex);

  (void) itf;

  uart_printf("  channelNum -> %x itf -> %x ctrlSel -> %x entityID -> %x (implemented parciali)", channelNum, itf, ctrlSel, entityID);

  // We do not support any set range requests here, only current value requests
  TU_VERIFY(p_request->bRequest == AUDIO_CS_REQ_CUR);

  // If request is for our feature unit
  if ( entityID == 2 )
  {
    switch ( ctrlSel )
    {
      case AUDIO_FU_CTRL_MUTE:
        // Request uses format layout 1
        TU_VERIFY(p_request->wLength == sizeof(audio_control_cur_1_t));

        mute[channelNum] = ((audio_control_cur_1_t*) pBuff)->bCur;

        uart_printf("    Set Mute: %d of channel: %u\r\n", mute[channelNum], channelNum);
      return true;

      case AUDIO_FU_CTRL_VOLUME:
        // Request uses format layout 2
        TU_VERIFY(p_request->wLength == sizeof(audio_control_cur_2_t));

        volume[channelNum] = (uint16_t) ((audio_control_cur_2_t*) pBuff)->bCur;

        signal_gain = 3 - (((volume[channelNum]/100) > 3) ? 3 : (volume[channelNum]/100)); 

        uart_printf("    Set Volume: %d dB of channel: %u\r\n", volume[channelNum], channelNum);
      return true;

        // Unknown/Unsupported control
      default:
        TU_BREAKPOINT();
      return false;
    }
  }
  return false;    // Yet not implemented
}

// Invoked when audio class specific get request received for an EP
bool tud_audio_get_req_ep_cb(uint8_t rhport, tusb_control_request_t const * p_request)
{
  (void) rhport;

  // Page 91 in UAC2 specification
  uint8_t channelNum = TU_U16_LOW(p_request->wValue);
  uint8_t ctrlSel = TU_U16_HIGH(p_request->wValue);
  uint8_t ep = TU_U16_LOW(p_request->wIndex);

  (void) channelNum; (void) ctrlSel; (void) ep;

  //	return tud_control_xfer(rhport, p_request, &tmp, 1);

  uart_printf("  channelNum -> %x ctrlSel -> %x ep -> %x (not implemented)", channelNum, ctrlSel, ep)	;

  return false; 	// Yet not implemented
}

// Invoked when audio class specific get request received for an interface
bool tud_audio_get_req_itf_cb(uint8_t rhport, tusb_control_request_t const * p_request)
{
  (void) rhport;

  // Page 91 in UAC2 specification
  uint8_t channelNum = TU_U16_LOW(p_request->wValue);
  uint8_t ctrlSel = TU_U16_HIGH(p_request->wValue);
  uint8_t itf = TU_U16_LOW(p_request->wIndex);

  (void) channelNum; (void) ctrlSel; (void) itf;

  uart_printf("  channelNum -> %x ctrlSel -> %x itf -> %x (not implemented)", channelNum, ctrlSel, itf);


  return false; 	// Yet not implemented
}

// Invoked when audio class specific get request received for an entity
bool tud_audio_get_req_entity_cb(uint8_t rhport, tusb_control_request_t const * p_request)
{
  (void) rhport;

  // Page 91 in UAC2 specification
  uint8_t channelNum = TU_U16_LOW(p_request->wValue);
  uint8_t ctrlSel = TU_U16_HIGH(p_request->wValue);
  uint8_t itf = TU_U16_LOW(p_request->wIndex); 			// Since we have only one audio function implemented, we do not need the itf value
  uint8_t entityID = TU_U16_HIGH(p_request->wIndex);

  uart_printf("  channelNum -> %x itf -> %x ctrlSel -> %x entityID -> %x", channelNum, itf, ctrlSel, entityID);
  
  // Input terminal (Microphone input)
  if (entityID == 1)
  {
     uart_printf("    Input terminal");
    switch ( ctrlSel )
    {
      case AUDIO_TE_CTRL_CONNECTOR:
      {
        // The terminal connector control only has a get request with only the CUR attribute.
        audio_desc_channel_cluster_t ret_;

        // Those are dummy values for now
        ret_.bNrChannels = 1;
        ret_.bmChannelConfig = AUDIO_CHANNEL_CONFIG_NON_PREDEFINED;
        ret_.iChannelNames = 0;

        uart_printf("    Get terminal connector\r\n");

        return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, (void*) &ret_, sizeof(ret_));
      }
      break;

        // Unknown/Unsupported control selector
      default:
        TU_BREAKPOINT();
        return false;
    }
  }

  // Feature unit
  if (entityID == 2)
  {
     uart_printf("    Feature unit");
    switch ( ctrlSel )
    {
      case AUDIO_FU_CTRL_MUTE:
        // Audio control mute cur parameter block consists of only one byte - we thus can send it right away
        // There does not exist a range parameter block for mute
        uart_printf("    Get Mute of channel: %u\r\n", channelNum);
        return tud_control_xfer(rhport, p_request, &mute[channelNum], 1);

      case AUDIO_FU_CTRL_VOLUME:
        switch ( p_request->bRequest )
        {
          case AUDIO_CS_REQ_CUR:
            uart_printf("    Get Volume of channel: %u\r\n", channelNum);
            return tud_control_xfer(rhport, p_request, &volume[channelNum], sizeof(volume[channelNum]));

          case AUDIO_CS_REQ_RANGE:
            uart_printf("    Get Volume range of channel: %u\r\n", channelNum);

            // Copy values - only for testing - better is version below
            audio_control_range_2_n_t(1) 
            ret;  
            //create 4 volume level  
            ret.wNumSubRanges = 1;
            ret.subrange[0].bMin = 000;   // -90 dB
            ret.subrange[0].bMax = 300;		// +90 dB
            ret.subrange[0].bRes = 100;   // 1 dB steps
        

            return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, (void*) &ret, sizeof(ret));

            // Unknown/Unsupported control
          default:
            TU_BREAKPOINT();
            return false;
        }
      break;

        // Unknown/Unsupported control
      default:
        TU_BREAKPOINT();
        uart_printf("\r\n");  /////////////////////////////////////////////////////
        return false;
    }
  }

  // Clock Source unit
  if ( entityID == 4 )
  {
    uart_printf("    Clock Source unit");
    switch ( ctrlSel )
    {
      case AUDIO_CS_CTRL_SAM_FREQ:
        // channelNum is always zero in this case
        switch ( p_request->bRequest )
        {
          case AUDIO_CS_REQ_CUR:
            uart_printf("    Get Sample Freq.\r\n");
            return tud_control_xfer(rhport, p_request, &sampFreq, sizeof(sampFreq));

          case AUDIO_CS_REQ_RANGE:
            sampleFreqRng.wNumSubRanges = 1;
            sampleFreqRng.subrange[0].bMin = AUDIO_SAMPLE_RATE;
            sampleFreqRng.subrange[0].bMax = AUDIO_SAMPLE_RATE;
            sampleFreqRng.subrange[0].bRes = 0;
            uart_printf("    Get Sample Freq. range\r\n");
            return tud_control_xfer(rhport, p_request, (void *)&sampleFreqRng, sizeof(sampleFreqRng));

           // Unknown/Unsupported control
          default:
            uart_printf("    Unsupported control\r\n");
            TU_BREAKPOINT();
            return false;
        }
      break;

      case AUDIO_CS_CTRL_CLK_VALID:
        // Only cur attribute exists for this request
        uart_printf("    Get Sample Freq. valid\r\n");
        return tud_control_xfer(rhport, p_request, &clkValid, sizeof(clkValid));

      // Unknown/Unsupported control
      default:
        TU_BREAKPOINT();
        return false;
    }
  }

  uart_printf("  Unsupported entity: %d\r\n", entityID);
  return false; 	// Yet not implemented
}

bool tud_audio_tx_done_pre_load_cb(uint8_t rhport, uint8_t itf, uint8_t ep_in, uint8_t cur_alt_setting)
{
  (void) rhport;
  (void) itf;
  (void) ep_in;
  (void) cur_alt_setting;

  blink_interval_ms = BLINK_STREAMING;
  uint8_t tmp[CFG_TUD_AUDIO_EP_SZ_IN_] = {0};
  if (wr_cnt++>150) {
    wr_cnt = 0;
    uart_printf("  audio_write\r\n");
  }
  
  if (mute[0] == 0 && mute[1] == 0) tud_audio_write ((uint8_t *)&samples[0], CFG_TUD_AUDIO_EP_SZ_IN_);
    else tud_audio_write (tmp, CFG_TUD_AUDIO_EP_SZ_IN_);

  return true;
}

bool tud_audio_tx_done_post_load_cb(uint8_t rhport, uint16_t n_bytes_copied, uint8_t itf, uint8_t ep_in, uint8_t cur_alt_setting)
{
  (void) rhport;
  (void) n_bytes_copied;
  (void) itf;
  (void) ep_in;
  (void) cur_alt_setting;


  int samples_read;
  samples_read = i2sSampler->read(samples, (CFG_TUD_AUDIO_EP_SZ_IN_)/2 , BITS_PER_SAMPLE);

  return true;
}

bool tud_audio_set_itf_close_EP_cb(uint8_t rhport, tusb_control_request_t const * p_request)
{
  (void) rhport;
  uint8_t const itf = tu_u16_low(tu_le16toh(p_request->wIndex));
  uint8_t const alt = tu_u16_low(tu_le16toh(p_request->wValue));

  return true;
}

bool tud_audio_set_itf_cb(uint8_t rhport, tusb_control_request_t const * p_request)
{
  (void)rhport;
  uint8_t const itf = tu_u16_low(tu_le16toh(p_request->wIndex));
  uint8_t const alt = tu_u16_low(tu_le16toh(p_request->wValue));

  uart_printf("Set interface %d alt %d\r\n", itf, alt);
  if (ITF_NUM_AUDIO_STREAMING == itf && alt != 0)
      blink_interval_ms = BLINK_STREAMING;
  if (ITF_NUM_AUDIO_STREAMING == itf && alt == 0)
      blink_interval_ms = BLINK_MOUNTED;
  
  if(alt != 0)
  {
    //
  }

  return true;
}



//--------------------------------------------------------------------+
// BLINKING TASK
//--------------------------------------------------------------------+
void led_blinking_task(void)
{
  static uint32_t start_ms = 0;
  static bool led_state = false;

  // Blink every interval ms
  if ( (board_millis() - start_ms) < blink_interval_ms) {
    return; // not enough time
  }
  start_ms += blink_interval_ms;

  digitalWrite(PIN_LED, (uint8_t)led_state);
  led_state = 1 - led_state; // toggle
}





int uart_printf(const char *c, ...){

  static char buffer[PRINTF_BUF_SZE];
  memset(buffer, 0 , sizeof buffer);
  va_list args;
  va_start(args, c);
  vsnprintf(&buffer[0], PRINTF_BUF_SZE, c, args);
  va_end(args);
  uartWriteBuf(UART, (const uint8_t*)buffer, strlen(buffer));
  return strlen(buffer);
}


//Fills the memory in 32-bit words for speed.
static void fill_mem_seed(int seed, void *mem, int len, int block_size)
{
    uint32_t *p = (uint32_t *)mem;
    unsigned int rseed = seed ^ 0xa5a5a5a5;
    for (int i = 0; i < len/block_size; i++) {
        rseed = i ^ rseed;
        for (int ii = 0; ii < block_size/4; ii++) {
          *p++ = rand_r(&rseed);
        }
    }
}

//Check the memory filled by fill_mem_seed. Returns true if the data matches the data
//that fill_mem_seed wrote (when given the same seed).
//Returns true if there's a match, false when the region differs from what should be there.
static bool check_mem_seed(int seed, void *mem, int len, int block_size)
{
    uint32_t *p = (uint32_t *)mem;
    unsigned int rseed = seed ^ 0xa5a5a5a5;
    for (int i = 0; i < len/block_size; i++) {
        rseed = i ^ rseed;
        for (int ii = 0; ii < block_size/4; ii++) {
          uint32_t ex = rand_r(&rseed);
          if (ex != *p) {
             Serial.printf("check_mem_seed: %x has 0x%08x expected 0x%08x\n", i*block_size+((char*)p-(char*)mem), *p, ex);
             return false;
          }
          p++;
        }
    }
    return true;
}