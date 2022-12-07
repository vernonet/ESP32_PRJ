/* 
 * The MIT License (MIT)
 *
 * Copyright (c) 2020 Jerzy Kasenbreg
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#ifndef _USB_DESCRIPTORS_H_
#define _USB_DESCRIPTORS_H_

#include "tusb.h"

// Unit numbers are arbitrary selected
// Microphone path
#define UAC2_ENTITY_MIC_INPUT_TERMINAL  0x01
#define UAC2_ENTITY_MIC_FEATURE_UNIT    0x02
#define UAC2_ENTITY_MIC_OUTPUT_TERMINAL 0x03
#define UAC2_ENTITY_CLOCK               0x04

#define UAC1_ENTITY_MIC_INPUT_TERMINAL  0x01
#define UAC1_ENTITY_MIC_OUTPUT_TERMINAL 0x03
#define UAC1_ENTITY_MIC_FEATURE_UNIT    0x05

enum
{
  ITF_NUM_AUDIO_CONTROL = 0,
  ITF_NUM_AUDIO_STREAMING,
  ITF_NUM_TOTAL
};
/////////////////////////////////////////////////////////////////////////////////////////////////  
#define TUD_AUDIO_MIC_ONE_CH_DESC_LEN_UAC1 (TUD_AUDIO_DESC_STD_AC_LEN\
  + TUD_AUDIO_DESC_CS_AC_LEN\
  + TUD_AUDIO_DESC_INPUT_TERM_LEN_UAC1\
  + TUD_AUDIO_DESC_FEATURE_UNIT_ONE_CHANNEL_LEN_UAC1\
  + TUD_AUDIO_DESC_OUTPUT_TERM_LEN_UAC1\
  + TUD_AUDIO_DESC_STD_AS_INT_LEN\
  + TUD_AUDIO_DESC_STD_AS_INT_LEN\
  + TUD_AUDIO_DESC_CS_AS_INT_LEN_UAC1\
  + TUD_AUDIO_DESC_TYPE_I_FORMAT_LEN_UAC1\
  + TUD_AUDIO_DESC_STD_AS_ISO_EP_LEN_UAC1\
  + TUD_AUDIO_DESC_CS_AS_ISO_EP_LEN_UAC1)

#define TUD_AUDIO_MIC_ONE_CH_DESC_N_AS_INT 1 	// Number of AS interfaces

#define TUD_AUDIO_MIC_ONE_CH_DESCRIPTOR_UAC1(_itfnum, _stridx, _nBytesPerSample, _nBitsUsedPerSample, _epin, _epsize, _protocol) \
  /* Standard AC Interface Descriptor(4.7.1) */\
  TUD_AUDIO_DESC_STD_AC(/*_itfnum*/ _itfnum, /*_nEPs*/ 0x00, /*_stridx*/ _stridx, /*_protocol*/ _protocol),\
  /* Class-Specific AC Interface Header Descriptor(4.7.2) */\
  TUD_AUDIO_DESC_CS_AC_UAC1(/*_bcdADC*/ 0x0100, /*_totallen*/ TUD_AUDIO_DESC_CS_AC_LEN+TUD_AUDIO_DESC_INPUT_TERM_LEN_UAC1+TUD_AUDIO_DESC_OUTPUT_TERM_LEN_UAC1+TUD_AUDIO_DESC_FEATURE_UNIT_ONE_CHANNEL_LEN_UAC1, /*_ctrl*/ 0x01),\
  /* Input Terminal Descriptor(4.7.2.4) */\
  TUD_AUDIO_DESC_INPUT_TERM_UAC1(/*_termid*/ UAC1_ENTITY_MIC_INPUT_TERMINAL, /*_termtype*/ AUDIO_TERM_TYPE_IN_GENERIC_MIC, /*_assocTerm*/ 0x00,  /*_nchannelslogical*/ 0x01, /*_channelcfg*/ AUDIO_CHANNEL_CONFIG_NON_PREDEFINED, /*_idxchannelnames*/ 0x00,  /*_stridx*/ 0x00),\
  /* Feature Unit Descriptor(4.7.2.8) */\
  TUD_AUDIO_DESC_FEATURE_UNIT_ONE_CHANNEL_UAC1(/*_unitid*/ UAC1_ENTITY_MIC_FEATURE_UNIT, /*_srcid*/ UAC1_ENTITY_MIC_INPUT_TERMINAL, /*_ctrlch0master*/ 0x0001 , /*_ctrlch1*/ 0x0002, /*_stridx*/ 0x00),\
  /* Output Terminal Descriptor(4.7.2.5) */\
  TUD_AUDIO_DESC_OUTPUT_TERM_UAC1(/*_termid*/ UAC1_ENTITY_MIC_OUTPUT_TERMINAL, /*_termtype*/ AUDIO_TERM_TYPE_USB_STREAMING, /*_assocTerm*/ 0x00, /*_srcid*/ UAC1_ENTITY_MIC_FEATURE_UNIT,  /*_stridx*/ 0x00),\
  /* Standard AS Interface Descriptor(4.9.1) */\
  /* Interface 1, Alternate 0 - default alternate setting with 0 bandwidth */\
  TUD_AUDIO_DESC_STD_AS_INT(/*_itfnum*/ (uint8_t)((_itfnum)+1), /*_altset*/ 0x00, /*_nEPs*/ 0x00,  _protocol, /*_stridx*/ 0x00),\
  /* Standard AS Interface Descriptor(4.9.1) */\
  /* Interface 1, Alternate 1 - alternate interface for data streaming */\
  TUD_AUDIO_DESC_STD_AS_INT(/*_itfnum*/ (uint8_t)((_itfnum)+1), /*_altset*/ 0x01, /*_nEPs*/ 0x01, _protocol, /*_stridx*/ 0x00),\
  /* Class-Specific AS Interface Descriptor(4.9.2) */\
  TUD_AUDIO_DESC_CS_AS_INT_UAC1(/*_termid*/ UAC1_ENTITY_MIC_OUTPUT_TERMINAL, 1, /*_formattype*/  AUDIO_DATA_FORMAT_TYPE_I_PCM),\
  /* Type I Format Type Descriptor(2.3.1.6 - Audio Formats) */\
  TUD_AUDIO_DESC_TYPE_I_FORMAT_UAC1(_nBytesPerSample, _nBitsUsedPerSample),\
  /* Standard AS Isochronous Audio Data Endpoint Descriptor(4.10.1.1) */\
  TUD_AUDIO_DESC_STD_AS_ISO_EP_UAC1(/*_ep*/ _epin, /*_attr*/ (TUSB_XFER_ISOCHRONOUS | TUSB_ISO_EP_ATT_ASYNCHRONOUS | TUSB_ISO_EP_ATT_DATA), /*_maxEPsize*/ _epsize, /*_interval*/ TUD_OPT_HIGH_SPEED ? 0x04 : 0x01),\
  /* Class-Specific AS Isochronous Audio Data Endpoint Descriptor(4.10.1.2) */\
  TUD_AUDIO_DESC_CS_AS_ISO_EP_UAC1(/*_attr*/ AUDIO_CS_AS_ISO_DATA_EP_ATT_NON_MAX_PACKETS_OK, /*_lockdelayunit*/ AUDIO_CS_AS_ISO_DATA_EP_LOCK_DELAY_UNIT_UNDEFINED, /*_lockdelay*/ 0x0000)



#define TUD_AUDIO_MIC_ONE_CH_DESC_LEN_UAC2 (TUD_AUDIO_DESC_IAD_LEN\
  + TUD_AUDIO_DESC_STD_AC_LEN\
  + TUD_AUDIO_DESC_CS_AC_LEN\
  + TUD_AUDIO_DESC_CLK_SRC_LEN\
  + TUD_AUDIO_DESC_INPUT_TERM_LEN_UAC2\
  + TUD_AUDIO_DESC_OUTPUT_TERM_LEN_UAC2\
  + TUD_AUDIO_DESC_FEATURE_UNIT_ONE_CHANNEL_LEN_UAC2\
  + TUD_AUDIO_DESC_STD_AS_INT_LEN\
  + TUD_AUDIO_DESC_STD_AS_INT_LEN\
  + TUD_AUDIO_DESC_CS_AS_INT_LEN_UAC2\
  + TUD_AUDIO_DESC_TYPE_I_FORMAT_LEN_UAC2\
  + TUD_AUDIO_DESC_STD_AS_ISO_EP_LEN_UAC2\
  + TUD_AUDIO_DESC_CS_AS_ISO_EP_LEN_UAC2)


#define TUD_AUDIO_MIC_ONE_CH_DESCRIPTOR_UAC2(_itfnum, _stridx, _nBytesPerSample, _nBitsUsedPerSample, _epin, _epsize, _protocol) \
  /* Standard Interface Association Descriptor (IAD) */\
  TUD_AUDIO_DESC_IAD(/*_firstitfs*/ _itfnum, /*_nitfs*/ 0x02, _protocol, /*_stridx*/ 0x00),\
  /* Standard AC Interface Descriptor(4.7.1) */\
  TUD_AUDIO_DESC_STD_AC(/*_itfnum*/ _itfnum, /*_nEPs*/ 0x00, /*_stridx*/ _stridx, _protocol),\
  /* Class-Specific AC Interface Header Descriptor(4.7.2) */\
  TUD_AUDIO_DESC_CS_AC_UAC2(/*_bcdADC*/ 0x0200, /*_category*/ AUDIO_FUNC_MICROPHONE, /*_totallen*/ TUD_AUDIO_DESC_CLK_SRC_LEN+TUD_AUDIO_DESC_INPUT_TERM_LEN_UAC2+TUD_AUDIO_DESC_OUTPUT_TERM_LEN_UAC2+TUD_AUDIO_DESC_FEATURE_UNIT_ONE_CHANNEL_LEN_UAC2, /*_ctrl*/ AUDIO_CS_AS_INTERFACE_CTRL_LATENCY_POS),\
  /* Clock Source Descriptor(4.7.2.1) */\
  TUD_AUDIO_DESC_CLK_SRC(/*_clkid*/ UAC2_ENTITY_CLOCK, /*_attr*/ AUDIO_CLOCK_SOURCE_ATT_INT_PRO_CLK, /*_ctrl*/ AUDIO_CTRL_RW << AUDIO_CLOCK_SOURCE_CTRL_CLK_FRQ_POS | AUDIO_CTRL_R << AUDIO_CLOCK_SOURCE_CTRL_CLK_VAL_POS, /*_assocTerm*/ 0x01,  /*_stridx*/ 0x00),\
  /* Input Terminal Descriptor(4.7.2.4) */\
  TUD_AUDIO_DESC_INPUT_TERM_UAC2(/*_termid*/ UAC2_ENTITY_MIC_INPUT_TERMINAL, /*_termtype*/ AUDIO_TERM_TYPE_IN_GENERIC_MIC, /*_assocTerm*/ UAC2_ENTITY_MIC_OUTPUT_TERMINAL, /*_clkid*/ 0x04, /*_nchannelslogical*/ 0x01, /*_channelcfg*/ AUDIO_CHANNEL_CONFIG_NON_PREDEFINED, /*_idxchannelnames*/ 0x00, /*_ctrl*/ AUDIO_CTRL_R << AUDIO_IN_TERM_CTRL_CONNECTOR_POS, /*_stridx*/ 0x00),\
  /* Output Terminal Descriptor(4.7.2.5) */\
  TUD_AUDIO_DESC_OUTPUT_TERM_UAC2(/*_termid*/ UAC2_ENTITY_MIC_OUTPUT_TERMINAL, /*_termtype*/ AUDIO_TERM_TYPE_USB_STREAMING, /*_assocTerm*/ UAC2_ENTITY_MIC_INPUT_TERMINAL, /*_srcid*/ 0x02, /*_clkid*/ 0x04, /*_ctrl*/ 0x0000, /*_stridx*/ 0x00),\
  /* Feature Unit Descriptor(4.7.2.8) */\
  TUD_AUDIO_DESC_FEATURE_UNIT_ONE_CHANNEL_UAC2(/*_unitid*/ 0x02, /*_srcid*/ 0x01, /*_ctrlch0master*/ AUDIO_CTRL_RW << AUDIO_FEATURE_UNIT_CTRL_MUTE_POS | AUDIO_CTRL_RW << AUDIO_FEATURE_UNIT_CTRL_VOLUME_POS, /*_ctrlch1*/ AUDIO_CTRL_RW << AUDIO_FEATURE_UNIT_CTRL_MUTE_POS | AUDIO_CTRL_RW << AUDIO_FEATURE_UNIT_CTRL_VOLUME_POS, /*_stridx*/ 0x00),\
  /* Standard AS Interface Descriptor(4.9.1) */\
  /* Interface 1, Alternate 0 - default alternate setting with 0 bandwidth */\
  TUD_AUDIO_DESC_STD_AS_INT(/*_itfnum*/ (uint8_t)((_itfnum)+1), /*_altset*/ 0x00, /*_nEPs*/ 0x00, _protocol, /*_stridx*/ 0x00),\
  /* Standard AS Interface Descriptor(4.9.1) */\
  /* Interface 1, Alternate 1 - alternate interface for data streaming */\
  TUD_AUDIO_DESC_STD_AS_INT(/*_itfnum*/ (uint8_t)((_itfnum)+1), /*_altset*/ 0x01, /*_nEPs*/ 0x01, _protocol, /*_stridx*/ 0x00),\
  /* Class-Specific AS Interface Descriptor(4.9.2) */\
  TUD_AUDIO_DESC_CS_AS_INT_UAC2(/*_termid*/ 0x03, /*_ctrl*/ AUDIO_CTRL_NONE, /*_formattype*/ AUDIO_FORMAT_TYPE_I, /*_formats*/ AUDIO_DATA_FORMAT_TYPE_I_PCM, /*_nchannelsphysical*/ 0x01, /*_channelcfg*/ AUDIO_CHANNEL_CONFIG_NON_PREDEFINED, /*_stridx*/ 0x00),\
  /* Type I Format Type Descriptor(2.3.1.6 - Audio Formats) */\
  TUD_AUDIO_DESC_TYPE_I_FORMAT_UAC2(_nBytesPerSample, _nBitsUsedPerSample),\
  /* Standard AS Isochronous Audio Data Endpoint Descriptor(4.10.1.1) */\
  TUD_AUDIO_DESC_STD_AS_ISO_EP_UAC2(/*_ep*/ _epin, /*_attr*/ (TUSB_XFER_ISOCHRONOUS | TUSB_ISO_EP_ATT_ASYNCHRONOUS | TUSB_ISO_EP_ATT_DATA), /*_maxEPsize*/ _epsize, /*_interval*/ TUD_OPT_HIGH_SPEED ? 0x04 : 0x01),\
  /* Class-Specific AS Isochronous Audio Data Endpoint Descriptor(4.10.1.2) */\
  TUD_AUDIO_DESC_CS_AS_ISO_EP_UAC2(/*_attr*/ AUDIO_CS_AS_ISO_DATA_EP_ATT_NON_MAX_PACKETS_OK, /*_ctrl*/ AUDIO_CTRL_NONE, /*_lockdelayunit*/ AUDIO_CS_AS_ISO_DATA_EP_LOCK_DELAY_UNIT_UNDEFINED, /*_lockdelay*/ 0x0000)
#endif