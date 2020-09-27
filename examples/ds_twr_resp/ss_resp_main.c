/*! ----------------------------------------------------------------------------
*  @file    ss_resp_main.c
*  @brief   Single-sided two-way ranging (SS TWR) responder example code
*
*           This is a simple code example which acts as the responder in a SS TWR distance measurement exchange. 
*           This application waits for a "poll" message (recording the RX time-stamp of the poll) expected from 
*           the "SS TWR initiator" example code (companion to this application), and
*           then sends a "response" message recording its TX time-stamp.
*
*           Notes at the end of this file, to expand on the inline comments.
*
* @attention
*
* Copyright 2018 (c) Decawave Ltd, Dublin, Ireland.
*
* All rights reserved.
*
* @author Decawave
*/
#include "sdk_config.h" 
#include <stdio.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "deca_device_api.h"
#include "deca_regs.h"
#include "port_platform.h"

#define THIS_ADDRESS_0 0x12
#define THIS_ADDRESS_1 0x37
#define ANT_DELAY 16456
#define RX_TIMEOUT 1000

/* Timestamps of frames transmission/reception.
* As they are 40-bit wide, we need to define a 64-bit int type to handle them. */
typedef signed long long int64;
typedef unsigned long long uint64;
static uint64 poll_rx_ts;

/* Declaration of static functions. */
//static uint64 get_tx_timestamp_u64(void);
static uint64 get_rx_timestamp_u64(void);
static void resp_msg_set_ts(uint8 *ts_field, const uint64 ts);
//static void final_msg_get_ts(const uint8 *ts_field, uint32 *ts);

/* Timestamps of frames transmission/reception.
* As they are 40-bit wide, we need to define a 64-bit int type to handle them. */
typedef unsigned long long uint64;
static uint64 poll_rx_ts;
static uint64 resp_tx_ts;

static uint8 rx_init_msg[] = {0x00, 0x00, THIS_ADDRESS_0, THIS_ADDRESS_1, 'I', 0, 0, 0};
static uint8 tx_ack_msg[] =  {THIS_ADDRESS_0, THIS_ADDRESS_1, 0x00, 0x00, 'A', 0, 0, 0};
static uint8 rx_res_msg[] =  {0x00, 0x00, THIS_ADDRESS_0, THIS_ADDRESS_1, 'R', 0, 0, 0};
static uint8 tx_last_msg[] = {THIS_ADDRESS_0, THIS_ADDRESS_1, 0x00, 0x00, 'L', 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

#define MSG_COMMON_LEN 6
#define MSG_FROM_ADDRESS_IX_0 0
#define MSG_FROM_ADDRESS_IX_1 1
#define MSG_TO_ADDRESS_IX_0 2
#define MSG_TO_ADDRESS_IX_1 3
#define MSG_COMMON_FLAG_IX 4
#define MSG_COMMON_SEQ_IX 5
#define MSG_DATA_DA_IX 6
#define MSG_DATA_RA_IX 10

/* Buffer to store received response message.
* Its size is adjusted to longest frame that this example code is supposed to handle. */
#define RX_BUF_LEN 20
static uint8 rx_buffer[RX_BUF_LEN];

/* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
static uint32 status_reg = 0;


void rx_error_reset()
{
  /* Clear RX error/timeout events in the DW1000 status register. */
  dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
  /* Reset RX to properly reinitialise LDE operation. */
  dwt_rxreset();
}


void send_msg_and_wait_res(uint8* msg, size_t size)
{
  dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
  dwt_writetxdata(size, msg, 0);
  dwt_writetxfctrl(size, 0, 1);
  dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
  
  // Receive ACK Message
  while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
  {};
}

void send_msg_immediate(uint8* msg, size_t size)
{
  dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
  dwt_writetxdata(size, msg, 0);
  dwt_writetxfctrl(size, 0, 1);
  int ret = dwt_starttx(DWT_START_TX_IMMEDIATE);

  /* If dwt_starttx() returns an error, abandon this ranging exchange and proceed to the next one. */
  if (ret == DWT_SUCCESS)
  {
    /* Poll DW1000 until TX frame sent event set. See NOTE 5 below. */
    while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))
    {};

    /* Clear TXFRS event. */
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
  }
  else
  {
    dwt_rxreset();
  }

}

void send_msg_delay(uint8* msg, size_t size)
{
  dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
  dwt_writetxdata(size, msg, 0);
  dwt_writetxfctrl(size, 0, 1);
  int ret = dwt_starttx(DWT_START_TX_DELAYED);

  /* If dwt_starttx() returns an error, abandon this ranging exchange and proceed to the next one. */
  if (ret == DWT_SUCCESS)
  {
    /* Poll DW1000 until TX frame sent event set. See NOTE 5 below. */
    while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))
    {};

    /* Clear TXFRS event. */
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
  }
  else
  {
    dwt_rxreset();
  }
}

uint32 len_rxdata()
{
    /* Clear good RX frame event in the DW1000 status register. */
  dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);
  /* A frame has been received, read it into the local buffer. */
  return dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
}

static uint64 get_tx_timestamp_u64(void)
{
  uint8 ts_tab[5];
  uint64 ts = 0;
  int i;
  dwt_readtxtimestamp(ts_tab);
  for (i = 4; i >= 0; i--)
  {
    ts <<= 8;
    ts |= ts_tab[i];
  }
  return ts;
}

static uint64 get_rx_timestamp_u64(void)
{
  uint8 ts_tab[5];
  uint64 ts = 0;
  int i;
  dwt_readrxtimestamp(ts_tab);
  for (i = 4; i >= 0; i--)
  {
    ts <<= 8;
    ts |= ts_tab[i];
  }
  return ts;
}

int ds_resp_run()
{
  // set to NO receive timeout 
  dwt_setrxtimeout(0);
  /* Activate reception immediately. */
  dwt_rxenable(DWT_START_RX_IMMEDIATE);
  /* Poll for reception of a frame or error/timeout. See NOTE 5 below. */
  while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
  {};


  if (!(status_reg & SYS_STATUS_RXFCG))
  {
    rx_error_reset();
    return -1;
  }

  uint32 frame_len = len_rxdata();
  if (frame_len != sizeof(rx_init_msg))
  {
    dwt_rxreset();
    return -1;
  }

  dwt_readrxdata(rx_buffer, frame_len, 0);

  if (rx_buffer[MSG_TO_ADDRESS_IX_0] != THIS_ADDRESS_0
      || rx_buffer[MSG_TO_ADDRESS_IX_1] != THIS_ADDRESS_1
      || rx_buffer[MSG_COMMON_FLAG_IX] != rx_init_msg[MSG_COMMON_FLAG_IX])
  {
    dwt_rxreset();
    return -1;
  }

  uint64 rxtimestamp = get_rx_timestamp_u64();

  dwt_setrxtimeout(RX_TIMEOUT);
  
  tx_ack_msg[MSG_TO_ADDRESS_IX_0] = rx_buffer[MSG_FROM_ADDRESS_IX_0];
  tx_ack_msg[MSG_TO_ADDRESS_IX_1] = rx_buffer[MSG_FROM_ADDRESS_IX_1];
  rx_res_msg[MSG_FROM_ADDRESS_IX_0] = rx_buffer[MSG_FROM_ADDRESS_IX_0];
  rx_res_msg[MSG_FROM_ADDRESS_IX_1] = rx_buffer[MSG_FROM_ADDRESS_IX_1];
  tx_last_msg[MSG_TO_ADDRESS_IX_0] = rx_buffer[MSG_FROM_ADDRESS_IX_0];
  tx_last_msg[MSG_TO_ADDRESS_IX_1] = rx_buffer[MSG_FROM_ADDRESS_IX_1];

  tx_ack_msg[MSG_COMMON_SEQ_IX] = rx_res_msg[MSG_COMMON_SEQ_IX] = tx_last_msg[MSG_COMMON_SEQ_IX] = rx_buffer[MSG_COMMON_SEQ_IX];

  send_msg_and_wait_res(tx_ack_msg, sizeof(tx_ack_msg));

  if (!(status_reg & SYS_STATUS_RXFCG))
  {
    rx_error_reset();
    return -1;
  }

  frame_len = len_rxdata();
  if (frame_len != sizeof(rx_res_msg))
  {
    dwt_rxreset();
    return -1;
  }

  dwt_readrxdata(rx_buffer, frame_len, 0);

  if (memcmp(rx_buffer, rx_res_msg, MSG_COMMON_LEN) != 0)
  {
    dwt_rxreset();
    return -1;
  }

  uint64 txtimestamp = get_tx_timestamp_u64();

  int64 Da = txtimestamp - rxtimestamp;
  if (Da < 0)
  {
    dwt_rxreset();
    return -1;
  }

  rxtimestamp = get_rx_timestamp_u64();

  int64 Ra = rxtimestamp - txtimestamp;
  if (Ra < 0)
  {
    dwt_rxreset();
    return -1;
  }

  *(uint32*)(&tx_last_msg[MSG_DATA_DA_IX]) = (uint32)Da;
  *(uint32*)(&tx_last_msg[MSG_DATA_RA_IX]) = (uint32)Ra;

  send_msg_immediate(tx_last_msg, sizeof(tx_last_msg));

  /* Retrieve poll reception timestamp. */
  //poll_rx_ts = get_rx_timestamp_u64();

  /* Response TX timestamp is the transmission time we programmed plus the antenna delay. */
  //resp_tx_ts = (((uint64)(resp_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

  /* Write all timestamps in the final message. See NOTE 8 below. */
  //resp_msg_set_ts(&tx_resp_msg[RESP_MSG_POLL_RX_TS_IDX], poll_rx_ts);
  //resp_msg_set_ts(&tx_resp_msg[RESP_MSG_RESP_TX_TS_IDX], resp_tx_ts);

  return 0;
}

/**@brief SS TWR Initiator task entry function.
*
* @param[in] pvParameter   Pointer that will be used as the parameter for the task.
*/
void ss_responder_task_function (void * pvParameter)
{
  UNUSED_PARAMETER(pvParameter);

  dwt_setrxantennadelay(ANT_DELAY);
  dwt_settxantennadelay(ANT_DELAY);

  dwt_setrxaftertxdelay(0);

  dwt_setleds(DWT_LEDS_ENABLE);

  while (true)
  {
    ds_resp_run();
    /* Delay a task for a given number of ticks */
    vTaskDelay(1);
    /* Tasks must be implemented to never return... */
  }
}
/*****************************************************************************************************************************************************
* NOTES:
*
* 1. This is the task delay when using FreeRTOS. Task is delayed a given number of ticks. Useful to be able to define this out to see the effect of the RTOS
*    on timing.
* 2. The frames used here are Decawave specific ranging frames, complying with the IEEE 802.15.4 standard data frame encoding. The frames are the
*    following:
*     - a poll message sent by the initiator to trigger the ranging exchange.
*     - a response message sent by the responder to complete the exchange and provide all information needed by the initiator to compute the
*       time-of-flight (distance) estimate.
*    The first 10 bytes of those frame are common and are composed of the following fields:
*     - byte 0/1: frame control (0x8841 to indicate a data frame using 16-bit addressing).
*     - byte 2: sequence number, incremented for each new frame.
*     - byte 3/4: PAN ID (0xDECA).
*     - byte 5/6: destination address, see NOTE 3 below.
*     - byte 7/8: source address, see NOTE 3 below.
*     - byte 9: function code (specific values to indicate which message it is in the ranging process).
*    The remaining bytes are specific to each message as follows:
*    Poll message:
*     - no more data
*    Response message:
*     - byte 10 -> 13: poll message reception timestamp.
*     - byte 14 -> 17: response message transmission timestamp.
*    All messages end with a 2-byte checksum automatically set by DW1000.
* 3. Source and destination addresses are hard coded constants in this example to keep it simple but for a real product every device should have a
*    unique ID. Here, 16-bit addressing is used to keep the messages as short as possible but, in an actual application, this should be done only
*    after an exchange of specific messages used to define those short addresses for each device participating to the ranging exchange.
* 4. dwt_writetxdata() takes the full size of the message as a parameter but only copies (size - 2) bytes as the check-sum at the end of the frame is
*    automatically appended by the DW1000. This means that our variable could be two bytes shorter without losing any data (but the sizeof would not
*    work anymore then as we would still have to indicate the full length of the frame to dwt_writetxdata()).
* 5. We use polled mode of operation here to keep the example as simple as possible but all status events can be used to generate interrupts. Please
*    refer to DW1000 User Manual for more details on "interrupts". It is also to be noted that STATUS register is 5 bytes long but, as the event we
*    use are all in the first bytes of the register, we can use the simple dwt_read32bitreg() API call to access it instead of reading the whole 5
*    bytes.
* 6. POLL_RX_TO_RESP_TX_DLY_UUS is a critical value for porting to different processors. For slower platforms where the SPI is at a slower speed 
*    or the processor is operating at a lower frequency (Comparing to STM32F, SPI of 18MHz and Processor internal 72MHz)this value needs to be increased.
*    Knowing the exact time when the responder is going to send its response is vital for time of flight calculation. The specification of the time of 
*    respnse must allow the processor enough time to do its calculations and put the packet in the Tx buffer. So more time required for a slower
*    system(processor).
* 7. As we want to send final TX timestamp in the final message, we have to compute it in advance instead of relying on the reading of DW1000
*    register. Timestamps and delayed transmission time are both expressed in device time units so we just have to add the desired response delay to
*    response RX timestamp to get final transmission time. The delayed transmission time resolution is 512 device time units which means that the
*    lower 9 bits of the obtained value must be zeroed. This also allows to encode the 40-bit value in a 32-bit words by shifting the all-zero lower
*    8 bits.
* 8. In this operation, the high order byte of each 40-bit timestamps is discarded. This is acceptable as those time-stamps are not separated by
*    more than 2**32 device time units (which is around 67 ms) which means that the calculation of the round-trip delays (needed in the
*    time-of-flight computation) can be handled by a 32-bit subtraction.
* 9. dwt_writetxdata() takes the full size of the message as a parameter but only copies (size - 2) bytes as the check-sum at the end of the frame is
*    automatically appended by the DW1000. This means that our variable could be two bytes shorter without losing any data (but the sizeof would not
*    work anymore then as we would still have to indicate the full length of the frame to dwt_writetxdata()).
*10. The user is referred to DecaRanging ARM application (distributed with EVK1000 product) for additional practical example of usage, and to the
*    DW1000 API Guide for more details on the DW1000 driver functions.
*
****************************************************************************************************************************************************/
 