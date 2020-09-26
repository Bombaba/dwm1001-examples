/*! ----------------------------------------------------------------------------
*  @file    ds_init_main.c
*  @brief   Double-sided two-way ranging (SS TWR) initiator example code
*
* @author Decawave
*/
#include <stdio.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "deca_device_api.h"
#include "deca_regs.h"
#include "port_platform.h"
#include "UART.h"

#define APP_NAME "DS TWR INIT v1.3"

#define THIS_ADDRESS_0 0x89
#define THIS_ADDRESS_1 0x13

/* Speed of light in air, in metres per second. */
//#define SPEED_OF_LIGHT 299702547
#define SPEED_OF_LIGHT 299711535.89

/* Hold copies of computed time of flight and distance here for reference so that it can be examined at a debug breakpoint. */
static double tof;
static double distance;
static unsigned short distance_ushort;

/* Declaration of static functions. */
static void resp_msg_get_ts(uint8 *ts_field, uint32 *ts);

static uint8 tx_init_msg[] = {THIS_ADDRESS_0, THIS_ADDRESS_1, 0x00, 0x00, 'I', 0, 0, 0};
static uint8 rx_ack_msg[] =  {0x00, 0x00, THIS_ADDRESS_0, THIS_ADDRESS_1, 'A', 0, 0, 0};
static uint8 tx_res_msg[] =  {THIS_ADDRESS_0, THIS_ADDRESS_1, 0x00, 0x00, 'R', 0, 0, 0};
static uint8 rx_last_msg[] = {0x00, 0x00, THIS_ADDRESS_0, THIS_ADDRESS_1, 'L', 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

static uint8 rx_init_msg[] = {0x00, 0x00, THIS_ADDRESS_0, THIS_ADDRESS_1, 'I', 0, 0, 0};
static uint8 tx_ack_msg[] =  {THIS_ADDRESS_0, THIS_ADDRESS_1, 0x00, 0x00, 'A', 0, 0, 0};
static uint8 rx_res_msg[] =  {0x00, 0x00, THIS_ADDRESS_0, THIS_ADDRESS_1, 'R', 0, 0, 0};
static uint8 tx_last_msg[] = {THIS_ADDRESS_0, THIS_ADDRESS_1, 0x00, 0x00, 'L', 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

static uint8 seq_no = 0;

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

void output_uart(uint8* pdata, int size)
{
  for (int i = 0; i < size; i++)
  {
    app_uart_put(pdata[i]);
  }

  printf("");
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

float ds_init_run(uint8 dest_address[2])
{
  tx_init_msg[MSG_TO_ADDRESS_IX_0] = dest_address[0];
  tx_init_msg[MSG_TO_ADDRESS_IX_1] = dest_address[1];
  rx_ack_msg[MSG_FROM_ADDRESS_IX_0] = dest_address[0];
  rx_ack_msg[MSG_FROM_ADDRESS_IX_1] = dest_address[1];
  tx_res_msg[MSG_TO_ADDRESS_IX_0] = dest_address[0];
  tx_res_msg[MSG_TO_ADDRESS_IX_1] = dest_address[1];
  rx_last_msg[MSG_FROM_ADDRESS_IX_0] = dest_address[0];
  rx_last_msg[MSG_FROM_ADDRESS_IX_1] = dest_address[1];

  tx_init_msg[MSG_COMMON_SEQ_IX] = rx_ack_msg[MSG_COMMON_SEQ_IX]
   = tx_res_msg[MSG_COMMON_SEQ_IX] = rx_last_msg[MSG_COMMON_SEQ_IX] = seq_no;
  seq_no++;

  // send Initial Message
  send_msg_and_wait_res(tx_init_msg, sizeof(tx_init_msg));

  uint32 txtimestamp = dwt_readtxtimestamplo32();

  if (!(status_reg & SYS_STATUS_RXFCG))
  {
    rx_error_reset();
    return 0;
  }

  uint32 frame_len = len_rxdata();
  if (frame_len != sizeof(rx_ack_msg))
  {
    return 0;
  }

  dwt_readrxdata(rx_buffer, frame_len, 0);

  if (memcmp(rx_buffer, rx_ack_msg, MSG_COMMON_LEN) != 0)
  {
    return 0;
  }
  uint32 rxtimestamp = dwt_readrxtimestamplo32();
  uint32 Rt = rxtimestamp - txtimestamp;

  // send Response message
  send_msg_and_wait_res(tx_res_msg, sizeof(tx_res_msg));

  txtimestamp = dwt_readtxtimestamplo32();
  uint32 Dt = txtimestamp - rxtimestamp;

    if (!(status_reg & SYS_STATUS_RXFCG))
  {
    rx_error_reset();
    return 0;
  }

  frame_len = len_rxdata();
  if (frame_len != sizeof(rx_last_msg))
  {
    return 0;
  }

  dwt_readrxdata(rx_buffer, frame_len, 0);

  if (memcmp(rx_buffer, rx_last_msg, MSG_COMMON_LEN) != 0)
  {
    return 0;
  }

  double unit = 1.0 / (128 * 499.2 * 1e6);
  double Da = *(uint32*)(&rx_buffer[MSG_DATA_DA_IX]);
  double Ra = *(uint32*)(&rx_buffer[MSG_DATA_RA_IX]);
  double time = (Ra * Rt - Da * Dt) * unit / (Ra + Da + Rt + Dt);
  double distance = time * SPEED_OF_LIGHT;

  return (float)distance;
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

  dwt_setrxtimeout(1000);

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

  uint32 rxtimestamp = dwt_readrxtimestamplo32();
  
  tx_ack_msg[MSG_TO_ADDRESS_IX_0] = rx_buffer[MSG_FROM_ADDRESS_IX_0];
  tx_ack_msg[MSG_TO_ADDRESS_IX_1] = rx_buffer[MSG_FROM_ADDRESS_IX_1];
  rx_res_msg[MSG_FROM_ADDRESS_IX_0] = rx_buffer[MSG_FROM_ADDRESS_IX_0];
  rx_res_msg[MSG_FROM_ADDRESS_IX_1] = rx_buffer[MSG_FROM_ADDRESS_IX_1];
  tx_last_msg[MSG_TO_ADDRESS_IX_0] = rx_buffer[MSG_FROM_ADDRESS_IX_0];
  tx_last_msg[MSG_TO_ADDRESS_IX_1] = rx_buffer[MSG_FROM_ADDRESS_IX_1];

  tx_ack_msg[MSG_COMMON_SEQ_IX] = rx_res_msg[MSG_COMMON_SEQ_IX] = tx_last_msg[MSG_COMMON_SEQ_IX] = rx_buffer[MSG_COMMON_SEQ_IX];

  send_msg_and_wait_res(tx_ack_msg, sizeof(tx_ack_msg));

  uint32 txtimestamp = dwt_readtxtimestamplo32();
  uint32 Da = txtimestamp - rxtimestamp;

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

  rxtimestamp = dwt_readrxtimestamplo32();
  uint32 Ra = rxtimestamp - txtimestamp;

  *(uint32*)(&tx_last_msg[MSG_DATA_DA_IX]) = Da;
  *(uint32*)(&tx_last_msg[MSG_DATA_RA_IX]) = Ra;

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

void output_address()
{
  app_uart_put(THIS_ADDRESS_0);
  app_uart_put(THIS_ADDRESS_1);
  printf("");
}

void set_delay()
{
  uint8 delay[2];
  int ix = 0;
  while (ix < 2)
  {
    if (boUART_getc(&delay[ix]))
    {
      ix++;
    }
  }
  dwt_setrxantennadelay(*(uint16*)delay);
  dwt_settxantennadelay(*(uint16*)delay);
}

void try_tx()
{
  uint8 dest_address[2];
  int ix = 0;
  while (ix < 2)
  {
    if (boUART_getc(&dest_address[ix]))
    {
      ix++;
    }
  }
  float ret = ds_init_run(dest_address);
  output_uart((uint8*)&ret, 4);
}


/**@brief SS TWR Initiator task entry function.
*
* @param[in] pvParameter   Pointer that will be used as the parameter for the task.
*/
void ss_initiator_task_function (void * pvParameter)
{
  UNUSED_PARAMETER(pvParameter);
  uint8 mode;

  //dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);

  dwt_setleds(DWT_LEDS_ENABLE);

  while (true)
  {
    if (boUART_getc(&mode))
    {
      switch (mode) {
      case 0x01:
        output_address();
        break;
      case 0x02:
        set_delay();
        break;
      case 0x03:
        try_tx();
        break;
      case 0x04:
        ds_resp_run();
        break;
      }
    }

    /* Delay a task for a given number of ticks */
    //vTaskDelay(RNG_DELAY_MS);
    /* Tasks must be implemented to never return... */
  }
}
/*****************************************************************************************************************************************************
* NOTES:
*
* 1. The frames used here are Decawave specific ranging frames, complying with the IEEE 802.15.4 standard data frame encoding. The frames are the
*    following:
*     - a poll message sent by the initiator to trigger the ranging exchange.
*     - a response message sent by the responder to complete the exchange and provide all information needed by the initiator to compute the
*       time-of-flight (distance) estimate.
*    The first 10 bytes of those frame are common and are composed of the following fields:
*     - byte 0/1: frame control (0x8841 to indicate a data frame using 16-bit addressing).
*     - byte 2: sequence number, incremented for each new frame.
*     - byte 3/4: PAN ID (0xDECA).
*     - byte 5/6: destination address, see NOTE 2 below.
*     - byte 7/8: source address, see NOTE 2 below.
*     - byte 9: function code (specific values to indicate which message it is in the ranging process).
*    The remaining bytes are specific to each message as follows:
*    Poll message:
*     - no more data
*    Response message:
*     - byte 10 -> 13: poll message reception timestamp.
*     - byte 14 -> 17: response message transmission timestamp.
*    All messages end with a 2-byte checksum automatically set by DW1000.
* 2. Source and destination addresses are hard coded constants in this example to keep it simple but for a real product every device should have a
*    unique ID. Here, 16-bit addressing is used to keep the messages as short as possible but, in an actual application, this should be done only
*    after an exchange of specific messages used to define those short addresses for each device participating to the ranging exchange.
* 3. dwt_writetxdata() takes the full size of the message as a parameter but only copies (size - 2) bytes as the check-sum at the end of the frame is
*    automatically appended by the DW1000. This means that our variable could be two bytes shorter without losing any data (but the sizeof would not
*    work anymore then as we would still have to indicate the full length of the frame to dwt_writetxdata()).
* 4. We use polled mode of operation here to keep the example as simple as possible but all status events can be used to generate interrupts. Please
*    refer to DW1000 User Manual for more details on "interrupts". It is also to be noted that STATUS register is 5 bytes long but, as the event we
*    use are all in the first bytes of the register, we can use the simple dwt_read32bitreg() API call to access it instead of reading the whole 5
*    bytes.
* 5. The high order byte of each 40-bit time-stamps is discarded here. This is acceptable as, on each device, those time-stamps are not separated by
*    more than 2**32 device time units (which is around 67 ms) which means that the calculation of the round-trip delays can be handled by a 32-bit
*    subtraction.
* 6. The user is referred to DecaRanging ARM application (distributed with EVK1000 product) for additional practical example of usage, and to the
*     DW1000 API Guide for more details on the DW1000 driver functions.
* 7. The use of the carrier integrator value to correct the TOF calculation, was added Feb 2017 for v1.3 of this example.  This significantly
*     improves the result of the SS-TWR where the remote responder unit's clock is a number of PPM offset from the local inmitiator unit's clock.
*     As stated in NOTE 2 a fixed offset in range will be seen unless the antenna delsy is calibratred and set correctly.
*
****************************************************************************************************************************************************/
