/*!
* @brief Component name:	UART
*
* Defines public interface to UART application level driver.
*
* @file UART.h
*/

#include "app_uart.h"

bool boUART_Init	(void);
bool boUART_getc	(uint8_t *u8ch);
