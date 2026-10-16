#ifndef SRC_DRIVER_BLUETOOTH_BLUETOOTH_H_
#define SRC_DRIVER_BLUETOOTH_BLUETOOTH_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "../../hal/uart/uart.h"

#define BLUETOOTH_RX_BUFFER_SIZE 128U
#define BLUETOOTH_TX_BUFFER_SIZE 1024U
#define BLUETOOTH_DEFAULT_BAUD_RATE UART_BRR_9600

void BLUETOOTH_Initialize(void);
void BLUETOOTH_ProcessRx(void);
void BLUETOOTH_ProcessTx(void);

bool BLUETOOTH_SendByte(uint8_t data);
size_t BLUETOOTH_SendData(const uint8_t *data, size_t length);
size_t BLUETOOTH_SendString(const char *string);

bool BLUETOOTH_IsDataAvailable(void);
bool BLUETOOTH_ReadByte(uint8_t *data);

bool BLUETOOTH_HasRxOverflow(void);
void BLUETOOTH_ClearRxOverflow(void);
uint32_t BLUETOOTH_GetTxDropCount(void);
void BLUETOOTH_SetLogMirrorEnabled(bool enabled);

#endif /* SRC_DRIVER_BLUETOOTH_BLUETOOTH_H_ */
