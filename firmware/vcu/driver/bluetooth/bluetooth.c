#include "bluetooth.h"
#include <stddef.h>
#include "xparameters.h"
#include "xuartlite_l.h"

static volatile uint8_t rx_buffer[BLUETOOTH_RX_BUFFER_SIZE];
static volatile uint16_t rx_head;
static volatile uint16_t rx_tail;
static volatile bool rx_overflow;
static volatile uint8_t tx_buffer[BLUETOOTH_TX_BUFFER_SIZE];
static volatile uint16_t tx_head;
static volatile uint16_t tx_tail;
static volatile uint32_t tx_drop_count;
static volatile bool log_mirror_enabled = true;

static uint16_t BLUETOOTH_NextIndex(uint16_t index)
{
	index++;
	if (index >= BLUETOOTH_RX_BUFFER_SIZE) {
		index = 0U;
	}
	return index;
}

static uint16_t BLUETOOTH_NextTxIndex(uint16_t index)
{
	index++;
	if (index >= BLUETOOTH_TX_BUFFER_SIZE) {
		index = 0U;
	}
	return index;
}

static void BLUETOOTH_PushRxByte(uint8_t data)
{
	uint16_t next = BLUETOOTH_NextIndex(rx_head);

	if (next == rx_tail) {
		rx_overflow = true;
		return;
	}

	rx_buffer[rx_head] = data;
	rx_head = next;
}

void BLUETOOTH_Initialize(void)
{
	rx_head = 0U;
	rx_tail = 0U;
	rx_overflow = false;
	tx_head = 0U;
	tx_tail = 0U;
	tx_drop_count = 0U;

	UART_Initialize(UART, BLUETOOTH_DEFAULT_BAUD_RATE);
	UART_ClearRxInterrupt(UART);
	/* VCU uses polling TX only; do not enable an unconnected RX interrupt. */
}

void BLUETOOTH_ProcessRx(void)
{
	uint8_t data;

	while (UART_ReadByte(UART, &data)) {
		BLUETOOTH_PushRxByte(data);
	}
}

bool BLUETOOTH_SendByte(uint8_t data)
{
	uint16_t next = BLUETOOTH_NextTxIndex(tx_head);

	if (next == tx_tail) {
		tx_drop_count++;
		return false;
	}

	tx_buffer[tx_head] = data;
	tx_head = next;
	return true;
}

void BLUETOOTH_ProcessTx(void)
{
	while (tx_tail != tx_head) {
		if (UART_WriteByte(UART, tx_buffer[tx_tail]) == false) {
			break;
		}
		tx_tail = BLUETOOTH_NextTxIndex(tx_tail);
	}
}

size_t BLUETOOTH_SendData(const uint8_t *data, size_t length)
{
	size_t sent = 0U;

	if (data == NULL) {
		return 0U;
	}

	while ((sent < length) && BLUETOOTH_SendByte(data[sent])) {
		sent++;
	}
	return sent;
}

size_t BLUETOOTH_SendString(const char *string)
{
	size_t sent = 0U;

	if (string == NULL) {
		return 0U;
	}

	while ((string[sent] != '\0') && BLUETOOTH_SendByte((uint8_t)string[sent])) {
		sent++;
	}
	return sent;
}

bool BLUETOOTH_IsDataAvailable(void)
{
	return (rx_head != rx_tail);
}

bool BLUETOOTH_ReadByte(uint8_t *data)
{
	if ((data == NULL) || (rx_head == rx_tail)) {
		return false;
	}

	*data = rx_buffer[rx_tail];
	rx_tail = BLUETOOTH_NextIndex(rx_tail);
	return true;
}

bool BLUETOOTH_HasRxOverflow(void)
{
	return rx_overflow;
}

void BLUETOOTH_ClearRxOverflow(void)
{
	rx_overflow = false;
}

uint32_t BLUETOOTH_GetTxDropCount(void)
{
	return tx_drop_count;
}

void BLUETOOTH_SetLogMirrorEnabled(bool enabled)
{
	log_mirror_enabled = enabled;
}

/* Send every xil_printf byte to both the PC UART and the Bluetooth UART. */
void outbyte(char data)
{
	XUartLite_SendByte(STDOUT_BASEADDRESS, (uint8_t)data);
	if (log_mirror_enabled != false) {
		(void)BLUETOOTH_SendByte((uint8_t)data);
	}
}
