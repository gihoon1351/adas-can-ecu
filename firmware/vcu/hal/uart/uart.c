#include "uart.h"
#include <stddef.h>

void UART_SetBaudRate(UART_TypeDef_t *uart, UART_BaudRate_t baud_rate)
{
	uint32_t control;

	if (uart == NULL) {
		return;
	}

	control = uart->CR & (uint32_t)~UART_CR_BRR_MASK;
	control |= (((uint32_t)baud_rate << UART_CR_BRR_POS) & UART_CR_BRR_MASK);
	uart->CR = control;
}

UART_BaudRate_t UART_GetBaudRate(const UART_TypeDef_t *uart)
{
	if (uart == NULL) {
		return UART_BRR_115200;
	}

	return (UART_BaudRate_t)((uart->CR & UART_CR_BRR_MASK) >> UART_CR_BRR_POS);
}

void UART_Enable(UART_TypeDef_t *uart)
{
	if (uart != NULL) uart->CR |= UART_CR_UART_EN_MASK;
}

void UART_Disable(UART_TypeDef_t *uart)
{
	if (uart != NULL) uart->CR &= (uint32_t)~UART_CR_UART_EN_MASK;
}

void UART_EnableTx(UART_TypeDef_t *uart)
{
	if (uart != NULL) uart->CR |= UART_CR_TX_EN_MASK;
}

void UART_DisableTx(UART_TypeDef_t *uart)
{
	if (uart != NULL) uart->CR &= (uint32_t)~UART_CR_TX_EN_MASK;
}

void UART_EnableRx(UART_TypeDef_t *uart)
{
	if (uart != NULL) uart->CR |= UART_CR_RX_EN_MASK;
}

void UART_DisableRx(UART_TypeDef_t *uart)
{
	if (uart != NULL) uart->CR &= (uint32_t)~UART_CR_RX_EN_MASK;
}

bool UART_IsRxEmpty(const UART_TypeDef_t *uart)
{
	return ((uart != NULL) && ((uart->SR & UART_SR_RX_EMPTY_MASK) != 0U));
}

bool UART_IsRxFull(const UART_TypeDef_t *uart)
{
	return ((uart != NULL) && ((uart->SR & UART_SR_RX_FULL_MASK) != 0U));
}

bool UART_IsTxEmpty(const UART_TypeDef_t *uart)
{
	return ((uart != NULL) && ((uart->SR & UART_SR_TX_EMPTY_MASK) != 0U));
}

bool UART_IsTxFull(const UART_TypeDef_t *uart)
{
	return ((uart != NULL) && ((uart->SR & UART_SR_TX_FULL_MASK) != 0U));
}

bool UART_IsTxBusy(const UART_TypeDef_t *uart)
{
	return ((uart != NULL) && ((uart->SR & UART_SR_TX_BUSY_MASK) != 0U));
}

bool UART_IsRxBusy(const UART_TypeDef_t *uart)
{
	return ((uart != NULL) && ((uart->SR & UART_SR_RX_BUSY_MASK) != 0U));
}

bool UART_WriteByte(UART_TypeDef_t *uart, uint8_t data)
{
	if ((uart == NULL) || UART_IsTxFull(uart)) {
		return false;
	}

	uart->DR = (uint32_t)data;
	return true;
}

bool UART_ReadByte(UART_TypeDef_t *uart, uint8_t *data)
{
	if ((uart == NULL) || (data == NULL) || UART_IsRxEmpty(uart)) {
		return false;
	}

	*data = (uint8_t)(uart->DR & UART_DR_DATA_MASK);
	return true;
}

void UART_EnableTxInterrupt(UART_TypeDef_t *uart)
{
	if (uart != NULL) uart->IER |= UART_TX_IRQ_MASK;
}

void UART_DisableTxInterrupt(UART_TypeDef_t *uart)
{
	if (uart != NULL) uart->IER &= (uint32_t)~UART_TX_IRQ_MASK;
}

void UART_EnableRxInterrupt(UART_TypeDef_t *uart)
{
	if (uart != NULL) uart->IER |= UART_RX_IRQ_MASK;
}

void UART_DisableRxInterrupt(UART_TypeDef_t *uart)
{
	if (uart != NULL) uart->IER &= (uint32_t)~UART_RX_IRQ_MASK;
}

bool UART_IsTxInterruptPending(const UART_TypeDef_t *uart)
{
	return ((uart != NULL) && ((uart->ISR & UART_TX_IRQ_MASK) != 0U));
}

bool UART_IsRxInterruptPending(const UART_TypeDef_t *uart)
{
	return ((uart != NULL) && ((uart->ISR & UART_RX_IRQ_MASK) != 0U));
}

void UART_ClearTxInterrupt(UART_TypeDef_t *uart)
{
	if (uart != NULL) uart->ICR = UART_TX_IRQ_MASK;
}

void UART_ClearRxInterrupt(UART_TypeDef_t *uart)
{
	if (uart != NULL) uart->ICR = UART_RX_IRQ_MASK;
}

void UART_Initialize(UART_TypeDef_t *uart, UART_BaudRate_t baud_rate)
{
	if (uart == NULL) {
		return;
	}

	uart->IER = 0U;
	uart->ICR = UART_TX_IRQ_MASK | UART_RX_IRQ_MASK;
	uart->CR = (((uint32_t)baud_rate << UART_CR_BRR_POS) & UART_CR_BRR_MASK);
	UART_EnableRx(uart);
	UART_EnableTx(uart);
	UART_Enable(uart);
}
