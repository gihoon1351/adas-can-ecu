#ifndef SRC_HAL_UART_UART_H_
#define SRC_HAL_UART_UART_H_

#include <stdbool.h>
#include <stdint.h>
#include "xparameters.h"

/* CR(0x00), SR(0x04), DR(0x08), IER(0x0C), ISR(0x10), ICR(0x14) */
typedef struct
{
	volatile uint32_t CR;
	volatile uint32_t SR;
	volatile uint32_t DR;
	volatile uint32_t IER;
	volatile uint32_t ISR;
	volatile uint32_t ICR;
} UART_TypeDef_t;

/*
 * Vitis 2020.2 may regenerate the application against an older BSP whose
 * instance name differs.  All variants map this UART to the address assigned
 * in the FINAL2 Vivado Address Editor.
 */
#if defined(XPAR_UART_V1_0_0_BASEADDR)
#define UART_BASEADDR XPAR_UART_V1_0_0_BASEADDR
#elif defined(XPAR_AXI_UART_V1_0_0_BASEADDR)
#define UART_BASEADDR XPAR_AXI_UART_V1_0_0_BASEADDR
#else
#define UART_BASEADDR 0x44A50000U
#endif
#define UART ((UART_TypeDef_t *)UART_BASEADDR)

/* UART_CR */
#define UART_CR_RX_EN_MASK (1U << 0U)
#define UART_CR_TX_EN_MASK (1U << 1U)
#define UART_CR_UART_EN_MASK (1U << 2U)
#define UART_CR_BRR_POS 3U
#define UART_CR_BRR_MASK (7U << UART_CR_BRR_POS)

/* UART_SR */
#define UART_SR_RX_EMPTY_MASK (1U << 0U)
#define UART_SR_RX_FULL_MASK (1U << 1U)
#define UART_SR_TX_EMPTY_MASK (1U << 2U)
#define UART_SR_TX_FULL_MASK (1U << 3U)
#define UART_SR_TX_BUSY_MASK (1U << 4U)
#define UART_SR_RX_BUSY_MASK (1U << 5U)

/* UART_IER, UART_ISR, UART_ICR */
#define UART_TX_IRQ_MASK (1U << 0U)
#define UART_RX_IRQ_MASK (1U << 1U)
#define UART_DR_DATA_MASK 0xFFU

/* uart_core.sv baud-rate selection values (80 MHz input clock). */
typedef enum
{
	UART_BRR_9600 = 0U,
	UART_BRR_19200 = 1U,
	UART_BRR_38400 = 2U,
	UART_BRR_57600 = 3U,
	UART_BRR_115200 = 4U,
	UART_BRR_230400 = 5U
} UART_BaudRate_t;

void UART_SetBaudRate(UART_TypeDef_t *uart, UART_BaudRate_t baud_rate);
UART_BaudRate_t UART_GetBaudRate(const UART_TypeDef_t *uart);

void UART_Enable(UART_TypeDef_t *uart);
void UART_Disable(UART_TypeDef_t *uart);
void UART_EnableTx(UART_TypeDef_t *uart);
void UART_DisableTx(UART_TypeDef_t *uart);
void UART_EnableRx(UART_TypeDef_t *uart);
void UART_DisableRx(UART_TypeDef_t *uart);

bool UART_IsRxEmpty(const UART_TypeDef_t *uart);
bool UART_IsRxFull(const UART_TypeDef_t *uart);
bool UART_IsTxEmpty(const UART_TypeDef_t *uart);
bool UART_IsTxFull(const UART_TypeDef_t *uart);
bool UART_IsTxBusy(const UART_TypeDef_t *uart);
bool UART_IsRxBusy(const UART_TypeDef_t *uart);

bool UART_WriteByte(UART_TypeDef_t *uart, uint8_t data);
bool UART_ReadByte(UART_TypeDef_t *uart, uint8_t *data);

void UART_EnableTxInterrupt(UART_TypeDef_t *uart);
void UART_DisableTxInterrupt(UART_TypeDef_t *uart);
void UART_EnableRxInterrupt(UART_TypeDef_t *uart);
void UART_DisableRxInterrupt(UART_TypeDef_t *uart);
bool UART_IsTxInterruptPending(const UART_TypeDef_t *uart);
bool UART_IsRxInterruptPending(const UART_TypeDef_t *uart);
void UART_ClearTxInterrupt(UART_TypeDef_t *uart);
void UART_ClearRxInterrupt(UART_TypeDef_t *uart);

void UART_Initialize(UART_TypeDef_t *uart, UART_BaudRate_t baud_rate);

#endif /* SRC_HAL_UART_UART_H_ */
