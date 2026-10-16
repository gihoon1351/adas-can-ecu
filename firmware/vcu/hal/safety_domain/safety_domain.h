#ifndef SRC_HAL_HAL_SAFETY_DOMAIN_H_
#define SRC_HAL_HAL_SAFETY_DOMAIN_H_

#include <stdbool.h>
#include <stdint.h>
#include "xparameters.h"

/* =========================================================
 * Register Map
 * SAFETY_CR (0x00): bit 0 - WDT kick (write 1)
 *                   bit 1 - Fault clear (write 1)
 *                   bit 2 - Monitor enable
 *                   bit 3 - Watchdog enable
 * SAFETY_SR (0x04): bit 0 - Fault interrupt
 *                   bit 1 - Reset request
 *                   bit 2 - Safe state
 * WDT_TIMEOUT (0x08): Watchdog timeout
 * REF_WINDOW (0x0C): bits 0-15 - Reference window
 * MIN_COUNT (0x10): Minimum count
 * MAX_COUNT (0x14): Maximum count
 * TS_DIVIDER (0x18): bits 0-15 - Timestamp divider
 * RESET_DELAY (0x1C): bits 0-7 - Reset delay
 * LOG_RD_ADDR (0x20): bits 0-7 - Fault log read address
 * LOG_DATA_L (0x24): Fault log data bits 0-31
 * LOG_DATA_H (0x28): Fault log data bits 32-55 (register bits 0-23)
 * ========================================================= */

typedef struct
{
	volatile uint32_t SAFETY_CR;
	volatile uint32_t SAFETY_SR;
	volatile uint32_t WDT_TIMEOUT;
	volatile uint32_t REF_WINDOW;
	volatile uint32_t MIN_COUNT;
	volatile uint32_t MAX_COUNT;
	volatile uint32_t TS_DIVIDER;
	volatile uint32_t RESET_DELAY;
	volatile uint32_t LOG_RD_ADDR;
	volatile uint32_t LOG_DATA_L;
	volatile uint32_t LOG_DATA_H;
} SAFETY_DOMAIN_TypeDef_t;

/* =========================================================
 * Base Address
 * ========================================================= */

#define SAFETY_DOMAIN_BASEADDR XPAR_SAFETY_DOMAIN_BASEADDR
#define SAFETY_DOMAIN0 ((SAFETY_DOMAIN_TypeDef_t *)SAFETY_DOMAIN_BASEADDR)

/* =========================================================
 * SAFETY_CR
 * ========================================================= */

#define SAFETY_DOMAIN_WDT_KICK (1U << 0)
#define SAFETY_DOMAIN_FAULT_CLEAR (1U << 1)
#define SAFETY_DOMAIN_MON_EN (1U << 2)
#define SAFETY_DOMAIN_WDT_EN (1U << 3)

/* =========================================================
 * SAFETY_SR
 * ========================================================= */

#define SAFETY_DOMAIN_FAULT_IRQ (1U << 0)
#define SAFETY_DOMAIN_RESET_REQ (1U << 1)
#define SAFETY_DOMAIN_SAFE_STATE (1U << 2)

/* =========================================================
 * Register Masks
 * ========================================================= */

#define SAFETY_DOMAIN_REF_WINDOW_POS 0U
#define SAFETY_DOMAIN_REF_WINDOW_MASK (0xFFFFU << SAFETY_DOMAIN_REF_WINDOW_POS)
#define SAFETY_DOMAIN_TS_DIVIDER_POS 0U
#define SAFETY_DOMAIN_TS_DIVIDER_MASK (0xFFFFU << SAFETY_DOMAIN_TS_DIVIDER_POS)
#define SAFETY_DOMAIN_RESET_DELAY_POS 0U
#define SAFETY_DOMAIN_RESET_DELAY_MASK (0xFFU << SAFETY_DOMAIN_RESET_DELAY_POS)
#define SAFETY_DOMAIN_LOG_RD_ADDR_POS 0U
#define SAFETY_DOMAIN_LOG_RD_ADDR_MASK (0xFFU << SAFETY_DOMAIN_LOG_RD_ADDR_POS)
#define SAFETY_DOMAIN_LOG_DATA_H_POS 0U
#define SAFETY_DOMAIN_LOG_DATA_H_MASK (0xFFFFFFU << SAFETY_DOMAIN_LOG_DATA_H_POS)

/* =========================================================
 * Default Configuration
 * ========================================================= */

#define SAFETY_DOMAIN_DEFAULT_WDT_TIMEOUT 10000000U /* 10 MHz x 1 s */
#define SAFETY_DOMAIN_DEFAULT_REF_WINDOW 10000U     /* 10 MHz x 1 ms */
#define SAFETY_DOMAIN_DEFAULT_MIN_COUNT 76000U      /* 80000 x (1 - 5%) */
#define SAFETY_DOMAIN_DEFAULT_MAX_COUNT 84000U      /* 80000 x (1 + 5%) */
#define SAFETY_DOMAIN_DEFAULT_TS_DIVIDER 10000U     /* 10 MHz / 10,000 = nominal 1 kHz */
#define SAFETY_DOMAIN_DEFAULT_RESET_DELAY 8U        /* Default count; main.c overrides with 10 */

/* =========================================================
 * Configuration Structure
 * ========================================================= */

typedef struct
{
	uint32_t wdt_timeout;
	uint16_t ref_window;
	uint32_t min_count;
	uint32_t max_count;
	uint16_t ts_divider;
	uint8_t reset_delay;
} SAFETY_DOMAIN_Config_t;

/* =========================================================
 * Initialization
 * SAFETY_DOMAIN_Initialization: Apply configuration to the registers.
 * ========================================================= */

void SAFETY_DOMAIN_Initialization(SAFETY_DOMAIN_TypeDef_t *const safety, const SAFETY_DOMAIN_Config_t *const config);

/* Quiesce state retained by the FPGA across a processor-only JTAG reset. */
void SAFETY_DOMAIN_PrepareForBoot(SAFETY_DOMAIN_TypeDef_t *const safety);

/* =========================================================
 * Control
 * SAFETY_DOMAIN_WdtKick: Issue a watchdog kick command.
 * SAFETY_DOMAIN_EnableWdt / DisableWdt: Enable or disable watchdog counting.
 * SAFETY_DOMAIN_EnableMonitor / DisableMonitor: Enable or disable the clock monitor.
 * SAFETY_DOMAIN_FaultClear: Issue a fault-clear command.
 * ========================================================= */

void SAFETY_DOMAIN_WdtKick(SAFETY_DOMAIN_TypeDef_t *const safety);

void SAFETY_DOMAIN_EnableWdt(SAFETY_DOMAIN_TypeDef_t *const safety);
void SAFETY_DOMAIN_DisableWdt(SAFETY_DOMAIN_TypeDef_t *const safety);

void SAFETY_DOMAIN_EnableMonitor(SAFETY_DOMAIN_TypeDef_t *const safety);
void SAFETY_DOMAIN_DisableMonitor(SAFETY_DOMAIN_TypeDef_t *const safety);

void SAFETY_DOMAIN_FaultClear(SAFETY_DOMAIN_TypeDef_t *const safety);

/* =========================================================
 * Status
 * SAFETY_DOMAIN_GetStatus: Return the raw status register value.
 * SAFETY_DOMAIN_IsFaultIrq: Check the fault interrupt status bit.
 * SAFETY_DOMAIN_IsResetReq: Check the reset-request status bit.
 * SAFETY_DOMAIN_IsSafeState: Check the safe-state status bit.
 * ========================================================= */

uint32_t SAFETY_DOMAIN_GetStatus(const SAFETY_DOMAIN_TypeDef_t *const safety);

bool SAFETY_DOMAIN_IsFaultIrq(const SAFETY_DOMAIN_TypeDef_t *const safety);
bool SAFETY_DOMAIN_IsResetReq(const SAFETY_DOMAIN_TypeDef_t *const safety);
bool SAFETY_DOMAIN_IsSafeState(const SAFETY_DOMAIN_TypeDef_t *const safety);

/* =========================================================
 * Watchdog
 * SAFETY_DOMAIN_SetWdtTimeout / GetWdtTimeout: Configure or read the timeout count.
 * ========================================================= */

void SAFETY_DOMAIN_SetWdtTimeout(SAFETY_DOMAIN_TypeDef_t *const safety, uint32_t timeout);
uint32_t SAFETY_DOMAIN_GetWdtTimeout(const SAFETY_DOMAIN_TypeDef_t *const safety);

/* =========================================================
 * Clock Monitor
 * SAFETY_DOMAIN_Set/GetRefWindow: Configure or read the reference window.
 * SAFETY_DOMAIN_Set/GetMinCount: Configure or read the minimum accepted count.
 * SAFETY_DOMAIN_Set/GetMaxCount: Configure or read the maximum accepted count.
 * ========================================================= */

void SAFETY_DOMAIN_SetRefWindow(SAFETY_DOMAIN_TypeDef_t *const safety, uint16_t ref_window);
uint16_t SAFETY_DOMAIN_GetRefWindow(const SAFETY_DOMAIN_TypeDef_t *const safety);

void SAFETY_DOMAIN_SetMinCount(SAFETY_DOMAIN_TypeDef_t *const safety, uint32_t min_count);
uint32_t SAFETY_DOMAIN_GetMinCount(const SAFETY_DOMAIN_TypeDef_t *const safety);

void SAFETY_DOMAIN_SetMaxCount(SAFETY_DOMAIN_TypeDef_t *const safety, uint32_t max_count);
uint32_t SAFETY_DOMAIN_GetMaxCount(const SAFETY_DOMAIN_TypeDef_t *const safety);

/* =========================================================
 * Timestamp
 * SAFETY_DOMAIN_SetTimestampDivider / GetTimestampDivider: Configure or read the divider.
 * ========================================================= */

void SAFETY_DOMAIN_SetTimestampDivider(SAFETY_DOMAIN_TypeDef_t *const safety, uint16_t divider);
uint16_t SAFETY_DOMAIN_GetTimestampDivider(const SAFETY_DOMAIN_TypeDef_t *const safety);

/* =========================================================
 * Reset
 * SAFETY_DOMAIN_SetResetDelay / GetResetDelay: Configure or read the reset delay count.
 * ========================================================= */

void SAFETY_DOMAIN_SetResetDelay(SAFETY_DOMAIN_TypeDef_t *const safety, uint8_t delay);
uint8_t SAFETY_DOMAIN_GetResetDelay(const SAFETY_DOMAIN_TypeDef_t *const safety);

/* =========================================================
 * Fault Log
 * SAFETY_DOMAIN_SetFaultLogAddress: Select the fault-log entry to read.
 * SAFETY_DOMAIN_ReadFaultLog: Return the selected 56-bit fault-log entry.
 * ========================================================= */

void SAFETY_DOMAIN_SetFaultLogAddress(SAFETY_DOMAIN_TypeDef_t *const safety, uint8_t address);
uint64_t SAFETY_DOMAIN_ReadFaultLog(SAFETY_DOMAIN_TypeDef_t *const safety, uint8_t address);

#endif /* SRC_HAL_HAL_SAFETY_DOMAIN_H_ */
