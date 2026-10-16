#include "safety_domain.h"
#include <stddef.h>

void SAFETY_DOMAIN_PrepareForBoot(SAFETY_DOMAIN_TypeDef_t *const safety)
{
	if (safety == NULL) {
		return;
	}

	/* The safety-domain peripheral is outside the processor reset domain.
	 * Stop a retained watchdog first, restore a non-immediate timeout, then
	 * pulse fault-clear while monitor and watchdog remain disabled. */
	safety->SAFETY_CR = 0U;
	safety->WDT_TIMEOUT = SAFETY_DOMAIN_DEFAULT_WDT_TIMEOUT;
	safety->SAFETY_CR = SAFETY_DOMAIN_FAULT_CLEAR;
	safety->SAFETY_CR = 0U;
}

void SAFETY_DOMAIN_Initialization(SAFETY_DOMAIN_TypeDef_t *const safety, const SAFETY_DOMAIN_Config_t *const config)
{
	if ((safety == NULL) || (config == NULL)) {
		return;
	}

	SAFETY_DOMAIN_SetWdtTimeout(safety, config->wdt_timeout);
	SAFETY_DOMAIN_SetRefWindow(safety, config->ref_window);
	SAFETY_DOMAIN_SetMinCount(safety, config->min_count);
	SAFETY_DOMAIN_SetMaxCount(safety, config->max_count);
	SAFETY_DOMAIN_SetTimestampDivider(safety, config->ts_divider);
	SAFETY_DOMAIN_SetResetDelay(safety, config->reset_delay);
}

void SAFETY_DOMAIN_WdtKick(SAFETY_DOMAIN_TypeDef_t *const safety)
{
	if (safety == NULL) {
		return;
	}
	safety->SAFETY_CR = SAFETY_DOMAIN_WDT_KICK | SAFETY_DOMAIN_MON_EN | SAFETY_DOMAIN_WDT_EN;
}

void SAFETY_DOMAIN_FaultClear(SAFETY_DOMAIN_TypeDef_t *const safety)
{
	if (safety == NULL) {
		return;
	}
	safety->SAFETY_CR = SAFETY_DOMAIN_FAULT_CLEAR | SAFETY_DOMAIN_MON_EN | SAFETY_DOMAIN_WDT_EN;
}

void SAFETY_DOMAIN_EnableWdt(SAFETY_DOMAIN_TypeDef_t *const safety)
{
	if (safety == NULL) {
		return;
	}
	safety->SAFETY_CR |= SAFETY_DOMAIN_WDT_EN;
}

void SAFETY_DOMAIN_DisableWdt(SAFETY_DOMAIN_TypeDef_t *const safety)
{
	if (safety == NULL) {
		return;
	}
	safety->SAFETY_CR &= ~SAFETY_DOMAIN_WDT_EN;
}

void SAFETY_DOMAIN_EnableMonitor(SAFETY_DOMAIN_TypeDef_t *const safety)
{
	if (safety == NULL) {
		return;
	}
	safety->SAFETY_CR |= SAFETY_DOMAIN_MON_EN;
}

void SAFETY_DOMAIN_DisableMonitor(SAFETY_DOMAIN_TypeDef_t *const safety)
{
	if (safety == NULL) {
		return;
	}
	safety->SAFETY_CR &= ~SAFETY_DOMAIN_MON_EN;
}

uint32_t SAFETY_DOMAIN_GetStatus(const SAFETY_DOMAIN_TypeDef_t *const safety)
{
	if (safety == NULL) {
		return 0U;
	}
	return safety->SAFETY_SR;
}

bool SAFETY_DOMAIN_IsFaultIrq(const SAFETY_DOMAIN_TypeDef_t *const safety)
{
	uint32_t status;

	if (safety == NULL) {
		return false;
	}
	status = safety->SAFETY_SR;
	return ((status & SAFETY_DOMAIN_FAULT_IRQ) != 0U);
}

bool SAFETY_DOMAIN_IsResetReq(const SAFETY_DOMAIN_TypeDef_t *const safety)
{
	uint32_t status;

	if (safety == NULL) {
		return false;
	}
	status = safety->SAFETY_SR;
	return ((status & SAFETY_DOMAIN_RESET_REQ) != 0U);
}

bool SAFETY_DOMAIN_IsSafeState(const SAFETY_DOMAIN_TypeDef_t *const safety)
{
	uint32_t status;

	if (safety == NULL) {
		return false;
	}
	status = safety->SAFETY_SR;
	return ((status & SAFETY_DOMAIN_SAFE_STATE) != 0U);
}

void SAFETY_DOMAIN_SetWdtTimeout(SAFETY_DOMAIN_TypeDef_t *const safety, uint32_t timeout)
{
	if (safety == NULL) {
		return;
	}
	safety->WDT_TIMEOUT = timeout;
}

uint32_t SAFETY_DOMAIN_GetWdtTimeout(const SAFETY_DOMAIN_TypeDef_t *const safety)
{
	if (safety == NULL) {
		return 0U;
	}
	return safety->WDT_TIMEOUT;
}

void SAFETY_DOMAIN_SetRefWindow(SAFETY_DOMAIN_TypeDef_t *const safety, uint16_t ref_window)
{
	if (safety == NULL) {
		return;
	}
	safety->REF_WINDOW = ((uint32_t)ref_window & SAFETY_DOMAIN_REF_WINDOW_MASK);
}

uint16_t SAFETY_DOMAIN_GetRefWindow(const SAFETY_DOMAIN_TypeDef_t *const safety)
{
	if (safety == NULL) {
		return 0U;
	}
	return (uint16_t)(safety->REF_WINDOW & SAFETY_DOMAIN_REF_WINDOW_MASK);
}

void SAFETY_DOMAIN_SetMinCount(SAFETY_DOMAIN_TypeDef_t *const safety, uint32_t min_count)
{
	if (safety == NULL) {
		return;
	}
	safety->MIN_COUNT = min_count;
}

uint32_t SAFETY_DOMAIN_GetMinCount(const SAFETY_DOMAIN_TypeDef_t *const safety)
{
	if (safety == NULL) {
		return 0U;
	}
	return safety->MIN_COUNT;
}

void SAFETY_DOMAIN_SetMaxCount(SAFETY_DOMAIN_TypeDef_t *const safety, uint32_t max_count)
{
	if (safety == NULL) {
		return;
	}
	safety->MAX_COUNT = max_count;
}

uint32_t SAFETY_DOMAIN_GetMaxCount(const SAFETY_DOMAIN_TypeDef_t *const safety)
{
	if (safety == NULL) {
		return 0U;
	}
	return safety->MAX_COUNT;
}

void SAFETY_DOMAIN_SetTimestampDivider(SAFETY_DOMAIN_TypeDef_t *const safety, uint16_t divider)
{
	if (safety == NULL) {
		return;
	}
	safety->TS_DIVIDER = ((uint32_t)divider & SAFETY_DOMAIN_TS_DIVIDER_MASK);
}

uint16_t SAFETY_DOMAIN_GetTimestampDivider(const SAFETY_DOMAIN_TypeDef_t *const safety)
{
	if (safety == NULL) {
		return 0U;
	}
	return (uint16_t)(safety->TS_DIVIDER & SAFETY_DOMAIN_TS_DIVIDER_MASK);
}

void SAFETY_DOMAIN_SetResetDelay(SAFETY_DOMAIN_TypeDef_t *const safety, uint8_t delay)
{
	if (safety == NULL) {
		return;
	}
	safety->RESET_DELAY = ((uint32_t)delay & SAFETY_DOMAIN_RESET_DELAY_MASK);
}

uint8_t SAFETY_DOMAIN_GetResetDelay(const SAFETY_DOMAIN_TypeDef_t *const safety)
{
	if (safety == NULL) {
		return 0U;
	}
	return (uint8_t)(safety->RESET_DELAY & SAFETY_DOMAIN_RESET_DELAY_MASK);
}

void SAFETY_DOMAIN_SetFaultLogAddress(SAFETY_DOMAIN_TypeDef_t *const safety, uint8_t address)
{
	if (safety == NULL) {
		return;
	}
	safety->LOG_RD_ADDR = ((uint32_t)address & SAFETY_DOMAIN_LOG_RD_ADDR_MASK);
}

uint64_t SAFETY_DOMAIN_ReadFaultLog(SAFETY_DOMAIN_TypeDef_t *const safety, uint8_t address)
{
	uint32_t data_low;
	uint32_t data_high;
	uint64_t data;

	if (safety == NULL) {
		return 0U;
	}
	SAFETY_DOMAIN_SetFaultLogAddress(safety, address);

	data_low = safety->LOG_DATA_L;

	data_high = safety->LOG_DATA_H & SAFETY_DOMAIN_LOG_DATA_H_MASK;

	data = ((uint64_t)data_high << 32U) | (uint64_t)data_low;

	return data;
}
