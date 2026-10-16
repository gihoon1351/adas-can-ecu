#ifndef SRC_HAL_TIM_TIM_H_
#define SRC_HAL_TIM_TIM_H_

#include "xparameters.h"
#include <stdint.h>

/* =========================================================
 * Register Map
 * CR (0x00): bit 0 - Timer enable, bit 1 - Timer interrupt enable
 * PSC (0x04): Prescaler
 * ARR (0x08): Auto-reload value
 * CNT (0x0C): Counter value
 * ========================================================= */
typedef struct
{
	uint32_t CR;
	uint32_t PSC;
	uint32_t ARR;
	uint32_t CNT;
} TIM_TypeDef_t;

#define TIM1_BASEADDR XPAR_TIME1_BASEADDR
#define TIM2_BASEADDR XPAR_TIME2_BASEADDR

#define TIM1 ((TIM_TypeDef_t *)TIM1_BASEADDR)

#define TIM2 ((TIM_TypeDef_t *)TIM2_BASEADDR)

/* CR bit map */
#define TIM_EN_BIT 0U
#define TIM_IE_BIT 1U
#define TIM_CR_EN_MASK (1U << TIM_EN_BIT)
#define TIM_CR_IE_MASK (1U << TIM_IE_BIT)

void TIM_SetPSC(TIM_TypeDef_t *tim, uint32_t psc);
uint32_t TIM_GetPSC(TIM_TypeDef_t *tim);

void TIM_SetARR(TIM_TypeDef_t *tim, uint32_t arr);
uint32_t TIM_GetARR(TIM_TypeDef_t *tim);

void TIM_SetCNT(TIM_TypeDef_t *tim, uint32_t cnt);
uint32_t TIM_GetCNT(TIM_TypeDef_t *tim);

void TIM_StartTimer(TIM_TypeDef_t *tim);
void TIM_StopTimer(TIM_TypeDef_t *tim);

void TIM_StartInterrupt(TIM_TypeDef_t *tim);
void TIM_StopInterrupt(TIM_TypeDef_t *tim);

/* Configure a timer in the stopped state and enable its interrupt. */
void TIM_Initialize(TIM_TypeDef_t *tim, uint32_t psc, uint32_t arr);

#endif /* SRC_HAL_TIM_TIM_H_ */
