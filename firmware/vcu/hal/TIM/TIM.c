#include "tim.h"

void TIM_SetPSC(TIM_TypeDef_t *tim, uint32_t psc)
{
	tim->PSC = psc;
}

uint32_t TIM_GetPSC(TIM_TypeDef_t *tim)
{
	return tim->PSC;
}

void TIM_SetARR(TIM_TypeDef_t *tim, uint32_t arr)
{
	tim->ARR = arr;
}

uint32_t TIM_GetARR(TIM_TypeDef_t *tim)
{
	return tim->ARR;
}

void TIM_SetCNT(TIM_TypeDef_t *tim, uint32_t cnt)
{
	tim->CNT = cnt;
}

uint32_t TIM_GetCNT(TIM_TypeDef_t *tim)
{
	return tim->CNT;
}

void TIM_StartTimer(TIM_TypeDef_t *tim)
{
	tim->CR |= TIM_CR_EN_MASK;
}

void TIM_StopTimer(TIM_TypeDef_t *tim)
{
	tim->CR &= (uint32_t)~TIM_CR_EN_MASK;
}

void TIM_StartInterrupt(TIM_TypeDef_t *tim)
{
	tim->CR |= TIM_CR_IE_MASK;
}

void TIM_StopInterrupt(TIM_TypeDef_t *tim)
{
	tim->CR &= (uint32_t)~TIM_CR_IE_MASK;
}

void TIM_Initialize(TIM_TypeDef_t *tim, uint32_t psc, uint32_t arr)
{
	if (tim == 0) {
		return;
	}

	TIM_StopTimer(tim);
	TIM_StopInterrupt(tim);
	TIM_SetPSC(tim, psc);
	TIM_SetARR(tim, arr);
	TIM_SetCNT(tim, 0U);
	TIM_StartInterrupt(tim);
}
