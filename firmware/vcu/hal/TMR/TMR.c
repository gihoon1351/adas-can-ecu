#include "TMR.h"

void TMR_SetPSC(TMR_TypeDef_t *tmr,uint32_t psc)
{
	tmr->PSC = psc;
}

uint32_t TMR_GetPSC(TMR_TypeDef_t *tmr)
{
	return tmr->PSC;
}

void TMR_SetARR(TMR_TypeDef_t *tmr,uint32_t arr)
{
	tmr->ARR = arr;
}

uint32_t TMR_GetARR(TMR_TypeDef_t *tmr)
{
	return tmr->ARR;
}

void TMR_SetCNT(TMR_TypeDef_t *tmr,uint32_t cnt)
{
	tmr->CNT = cnt;
}

uint32_t TMR_GetCNT(TMR_TypeDef_t *tmr)
{
	return tmr->CNT;
}


void TMR_StartTimer(TMR_TypeDef_t *tmr)
{
	tmr->CR |= (UINT32_C(1) << TMR_EN_BIT);
}

void TMR_StopTimer(TMR_TypeDef_t *tmr)
{
	tmr->CR &= (uint32_t)~(UINT32_C(1) << TMR_EN_BIT);
}

void TMR_StartInterrupt(TMR_TypeDef_t *tmr)
{
	tmr->CR |= (UINT32_C(1) << TMR_IE_BIT);
}

void TMR_StopInterrupt(TMR_TypeDef_t *tmr)
{
	tmr->CR &= (uint32_t)~(UINT32_C(1) << TMR_IE_BIT);
}
