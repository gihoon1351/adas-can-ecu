/*
 * TMR.h
 *
 *  Created on: 2026. 6. 26.
 *      Author: kccistc
 */

#ifndef SRC_HAL_TMR_TMR_H_
#define SRC_HAL_TMR_TMR_H_

#include "xparameters.h"
#include <stdint.h>

typedef struct
{
    uint32_t CR;
    uint32_t PSC;
    uint32_t ARR;
    uint32_t CNT;
} TMR_TypeDef_t;


#define TMR1_BASEADDR    XPAR_TIME1_BASEADDR
#define TMR2_BASEADDR    XPAR_TIME2_BASEADDR

#define TMR1 \
    ((TMR_TypeDef_t *)TMR1_BASEADDR)

#define TMR2 \
    ((TMR_TypeDef_t *)TMR2_BASEADDR)

#define TMR_EN_BIT		0
#define TMR_IE_BIT		1

void TMR_SetPSC(TMR_TypeDef_t *tmr,uint32_t psc);
uint32_t TMR_GetPSC(TMR_TypeDef_t *tmr);
void TMR_SetARR(TMR_TypeDef_t *tmr,uint32_t arr);
uint32_t TMR_GetARR(TMR_TypeDef_t *tmr);
void TMR_SetCNT(TMR_TypeDef_t *tmr,uint32_t cnt);
uint32_t TMR_GetCNT(TMR_TypeDef_t *tmr);
void TMR_StartTimer(TMR_TypeDef_t *tmr);
void TMR_StopTimer(TMR_TypeDef_t *tmr);
void TMR_StartInterrupt(TMR_TypeDef_t *tmr);
void TMR_StopInterrupt(TMR_TypeDef_t *tmr);

#endif /* SRC_HAL_TMR_TMR_H_ */
