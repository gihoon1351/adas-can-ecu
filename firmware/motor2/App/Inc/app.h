#ifndef APP_H_
#define APP_H_

#include "tim.h"

void App_Init(void);
void App_Run(void);
void App_OnTimerElapsed(TIM_HandleTypeDef *htim);

#endif /* APP_H_ */
