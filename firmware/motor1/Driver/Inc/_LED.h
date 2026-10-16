#ifndef LED_H_
#define LED_H_

#include "main.h"
#include <stdbool.h>

extern volatile bool led_flag;

void LED1_On(void);
void LED1_Off(void);
void LED2_On(void);
void LED2_Off(void);

#endif /* LED_H_ */
