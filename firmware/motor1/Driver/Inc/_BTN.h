#ifndef BTN_H_
#define BTN_H_

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

typedef enum {
    RELEASE,
    isPRESS,
    PRESS,
    isRELEASE
} btnState_t;

typedef struct {
    GPIO_TypeDef *GPIOx;
    uint16_t GPIO_Pin;
    btnState_t state;
    volatile bool flag;
} Button_t;

void Button_DebounceTask(Button_t *btn);

extern Button_t buttons[2];

#endif /* BTN_H_ */
