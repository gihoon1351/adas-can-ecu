#include "../Inc/_BTN.h"

Button_t buttons[2] = {
    {GPIOB, GPIO_PIN_2, RELEASE, false},
    {GPIOB, GPIO_PIN_12, RELEASE, false}
};

void Button_DebounceTask(Button_t *btn)
{
    bool btnValue;

    btnValue = HAL_GPIO_ReadPin(btn->GPIOx, btn->GPIO_Pin);

    switch (btn->state)
    {
        case RELEASE:
            if (btnValue == GPIO_PIN_RESET) {
                btn->state = isPRESS;
            }
            break;

        case isPRESS:
            if (btnValue == GPIO_PIN_RESET) {
                btn->state = PRESS;
                btn->flag = true;
            } else {
                btn->state = RELEASE;
            }
            break;

        case PRESS:
            if (btnValue == GPIO_PIN_SET) {
                btn->state = isRELEASE;
            }
            break;

        case isRELEASE:
            if (btnValue == GPIO_PIN_SET) {
                btn->state = RELEASE;
            } else {
                btn->state = PRESS;
            }
            break;

        default:
            btn->state = RELEASE;
            break;
    }
}
