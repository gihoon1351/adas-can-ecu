#ifndef SRC_HAL_GPIO_GPIO_H_
#define SRC_HAL_GPIO_GPIO_H_

#include "xparameters.h"
#include <stdint.h>

/* =========================================================
 * Register Map
 * CR (0x00): bits 0-7 - Direction of GPIO pins 0-7
 *            (0: input/high impedance, 1: output)
 * IDR (0x04): bits 0-7 - Input state of GPIO pins 0-7
 * ODR (0x08): bits 0-7 - Output state of GPIO pins 0-7
 * ========================================================= */
typedef struct
{
	uint32_t CR;
	uint32_t IDR;
	uint32_t ODR;
} GPIO_TypeDef;

#define GPIO_BASEADDR XPAR_GPIO_BASEADDR
#define GPIO ((GPIO_TypeDef *)GPIO_BASEADDR)

#define GPIO_CR_MODE_MASK 0xFFU

#define GPIO_INPUT 0x00U
#define GPIO_OUTPUT GPIO_CR_MODE_MASK
#define GPIO_PIN_0 0x01
#define GPIO_PIN_1 0x02
#define GPIO_PIN_2 0x04
#define GPIO_PIN_3 0x08
#define GPIO_PIN_4 0x10
#define GPIO_PIN_5 0x20
#define GPIO_PIN_6 0x40
#define GPIO_PIN_7 0x80

#define GPIO_SET 1U
#define GPIO_RESET 0U

void GPIO_SetMode(GPIO_TypeDef *GPIOx, uint32_t mode);
void GPIO_WritePort(GPIO_TypeDef *GPIOx, uint32_t data);
void GPIO_WritePin(GPIO_TypeDef *GPIOx, uint32_t gpio_pin, uint32_t gpio_pin_state);
uint32_t GPIO_ReadPort(GPIO_TypeDef *GPIOx);
uint32_t GPIO_ReadPin(GPIO_TypeDef *GPIOx, uint32_t gpio_pin);
uint32_t GPIO_GetODR(GPIO_TypeDef *GPIOx);
uint32_t GPIO_GetCR(GPIO_TypeDef *GPIOx);

#endif /* SRC_HAL_GPIO_GPIO_H_ */
