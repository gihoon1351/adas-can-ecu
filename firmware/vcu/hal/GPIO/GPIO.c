#include "gpio.h"

void GPIO_SetMode(GPIO_TypeDef *GPIOx, uint32_t mode)
{
	GPIOx->CR = mode & GPIO_CR_MODE_MASK;
}

uint32_t GPIO_GetODR(GPIO_TypeDef *GPIOx)
{
	return GPIOx->ODR;
}

uint32_t GPIO_GetCR(GPIO_TypeDef *GPIOx)
{
	return GPIOx->CR;
}

void GPIO_WritePort(GPIO_TypeDef *GPIOx, uint32_t data)
{
	GPIOx->ODR = data;
}

void GPIO_WritePin(GPIO_TypeDef *GPIOx, uint32_t gpio_pin, uint32_t gpio_pin_state)
{
	if (gpio_pin_state == GPIO_SET) {
		GPIOx->ODR |= gpio_pin;
	} else {
		GPIOx->ODR &= ~gpio_pin;
	}
}

uint32_t GPIO_ReadPort(GPIO_TypeDef *GPIOx)
{
	return GPIOx->IDR;
}

uint32_t GPIO_ReadPin(GPIO_TypeDef *GPIOx, uint32_t gpio_pin)
{
	return (GPIOx->IDR & gpio_pin) ? 1 : 0;
}
