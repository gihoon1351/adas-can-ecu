
#include "../Inc/_LED.h"

void LED1_On(void)
{
	HAL_GPIO_WritePin(GPIOA, LED1_Pin, GPIO_PIN_SET);
}

void LED1_Off(void)
{
	HAL_GPIO_WritePin(GPIOA, LED1_Pin, GPIO_PIN_RESET);
}

void LED2_On(void)
{
	HAL_GPIO_WritePin(GPIOA, LED2_Pin, GPIO_PIN_SET);
}

void LED2_Off(void)
{
	HAL_GPIO_WritePin(GPIOA, LED2_Pin, GPIO_PIN_RESET);
}

