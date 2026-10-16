#include "delay.h"

static volatile uint32_t m_tick = 0U;

uint32_t millis(void)
{
	return m_tick;
}

void incTick(void)
{
	m_tick++;
}

void delay_sec(u32 seconds)
{
	sleep(seconds);
}

void delay_ms(uint32_t msec)
{
	uint32_t usec = msec * 1000U;

	usleep(usec);
}

void delay_us(uint32_t usec)
{
	usleep(usec);
}
