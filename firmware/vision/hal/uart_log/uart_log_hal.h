#ifndef UART_LOG_HAL_H
#define UART_LOG_HAL_H

#include <stdbool.h>
#include <stdint.h>

/* Main-context only. Startup output is synchronous; runtime output is queued.
 * No runtime wait for UART FIFO space. Full queue drops LOGS, not lane data. */
void UartLogHal_SetAsync(bool enabled);
void UartLogHal_Printf(const char *format, ...);
void UartLogHal_Poll(void);
uint32_t UartLogHal_GetDroppedCount(void);

#endif
