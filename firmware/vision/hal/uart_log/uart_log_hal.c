#include "uart_log_hal.h"
#include <stdarg.h>
#include <stdio.h>
#include <stddef.h>
#include "xil_printf.h"
#include "xparameters.h"
#include "xuartps_hw.h"

#if defined(STDOUT_BASEADDRESS)
#define UART_LOG_BASE ((UINTPTR)STDOUT_BASEADDRESS)
#elif defined(XPAR_XUARTPS_0_BASEADDR)
#define UART_LOG_BASE ((UINTPTR)XPAR_XUARTPS_0_BASEADDR)
#elif defined(XPAR_PS7_UART_1_BASEADDR)
#define UART_LOG_BASE ((UINTPTR)XPAR_PS7_UART_1_BASEADDR)
#else
#error "UART stdout base address missing"
#endif

#define UART_LOG_CAPACITY (4096U)
#define UART_LOG_LINE_MAX (512U)
#define UART_LOG_POLL_MAX (64U)
static char s_queue[UART_LOG_CAPACITY];
static uint32_t s_head;
static uint32_t s_tail;
static uint32_t s_count;
static uint32_t s_dropped;
static bool s_async;

void UartLogHal_SetAsync(bool enabled)
{
    s_async = enabled;
}

void UartLogHal_Printf(const char *format, ...)
{
    char line[UART_LOG_LINE_MAX];
    va_list args;
    int length;
    uint32_t index;
    if (format == NULL) return;
    va_start(args, format);
    length = vsnprintf(line, sizeof(line), format, args);
    va_end(args);
    /* C library varargs/formatting is an explicit project deviation. */
    if (length < 0 || (uint32_t)length >= UART_LOG_LINE_MAX)
    {
        ++s_dropped;
        return;
    }
    if (!s_async)
    {
        xil_printf("%s", line);
        return;
    }
    if ((uint32_t)length > (UART_LOG_CAPACITY - s_count))
    {
        ++s_dropped;
        return;
    }
    for (index = 0U; index < (uint32_t)length; ++index)
    {
        s_queue[s_head] = line[index];
        s_head = (s_head + 1U) % UART_LOG_CAPACITY;
    }
    s_count += (uint32_t)length;
}

void UartLogHal_Poll(void)
{
    uint32_t sent = 0U;
    while (s_count > 0U && sent < UART_LOG_POLL_MAX &&
           !XUartPs_IsTransmitFull(UART_LOG_BASE))
    {
        XUartPs_WriteReg(UART_LOG_BASE, XUARTPS_FIFO_OFFSET,
                         (uint32_t)(uint8_t)s_queue[s_tail]);
        s_tail = (s_tail + 1U) % UART_LOG_CAPACITY;
        --s_count;
        ++sent;
    }
}

uint32_t UartLogHal_GetDroppedCount(void)
{
    return s_dropped;
}
