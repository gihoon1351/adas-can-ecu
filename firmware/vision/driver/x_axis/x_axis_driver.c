#include "x_axis_driver.h"
#include <stddef.h>
#include "../../hal/irq/irq_hal.h"
#include "xtime_l.h"

bool XAxisDriver_Init(XAxisDriver *driver, X_AXIS_TypeDef *hal)
{
    if (driver == NULL || hal == NULL) return false;
    driver->hal = hal;
    driver->head = 0U;
    driver->tail = 0U;
    driver->count = 0U;
    driver->irq_count = 0U;
    driver->dropped_count = 0U;
    driver->initialized = true;
    x_axis_irq_disable(hal);
    return true;
}

void XAxisDriver_Start(XAxisDriver *driver)
{
    if (driver != NULL && driver->initialized)
    {
        x_axis_done_clear(driver->hal);
        x_axis_irq_enable(driver->hal);
        x_axis_enable(driver->hal);
    }
}

void XAxisDriver_IrqHandler(void *context)
{
    XAxisDriver *driver = (XAxisDriver *)context;
    XAxisDriverSample sample;
    XTime ticks;
    if (driver == NULL || !driver->initialized) return;
    /* Read DR, then clear sticky DONE. No averaging/UART/CAN/DMA in ISR. */
    if (!x_axis_read_dual_result(driver->hal, &sample.result)) return;
    XTime_GetTime(&ticks);
    sample.captured_ms = (uint32_t)(((uint64_t)ticks * 1000ULL) /
                                     (uint64_t)COUNTS_PER_SECOND);
    ++driver->irq_count;
    if (driver->count == X_AXIS_DRIVER_QUEUE_DEPTH)
    {
        /* Main pops under a short IRQ mask; keep the most recent samples. */
        driver->tail = (driver->tail + 1U) % X_AXIS_DRIVER_QUEUE_DEPTH;
        --driver->count;
        ++driver->dropped_count;
    }
    driver->queue[driver->head] = sample;
    driver->head = (driver->head + 1U) % X_AXIS_DRIVER_QUEUE_DEPTH;
    ++driver->count;
}

bool XAxisDriver_Pop(XAxisDriver *driver, XAxisDriverSample *sample)
{
    uint32_t saved;
    bool available = false;
    if (driver == NULL || !driver->initialized || sample == NULL) return false;
    saved = IrqHal_SaveDisable();
    if (driver->count > 0U)
    {
        *sample = driver->queue[driver->tail];
        driver->tail = (driver->tail + 1U) % X_AXIS_DRIVER_QUEUE_DEPTH;
        --driver->count;
        available = true;
    }
    IrqHal_Restore(saved);
    return available;
}
