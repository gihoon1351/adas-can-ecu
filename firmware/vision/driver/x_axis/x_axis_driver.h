#ifndef X_AXIS_DRIVER_H
#define X_AXIS_DRIVER_H

#include <stdbool.h>
#include <stdint.h>
#include "../../hal/x_axis/x_axis.h"

#define X_AXIS_DRIVER_QUEUE_DEPTH (8U)

typedef struct
{
    XAxisDualResult result;
    uint32_t captured_ms;
} XAxisDriverSample;

typedef struct
{
    X_AXIS_TypeDef *hal;
    XAxisDriverSample queue[X_AXIS_DRIVER_QUEUE_DEPTH];
    volatile uint32_t head;
    volatile uint32_t tail;
    volatile uint32_t count;
    volatile uint32_t irq_count;
    volatile uint32_t dropped_count;
    bool initialized;
} XAxisDriver;

bool XAxisDriver_Init(XAxisDriver *driver, X_AXIS_TypeDef *hal);
void XAxisDriver_Start(XAxisDriver *driver);
void XAxisDriver_IrqHandler(void *context);
bool XAxisDriver_Pop(XAxisDriver *driver, XAxisDriverSample *sample);

#endif
