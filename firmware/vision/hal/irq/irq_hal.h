#ifndef IRQ_HAL_H
#define IRQ_HAL_H

#include <stdbool.h>
#include <stdint.h>
#include "xscugic.h"

/* Owned by this application. Do not initialize a second GIC instance. */
typedef struct
{
    XScuGic instance;
    bool initialized;
} IrqHal;

bool IrqHal_Init(IrqHal *hal, uint16_t device_id);
bool IrqHal_Connect(IrqHal *hal, uint32_t irq_id,
                    Xil_InterruptHandler handler, void *context,
                    uint8_t priority, uint8_t trigger);
void IrqHal_Enable(IrqHal *hal, uint32_t irq_id);
void IrqHal_Disable(IrqHal *hal, uint32_t irq_id);
void IrqHal_EnableCpu(void);
/* Nestable, single-core critical sections; restore the original IRQ mask. */
uint32_t IrqHal_SaveDisable(void);
void IrqHal_Restore(uint32_t saved_mask);

#endif
