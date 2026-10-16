#include "irq_hal.h"
#include <stddef.h>
#include "xil_exception.h"
#include "xpseudo_asm.h"
#include "xstatus.h"

bool IrqHal_Init(IrqHal *hal, uint16_t device_id)
{
    XScuGic_Config *config;
    if (hal == NULL) return false;
    hal->initialized = false;
    config = XScuGic_LookupConfig((u16)device_id);
    if (config == NULL) return false;
    if (XScuGic_CfgInitialize(&hal->instance, config,
                              config->CpuBaseAddress) != XST_SUCCESS)
        return false;
    Xil_ExceptionInit();
    /* BSP-required handler cast: vendor exception/IRQ typedefs differ. */
    Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT,
        (Xil_ExceptionHandler)XScuGic_InterruptHandler, &hal->instance);
    hal->initialized = true;
    return true;
}

bool IrqHal_Connect(IrqHal *hal, uint32_t irq_id,
                    Xil_InterruptHandler handler, void *context,
                    uint8_t priority, uint8_t trigger)
{
    if (hal == NULL || !hal->initialized || handler == NULL ||
        irq_id < 32U || irq_id >= XSCUGIC_MAX_NUM_INTR_INPUTS ||
        (trigger != 1U && trigger != 3U)) return false;
    XScuGic_Disable(&hal->instance, irq_id);
    if (XScuGic_Connect(&hal->instance, irq_id, handler, context) != XST_SUCCESS)
        return false;
    XScuGic_SetPriorityTriggerType(&hal->instance, irq_id, priority, trigger);
    return true;
}

void IrqHal_Enable(IrqHal *hal, uint32_t irq_id)
{
    if (hal != NULL && hal->initialized)
        XScuGic_Enable(&hal->instance, irq_id);
}

void IrqHal_Disable(IrqHal *hal, uint32_t irq_id)
{
    if (hal != NULL && hal->initialized)
        XScuGic_Disable(&hal->instance, irq_id);
}

void IrqHal_EnableCpu(void)
{
    Xil_ExceptionEnableMask(XIL_EXCEPTION_IRQ);
}

uint32_t IrqHal_SaveDisable(void)
{
    const uint32_t saved = (uint32_t)mfcpsr() & XIL_EXCEPTION_IRQ;
    Xil_ExceptionDisableMask(XIL_EXCEPTION_IRQ);
    dmb();
    return saved;
}

void IrqHal_Restore(uint32_t saved_mask)
{
    dmb();
    if ((saved_mask & XIL_EXCEPTION_IRQ) == 0U)
        Xil_ExceptionEnableMask(XIL_EXCEPTION_IRQ);
}
