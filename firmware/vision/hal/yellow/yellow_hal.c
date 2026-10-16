#include "yellow_hal.h"

#include <stddef.h>
#include "xil_io.h"

static bool YellowHal_IsValid(const YellowHal *hal)
{
    return (hal != NULL) && (hal->base_address != (UINTPTR)0U) &&
           ((hal->base_address & (UINTPTR)3U) == (UINTPTR)0U) &&
           (hal->base_address <= (UINTPTR)(~(UINTPTR)0U) -
                                  (UINTPTR)YELLOW_HAL_CR_OFFSET);
}

YellowHalStatus YellowHal_Init(YellowHal *hal, UINTPTR base_address)
{
    if (hal == NULL)
    {
        return YELLOW_HAL_STATUS_INVALID_ARGUMENT;
    }
    hal->base_address = (UINTPTR)0U;
    if ((base_address == (UINTPTR)0U) ||
        ((base_address & (UINTPTR)3U) != (UINTPTR)0U) ||
        (base_address > (UINTPTR)(~(UINTPTR)0U) -
                         (UINTPTR)YELLOW_HAL_CR_OFFSET))
    {
        return YELLOW_HAL_STATUS_INVALID_ARGUMENT;
    }
    hal->base_address = base_address;
    return YELLOW_HAL_STATUS_OK;
}

YellowHalStatus YellowHal_SetEnabled(const YellowHal *hal, bool enabled)
{
    uint32_t control;
    if (hal == NULL)
    {
        return YELLOW_HAL_STATUS_INVALID_ARGUMENT;
    }
    if (!YellowHal_IsValid(hal))
    {
        return YELLOW_HAL_STATUS_NOT_INITIALIZED;
    }
    control = (uint32_t)Xil_In32(hal->base_address + YELLOW_HAL_CTRL_OFFSET);
    control &= ~YELLOW_HAL_ENABLE_MASK;
    if (enabled)
    {
        control |= YELLOW_HAL_ENABLE_MASK;
    }
    Xil_Out32(hal->base_address + YELLOW_HAL_CTRL_OFFSET, (u32)control);
    return YELLOW_HAL_STATUS_OK;
}

YellowHalStatus YellowHal_WriteThreshold(const YellowHal *hal,
                                       YellowHalChannel channel,
                                       uint16_t packed_threshold)
{
    uint32_t offset;
    uint32_t value;
    if (hal == NULL)
    {
        return YELLOW_HAL_STATUS_INVALID_ARGUMENT;
    }
    switch (channel)
    {
        case YELLOW_HAL_CHANNEL_Y:  offset = YELLOW_HAL_Y_OFFSET;  break;
        case YELLOW_HAL_CHANNEL_CB: offset = YELLOW_HAL_CB_OFFSET; break;
        case YELLOW_HAL_CHANNEL_CR: offset = YELLOW_HAL_CR_OFFSET; break;
        default: return YELLOW_HAL_STATUS_INVALID_ARGUMENT;
    }
    if (!YellowHal_IsValid(hal))
    {
        return YELLOW_HAL_STATUS_NOT_INITIALIZED;
    }
    value = (uint32_t)Xil_In32(hal->base_address + (UINTPTR)offset);
    value = (value & ~YELLOW_HAL_THRESHOLD_MASK) | (uint32_t)packed_threshold;
    Xil_Out32(hal->base_address + (UINTPTR)offset, (u32)value);
    return YELLOW_HAL_STATUS_OK;
}

YellowHalStatus YellowHal_ReadRegisters(const YellowHal *hal,
                                      YellowHalRegisters *registers)
{
    if ((hal == NULL) || (registers == NULL))
    {
        return YELLOW_HAL_STATUS_INVALID_ARGUMENT;
    }
    if (!YellowHal_IsValid(hal))
    {
        return YELLOW_HAL_STATUS_NOT_INITIALIZED;
    }
    registers->control = (uint32_t)Xil_In32(hal->base_address + YELLOW_HAL_CTRL_OFFSET);
    registers->y = (uint32_t)Xil_In32(hal->base_address + YELLOW_HAL_Y_OFFSET);
    registers->cb = (uint32_t)Xil_In32(hal->base_address + YELLOW_HAL_CB_OFFSET);
    registers->cr = (uint32_t)Xil_In32(hal->base_address + YELLOW_HAL_CR_OFFSET);
    return YELLOW_HAL_STATUS_OK;
}
