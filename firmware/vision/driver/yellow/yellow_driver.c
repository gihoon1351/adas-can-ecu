#include "yellow_driver.h"

#include <stddef.h>

static bool YellowDriver_RangeIsValid(YellowRange range)
{
    return (range.low <= range.high) && (range.high <= 255U);
}

static bool YellowDriver_ThresholdsAreValid(const YellowThresholds *thresholds)
{
    return (thresholds != NULL) && YellowDriver_RangeIsValid(thresholds->y) &&
           YellowDriver_RangeIsValid(thresholds->cb) &&
           YellowDriver_RangeIsValid(thresholds->cr);
}

static uint16_t YellowDriver_Pack(YellowRange range)
{
    return (uint16_t)(((uint32_t)range.high << 8U) | (uint32_t)range.low);
}

static YellowRange YellowDriver_Unpack(uint32_t value)
{
    YellowRange range;
    range.low = (uint16_t)(value & 0xFFU);
    range.high = (uint16_t)((value >> 8U) & 0xFFU);
    return range;
}

static bool YellowDriver_ThresholdsMatch(const YellowHalRegisters *registers,
                                       const YellowThresholds *thresholds)
{
    return ((registers->y & YELLOW_HAL_THRESHOLD_MASK) == YellowDriver_Pack(thresholds->y)) &&
           ((registers->cb & YELLOW_HAL_THRESHOLD_MASK) == YellowDriver_Pack(thresholds->cb)) &&
           ((registers->cr & YELLOW_HAL_THRESHOLD_MASK) == YellowDriver_Pack(thresholds->cr));
}

static YellowDriverStatus YellowDriver_WriteAndVerifyThresholds(
    const YellowHal *hal, const YellowThresholds *thresholds)
{
    YellowHalRegisters registers;
    if ((YellowHal_WriteThreshold(hal, YELLOW_HAL_CHANNEL_Y,
                                 YellowDriver_Pack(thresholds->y)) != YELLOW_HAL_STATUS_OK) ||
        (YellowHal_WriteThreshold(hal, YELLOW_HAL_CHANNEL_CB,
                                 YellowDriver_Pack(thresholds->cb)) != YELLOW_HAL_STATUS_OK) ||
        (YellowHal_WriteThreshold(hal, YELLOW_HAL_CHANNEL_CR,
                                 YellowDriver_Pack(thresholds->cr)) != YELLOW_HAL_STATUS_OK) ||
        (YellowHal_ReadRegisters(hal, &registers) != YELLOW_HAL_STATUS_OK))
    {
        return YELLOW_DRIVER_STATUS_HAL_ERROR;
    }
    return YellowDriver_ThresholdsMatch(&registers, thresholds) ?
           YELLOW_DRIVER_STATUS_OK : YELLOW_DRIVER_STATUS_VERIFY_FAILED;
}

static YellowDriverStatus YellowDriver_WriteAndVerifyEnable(const YellowHal *hal,
                                                          bool enabled)
{
    YellowHalRegisters registers;
    if ((YellowHal_SetEnabled(hal, enabled) != YELLOW_HAL_STATUS_OK) ||
        (YellowHal_ReadRegisters(hal, &registers) != YELLOW_HAL_STATUS_OK))
    {
        return YELLOW_DRIVER_STATUS_HAL_ERROR;
    }
    return (((registers.control & YELLOW_HAL_ENABLE_MASK) != 0U) == enabled) ?
           YELLOW_DRIVER_STATUS_OK : YELLOW_DRIVER_STATUS_VERIFY_FAILED;
}

YellowThresholds YellowDriver_DefaultThresholds(void)
{
    const YellowThresholds thresholds = {{120U, 255U}, {0U, 140U}, {105U, 200U}};
    return thresholds;
}

YellowDriverStatus YellowDriver_Init(YellowDriver *driver,
                                    const YellowDriverConfig *config)
{
    YellowDriverStatus status;
    if (driver == NULL)
    {
        return YELLOW_DRIVER_STATUS_INVALID_ARGUMENT;
    }
    driver->initialized = false;
    driver->hal.base_address = (UINTPTR)0U;
    if ((config == NULL) || !YellowDriver_ThresholdsAreValid(&config->thresholds))
    {
        return YELLOW_DRIVER_STATUS_INVALID_ARGUMENT;
    }
    if (YellowHal_Init(&driver->hal, config->base_address) != YELLOW_HAL_STATUS_OK)
    {
        return YELLOW_DRIVER_STATUS_INVALID_ARGUMENT;
    }
    status = YellowDriver_WriteAndVerifyEnable(&driver->hal, false);
    if (status != YELLOW_DRIVER_STATUS_OK)
    {
        return status;
    }
    status = YellowDriver_WriteAndVerifyThresholds(&driver->hal, &config->thresholds);
    if (status != YELLOW_DRIVER_STATUS_OK)
    {
        return status;
    }
    if (config->enabled)
    {
        status = YellowDriver_WriteAndVerifyEnable(&driver->hal, true);
        if (status != YELLOW_DRIVER_STATUS_OK)
        {
            /* Best-effort request for bypass; the core still updates on SOF. */
            (void)YellowHal_SetEnabled(&driver->hal, false);
            return status;
        }
    }
    driver->initialized = true;
    return YELLOW_DRIVER_STATUS_OK;
}

YellowDriverStatus YellowDriver_SetEnabled(YellowDriver *driver, bool enabled)
{
    if (driver == NULL)
    {
        return YELLOW_DRIVER_STATUS_INVALID_ARGUMENT;
    }
    if (!driver->initialized)
    {
        return YELLOW_DRIVER_STATUS_NOT_INITIALIZED;
    }
    return YellowDriver_WriteAndVerifyEnable(&driver->hal, enabled);
}

YellowDriverStatus YellowDriver_SetThresholds(YellowDriver *driver,
                                             const YellowThresholds *thresholds)
{
    if ((driver == NULL) || !YellowDriver_ThresholdsAreValid(thresholds))
    {
        return YELLOW_DRIVER_STATUS_INVALID_ARGUMENT;
    }
    if (!driver->initialized)
    {
        return YELLOW_DRIVER_STATUS_NOT_INITIALIZED;
    }
    return YellowDriver_WriteAndVerifyThresholds(&driver->hal, thresholds);
}

YellowDriverStatus YellowDriver_ReadConfig(const YellowDriver *driver,
                                          YellowDriverConfig *config)
{
    YellowHalRegisters registers;
    if ((driver == NULL) || (config == NULL))
    {
        return YELLOW_DRIVER_STATUS_INVALID_ARGUMENT;
    }
    if (!driver->initialized)
    {
        return YELLOW_DRIVER_STATUS_NOT_INITIALIZED;
    }
    if (YellowHal_ReadRegisters(&driver->hal, &registers) != YELLOW_HAL_STATUS_OK)
    {
        return YELLOW_DRIVER_STATUS_HAL_ERROR;
    }
    config->base_address = driver->hal.base_address;
    config->enabled = (registers.control & YELLOW_HAL_ENABLE_MASK) != 0U;
    config->thresholds.y = YellowDriver_Unpack(registers.y);
    config->thresholds.cb = YellowDriver_Unpack(registers.cb);
    config->thresholds.cr = YellowDriver_Unpack(registers.cr);
    return YELLOW_DRIVER_STATUS_OK;
}

YellowDriverStatus YellowDriver_SetRange(YellowDriver *driver,
                                        YellowHalChannel channel,
                                        YellowRange range)
{
    YellowHalRegisters registers;
    uint32_t value;
    if ((driver == NULL) || !YellowDriver_RangeIsValid(range) ||
        ((channel != YELLOW_HAL_CHANNEL_Y) &&
         (channel != YELLOW_HAL_CHANNEL_CB) &&
         (channel != YELLOW_HAL_CHANNEL_CR)))
        return YELLOW_DRIVER_STATUS_INVALID_ARGUMENT;
    if (!driver->initialized)
        return YELLOW_DRIVER_STATUS_NOT_INITIALIZED;
    if ((YellowHal_WriteThreshold(&driver->hal, channel,
                                 YellowDriver_Pack(range)) != YELLOW_HAL_STATUS_OK) ||
        (YellowHal_ReadRegisters(&driver->hal, &registers) != YELLOW_HAL_STATUS_OK))
        return YELLOW_DRIVER_STATUS_HAL_ERROR;
    value = (channel == YELLOW_HAL_CHANNEL_Y) ? registers.y :
            ((channel == YELLOW_HAL_CHANNEL_CB) ? registers.cb : registers.cr);
    return ((value & YELLOW_HAL_THRESHOLD_MASK) == YellowDriver_Pack(range)) ?
           YELLOW_DRIVER_STATUS_OK : YELLOW_DRIVER_STATUS_VERIFY_FAILED;
}
