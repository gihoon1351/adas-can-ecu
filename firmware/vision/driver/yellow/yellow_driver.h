#ifndef YELLOW_DRIVER_H
#define YELLOW_DRIVER_H

#include <stdbool.h>
#include <stdint.h>
#include "../../hal/yellow/yellow_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    YELLOW_DRIVER_STATUS_OK = 0,
    YELLOW_DRIVER_STATUS_INVALID_ARGUMENT,
    YELLOW_DRIVER_STATUS_NOT_INITIALIZED,
    YELLOW_DRIVER_STATUS_HAL_ERROR,
    YELLOW_DRIVER_STATUS_VERIFY_FAILED
} YellowDriverStatus;

typedef struct
{
    /* Wider than 8 bits so an input such as 256 is rejected, not wrapped. */
    uint16_t low;
    uint16_t high;
} YellowRange;

typedef struct
{
    YellowRange y;
    YellowRange cb;
    YellowRange cr;
} YellowThresholds;

typedef struct
{
    UINTPTR base_address;
    YellowThresholds thresholds;
    bool enabled;
} YellowDriverConfig;

typedef struct
{
    YellowHal hal;
    bool initialized;
} YellowDriver;

/* Example tuning values, not measured/calibrated camera thresholds. */
YellowThresholds YellowDriver_DefaultThresholds(void);

/* Call with the stream stopped/before video starts. Zero-initialize the handle.
 * Writes disable -> Y/Cb/Cr -> readback -> requested enable -> readback.
 * Success verifies programmed registers only; changes become active at SOF.
 * A failed Init leaves initialized=false; it does not trap AXI bus faults.
 */
YellowDriverStatus YellowDriver_Init(YellowDriver *driver,
                                    const YellowDriverConfig *config);

/* Enable/bypass takes effect on the next accepted input SOF. */
YellowDriverStatus YellowDriver_SetEnabled(YellowDriver *driver, bool enabled);

/* All three ranges are validated before any write. Does not change CTRL.
 * For frame-consistent changes, stop the stream at a frame boundary before
 * this call, then resume with SOF. The current RTL has no atomic commit.
 * On readback failure, some registers may already contain new values.
 */
YellowDriverStatus YellowDriver_SetThresholds(YellowDriver *driver,
                                             const YellowThresholds *thresholds);

/* One range = one AXI write; low/high cannot be split across SOF.
 * Writes only the selected channel. One software owner is required. */
YellowDriverStatus YellowDriver_SetRange(YellowDriver *driver,
                                        YellowHalChannel channel,
                                        YellowRange range);

/* Reads programmed AXI registers, NOT the core's latched frame settings.
 * Readback is not a multi-register atomic snapshot. Use one software owner.
 */
YellowDriverStatus YellowDriver_ReadConfig(const YellowDriver *driver,
                                          YellowDriverConfig *config);

#ifdef __cplusplus
}
#endif
#endif
