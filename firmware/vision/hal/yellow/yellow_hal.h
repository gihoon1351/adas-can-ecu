#ifndef YELLOW_HAL_H
#define YELLOW_HAL_H

#include <stdbool.h>
#include <stdint.h>
#include "xil_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define YELLOW_HAL_CTRL_OFFSET       (0x00U)
#define YELLOW_HAL_Y_OFFSET          (0x04U)
#define YELLOW_HAL_CB_OFFSET         (0x08U)
#define YELLOW_HAL_CR_OFFSET         (0x0CU)
#define YELLOW_HAL_ENABLE_MASK       (0x00000001U)
#define YELLOW_HAL_THRESHOLD_MASK    (0x0000FFFFU)

typedef enum
{
    YELLOW_HAL_STATUS_OK = 0,
    YELLOW_HAL_STATUS_INVALID_ARGUMENT,
    YELLOW_HAL_STATUS_NOT_INITIALIZED
} YellowHalStatus;

typedef enum
{
    YELLOW_HAL_CHANNEL_Y = 0,
    YELLOW_HAL_CHANNEL_CB,
    YELLOW_HAL_CHANNEL_CR
} YellowHalChannel;

typedef struct
{
    UINTPTR base_address;
} YellowHal;

/* These are software-visible configuration registers, NOT active-frame state. */
typedef struct
{
    uint32_t control;
    uint32_t y;
    uint32_t cb;
    uint32_t cr;
} YellowHalRegisters;

/* Init binds an address only; it does not access or probe the hardware. */
YellowHalStatus YellowHal_Init(YellowHal *hal, UINTPTR base_address);
YellowHalStatus YellowHal_SetEnabled(const YellowHal *hal, bool enabled);
/* packed_threshold: [15:8] upper bound, [7:0] lower bound, inclusive. */
YellowHalStatus YellowHal_WriteThreshold(const YellowHal *hal,
                                       YellowHalChannel channel,
                                       uint16_t packed_threshold);
YellowHalStatus YellowHal_ReadRegisters(const YellowHal *hal,
                                      YellowHalRegisters *registers);

#ifdef __cplusplus
}
#endif
#endif
