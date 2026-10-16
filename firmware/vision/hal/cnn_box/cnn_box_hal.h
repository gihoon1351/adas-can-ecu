#ifndef CNN_BOX_HAL_H
#define CNN_BOX_HAL_H

#include <stdbool.h>
#include <stdint.h>
#include "xil_types.h"

/* Byte offsets from cnn_result_box_v1_0_0's AXI-Lite base address. */
#define CNN_BOX_HAL_CONTROL_OFFSET   (0x00U)
#define CNN_BOX_HAL_COORDS_OFFSET    (0x04U)
#define CNN_BOX_HAL_COLOR_OFFSET     (0x08U)
#define CNN_BOX_HAL_THICKNESS_OFFSET (0x0CU)
#define CNN_BOX_HAL_COMMIT_OFFSET    (0x10U)
#define CNN_BOX_HAL_VIDEO_SIZE_OFFSET (0x14U)
#define CNN_BOX_HAL_CNN_SIZE_OFFSET  (0x18U)
#define CNN_BOX_HAL_VERSION_OFFSET   (0x1CU)
#define CNN_BOX_HAL_VERSION          (0x00010000UL)

typedef struct
{
    UINTPTR base_address;
} CnnBoxHal;

/* Inclusive endpoints in the 224x224 CNN coordinate system. */
typedef struct
{
    uint8_t x0;
    uint8_t y0;
    uint8_t x1;
    uint8_t y1;
} CnnBoxCoordinates;

typedef struct
{
    uint32_t version;
    uint16_t video_width;
    uint16_t video_height;
    uint16_t cnn_width;
    uint16_t cnn_height;
} CnnBoxHalInfo;

bool CnnBoxHal_Init(CnnBoxHal *instance, UINTPTR base_address);
bool CnnBoxHal_ReadInfo(const CnnBoxHal *instance, CnnBoxHalInfo *info);
/* Writes all shadows, then COMMIT last. RTL applies settings at a later SOF.
 * Serialize calls in one execution context; this API does not arbitrate ISRs. */
bool CnnBoxHal_Show(const CnnBoxHal *instance,
                    const CnnBoxCoordinates *coordinates,
                    uint32_t color, uint8_t thickness);
bool CnnBoxHal_Hide(const CnnBoxHal *instance);

#endif
