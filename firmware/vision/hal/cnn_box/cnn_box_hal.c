#include "cnn_box_hal.h"

#include <stddef.h>
#include "xil_io.h"

static bool CnnBoxHal_IsValid(const CnnBoxHal *instance)
{
    return (instance != NULL) &&
           (instance->base_address != (UINTPTR)0U);
}

bool CnnBoxHal_Init(CnnBoxHal *instance, UINTPTR base_address)
{
    bool success = false;

    if (instance != NULL)
    {
        instance->base_address = (UINTPTR)0U;
        if ((base_address != (UINTPTR)0U) &&
            ((base_address & (UINTPTR)0x03U) == (UINTPTR)0U))
        {
            instance->base_address = base_address;
            success = true;
        }
    }
    return success;
}

bool CnnBoxHal_ReadInfo(const CnnBoxHal *instance, CnnBoxHalInfo *info)
{
    bool success = false;
    uint32_t video_size;
    uint32_t cnn_size;

    if (CnnBoxHal_IsValid(instance) && (info != NULL))
    {
        info->version = (uint32_t)Xil_In32(instance->base_address +
                            (UINTPTR)CNN_BOX_HAL_VERSION_OFFSET);
        video_size = (uint32_t)Xil_In32(instance->base_address +
                            (UINTPTR)CNN_BOX_HAL_VIDEO_SIZE_OFFSET);
        cnn_size = (uint32_t)Xil_In32(instance->base_address +
                            (UINTPTR)CNN_BOX_HAL_CNN_SIZE_OFFSET);
        info->video_width = (uint16_t)(video_size & 0xFFFFUL);
        info->video_height = (uint16_t)(video_size >> 16U);
        info->cnn_width = (uint16_t)(cnn_size & 0xFFFFUL);
        info->cnn_height = (uint16_t)(cnn_size >> 16U);
        success = true;
    }
    return success;
}

bool CnnBoxHal_Show(const CnnBoxHal *instance,
                    const CnnBoxCoordinates *coordinates,
                    uint32_t color, uint8_t thickness)
{
    bool success = false;
    uint32_t packed;

    if (CnnBoxHal_IsValid(instance) && (coordinates != NULL) &&
        (coordinates->x0 <= coordinates->x1) &&
        (coordinates->y0 <= coordinates->y1))
    {
        /* COORDS: [7:0] x0, [15:8] x1, [23:16] y0, [31:24] y1. */
        packed = (uint32_t)coordinates->x0 |
                 ((uint32_t)coordinates->x1 << 8U) |
                 ((uint32_t)coordinates->y0 << 16U) |
                 ((uint32_t)coordinates->y1 << 24U);
        Xil_Out32(instance->base_address + (UINTPTR)CNN_BOX_HAL_COORDS_OFFSET,
                  (u32)packed);
        Xil_Out32(instance->base_address + (UINTPTR)CNN_BOX_HAL_COLOR_OFFSET,
                  (u32)(color & 0x00FFFFFFUL));
        Xil_Out32(instance->base_address + (UINTPTR)CNN_BOX_HAL_THICKNESS_OFFSET,
                  (u32)((thickness == 0U) ? 1U : thickness));
        /* enable=1, fill=0: outline only. */
        Xil_Out32(instance->base_address + (UINTPTR)CNN_BOX_HAL_CONTROL_OFFSET,
                  (u32)1U);
        Xil_Out32(instance->base_address + (UINTPTR)CNN_BOX_HAL_COMMIT_OFFSET,
                  (u32)1U);
        success = true;
    }
    return success;
}

bool CnnBoxHal_Hide(const CnnBoxHal *instance)
{
    bool success = false;

    if (CnnBoxHal_IsValid(instance))
    {
        Xil_Out32(instance->base_address + (UINTPTR)CNN_BOX_HAL_CONTROL_OFFSET,
                  (u32)0U);
        Xil_Out32(instance->base_address + (UINTPTR)CNN_BOX_HAL_COMMIT_OFFSET,
                  (u32)1U);
        success = true;
    }
    return success;
}
