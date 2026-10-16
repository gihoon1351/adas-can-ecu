#include "cnn_box_driver.h"

#include <float.h>
#include <stddef.h>

static bool CnnBoxDriver_IsFinite(float value)
{
    return (value >= -FLT_MAX) && (value <= FLT_MAX);
}

static uint8_t CnnBoxDriver_ToCoordinate(float value)
{
    if (value < 0.0F)
    {
        value = 0.0F;
    }
    else if (value > (float)CNN_BOX_DRIVER_COORD_MAX)
    {
        value = (float)CNN_BOX_DRIVER_COORD_MAX;
    }
    else
    {
        /* Already within the CNN image. */
    }
    /* Input is finite and clamped to [0,223], so narrowing is bounded. */
    return (uint8_t)(value + 0.5F);
}

static bool CnnBoxDriver_GetCoordinates(const CnnBoxCandidate *candidate,
                                        CnnBoxCoordinates *coordinates)
{
    bool valid = false;

    if (candidate->valid &&
        (candidate->class_id < CNN_BOX_DRIVER_CLASS_COUNT) &&
        (candidate->score >= 0.0F) && (candidate->score <= 1.0F) &&
        CnnBoxDriver_IsFinite(candidate->x0) &&
        CnnBoxDriver_IsFinite(candidate->y0) &&
        CnnBoxDriver_IsFinite(candidate->x1) &&
        CnnBoxDriver_IsFinite(candidate->y1) &&
        (candidate->x0 <= candidate->x1) &&
        (candidate->y0 <= candidate->y1) &&
        (candidate->x1 >= 0.0F) && (candidate->y1 >= 0.0F) &&
        (candidate->x0 <= (float)CNN_BOX_DRIVER_COORD_MAX) &&
        (candidate->y0 <= (float)CNN_BOX_DRIVER_COORD_MAX))
    {
        coordinates->x0 = CnnBoxDriver_ToCoordinate(candidate->x0);
        coordinates->y0 = CnnBoxDriver_ToCoordinate(candidate->y0);
        coordinates->x1 = CnnBoxDriver_ToCoordinate(candidate->x1);
        coordinates->y1 = CnnBoxDriver_ToCoordinate(candidate->y1);
        valid = true;
    }
    return valid;
}

CnnBoxDriverStatus CnnBoxDriver_Init(CnnBoxDriver *driver,
                                     UINTPTR base_address,
                                     uint32_t color, uint8_t thickness)
{
    CnnBoxDriverStatus status = CNN_BOX_DRIVER_INVALID_ARGUMENT;
    CnnBoxHalInfo info = {0};

    if (driver != NULL)
    {
        driver->initialized = false;
        if (CnnBoxHal_Init(&driver->hal, base_address))
        {
            status = CNN_BOX_DRIVER_HAL_FAILED;
            if (CnnBoxHal_ReadInfo(&driver->hal, &info))
            {
                status = CNN_BOX_DRIVER_IP_MISMATCH;
                if ((info.version == CNN_BOX_HAL_VERSION) &&
                    (info.video_width == 1280U) &&
                    (info.video_height == 720U) &&
                    (info.cnn_width == 224U) && (info.cnn_height == 224U))
                {
                    driver->color = color & 0x00FFFFFFUL;
                    driver->thickness = (thickness == 0U) ? 1U : thickness;
                    status = CNN_BOX_DRIVER_HAL_FAILED;
                    if (CnnBoxHal_Hide(&driver->hal))
                    {
                        driver->initialized = true;
                        status = CNN_BOX_DRIVER_OK;
                    }
                }
            }
        }
    }
    return status;
}

CnnBoxDriverStatus CnnBoxDriver_Hide(CnnBoxDriver *driver)
{
    CnnBoxDriverStatus status = CNN_BOX_DRIVER_INVALID_ARGUMENT;

    if (driver != NULL)
    {
        status = CNN_BOX_DRIVER_NOT_INITIALIZED;
        if (driver->initialized)
        {
            status = CnnBoxHal_Hide(&driver->hal)
                   ? CNN_BOX_DRIVER_OK : CNN_BOX_DRIVER_HAL_FAILED;
        }
    }
    return status;
}

CnnBoxDriverStatus CnnBoxDriver_ShowHighest(
    CnnBoxDriver *driver, const CnnBoxCandidate *candidates,
    uint32_t candidate_count, CnnBoxSelection *selection)
{
    CnnBoxDriverStatus status = CNN_BOX_DRIVER_INVALID_ARGUMENT;
    CnnBoxSelection chosen = {0};
    CnnBoxCoordinates coordinates;
    uint32_t index;

    if (selection != NULL)
    {
        *selection = chosen;
    }
    if ((driver != NULL) && (selection != NULL) &&
        ((candidates != NULL) || (candidate_count == 0U)))
    {
        status = CNN_BOX_DRIVER_NOT_INITIALIZED;
        if (driver->initialized)
        {
            for (index = 0U; index < candidate_count; ++index)
            {
                if (CnnBoxDriver_GetCoordinates(&candidates[index],
                                                &coordinates) &&
                    ((!chosen.visible) ||
                     (candidates[index].score > chosen.score)))
                {
                    chosen.visible = true;
                    chosen.candidate_index = index;
                    chosen.class_id = candidates[index].class_id;
                    chosen.score = candidates[index].score;
                    chosen.coordinates = coordinates;
                }
            }
            status = CNN_BOX_DRIVER_HAL_FAILED;
            if (chosen.visible)
            {
                if (CnnBoxHal_Show(&driver->hal, &chosen.coordinates,
                                   driver->color, driver->thickness))
                {
                    *selection = chosen;
                    status = CNN_BOX_DRIVER_OK;
                }
            }
            else if (CnnBoxHal_Hide(&driver->hal))
            {
                status = CNN_BOX_DRIVER_NO_DETECTION;
            }
        }
    }
    return status;
}
