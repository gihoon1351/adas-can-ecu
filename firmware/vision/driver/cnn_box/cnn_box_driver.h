#ifndef CNN_BOX_DRIVER_H
#define CNN_BOX_DRIVER_H

#include <stdbool.h>
#include <stdint.h>
#include "../../hal/cnn_box/cnn_box_hal.h"

#define CNN_BOX_DRIVER_COORD_MAX (223U)
#define CNN_BOX_DRIVER_CLASS_COUNT (5U)

typedef enum
{
    CNN_BOX_DRIVER_OK = 0,
    CNN_BOX_DRIVER_NO_DETECTION,
    CNN_BOX_DRIVER_INVALID_ARGUMENT,
    CNN_BOX_DRIVER_NOT_INITIALIZED,
    CNN_BOX_DRIVER_IP_MISMATCH,
    CNN_BOX_DRIVER_HAL_FAILED
} CnnBoxDriverStatus;

/* Model-independent adapter shared by fixed PCam and float ADAS decoders.
 * The caller sets valid only for candidates passing its existing thresholds. */
typedef struct
{
    bool valid;
    uint8_t class_id;
    float score; /* objectness * class probability, in [0,1] */
    float x0;
    float y0;
    float x1;
    float y1;
} CnnBoxCandidate;

typedef struct
{
    bool visible;
    uint32_t candidate_index;
    uint8_t class_id;
    float score;
    CnnBoxCoordinates coordinates;
} CnnBoxSelection;

typedef struct
{
    CnnBoxHal hal;
    uint32_t color;
    uint8_t thickness;
    bool initialized;
} CnnBoxDriver;

CnnBoxDriverStatus CnnBoxDriver_Init(CnnBoxDriver *driver,
                                     UINTPTR base_address,
                                     uint32_t color, uint8_t thickness);
/* Chooses one highest-score valid current-frame candidate. Ties retain the
 * first candidate. Empty/invalid-only input hides the previous box. */
CnnBoxDriverStatus CnnBoxDriver_ShowHighest(
    CnnBoxDriver *driver, const CnnBoxCandidate *candidates,
    uint32_t candidate_count, CnnBoxSelection *selection);
CnnBoxDriverStatus CnnBoxDriver_Hide(CnnBoxDriver *driver);

#endif
