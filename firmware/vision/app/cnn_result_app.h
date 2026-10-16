#ifndef CNN_RESULT_APP_H
#define CNN_RESULT_APP_H

#include <stdint.h>

#include "adas_can_app.h"
#include "cnn_postprocess_fixed.h"

typedef enum
{
    CNN_RESULT_APP_NO_DETECTION = 0,
    CNN_RESULT_APP_UPDATED,
    CNN_RESULT_APP_INVALID_ARGUMENT,
    CNN_RESULT_APP_CAN_UPDATE_FAILED
} CnnResultAppStatus;

typedef struct
{
    uint8_t model_class;
    uint8_t y0;
    uint8_t y1;
    uint32_t votes;
    uint32_t valid_candidate_count;
    float score;
} CnnResultSelection;

CnnResultAppStatus CnnResultApp_UpdateCan(
    AdasCanApp *can_app,
    const CnnDetection *candidates,
    uint32_t candidate_count,
    uint32_t now_ms,
    CnnResultSelection *selection);

#endif
