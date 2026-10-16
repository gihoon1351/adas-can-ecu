#include "cnn_result_app.h"

#include <stdbool.h>
#include <stddef.h>

static bool CnnResultApp_GetYCoordinates(const CnnDetection *candidate,
                                         uint8_t *y0,
                                         uint8_t *y1)
{
    bool valid = false;
    int32_t lower;
    int32_t upper;

    if ((candidate != NULL) && (y0 != NULL) && (y1 != NULL))
    {
        lower = (int32_t)(candidate->y0 + 0.5F);
        upper = (int32_t)(candidate->y1 + 0.5F);

        if (lower < 0)
        {
            lower = 0;
        }
        if (upper > (int32_t)ADAS_BOX_Y_MAX)
        {
            upper = (int32_t)ADAS_BOX_Y_MAX;
        }

        if (lower < upper)
        {
            *y0 = (uint8_t)lower;
            *y1 = (uint8_t)upper;
            valid = true;
        }
    }

    return valid;
}

CnnResultAppStatus CnnResultApp_UpdateCan(
    AdasCanApp *can_app,
    const CnnDetection *candidates,
    uint32_t candidate_count,
    uint32_t now_ms,
    CnnResultSelection *selection)
{
    uint32_t votes[CNN_PP_NUM_CLASSES] = {0U};
    uint32_t best_index[CNN_PP_NUM_CLASSES];
    uint32_t valid_count = 0U;
    uint32_t winner = CNN_PP_NUM_CLASSES;
    uint32_t index;
    uint8_t candidate_y0;
    uint8_t candidate_y1;
    const CnnDetection *candidate;
    const CnnDetection *selected;
    CnnResultAppStatus status = CNN_RESULT_APP_INVALID_ARGUMENT;

    if ((can_app != NULL) &&
        ((candidates != NULL) || (candidate_count == 0U)) &&
        (selection != NULL))
    {
        selection->model_class = 0U;
        selection->y0 = 0U;
        selection->y1 = 0U;
        selection->votes = 0U;
        selection->valid_candidate_count = 0U;
        selection->score = 0.0F;

        for (index = 0U; index < CNN_PP_NUM_CLASSES; ++index)
        {
            best_index[index] = candidate_count;
        }

        for (index = 0U; index < candidate_count; ++index)
        {
            candidate = &candidates[index];
            if ((candidate->class_id < CNN_PP_NUM_CLASSES) &&
                CnnResultApp_GetYCoordinates(candidate,
                                             &candidate_y0,
                                             &candidate_y1))
            {
                ++votes[candidate->class_id];
                ++valid_count;

                if ((best_index[candidate->class_id] == candidate_count) ||
                    (candidate->score >
                     candidates[best_index[candidate->class_id]].score))
                {
                    best_index[candidate->class_id] = index;
                }
            }
        }

        for (index = 0U; index < CNN_PP_NUM_CLASSES; ++index)
        {
            if (votes[index] > 0U)
            {
                if ((winner == CNN_PP_NUM_CLASSES) ||
                    (votes[index] > votes[winner]) ||
                    ((votes[index] == votes[winner]) &&
                     (candidates[best_index[index]].score >
                      candidates[best_index[winner]].score)))
                {
                    winner = index;
                }
            }
        }

        if (winner == CNN_PP_NUM_CLASSES)
        {
            AdasCanApp_ClearDetection(can_app);
            status = CNN_RESULT_APP_NO_DETECTION;
        }
        else
        {
            selected = &candidates[best_index[winner]];
            if (!CnnResultApp_GetYCoordinates(selected,
                                              &candidate_y0,
                                              &candidate_y1))
            {
                AdasCanApp_ClearDetection(can_app);
                status = CNN_RESULT_APP_NO_DETECTION;
            }
            else if (!AdasCanApp_UpdateModelDetection(
                         can_app,
                         (uint8_t)winner,
                         candidate_y0,
                         candidate_y1,
                         now_ms))
            {
                status = CNN_RESULT_APP_CAN_UPDATE_FAILED;
            }
            else
            {
                selection->model_class = (uint8_t)winner;
                selection->y0 = candidate_y0;
                selection->y1 = candidate_y1;
                selection->votes = votes[winner];
                selection->valid_candidate_count = valid_count;
                selection->score = selected->score;
                status = CNN_RESULT_APP_UPDATED;
            }
        }
    }

    return status;
}
