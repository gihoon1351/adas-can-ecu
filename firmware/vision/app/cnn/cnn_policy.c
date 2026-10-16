#include "cnn_policy.h"
#include <stddef.h>
#include <string.h>

static float max_f32(float a, float b) { return (a > b) ? a : b; }
static float min_f32(float a, float b) { return (a < b) ? a : b; }

static float box_iou(const CnnDetection *a, const CnnDetection *b)
{
    const float w = max_f32(0.0f, min_f32(a->x1,b->x1)-max_f32(a->x0,b->x0));
    const float h = max_f32(0.0f, min_f32(a->y1,b->y1)-max_f32(a->y0,b->y0));
    const float intersection = w*h;
    const float area_a = max_f32(0.0f,a->x1-a->x0)*max_f32(0.0f,a->y1-a->y0);
    const float area_b = max_f32(0.0f,b->x1-b->x0)*max_f32(0.0f,b->y1-b->y0);
    const float union_area = area_a+area_b-intersection;
    return union_area>0.0f ? intersection/union_area : 0.0f;
}

void CnnPolicy_Reset(CnnPolicy *policy)
{
    if (policy != NULL) (void)memset(policy, 0, sizeof(*policy));
}

CnnPolicyDecision CnnPolicy_Update(CnnPolicy *policy,
                                    const CnnDetection *detections,
                                    uint32_t count)
{
    /* Red, child, bump, 70, green. The model still decodes all five classes. */
    static const uint8_t priority[CNN_PP_NUM_CLASSES] = {3U, 0U, 2U, 1U, 4U};
    /* Red light acts immediately; the other classes require two matching
     * frames. Array order is child, 70, bump, red, green. */
    static const uint8_t frames_needed[CNN_PP_NUM_CLASSES] =
        {2U, 2U, 2U, 1U, 2U};
    CnnPolicyDecision decision = {0};
    const CnnDetection *current[CNN_PP_NUM_CLASSES] = {NULL};
    uint32_t i, c;

    if (policy == NULL || (count != 0U && detections == NULL)) return decision;
    for (i = 0U; i < count; ++i)
    {
        c = detections[i].class_id;
        if (c < CNN_PP_NUM_CLASSES && detections[i].valid != 0U &&
            (current[c] == NULL || detections[i].score > current[c]->score))
            current[c] = &detections[i];
    }
    for (c = 0U; c < CNN_PP_NUM_CLASSES; ++c)
    {
        const CnnDetection *d = current[c];
        if (d == NULL || !(d->x0 < d->x1 && d->y0 < d->y1))
        {
            policy->streak[c] = 0U;
            if (d != NULL) decision.uncertain = true;
        }
        else
        {
            if (policy->streak[c] > 0U &&
                box_iou(&policy->previous[c], d) >= CNN_POLICY_MIN_BOX_IOU)
            {
                if (policy->streak[c] < frames_needed[c]) ++policy->streak[c];
            }
            else policy->streak[c] = 1U;
            policy->previous[c] = *d;
            if (policy->streak[c] < frames_needed[c]) decision.uncertain = true;
        }
    }
    for (i = 0U; i < CNN_PP_NUM_CLASSES; ++i)
    {
        c = priority[i];
        if (current[c] != NULL && policy->streak[c] >= frames_needed[c])
        {
            decision.confirmed = true;
            decision.class_id = (uint8_t)c;
            decision.detection = *current[c];
            break;
        }
    }
    return decision;
}
