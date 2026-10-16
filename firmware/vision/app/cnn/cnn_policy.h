#ifndef CNN_POLICY_H
#define CNN_POLICY_H

#include <stdbool.h>
#include <stdint.h>
#include "../cnn_postprocess_fixed.h"

/* Score gates are applied by cnn_postprocess_fixed before this temporal
 * confirmation policy receives the detections. */
#define CNN_POLICY_MIN_BOX_IOU       0.30f

typedef struct {
    CnnDetection previous[CNN_PP_NUM_CLASSES];
    uint8_t streak[CNN_PP_NUM_CLASSES];
} CnnPolicy;

typedef struct {
    bool confirmed;
    bool uncertain;
    uint8_t class_id;
    CnnDetection detection;
} CnnPolicyDecision;

void CnnPolicy_Reset(CnnPolicy *policy);
/* Call once for every completed inference, even when count is zero.
 * Only current-frame candidates can be selected. A missing/weak frame resets
 * confirmation, so prior detections never remain live inside this policy. */
CnnPolicyDecision CnnPolicy_Update(CnnPolicy *policy,
                                    const CnnDetection *detections,
                                    uint32_t count);

#endif
