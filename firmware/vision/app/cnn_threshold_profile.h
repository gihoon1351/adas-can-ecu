#ifndef CNN_THRESHOLD_PROFILE_H
#define CNN_THRESHOLD_PROFILE_H

/* V5 PCam score thresholds.
 * Score is objectness(sigmoid) * winning class(sigmoid), matching the
 * model's BCE training and Python evaluation.
 */
#define CNN_THRESHOLD_OBSERVE 0.50f
#define CNN_THRESHOLD_DIAGNOSTIC 0.30f
#define CNN_THRESHOLD_BASE_CHILD 0.70f
#define CNN_THRESHOLD_BASE_70 0.70f
#define CNN_THRESHOLD_BASE_BUMP 0.55f
#define CNN_THRESHOLD_BASE_RED 0.6f
#define CNN_THRESHOLD_BASE_GREEN 0.45f

#endif
