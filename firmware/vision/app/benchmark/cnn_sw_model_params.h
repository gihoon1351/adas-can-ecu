#ifndef CNN_SW_MODEL_PARAMS_H
#define CNN_SW_MODEL_PARAMS_H

#include <stdint.h>

#define CNN_SW_MODEL_LAYER_COUNT     (7U)
#define CNN_SW_MODEL_PARAMETER_COUNT (1038U)

extern const float
    g_cnn_sw_post_multiply[CNN_SW_MODEL_PARAMETER_COUNT];
extern const float
    g_cnn_sw_post_add[CNN_SW_MODEL_PARAMETER_COUNT];

#endif
