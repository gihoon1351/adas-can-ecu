#ifndef CNN_SW_REFERENCE_H
#define CNN_SW_REFERENCE_H

#include <stdbool.h>
#include <stdint.h>

#include "../cnn_postprocess_fixed.h"

#define CNN_SW_INPUT_WIDTH     (224U)
#define CNN_SW_INPUT_HEIGHT    (224U)
#define CNN_SW_INPUT_CHANNELS  (3U)
#define CNN_SW_INPUT_BYTE_COUNT \
    (CNN_SW_INPUT_WIDTH * CNN_SW_INPUT_HEIGHT * CNN_SW_INPUT_CHANNELS)

typedef struct
{
    const uint8_t *input;
    uint32_t input_byte_count;
} CnnSwReferenceContext;

bool CnnSwReference_Init(void);

bool CnnSwReference_Run(const uint8_t *input,
                        uint32_t input_byte_count,
                        uint32_t *output_words,
                        uint32_t output_capacity_words);

bool CnnSwReference_BenchmarkRun(void *context,
                                 uint32_t *output_words,
                                 uint32_t output_capacity_words);

void CnnSwReference_FillTestInput(uint8_t *input,
                                  uint32_t input_byte_count);

#endif
