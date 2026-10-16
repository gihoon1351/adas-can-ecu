#include "cnn_sw_reference.h"

#include <float.h>
#include <stddef.h>

#include "cnn_sw_model_params.h"
#include "../cnn_activation_scale.h"
#include "../../driver/cnn/cnn_weight_data.h"

#define CNN_SW_BUFFER_ELEMENT_COUNT (112U * 112U * 16U)
#define CNN_SW_OUTPUT_SCALE         CNN_ACTIVATION_OUTPUT_SCALE
#define CNN_SW_WEIGHT_LANES         (8U)
#define CNN_SW_OUTPUT_LANES         (16U)
#define CNN_SW_BYTES_PER_WORD       (4U)
#define CNN_SW_MODEL_WEIGHT_COUNT   (1587120U)

typedef struct
{
    uint32_t input_channels;
    uint32_t output_channels;
    uint32_t kernel_size;
    uint32_t weight_byte_offset;
    uint32_t model_weight_offset;
    uint32_t parameter_offset;
} CnnSwLayer;

static const CnnSwLayer s_layers[CNN_SW_MODEL_LAYER_COUNT] =
{
    {3U,   16U, 3U,       0U,       0U,    0U},
    {16U,  32U, 3U,    1152U,     432U,   16U},
    {32U,  64U, 3U,    5760U,    5040U,   48U},
    {64U, 128U, 3U,   24192U,   23472U,  112U},
    {128U,256U, 3U,   97920U,   97200U,  240U},
    {256U,512U, 3U,  392832U,  392112U,  496U},
    {512U, 30U, 1U, 1572480U, 1571760U, 1008U}
};

/* Two ping-pong feature buffers. They are static to avoid stack/heap use. */
static float s_feature_a[CNN_SW_BUFFER_ELEMENT_COUNT]
    __attribute__((aligned(64)));
static float s_feature_b[CNN_SW_BUFFER_ELEMENT_COUNT]
    __attribute__((aligned(64)));
static int8_t s_model_weights[CNN_SW_MODEL_WEIGHT_COUNT]
    __attribute__((aligned(64)));
static bool s_model_weights_initialized = false;

static inline uint32_t CnnSw_RoundUp(uint32_t value, uint32_t multiple)
{
    return ((value + multiple - 1U) / multiple) * multiple;
}

static int8_t CnnSw_GetPackedWeight(uint32_t layer_index,
                                    uint32_t output_channel,
                                    uint32_t input_channel,
                                    uint32_t kernel_index)
{
    const CnnSwLayer *layer = &s_layers[layer_index];
    uint32_t mapped_input_channel = input_channel;
    uint32_t input_group_count;
    uint32_t output_group;
    uint32_t output_lane;
    uint32_t input_group;
    uint32_t input_lane;
    uint32_t kernel_count;
    uint32_t byte_offset;
    uint32_t word;
    uint32_t shift;

    /* The first-layer export swaps input lanes 1 and 2. */
    if (layer_index == 0U)
    {
        if (input_channel == 1U)
        {
            mapped_input_channel = 2U;
        }
        else if (input_channel == 2U)
        {
            mapped_input_channel = 1U;
        }
        else
        {
            /* Channel zero is unchanged. */
        }
    }

    input_group_count = CnnSw_RoundUp(
        layer->input_channels, CNN_SW_WEIGHT_LANES) /
        CNN_SW_WEIGHT_LANES;
    output_group = output_channel / CNN_SW_OUTPUT_LANES;
    output_lane = output_channel % CNN_SW_OUTPUT_LANES;
    input_group = mapped_input_channel / CNN_SW_WEIGHT_LANES;
    input_lane = mapped_input_channel % CNN_SW_WEIGHT_LANES;
    kernel_count = layer->kernel_size * layer->kernel_size;

    byte_offset = layer->weight_byte_offset +
        (((((output_group * input_group_count) + input_group) *
            kernel_count + kernel_index) * CNN_SW_OUTPUT_LANES +
           output_lane) * CNN_SW_WEIGHT_LANES) + input_lane;

    word = g_cnn_weight_words[byte_offset / CNN_SW_BYTES_PER_WORD];
    shift = (3U - (byte_offset % CNN_SW_BYTES_PER_WORD)) * 8U;

    return (int8_t)(uint8_t)((word >> shift) & 0xFFU);
}

static inline int8_t CnnSw_GetWeight(uint32_t layer_index,
                                     uint32_t output_channel,
                                     uint32_t input_channel,
                                     uint32_t kernel_index)
{
    const CnnSwLayer *layer = &s_layers[layer_index];
    const uint32_t kernel_count =
        layer->kernel_size * layer->kernel_size;
    const uint32_t weight_index = layer->model_weight_offset +
        (((output_channel * layer->input_channels) + input_channel) *
         kernel_count) + kernel_index;

    return s_model_weights[weight_index];
}

bool CnnSwReference_Init(void)
{
    uint32_t layer_index;
    uint32_t output_channel;
    uint32_t input_channel;
    uint32_t kernel_index;
    uint32_t kernel_count;
    uint32_t weight_index;

    if (!s_model_weights_initialized)
    {
        for (layer_index = 0U;
             layer_index < CNN_SW_MODEL_LAYER_COUNT;
             ++layer_index)
        {
            kernel_count = s_layers[layer_index].kernel_size *
                           s_layers[layer_index].kernel_size;
            for (output_channel = 0U;
                 output_channel < s_layers[layer_index].output_channels;
                 ++output_channel)
            {
                for (input_channel = 0U;
                     input_channel < s_layers[layer_index].input_channels;
                     ++input_channel)
                {
                    for (kernel_index = 0U;
                         kernel_index < kernel_count;
                         ++kernel_index)
                    {
                        weight_index =
                            s_layers[layer_index].model_weight_offset +
                            (((output_channel *
                               s_layers[layer_index].input_channels) +
                              input_channel) * kernel_count) +
                            kernel_index;
                        s_model_weights[weight_index] =
                            CnnSw_GetPackedWeight(layer_index,
                                                  output_channel,
                                                  input_channel,
                                                  kernel_index);
                    }
                }
            }
        }
        s_model_weights_initialized = true;
    }

    return s_model_weights_initialized;
}

static float CnnSw_ApplyPostOperation(uint32_t layer_index,
                                      uint32_t output_channel,
                                      float accumulator,
                                      bool apply_relu)
{
    const uint32_t parameter_index =
        s_layers[layer_index].parameter_offset + output_channel;
    float value =
        (accumulator * g_cnn_sw_post_multiply[parameter_index]) +
        g_cnn_sw_post_add[parameter_index];

    if (apply_relu && (value < 0.0F))
    {
        value = 0.0F;
    }

    return value;
}

static void CnnSw_ConvReluPoolU8(const uint8_t *input,
                                 float *output)
{
    const CnnSwLayer *layer = &s_layers[0];
    uint32_t output_y;
    uint32_t output_x;
    uint32_t output_channel;
    uint32_t pool_y;
    uint32_t pool_x;
    uint32_t kernel_y;
    uint32_t kernel_x;
    uint32_t input_channel;
    uint32_t input_y;
    uint32_t input_x;
    uint32_t input_index;
    uint32_t output_index;
    uint32_t kernel_index;
    float accumulator;
    float value;
    float maximum;

    for (output_y = 0U; output_y < 112U; ++output_y)
    {
        for (output_x = 0U; output_x < 112U; ++output_x)
        {
            for (output_channel = 0U;
                 output_channel < layer->output_channels;
                 ++output_channel)
            {
                maximum = 0.0F;
                for (pool_y = 0U; pool_y < 2U; ++pool_y)
                {
                    for (pool_x = 0U; pool_x < 2U; ++pool_x)
                    {
                        accumulator = 0.0F;
                        input_y = (output_y * 2U) + pool_y;
                        input_x = (output_x * 2U) + pool_x;

                        for (kernel_y = 0U; kernel_y < 3U; ++kernel_y)
                        {
                            if (((input_y + kernel_y) > 0U) &&
                                ((input_y + kernel_y) <= 224U))
                            {
                                for (kernel_x = 0U;
                                     kernel_x < 3U;
                                     ++kernel_x)
                                {
                                    if (((input_x + kernel_x) > 0U) &&
                                        ((input_x + kernel_x) <= 224U))
                                    {
                                        kernel_index =
                                            (kernel_y * 3U) + kernel_x;
                                        for (input_channel = 0U;
                                             input_channel < 3U;
                                             ++input_channel)
                                        {
                                            input_index =
                                                ((((input_y + kernel_y) - 1U) *
                                                  224U +
                                                  ((input_x + kernel_x) - 1U)) *
                                                 3U) + input_channel;
                                            accumulator +=
                                                (float)input[input_index] *
                                                (float)CnnSw_GetWeight(
                                                    0U,
                                                    output_channel,
                                                    input_channel,
                                                    kernel_index);
                                        }
                                    }
                                }
                            }
                        }

                        value = CnnSw_ApplyPostOperation(
                            0U, output_channel, accumulator, true);
                        if (value > maximum)
                        {
                            maximum = value;
                        }
                    }
                }

                output_index =
                    ((output_y * 112U + output_x) * 16U) +
                    output_channel;
                output[output_index] = maximum;
            }
        }
    }
}

static void CnnSw_ConvReluPoolFloat(const float *input,
                                    float *output,
                                    uint32_t layer_index,
                                    uint32_t input_width)
{
    const CnnSwLayer *layer = &s_layers[layer_index];
    const uint32_t output_width = input_width / 2U;
    uint32_t output_y;
    uint32_t output_x;
    uint32_t output_channel;
    uint32_t pool_y;
    uint32_t pool_x;
    uint32_t kernel_y;
    uint32_t kernel_x;
    uint32_t input_channel;
    uint32_t input_y;
    uint32_t input_x;
    uint32_t input_index;
    uint32_t output_index;
    uint32_t kernel_index;
    float accumulator;
    float value;
    float maximum;

    for (output_y = 0U; output_y < output_width; ++output_y)
    {
        for (output_x = 0U; output_x < output_width; ++output_x)
        {
            for (output_channel = 0U;
                 output_channel < layer->output_channels;
                 ++output_channel)
            {
                maximum = 0.0F;
                for (pool_y = 0U; pool_y < 2U; ++pool_y)
                {
                    for (pool_x = 0U; pool_x < 2U; ++pool_x)
                    {
                        accumulator = 0.0F;
                        input_y = (output_y * 2U) + pool_y;
                        input_x = (output_x * 2U) + pool_x;

                        for (kernel_y = 0U; kernel_y < 3U; ++kernel_y)
                        {
                            if (((input_y + kernel_y) > 0U) &&
                                ((input_y + kernel_y) <= input_width))
                            {
                                for (kernel_x = 0U;
                                     kernel_x < 3U;
                                     ++kernel_x)
                                {
                                    if (((input_x + kernel_x) > 0U) &&
                                        ((input_x + kernel_x) <= input_width))
                                    {
                                        kernel_index =
                                            (kernel_y * 3U) + kernel_x;
                                        for (input_channel = 0U;
                                             input_channel <
                                                 layer->input_channels;
                                             ++input_channel)
                                        {
                                            input_index =
                                                ((((input_y + kernel_y) - 1U) *
                                                  input_width +
                                                  ((input_x + kernel_x) - 1U)) *
                                                 layer->input_channels) +
                                                input_channel;
                                            accumulator +=
                                                input[input_index] *
                                                (float)CnnSw_GetWeight(
                                                    layer_index,
                                                    output_channel,
                                                    input_channel,
                                                    kernel_index);
                                        }
                                    }
                                }
                            }
                        }

                        value = CnnSw_ApplyPostOperation(
                            layer_index,
                            output_channel,
                            accumulator,
                            true);
                        if (value > maximum)
                        {
                            maximum = value;
                        }
                    }
                }

                output_index =
                    ((output_y * output_width + output_x) *
                     layer->output_channels) + output_channel;
                output[output_index] = maximum;
            }
        }
    }
}

static void CnnSw_ConvReluFloat(const float *input,
                                float *output,
                                uint32_t layer_index,
                                uint32_t width)
{
    const CnnSwLayer *layer = &s_layers[layer_index];
    uint32_t y;
    uint32_t x;
    uint32_t output_channel;
    uint32_t kernel_y;
    uint32_t kernel_x;
    uint32_t input_channel;
    uint32_t input_index;
    uint32_t output_index;
    uint32_t kernel_index;
    float accumulator;

    for (y = 0U; y < width; ++y)
    {
        for (x = 0U; x < width; ++x)
        {
            for (output_channel = 0U;
                 output_channel < layer->output_channels;
                 ++output_channel)
            {
                accumulator = 0.0F;
                for (kernel_y = 0U; kernel_y < 3U; ++kernel_y)
                {
                    if (((y + kernel_y) > 0U) &&
                        ((y + kernel_y) <= width))
                    {
                        for (kernel_x = 0U; kernel_x < 3U; ++kernel_x)
                        {
                            if (((x + kernel_x) > 0U) &&
                                ((x + kernel_x) <= width))
                            {
                                kernel_index =
                                    (kernel_y * 3U) + kernel_x;
                                for (input_channel = 0U;
                                     input_channel < layer->input_channels;
                                     ++input_channel)
                                {
                                    input_index =
                                        ((((y + kernel_y) - 1U) * width +
                                          ((x + kernel_x) - 1U)) *
                                         layer->input_channels) +
                                        input_channel;
                                    accumulator +=
                                        input[input_index] *
                                        (float)CnnSw_GetWeight(
                                            layer_index,
                                            output_channel,
                                            input_channel,
                                            kernel_index);
                                }
                            }
                        }
                    }
                }

                output_index =
                    ((y * width + x) * layer->output_channels) +
                    output_channel;
                output[output_index] = CnnSw_ApplyPostOperation(
                    layer_index, output_channel, accumulator, true);
            }
        }
    }
}

static int8_t CnnSw_QuantizeOutput(float value)
{
    float scaled = value / CNN_SW_OUTPUT_SCALE;
    int32_t quantized;

    if (scaled >= 0.0F)
    {
        quantized = (int32_t)(scaled + 0.5F);
    }
    else
    {
        quantized = (int32_t)(scaled - 0.5F);
    }

    if (quantized > 127)
    {
        quantized = 127;
    }
    else if (quantized < -128)
    {
        quantized = -128;
    }
    else
    {
        /* Value is already in range. */
    }

    return (int8_t)quantized;
}

static void CnnSw_DetectAndPack(const float *input,
                                uint32_t *output_words)
{
    const CnnSwLayer *layer = &s_layers[6];
    uint32_t cell;
    uint32_t output_channel;
    uint32_t input_channel;
    uint32_t group;
    uint32_t lane;
    uint32_t word_index;
    uint32_t shift;
    float accumulator;
    float value;
    int8_t quantized;

    for (cell = 0U; cell < (7U * 7U); ++cell)
    {
        for (output_channel = 0U;
             output_channel < layer->output_channels;
             ++output_channel)
        {
            accumulator = 0.0F;
            for (input_channel = 0U;
                 input_channel < layer->input_channels;
                 ++input_channel)
            {
                accumulator +=
                    input[(cell * layer->input_channels) + input_channel] *
                    (float)CnnSw_GetWeight(
                        6U, output_channel, input_channel, 0U);
            }

            value = CnnSw_ApplyPostOperation(
                6U, output_channel, accumulator, false);
            quantized = CnnSw_QuantizeOutput(value);
            group = output_channel >> 4U;
            lane = output_channel & 15U;
            word_index = (group * 196U) + (cell * 4U) + (lane >> 2U);
            shift = (3U - (lane & 3U)) * 8U;
            output_words[word_index] |=
                (uint32_t)(uint8_t)quantized << shift;
        }
    }
}

bool CnnSwReference_Run(const uint8_t *input,
                        uint32_t input_byte_count,
                        uint32_t *output_words,
                        uint32_t output_capacity_words)
{
    uint32_t index;
    bool success = false;

    if ((input != NULL) &&
        (input_byte_count >= CNN_SW_INPUT_BYTE_COUNT) &&
        (output_words != NULL) &&
        (output_capacity_words >= CNN_PP_OA_WORDS) &&
        CnnSwReference_Init())
    {
        for (index = 0U; index < CNN_PP_OA_WORDS; ++index)
        {
            output_words[index] = 0U;
        }

        CnnSw_ConvReluPoolU8(input, s_feature_a);
        CnnSw_ConvReluPoolFloat(s_feature_a, s_feature_b, 1U, 112U);
        CnnSw_ConvReluPoolFloat(s_feature_b, s_feature_a, 2U, 56U);
        CnnSw_ConvReluPoolFloat(s_feature_a, s_feature_b, 3U, 28U);
        CnnSw_ConvReluPoolFloat(s_feature_b, s_feature_a, 4U, 14U);
        CnnSw_ConvReluFloat(s_feature_a, s_feature_b, 5U, 7U);
        CnnSw_DetectAndPack(s_feature_b, output_words);
        success = true;
    }

    return success;
}

bool CnnSwReference_BenchmarkRun(void *context,
                                 uint32_t *output_words,
                                 uint32_t output_capacity_words)
{
    const CnnSwReferenceContext *sw_context =
        (const CnnSwReferenceContext *)context;
    bool success = false;

    if (sw_context != NULL)
    {
        success = CnnSwReference_Run(sw_context->input,
                                     sw_context->input_byte_count,
                                     output_words,
                                     output_capacity_words);
    }

    return success;
}

void CnnSwReference_FillTestInput(uint8_t *input,
                                  uint32_t input_byte_count)
{
    uint32_t y;
    uint32_t x;
    uint32_t index;

    if ((input != NULL) && (input_byte_count >= CNN_SW_INPUT_BYTE_COUNT))
    {
        for (y = 0U; y < CNN_SW_INPUT_HEIGHT; ++y)
        {
            for (x = 0U; x < CNN_SW_INPUT_WIDTH; ++x)
            {
                index = ((y * CNN_SW_INPUT_WIDTH + x) *
                         CNN_SW_INPUT_CHANNELS);
                input[index] = (uint8_t)x;
                input[index + 1U] = (uint8_t)y;
                input[index + 2U] = (uint8_t)((x + y) & 0xFFU);
            }
        }
    }
}
