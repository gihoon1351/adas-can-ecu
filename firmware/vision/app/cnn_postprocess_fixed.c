#include "cnn_postprocess_fixed.h"
#include "cnn_activation_scale.h"
#include "cnn_threshold_profile.h"

#include <math.h>
#include "xil_printf.h"
#include "../hal/uart_log/uart_log_hal.h"
#define xil_printf UartLogHal_Printf

#ifndef CNN_PP_DEBUG_PRINT
#define CNN_PP_DEBUG_PRINT 0
#endif

#define CNN_GRID_SIZE       7U
#define CNN_CELL_WORDS      8U
#define CNN_CHANNELS_PADDED 32U
#define CNN_ANCHORS         3U
#define CNN_VALUES_ANCHOR   10U

/*
 * Output layout:
 *   two 16-channel groups; 49 cells x 4 words per group.
 *   The last group has 14 valid channels (30 channels total).
 *   3 anchors x 10 values:
 *     tx, ty, tw, th, obj, child_sign, 70, speed_bump, red_light, green_light
 *
 * Final-layer output activation scale for the installed V5 parameter data.
 * This is separate from the PPU's internal Q20 scale RAM values.
 */
#define CNN_OUTPUT_SCALE CNN_ACTIVATION_OUTPUT_SCALE

static const char *const k_class_names[CNN_PP_NUM_CLASSES] = {
    "child_sign",
    "70",
    "speed_bump",
    "red_light",
    "green_light"
};

static float g_thresholds[CNN_PP_NUM_CLASSES] = {
    CNN_THRESHOLD_BASE_CHILD,
    CNN_THRESHOLD_BASE_70,
    CNN_THRESHOLD_BASE_BUMP,
    CNN_THRESHOLD_BASE_RED,
    CNN_THRESHOLD_BASE_GREEN
};

static CnnDetection g_detections[CNN_PP_NUM_CLASSES];
static CnnDetection g_best_candidates[CNN_PP_NUM_CLASSES];
static CnnClassProbe g_class_probes[CNN_PP_NUM_CLASSES];
static u8 g_layout = CNN_PP_LAYOUT_GROUP_MAJOR;
static s8 g_max_objectness_raw;

static float exp_approx(float x)
{
    int i;
    float y;

    if (x < -16.0f) return 0.0f;
    if (x > 16.0f)  x = 16.0f;

    y = 1.0f + (x / 256.0f);
    for (i = 0; i < 8; ++i) {
        y *= y;
    }
    return y;
}

static float sigmoid_from_q8(s8 q)
{
    float x = ((float)q) * CNN_OUTPUT_SCALE;
    return 1.0f / (1.0f + exp_approx(-x));
}

static float clamp_unit(float value)
{
    if (value < 0.0f) value = 0.0f;
    else if (value > 1.0f) value = 1.0f;
    else { /* Already normalized. */ }
    return value;
}

/* Restore the notebook's box equations. Width/height use sigmoid, with no
 * anchor-template multiplication; the anchors are assignment-only. Preserve
 * the existing fast score calculation and use expf only for box coordinates. */
static float box_sigmoid(s8 q)
{
    return 1.0f / (1.0f + expf(-((float)q * CNN_OUTPUT_SCALE)));
}

static void decode_box(CnnDetection *detection)
{
    const float cx = ((float)detection->grid_x + box_sigmoid(detection->tx))
                   / (float)CNN_GRID_SIZE;
    const float cy = ((float)detection->grid_y + box_sigmoid(detection->ty))
                   / (float)CNN_GRID_SIZE;
    const float width = box_sigmoid(detection->tw);
    const float height = box_sigmoid(detection->th);

    detection->x0 = clamp_unit(cx - 0.5f * width) * 223.0f;
    detection->y0 = clamp_unit(cy - 0.5f * height) * 223.0f;
    detection->x1 = clamp_unit(cx + 0.5f * width) * 223.0f;
    detection->y1 = clamp_unit(cy + 0.5f * height) * 223.0f;
}

static s8 get_cell_channel(const u32 result_words[392], u32 cell, u32 channel)
{
    u32 word;
    u32 byte_sel;
    u32 v;

    if (g_layout == CNN_PP_LAYOUT_CELL_MAJOR)
        word = result_words[cell * CNN_CELL_WORDS + (channel >> 2)];
    else
        word = result_words[(channel >> 4) * (CNN_GRID_SIZE * CNN_GRID_SIZE * 4U)
                          + cell * 4U + ((channel & 15U) >> 2)];
    byte_sel = channel & 3U;

    if (byte_sel == 0U) {
        v = (word >> 24) & 0xffU;
    } else if (byte_sel == 1U) {
        v = (word >> 16) & 0xffU;
    } else if (byte_sel == 2U) {
        v = (word >> 8) & 0xffU;
    } else {
        v = word & 0xffU;
    }

    return (s8)((u8)v);
}

/* Training uses independent BCE class logits. Decode each class with sigmoid
 * and retain the highest class for this anchor, matching Python inference. */
static void class_sigmoid_probs(const s8 q[CNN_PP_NUM_CLASSES],
                                float probabilities[CNN_PP_NUM_CLASSES],
                                u8 *class_id)
{
    u32 i;

    *class_id = 0U;
    for (i = 0U; i < CNN_PP_NUM_CLASSES; ++i) {
        probabilities[i] = sigmoid_from_q8(q[i]);
        if (q[i] > q[*class_id])
            *class_id = (u8)i;
    }
}

void CnnPostprocess_Run(const u32 result_words[392])
{
    u32 i;
    u32 cell;
    u32 anchor;

    g_max_objectness_raw = (s8)-128;
    for (i = 0U; i < CNN_PP_NUM_CLASSES; ++i) {
        g_detections[i].valid = 0U;
        g_detections[i].class_id = (u8)i;
        g_detections[i].anchor = 0U;
        g_detections[i].grid_x = 0U;
        g_detections[i].grid_y = 0U;
        g_detections[i].score = 0.0f;
        g_detections[i].tx = 0;
        g_detections[i].ty = 0;
        g_detections[i].tw = 0;
        g_detections[i].th = 0;
        g_detections[i].x0 = 0.0f;
        g_detections[i].y0 = 0.0f;
        g_detections[i].x1 = 0.0f;
        g_detections[i].y1 = 0.0f;
        g_best_candidates[i] = g_detections[i];
        g_class_probes[i].valid = 0U;
        g_class_probes[i].score = 0.0f;
    }

    for (cell = 0U; cell < (CNN_GRID_SIZE * CNN_GRID_SIZE); ++cell) {
        u8 gx = (u8)(cell % CNN_GRID_SIZE);
        u8 gy = (u8)(cell / CNN_GRID_SIZE);

        for (anchor = 0U; anchor < CNN_ANCHORS; ++anchor) {
            u32 base = anchor * CNN_VALUES_ANCHOR;
            s8 tx = get_cell_channel(result_words, cell, base + 0U);
            s8 ty = get_cell_channel(result_words, cell, base + 1U);
            s8 tw = get_cell_channel(result_words, cell, base + 2U);
            s8 th = get_cell_channel(result_words, cell, base + 3U);
            s8 obj = get_cell_channel(result_words, cell, base + 4U);
            s8 c0 = get_cell_channel(result_words, cell, base + 5U);
            s8 c1 = get_cell_channel(result_words, cell, base + 6U);
            s8 c2 = get_cell_channel(result_words, cell, base + 7U);
            s8 c3 = get_cell_channel(result_words, cell, base + 8U);
            s8 c4 = get_cell_channel(result_words, cell, base + 9U);
            const s8 class_q[CNN_PP_NUM_CLASSES] = {c0, c1, c2, c3, c4};
            float class_probs[CNN_PP_NUM_CLASSES];
            u8 class_id;
            u32 probe_class;
            if (obj > g_max_objectness_raw) g_max_objectness_raw = obj;
            float obj_prob = sigmoid_from_q8(obj);
            float class_prob;
            class_sigmoid_probs(class_q, class_probs, &class_id);
            class_prob = class_probs[class_id];
            float score = obj_prob * class_prob;

            for (probe_class = 0U; probe_class < CNN_PP_NUM_CLASSES; ++probe_class) {
                const float probe_prob = class_probs[probe_class];
                const float probe_score = obj_prob * probe_prob;
                CnnClassProbe *probe = &g_class_probes[probe_class];
                if (!probe->valid || probe_score > probe->score) {
                    probe->valid = 1U;
                    probe->anchor = (u8)anchor;
                    probe->grid_x = gx;
                    probe->grid_y = gy;
                    probe->winning_class = class_id;
                    probe->objectness_q = obj;
                    probe->class_q = class_q[probe_class];
                    probe->score = probe_score;
                    probe->objectness = obj_prob;
                    probe->class_probability = probe_prob;
                }
            }

            if (!g_best_candidates[class_id].valid ||
                score > g_best_candidates[class_id].score) {
                CnnDetection *best = &g_best_candidates[class_id];
                best->valid = 1U; /* a candidate exists, even below threshold */
                best->class_id = class_id;
                best->anchor = (u8)anchor;
                best->grid_x = gx;
                best->grid_y = gy;
                best->score = score;
                best->tx = tx;
                best->ty = ty;
                best->tw = tw;
                best->th = th;
            }
        }
    }
    for (i = 0U; i < CNN_PP_NUM_CLASSES; ++i) {
        if (g_best_candidates[i].valid)
            decode_box(&g_best_candidates[i]);
        if (g_best_candidates[i].valid &&
            g_best_candidates[i].score >= g_thresholds[i])
            g_detections[i] = g_best_candidates[i];
    }

#if CNN_PP_DEBUG_PRINT
    xil_printf("CNN postprocess:\r\n");
    for (i = 0U; i < CNN_PP_NUM_CLASSES; ++i) {
        const CnnDetection *d = &g_detections[i];
        xil_printf("  %d %-11s valid=%d score=%d.%03d grid=(%d,%d) anchor=%d raw=[%d,%d,%d,%d]\r\n",
                   (int)i,
                   k_class_names[i],
                   (int)d->valid,
                   (int)d->score,
                   (int)((d->score - (float)((int)d->score)) * 1000.0f),
                   (int)d->grid_x,
                   (int)d->grid_y,
                   (int)d->anchor,
                   (int)d->tx,
                   (int)d->ty,
                   (int)d->tw,
                   (int)d->th);
    }

    if (g_detections[3].valid) {
        xil_printf("CNN control hint: red_light detected\r\n");
    } else if (g_detections[4].valid) {
        xil_printf("CNN control hint: green_light detected\r\n");
    } else {
        xil_printf("CNN control hint: no traffic-light action\r\n");
    }
#endif
}

const CnnDetection *CnnPostprocess_GetDetections(void)
{
    return g_detections;
}

const CnnDetection *CnnPostprocess_GetBestCandidates(void)
{
    return g_best_candidates;
}

const CnnClassProbe *CnnPostprocess_GetClassProbes(void)
{
    return g_class_probes;
}

float CnnPostprocess_GetThreshold(u32 class_id)
{
    return class_id < CNN_PP_NUM_CLASSES ? g_thresholds[class_id] : 0.0f;
}

int CnnPostprocess_SetThresholds(const float thresholds[CNN_PP_NUM_CLASSES])
{
    u32 i;
    if (thresholds == 0) return 0;
    for (i = 0U; i < CNN_PP_NUM_CLASSES; ++i)
        if (!(thresholds[i] >= 0.0f && thresholds[i] <= 1.0f))
            return 0;
    for (i = 0U; i < CNN_PP_NUM_CLASSES; ++i)
        g_thresholds[i] = thresholds[i];
    return 1;
}

void CnnPostprocess_SetLayout(u8 layout)
{
    if (layout == CNN_PP_LAYOUT_GROUP_MAJOR || layout == CNN_PP_LAYOUT_CELL_MAJOR)
        g_layout = layout;
}

u8 CnnPostprocess_GetLayout(void)
{
    return g_layout;
}

s8 CnnPostprocess_GetMaxObjectnessRaw(void)
{
    return g_max_objectness_raw;
}

float CnnPostprocess_GetOutputScale(void)
{
    return CNN_OUTPUT_SCALE;
}

const char *CnnPostprocess_ClassName(u32 class_id)
{
    return (class_id < CNN_PP_NUM_CLASSES)
         ? k_class_names[class_id]
         : "unknown";
}

void CnnPostprocess_Print(const CnnDetection detections[CNN_PP_NUM_CLASSES])
{
    u32 i;
    u32 count = 0U;

    if (detections == 0)
    {
        xil_printf("CNN threshold detections: 0\r\n");
        return;
    }

    for (i = 0U; i < CNN_PP_NUM_CLASSES; ++i)
    {
        if (detections[i].valid != 0U)
        {
            ++count;
        }
    }
    xil_printf("CNN threshold detections: %u\r\n", (unsigned int)count);

    for (i = 0U; i < CNN_PP_NUM_CLASSES; ++i)
    {
        const CnnDetection *d = &detections[i];
        u32 score_milli_percent;

        if (d->valid == 0U)
        {
            continue;
        }
        score_milli_percent = (u32)(d->score * 100000.0f + 0.5f);
        xil_printf("det class=%u(%s) score=%u.%03u%% "
                   "box=(%u,%u)-(%u,%u) grid=(%u,%u) anchor=%u\r\n",
                   (unsigned int)d->class_id,
                   CnnPostprocess_ClassName(d->class_id),
                   (unsigned int)(score_milli_percent / 1000U),
                   (unsigned int)(score_milli_percent % 1000U),
                   (unsigned int)(d->x0 + 0.5f),
                   (unsigned int)(d->y0 + 0.5f),
                   (unsigned int)(d->x1 + 0.5f),
                   (unsigned int)(d->y1 + 0.5f),
                   (unsigned int)d->grid_x,
                   (unsigned int)d->grid_y,
                   (unsigned int)d->anchor);
    }
}
