#ifndef CNN_POSTPROCESS_FIXED_H
#define CNN_POSTPROCESS_FIXED_H

#include "xil_types.h"

#define CNN_PP_NUM_CLASSES 5U
#define CNN_PP_OA_WORDS 392U
#define CNN_PP_LAYOUT_GROUP_MAJOR 0U
#define CNN_PP_LAYOUT_CELL_MAJOR  1U

#define CNN_CLASS_CHILD_SIGN      0U
#define CNN_CLASS_SIGN_70         1U
#define CNN_CLASS_SPEED_BUMP      2U
#define CNN_CLASS_RED_LIGHT       3U
#define CNN_CLASS_GREEN_LIGHT     4U

typedef struct {
    u8 valid;
    u8 class_id;
    u8 anchor;
    u8 grid_x;
    u8 grid_y;
    float score;
    s8 tx;
    s8 ty;
    s8 tw;
    s8 th;
    /* Float pixel endpoints in the 224x224 CNN input, before rounding. */
    float x0;
    float y0;
    float x1;
    float y1;
} CnnDetection;

/* Diagnostic score for each class independently at every grid/anchor.
 * This does not change the winning-class detection decision. */
typedef struct {
    u8 valid;
    u8 anchor;
    u8 grid_x;
    u8 grid_y;
    u8 winning_class;
    s8 objectness_q;
    s8 class_q;
    float score;
    float objectness;
    float class_probability;
} CnnClassProbe;

void CnnPostprocess_Run(const u32 result_words[392]);
const CnnDetection *CnnPostprocess_GetDetections(void);
/* Per-class strongest candidates before score thresholds. */
const CnnDetection *CnnPostprocess_GetBestCandidates(void);
const CnnClassProbe *CnnPostprocess_GetClassProbes(void);
float CnnPostprocess_GetThreshold(u32 class_id);
/* Applied by the current PCam test on the next inference. Invalid values
 * leave the active thresholds unchanged. */
int CnnPostprocess_SetThresholds(const float thresholds[CNN_PP_NUM_CLASSES]);
void CnnPostprocess_SetLayout(u8 layout);
u8 CnnPostprocess_GetLayout(void);
s8 CnnPostprocess_GetMaxObjectnessRaw(void);
float CnnPostprocess_GetOutputScale(void);
const char *CnnPostprocess_ClassName(u32 class_id);
void CnnPostprocess_Print(
    const CnnDetection detections[CNN_PP_NUM_CLASSES]);

#endif
