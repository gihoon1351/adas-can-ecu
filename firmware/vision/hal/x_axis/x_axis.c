#include "x_axis.h"

#include <stddef.h>

#include "xil_printf.h"
#include "xstatus.h"
#include "../irq/irq_hal.h"
#include "../uart_log/uart_log_hal.h"
/* Preserve existing messages; queue them without UART waits at runtime. */
#define xil_printf UartLogHal_Printf

#define X_AXIS_IMAGE_WIDTH       (1280U)
#define X_AXIS_IMAGE_HEIGHT      (720U)
#define X_AXIS_MAX_SIZE_LOG2     (7U)
#define X_AXIS_Y_STEP_SMALL      (4U)
#define X_AXIS_Y_STEP_LARGE      (16U)
#define X_AXIS_WIDTH_STEP_SMALL  (8U)
#define X_AXIS_WIDTH_STEP_LARGE  (32U)

typedef enum
{
    X_AXIS_FIELD_FAR_START = 0,
    X_AXIS_FIELD_NEAR_START,
    X_AXIS_FIELD_FAR_WIDTH,
    X_AXIS_FIELD_NEAR_WIDTH,
    X_AXIS_FIELD_FAR_CENTER,
    X_AXIS_FIELD_NEAR_CENTER,
    X_AXIS_FIELD_FAR_HEIGHT,
    X_AXIS_FIELD_NEAR_HEIGHT,
    X_AXIS_FIELD_COUNT
} XAxisRoiField;

static const XAxisDualRoiConfig s_default_config =
{
    980U,  /* far_width: centered x=150..1129 ([150, 1130)) */
    1240U, /* near_width: centered x=20..1259 ([20, 1260)) */
    640U,  /* far_center: adaptive search center starts at image center */
    640U,  /* near_center: adaptive search center starts at image center */
    420U,  /* far_start: 420..459 ([420, 460)) */
    530U,  /* near_start: 530..593 ([530, 594)) */
    5U,    /* FAR rows = 40 (dual-ROI size code 5) */
    6U     /* NEAR rows = 64 */
};

static XAxisDualRoiConfig s_config;
static uint16_t s_effective_near_start;
static XAxisRoiField s_selected_field;
static bool s_ready;
static uint16_t s_far_miss_count;
static uint16_t s_near_miss_count;
static uint16_t s_far_valid_count;
static bool s_far_trusted_valid;
static uint16_t s_far_trusted_x;
static uint16_t s_far_candidate_x;
static uint8_t s_far_candidate_count;
static bool s_far_reacquire_active;
static bool s_near_trusted_valid;
static uint16_t s_near_trusted_x;
static uint16_t s_near_candidate_x;
static uint8_t s_near_candidate_count;
static bool s_near_reacquire_active;

static uint32_t XAxis_RowsFromSizeCode(uint8_t size_code)
{
    /* Code 5 is reserved by this dual-ROI design for the tuned 40-row band.
     * The remaining codes retain the original power-of-two interpretation. */
    return (size_code == 5U) ? 40UL : (1UL << size_code);
}

static const char *XAxis_FieldName(XAxisRoiField field)
{
    switch (field)
    {
        case X_AXIS_FIELD_FAR_START:  return "FAR Y start";
        case X_AXIS_FIELD_NEAR_START: return "NEAR Y start";
        case X_AXIS_FIELD_FAR_WIDTH:  return "FAR width";
        case X_AXIS_FIELD_NEAR_WIDTH: return "NEAR width";
        case X_AXIS_FIELD_FAR_CENTER: return "FAR center X";
        case X_AXIS_FIELD_NEAR_CENTER:return "NEAR center X";
        case X_AXIS_FIELD_FAR_HEIGHT: return "FAR height";
        case X_AXIS_FIELD_NEAR_HEIGHT:return "NEAR height";
        default:                      return "unknown";
    }
}

static uint16_t XAxis_SubSaturate(uint16_t value,
                                  uint16_t amount,
                                  uint16_t minimum)
{
    if (value <= minimum || value < (uint16_t)(minimum + amount))
    {
        return minimum;
    }
    return (uint16_t)(value - amount);
}

static uint16_t XAxis_AddSaturate(uint16_t value,
                                  uint16_t amount,
                                  uint16_t maximum)
{
    uint32_t sum = (uint32_t)value + (uint32_t)amount;
    return (sum > (uint32_t)maximum) ? maximum : (uint16_t)sum;
}

static uint16_t XAxis_ClampCenter(uint16_t center, uint16_t width)
{
    uint16_t half_width = (uint16_t)(width >> 1U);
    uint16_t minimum = half_width;
    uint16_t maximum = (uint16_t)(X_AXIS_IMAGE_WIDTH - width + half_width);

    if (center < minimum)
    {
        return minimum;
    }
    if (center > maximum)
    {
        return maximum;
    }
    return center;
}

static void XAxis_WriteCenters(X_AXIS_TypeDef *instance,
                               uint16_t far_center,
                               uint16_t near_center)
{
    /* 0x08 is read-as-status and write-as-center configuration. */
    instance->SR =
        ((uint32_t)far_center & X_AXIS_CENTER_FAR_MASK) |
        (((uint32_t)near_center << X_AXIS_CENTER_NEAR_SHIFT) &
         X_AXIS_CENTER_NEAR_MASK);
}

static void XAxis_WriteTrackingWindow(X_AXIS_TypeDef *instance,
                                      const XAxisDualRoiConfig *config)
{
    uint32_t cr0;
    uint32_t cr1;
    uint32_t saved = IrqHal_SaveDisable();

    /* The previous frame has already completed when tracking_update runs.
     * Update width and center together for the detector and red overlay to
     * sample at the next SOF. Do not replay the write-one DONE_CLEAR bit. */
    cr0 = instance->CR0 &
        ~(X_AXIS_CR0_FAR_WIDTH_MASK | X_AXIS_CR0_DONE_CLEAR_MASK);
    cr0 |= (((uint32_t)config->far_width <<
             X_AXIS_CR0_FAR_WIDTH_SHIFT) & X_AXIS_CR0_FAR_WIDTH_MASK);
    cr1 = instance->CR1 & ~X_AXIS_CR1_NEAR_WIDTH_MASK;
    cr1 |= (((uint32_t)config->near_width <<
             X_AXIS_CR1_NEAR_WIDTH_SHIFT) & X_AXIS_CR1_NEAR_WIDTH_MASK);

    instance->CR0 = cr0;
    instance->CR1 = cr1;
    XAxis_WriteCenters(instance, config->far_center, config->near_center);
    IrqHal_Restore(saved);
}

static uint16_t XAxis_WidthShrink(uint16_t width,
                                  uint16_t minimum,
                                  uint16_t step)
{
    return XAxis_SubSaturate(width, step, minimum);
}

static uint16_t XAxis_WidthExpand(uint16_t width,
                                  uint16_t maximum,
                                  uint16_t step)
{
    return XAxis_AddSaturate(width, step, maximum);
}

static bool XAxis_DetectionNearEdge(uint16_t detected,
                                    uint16_t center,
                                    uint16_t width)
{
    uint16_t left = (uint16_t)(center - (width >> 1U));
    uint16_t right = (uint16_t)(left + width - 1U);

    return ((detected <= (uint16_t)(left + X_AXIS_WIDTH_EDGE_GUARD)) ||
            ((uint32_t)detected + X_AXIS_WIDTH_EDGE_GUARD >= right));
}

static bool XAxis_ValidateConfig(const XAxisDualRoiConfig *config,
                                 uint16_t *effective_near_start)
{
    uint32_t far_rows;
    uint32_t near_rows;
    uint32_t far_end;
    uint32_t near_start;

    if (config == NULL || effective_near_start == NULL)
    {
        return false;
    }

    if (config->far_width < 1U ||
        config->far_width > (X_AXIS_IMAGE_WIDTH - 2U) ||
        config->near_width < 1U ||
        config->near_width > (X_AXIS_IMAGE_WIDTH - 2U) ||
        config->far_size_log2 > X_AXIS_MAX_SIZE_LOG2 ||
        config->near_size_log2 > X_AXIS_MAX_SIZE_LOG2)
    {
        return false;
    }

    far_rows = XAxis_RowsFromSizeCode(config->far_size_log2);
    near_rows = XAxis_RowsFromSizeCode(config->near_size_log2);

    if (((uint32_t)config->far_start + far_rows) > X_AXIS_IMAGE_HEIGHT)
    {
        return false;
    }

    far_end = (uint32_t)config->far_start + far_rows - 1UL;
    near_start = (uint32_t)config->near_start;
    if (near_start <= far_end)
    {
        near_start = far_end + 1UL;
    }

    if ((near_start + near_rows) > X_AXIS_IMAGE_HEIGHT)
    {
        return false;
    }

    *effective_near_start = (uint16_t)near_start;
    return true;
}

static void XAxis_WriteConfig(X_AXIS_TypeDef *instance,
                              const XAxisDualRoiConfig *config,
                              uint16_t effective_near_start)
{
    uint32_t control;
    uint32_t cr0;
    uint32_t cr1;
    uint32_t saved = IrqHal_SaveDisable();

    control = instance->CR0 &
        (X_AXIS_CR0_EN_MASK | X_AXIS_CR0_IRQ_EN_MASK);

    cr0 = control |
        (((uint32_t)config->far_size_log2 << X_AXIS_CR0_FAR_SIZE_SHIFT) &
         X_AXIS_CR0_FAR_SIZE_MASK) |
        (((uint32_t)config->far_start << X_AXIS_CR0_FAR_START_SHIFT) &
         X_AXIS_CR0_FAR_START_MASK) |
        (((uint32_t)config->far_width << X_AXIS_CR0_FAR_WIDTH_SHIFT) &
         X_AXIS_CR0_FAR_WIDTH_MASK);

    cr1 =
        (((uint32_t)config->near_size_log2 << X_AXIS_CR1_NEAR_SIZE_SHIFT) &
         X_AXIS_CR1_NEAR_SIZE_MASK) |
        (((uint32_t)effective_near_start << X_AXIS_CR1_NEAR_START_SHIFT) &
         X_AXIS_CR1_NEAR_START_MASK) |
        (((uint32_t)config->near_width << X_AXIS_CR1_NEAR_WIDTH_SHIFT) &
         X_AXIS_CR1_NEAR_WIDTH_MASK);

    instance->CR0 = cr0 & ~X_AXIS_CR0_EN_MASK;
    instance->CR1 = cr1;
    XAxis_WriteCenters(instance, config->far_center, config->near_center);
    IrqHal_Restore(saved);
}

static bool XAxis_ApplyConfig(const XAxisDualRoiConfig *config,
                              bool start_after_apply)
{
    XAxisDualRoiConfig applied;
    uint16_t effective_near_start;

    if (config == NULL)
    {
        return false;
    }

    applied = *config;
    applied.far_center = XAxis_ClampCenter(applied.far_center,
                                           applied.far_width);
    applied.near_center = XAxis_ClampCenter(applied.near_center,
                                            applied.near_width);

    if (!XAxis_ValidateConfig(&applied, &effective_near_start))
    {
        return false;
    }

    x_axis_disable(X_AXIS0);
    XAxis_WriteConfig(X_AXIS0, &applied, effective_near_start);
    x_axis_done_clear(X_AXIS0);

    s_config = applied;
    s_effective_near_start = effective_near_start;

    if (start_after_apply)
    {
        x_axis_enable(X_AXIS0);
    }
    return true;
}

void x_axis_enable(X_AXIS_TypeDef *instance)
{
    uint32_t saved = IrqHal_SaveDisable();
    instance->CR0 = (instance->CR0 & ~X_AXIS_CR0_DONE_CLEAR_MASK) |
                     X_AXIS_CR0_EN_MASK;
    IrqHal_Restore(saved);
}

void x_axis_disable(X_AXIS_TypeDef *instance)
{
    uint32_t saved = IrqHal_SaveDisable();
    instance->CR0 &= ~(X_AXIS_CR0_EN_MASK | X_AXIS_CR0_DONE_CLEAR_MASK);
    IrqHal_Restore(saved);
}

void x_axis_irq_enable(X_AXIS_TypeDef *instance)
{
    uint32_t saved = IrqHal_SaveDisable();
    instance->CR0 = (instance->CR0 & ~X_AXIS_CR0_DONE_CLEAR_MASK) |
                     X_AXIS_CR0_IRQ_EN_MASK;
    IrqHal_Restore(saved);
}

void x_axis_irq_disable(X_AXIS_TypeDef *instance)
{
    uint32_t saved = IrqHal_SaveDisable();
    instance->CR0 &= ~(X_AXIS_CR0_IRQ_EN_MASK | X_AXIS_CR0_DONE_CLEAR_MASK);
    IrqHal_Restore(saved);
}

void x_axis_done_clear(X_AXIS_TypeDef *instance)
{
    uint32_t control;
    uint32_t saved = IrqHal_SaveDisable();
    if ((instance->SR & X_AXIS_SR_DONE_MASK) != 0UL)
    {
        control = instance->CR0 & ~X_AXIS_CR0_DONE_CLEAR_MASK;
        instance->CR0 = control | X_AXIS_CR0_DONE_CLEAR_MASK;
        instance->CR0 = control;
    }
    IrqHal_Restore(saved);
}

bool x_axis_read_done(const X_AXIS_TypeDef *instance)
{
    return ((instance->SR & X_AXIS_SR_DONE_MASK) != 0UL);
}

bool x_axis_read_dual_result(X_AXIS_TypeDef *instance,
                             XAxisDualResult *result)
{
    uint32_t raw;
    uint32_t combined;

    if (instance == NULL || result == NULL || !x_axis_read_done(instance))
    {
        return false;
    }

    raw = instance->DR;
    result->far_x = (uint16_t)(raw & X_AXIS_DR_FAR_MASK);
    result->near_x = (uint16_t)((raw & X_AXIS_DR_NEAR_MASK) >>
                                X_AXIS_DR_NEAR_SHIFT);
	result->far_valid = ((raw & X_AXIS_DR_FAR_VALID_MASK) != 0UL);
	result->near_valid = ((raw & X_AXIS_DR_NEAR_VALID_MASK) != 0UL);

    if (result->far_valid && result->near_valid) {
        combined = (3UL * (uint32_t)result->far_x) +
                   (5UL * (uint32_t)result->near_x) + 4UL;
        result->combined_x = (uint16_t)(combined >> 3U);
    } else if (result->near_valid) {
        result->combined_x = result->near_x;
    } else if (result->far_valid) {
        result->combined_x = result->far_x;
    } else {
        result->combined_x = 640U;
    }

    /* Capture both coordinates first, then acknowledge the sticky result. */
    x_axis_done_clear(instance);
    return true;
}

static uint16_t XAxis_TrackingStep(uint16_t current,
                                   int32_t detected,
                                   uint16_t width)
{
    int32_t filtered;
    int32_t delta;
    int32_t next;

    filtered = (((int32_t)current * X_AXIS_ROI_LPF_OLD_WEIGHT) +
                (detected * X_AXIS_ROI_LPF_NEW_WEIGHT) +
                (X_AXIS_ROI_LPF_WEIGHT_SUM / 2L)) /
               X_AXIS_ROI_LPF_WEIGHT_SUM;
    delta = filtered - (int32_t)current;

    if (delta > X_AXIS_ROI_MAX_MOVE)
    {
        delta = X_AXIS_ROI_MAX_MOVE;
    }
    else if (delta < -X_AXIS_ROI_MAX_MOVE)
    {
        delta = -X_AXIS_ROI_MAX_MOVE;
    }

    next = (int32_t)current + delta;
    if (next < 0L)
    {
        next = 0L;
    }
    else if (next >= (int32_t)X_AXIS_IMAGE_WIDTH)
    {
        next = (int32_t)X_AXIS_IMAGE_WIDTH - 1L;
    }

    return XAxis_ClampCenter((uint16_t)next, width);
}

static uint16_t XAxis_MoveTowardDefault(uint16_t current,
                                        uint16_t default_center,
                                        uint16_t width)
{
    int32_t delta = (int32_t)default_center - (int32_t)current;

    if (delta > X_AXIS_ROI_RECOVER_STEP)
    {
        delta = X_AXIS_ROI_RECOVER_STEP;
    }
    else if (delta < -X_AXIS_ROI_RECOVER_STEP)
    {
        delta = -X_AXIS_ROI_RECOVER_STEP;
    }

    return XAxis_ClampCenter((uint16_t)((int32_t)current + delta), width);
}

static uint16_t XAxis_AbsDiff16(uint16_t a, uint16_t b)
{
    return (a >= b) ? (uint16_t)(a - b) : (uint16_t)(b - a);
}

static uint16_t XAxis_MoveTowardTracked(uint16_t current,
                                        uint16_t target,
                                        uint16_t width)
{
    int32_t delta = (int32_t)target - (int32_t)current;

    if (delta > X_AXIS_ROI_MAX_MOVE)
    {
        delta = X_AXIS_ROI_MAX_MOVE;
    }
    else if (delta < -X_AXIS_ROI_MAX_MOVE)
    {
        delta = -X_AXIS_ROI_MAX_MOVE;
    }
    return XAxis_ClampCenter((uint16_t)((int32_t)current + delta), width);
}

static void XAxis_ResetFarCandidate(void)
{
    s_far_candidate_x = s_far_trusted_x;
    s_far_candidate_count = 0U;
    s_far_reacquire_active = false;
}

static void XAxis_ResetNearCandidate(void)
{
    s_near_candidate_x = s_near_trusted_x;
    s_near_candidate_count = 0U;
    s_near_reacquire_active = false;
}

/* Replace a raw FAR sample with a temporally trusted one. Large changes must
 * form a stable cluster before they can move either the ROI or the VCU. */
static void XAxis_FilterFarResult(XAxisDualResult *result)
{
    uint16_t raw_x = result->far_x;
    uint16_t delta;
    uint16_t required_count;

    if (!result->far_valid || raw_x >= X_AXIS_IMAGE_WIDTH)
    {
        result->far_x = s_far_trusted_valid ? s_far_trusted_x :
                        s_default_config.far_center;
        result->far_valid = false;
        XAxis_ResetFarCandidate();
        return;
    }

    /*
    if (!s_far_trusted_valid)
    {
        s_far_trusted_x = raw_x;
        s_far_trusted_valid = true;
        XAxis_ResetFarCandidate();
        result->far_x = s_far_trusted_x;
        return;
    }
    */

    if (s_far_reacquire_active)
    {
        if (XAxis_AbsDiff16(raw_x, s_far_candidate_x) <=
            X_AXIS_FAR_CANDIDATE_CLUSTER)
        {
            s_far_candidate_x = XAxis_TrackingStep(s_far_candidate_x,
                                                   (int32_t)raw_x, 1U);
            s_far_trusted_x = XAxis_MoveTowardTracked(s_far_trusted_x,
                                                      s_far_candidate_x, 1U);
            result->far_x = s_far_trusted_x;
            if (XAxis_AbsDiff16(raw_x, s_far_trusted_x) <=
                X_AXIS_FAR_NORMAL_DELTA)
            {
                XAxis_ResetFarCandidate();
            }
            return;
        }
        XAxis_ResetFarCandidate();
    }

    delta = XAxis_AbsDiff16(raw_x, s_far_trusted_x);
    if (delta <= X_AXIS_FAR_NORMAL_DELTA)
    {
        s_far_trusted_x = XAxis_TrackingStep(s_far_trusted_x,
                                             (int32_t)raw_x, 1U);
        XAxis_ResetFarCandidate();
        result->far_x = s_far_trusted_x;
        return;
    }

    required_count = (delta > X_AXIS_FAR_LARGE_JUMP_DELTA) ?
        X_AXIS_FAR_LARGE_CONFIRM : X_AXIS_FAR_MODERATE_CONFIRM;
    if ((s_far_candidate_count == 0U) ||
        (XAxis_AbsDiff16(raw_x, s_far_candidate_x) >
         X_AXIS_FAR_CANDIDATE_CLUSTER))
    {
        s_far_candidate_x = raw_x;
        s_far_candidate_count = 1U;
    }
    else
    {
        s_far_candidate_x = XAxis_TrackingStep(s_far_candidate_x,
                                               (int32_t)raw_x, 1U);
        if (s_far_candidate_count < 0xFFU)
        {
            s_far_candidate_count++;
        }
    }

    if (s_far_candidate_count >= required_count)
    {
        s_far_reacquire_active = true;
        s_far_trusted_x = XAxis_MoveTowardTracked(s_far_trusted_x,
                                                  s_far_candidate_x, 1U);
        result->far_x = s_far_trusted_x;
        result->far_valid = true;
    }
    else
    {
        result->far_x = s_far_trusted_x;
        result->far_valid = false;
    }
}

/* Keep NEAR's wide fixed ROI, but do not let a one-frame floor reflection or
 * isolated edge immediately steer the vehicle. A persistent change is
 * accepted after two/three clustered frames and approached at 12 px/frame. */
static void XAxis_FilterNearResult(XAxisDualResult *result)
{
    uint16_t raw_x = result->near_x;
    uint16_t delta;
    uint16_t required_count;

    if (!result->near_valid || raw_x >= X_AXIS_IMAGE_WIDTH)
    {
        result->near_x = s_near_trusted_valid ? s_near_trusted_x :
                         s_default_config.near_center;
        result->near_valid = false;
        XAxis_ResetNearCandidate();
        return;
    }

    /*
    if (!s_near_trusted_valid)
    {
        s_near_trusted_x = raw_x;
        s_near_trusted_valid = true;
        XAxis_ResetNearCandidate();
        result->near_x = s_near_trusted_x;
        return;
    }
    */

    if (s_near_reacquire_active)
    {
        if (XAxis_AbsDiff16(raw_x, s_near_candidate_x) <=
            X_AXIS_NEAR_CANDIDATE_CLUSTER)
        {
            s_near_candidate_x = XAxis_TrackingStep(s_near_candidate_x,
                                                    (int32_t)raw_x, 1U);
            s_near_trusted_x = XAxis_MoveTowardTracked(s_near_trusted_x,
                                                       s_near_candidate_x, 1U);
            result->near_x = s_near_trusted_x;
            if (XAxis_AbsDiff16(raw_x, s_near_trusted_x) <=
                X_AXIS_NEAR_NORMAL_DELTA)
            {
                XAxis_ResetNearCandidate();
            }
            return;
        }
        XAxis_ResetNearCandidate();
    }

    delta = XAxis_AbsDiff16(raw_x, s_near_trusted_x);
    if (delta <= X_AXIS_NEAR_NORMAL_DELTA)
    {
        s_near_trusted_x = XAxis_TrackingStep(s_near_trusted_x,
                                              (int32_t)raw_x, 1U);
        XAxis_ResetNearCandidate();
        result->near_x = s_near_trusted_x;
        return;
    }

    required_count = (delta > X_AXIS_NEAR_LARGE_JUMP_DELTA) ?
        X_AXIS_NEAR_LARGE_CONFIRM : X_AXIS_NEAR_MODERATE_CONFIRM;
    if ((s_near_candidate_count == 0U) ||
        (XAxis_AbsDiff16(raw_x, s_near_candidate_x) >
         X_AXIS_NEAR_CANDIDATE_CLUSTER))
    {
        s_near_candidate_x = raw_x;
        s_near_candidate_count = 1U;
    }
    else
    {
        s_near_candidate_x = XAxis_TrackingStep(s_near_candidate_x,
                                                (int32_t)raw_x, 1U);
        if (s_near_candidate_count < 0xFFU)
        {
            s_near_candidate_count++;
        }
    }

    if (s_near_candidate_count >= required_count)
    {
        s_near_reacquire_active = true;
        s_near_trusted_x = XAxis_MoveTowardTracked(s_near_trusted_x,
                                                   s_near_candidate_x, 1U);
        result->near_x = s_near_trusted_x;
        result->near_valid = true;
    }
    else
    {
        result->near_x = s_near_trusted_x;
        result->near_valid = false;
    }
}

void x_axis_tracking_update(XAxisDualResult *result)
{
    bool far_near_edge = false;

    if (!s_ready || result == NULL)
    {
        return;
    }

    XAxis_FilterFarResult(result);
    XAxis_FilterNearResult(result);

    /* NEAR is intentionally a fixed, wide search band. Its job is current
     * lateral-position feedback, so do not allow a noisy edge to drag or
     * narrow the search window. */
    s_config.near_center = s_default_config.near_center;
    s_config.near_width = X_AXIS_NEAR_WIDTH_FIXED;
    if (result->near_valid && result->near_x < X_AXIS_IMAGE_WIDTH)
    {
        s_near_miss_count = 0U;
    }
    else
    {
        if (s_near_miss_count < 0xFFFFU)
        {
            s_near_miss_count++;
        }
    }

    if (result->far_valid && result->far_x < X_AXIS_IMAGE_WIDTH)
    {
        far_near_edge = XAxis_DetectionNearEdge(result->far_x,
                                                s_config.far_center,
                                                s_config.far_width);
        s_far_miss_count = 0U;
        if (s_far_valid_count < 0xFFFFU)
        {
            s_far_valid_count++;
        }
        s_config.far_center = XAxis_MoveTowardTracked(
            s_config.far_center, result->far_x, s_config.far_width);
        if (far_near_edge)
        {
            s_config.far_width = XAxis_WidthExpand(
                s_config.far_width, X_AXIS_FAR_WIDTH_MAX,
                X_AXIS_FAR_WIDTH_EDGE_STEP);
            s_far_valid_count = 0U;
        }
        else if (s_far_valid_count >= X_AXIS_WIDTH_SHRINK_DELAY)
        {
            s_config.far_width = XAxis_WidthShrink(
                s_config.far_width, X_AXIS_FAR_WIDTH_MIN,
                X_AXIS_FAR_WIDTH_SHRINK_STEP);
        }
    }
    else
    {
        s_far_valid_count = 0U;
        if (s_far_miss_count < 0xFFFFU)
        {
            s_far_miss_count++;
        }
        if (s_far_candidate_count > 0U)
        {
            /* A coherent but unconfirmed jump may be a real curve. Keep the
             * center frozen and create search margin without trusting it. */
            s_config.far_width = XAxis_WidthExpand(
                s_config.far_width, X_AXIS_FAR_WIDTH_MAX,
                X_AXIS_FAR_WIDTH_EDGE_STEP);
        }
        else if (s_far_miss_count > X_AXIS_WIDTH_EXPAND_DELAY)
        {
            s_config.far_width = XAxis_WidthExpand(
                s_config.far_width, X_AXIS_FAR_WIDTH_MAX,
                X_AXIS_FAR_WIDTH_MISS_STEP);
        }
        if (s_far_miss_count > X_AXIS_ROI_RECOVER_DELAY)
        {
            s_config.far_center = XAxis_MoveTowardDefault(
                s_config.far_center,
                s_default_config.far_center,
                s_config.far_width);
        }
    }

    if (result->far_valid && result->near_valid)
    {
        result->combined_x = (uint16_t)(((3UL * result->far_x) +
                                  (5UL * result->near_x) + 4UL) >> 3U);
    }
    else if (result->near_valid)
    {
        result->combined_x = result->near_x;
    }
    else if (result->far_valid)
    {
        result->combined_x = result->far_x;
    }
    else
    {
        result->combined_x = s_default_config.near_center;
    }

    /* Width changes alter legal center limits. Clamp both before publishing
     * one shared next-frame window to the detector and red overlay. */
    s_config.far_center = XAxis_ClampCenter(s_config.far_center,
                                            s_config.far_width);
    s_config.near_center = XAxis_ClampCenter(s_config.near_center,
                                             s_config.near_width);
    XAxis_WriteTrackingWindow(X_AXIS0, &s_config);
}

bool x_axis_tracking_get_status(XAxisTrackingStatus *status)
{
    if (!s_ready || status == NULL)
    {
        return false;
    }

    status->far_center = s_config.far_center;
    status->near_center = s_config.near_center;
    status->far_left = (uint16_t)(s_config.far_center -
                                  (s_config.far_width >> 1U));
    status->far_right = (uint16_t)(status->far_left +
                                   s_config.far_width - 1U);
    status->near_left = (uint16_t)(s_config.near_center -
                                   (s_config.near_width >> 1U));
    status->near_right = (uint16_t)(status->near_left +
                                    s_config.near_width - 1U);
    status->far_miss_count = s_far_miss_count;
    status->near_miss_count = s_near_miss_count;
    return true;
}

int x_axis_apply_fixed_roi(void)
{
    s_selected_field = X_AXIS_FIELD_FAR_START;
    s_ready = false;
    s_far_miss_count = 0U;
    s_near_miss_count = 0U;
    s_far_valid_count = 0U;
    s_far_trusted_valid = false;
    s_far_trusted_x = s_default_config.far_center;
    s_far_candidate_x = s_default_config.far_center;
    s_far_candidate_count = 0U;
    s_far_reacquire_active = false;
    s_near_trusted_valid = false;
    s_near_trusted_x = s_default_config.near_center;
    s_near_candidate_x = s_default_config.near_center;
    s_near_candidate_count = 0U;
    s_near_reacquire_active = false;

    if (!XAxis_ApplyConfig(&s_default_config, false))
    {
        xil_printf("ERROR: dual x_axis ROI configuration rejected\r\n");
        return XST_FAILURE;
    }

    s_ready = true;
    x_axis_roi_print_help();
    x_axis_roi_print_status();
    return XST_SUCCESS;
}

static void XAxis_Adjust(XAxisDualRoiConfig *trial,
                         int32_t direction,
                         bool large_step)
{
    uint16_t y_step = large_step ? X_AXIS_Y_STEP_LARGE :
                                   X_AXIS_Y_STEP_SMALL;
    uint16_t width_step = large_step ? X_AXIS_WIDTH_STEP_LARGE :
                                       X_AXIS_WIDTH_STEP_SMALL;

    switch (s_selected_field)
    {
        case X_AXIS_FIELD_FAR_START:
            trial->far_start = (direction < 0) ?
                XAxis_SubSaturate(trial->far_start, y_step, 0U) :
                XAxis_AddSaturate(trial->far_start, y_step,
                                  X_AXIS_IMAGE_HEIGHT - 1U);
            break;

        case X_AXIS_FIELD_NEAR_START:
            trial->near_start = (direction < 0) ?
                XAxis_SubSaturate(trial->near_start, y_step, 0U) :
                XAxis_AddSaturate(trial->near_start, y_step,
                                  X_AXIS_IMAGE_HEIGHT - 1U);
            break;

        case X_AXIS_FIELD_FAR_WIDTH:
            trial->far_width = (direction < 0) ?
                XAxis_SubSaturate(trial->far_width, width_step, 1U) :
                XAxis_AddSaturate(trial->far_width, width_step,
                                  X_AXIS_IMAGE_WIDTH - 2U);
            break;

        case X_AXIS_FIELD_NEAR_WIDTH:
            trial->near_width = (direction < 0) ?
                XAxis_SubSaturate(trial->near_width, width_step, 1U) :
                XAxis_AddSaturate(trial->near_width, width_step,
                                  X_AXIS_IMAGE_WIDTH - 2U);
            break;

        case X_AXIS_FIELD_FAR_CENTER:
            trial->far_center = (direction < 0) ?
                XAxis_SubSaturate(trial->far_center, width_step, 0U) :
                XAxis_AddSaturate(trial->far_center, width_step,
                                  X_AXIS_IMAGE_WIDTH - 1U);
            trial->far_center = XAxis_ClampCenter(trial->far_center,
                                                  trial->far_width);
            break;

        case X_AXIS_FIELD_NEAR_CENTER:
            trial->near_center = (direction < 0) ?
                XAxis_SubSaturate(trial->near_center, width_step, 0U) :
                XAxis_AddSaturate(trial->near_center, width_step,
                                  X_AXIS_IMAGE_WIDTH - 1U);
            trial->near_center = XAxis_ClampCenter(trial->near_center,
                                                   trial->near_width);
            break;

        case X_AXIS_FIELD_FAR_HEIGHT:
            if (direction < 0 && trial->far_size_log2 > 0U)
            {
                trial->far_size_log2--;
            }
            else if (direction > 0 &&
                     trial->far_size_log2 < X_AXIS_MAX_SIZE_LOG2)
            {
                trial->far_size_log2++;
            }
            break;

        case X_AXIS_FIELD_NEAR_HEIGHT:
            if (direction < 0 && trial->near_size_log2 > 0U)
            {
                trial->near_size_log2--;
            }
            else if (direction > 0 &&
                     trial->near_size_log2 < X_AXIS_MAX_SIZE_LOG2)
            {
                trial->near_size_log2++;
            }
            break;

        default:
            break;
    }
}

static void XAxis_ApplyTrial(const XAxisDualRoiConfig *trial)
{
    if (XAxis_ApplyConfig(trial, true))
    {
        x_axis_roi_print_status();
    }
    else
    {
        xil_printf("ROI update rejected: check image bounds and FAR/NEAR overlap\r\n");
    }
}

bool x_axis_roi_uart_handle(uint8_t command)
{
    XAxisDualRoiConfig trial;

    if (!s_ready)
    {
        return false;
    }

    trial = s_config;

    switch (command)
    {
        case (uint8_t)'r':
            s_selected_field =
                (XAxisRoiField)((uint32_t)s_selected_field + 1UL);
            if (s_selected_field >= X_AXIS_FIELD_COUNT)
            {
                s_selected_field = X_AXIS_FIELD_FAR_START;
            }
            xil_printf("ROI selected: %s\r\n",
                       XAxis_FieldName(s_selected_field));
            return true;

        case (uint8_t)',':
            XAxis_Adjust(&trial, -1, false);
            XAxis_ApplyTrial(&trial);
            return true;

        case (uint8_t)'.':
            XAxis_Adjust(&trial, 1, false);
            XAxis_ApplyTrial(&trial);
            return true;

        case (uint8_t)'<':
            XAxis_Adjust(&trial, -1, true);
            XAxis_ApplyTrial(&trial);
            return true;

        case (uint8_t)'>':
            XAxis_Adjust(&trial, 1, true);
            XAxis_ApplyTrial(&trial);
            return true;

        case (uint8_t)'z':
            s_far_miss_count = 0U;
            s_near_miss_count = 0U;
            s_far_valid_count = 0U;
            s_far_trusted_valid = false;
            s_far_trusted_x = s_default_config.far_center;
            s_far_candidate_x = s_default_config.far_center;
            s_far_candidate_count = 0U;
            s_far_reacquire_active = false;
            s_near_trusted_valid = false;
            s_near_trusted_x = s_default_config.near_center;
            s_near_candidate_x = s_default_config.near_center;
            s_near_candidate_count = 0U;
            s_near_reacquire_active = false;
            XAxis_ApplyTrial(&s_default_config);
            return true;

        case (uint8_t)'o':
            x_axis_roi_print_status();
            return true;

        case (uint8_t)'O':
            x_axis_roi_print_help();
            return true;

        default:
            return false;
    }
}

void x_axis_roi_print_status(void)
{
    uint32_t far_rows;
    uint32_t near_rows;
    uint32_t far_x_start;
    uint32_t far_x_end;
    uint32_t near_x_start;
    uint32_t near_x_end;

    if (!s_ready)
    {
        xil_printf("dual ROI is not initialized\r\n");
        return;
    }

    far_rows = XAxis_RowsFromSizeCode(s_config.far_size_log2);
    near_rows = XAxis_RowsFromSizeCode(s_config.near_size_log2);
    far_x_start = (uint32_t)s_config.far_center -
                  ((uint32_t)s_config.far_width >> 1U);
    far_x_end = far_x_start + s_config.far_width - 1UL;
    near_x_start = (uint32_t)s_config.near_center -
                   ((uint32_t)s_config.near_width >> 1U);
    near_x_end = near_x_start + s_config.near_width - 1UL;

    xil_printf("\r\n[dual ROI status]\r\n");
    xil_printf(" selected: %s\r\n", XAxis_FieldName(s_selected_field));
    xil_printf(" FAR : x=%u~%u center=%u width=%u, y=%u~%u rows=%u\r\n",
               (unsigned int)far_x_start,
               (unsigned int)far_x_end,
               (unsigned int)s_config.far_center,
               (unsigned int)s_config.far_width,
               (unsigned int)s_config.far_start,
               (unsigned int)((uint32_t)s_config.far_start + far_rows - 1UL),
               (unsigned int)far_rows);
    xil_printf(" NEAR: x=%u~%u center=%u width=%u, requested_y=%u actual_y=%u~%u rows=%u\r\n",
               (unsigned int)near_x_start,
               (unsigned int)near_x_end,
               (unsigned int)s_config.near_center,
               (unsigned int)s_config.near_width,
               (unsigned int)s_config.near_start,
               (unsigned int)s_effective_near_start,
               (unsigned int)((uint32_t)s_effective_near_start + near_rows - 1UL),
               (unsigned int)near_rows);
    xil_printf(" output: COMBINED=(3*FAR + 5*NEAR)/8\r\n");
    xil_printf(" tracking: max_move=%d recover_delay=%u recover_step=%d "
               "far_pred_gain=%d misses(F/N)=%u/%u\r\n",
               (int)X_AXIS_ROI_MAX_MOVE,
               (unsigned int)X_AXIS_ROI_RECOVER_DELAY,
               (int)X_AXIS_ROI_RECOVER_STEP,
               (int)X_AXIS_FAR_PRED_GAIN,
               (unsigned int)s_far_miss_count,
               (unsigned int)s_near_miss_count);
}

void x_axis_roi_print_help(void)
{
    xil_printf("\r\n[dual ROI UART control]\r\n");
    xil_printf(" r     : select FAR/NEAR Y, width, center, or height\r\n");
    xil_printf(" , / . : selected value -/+ small step\r\n");
    xil_printf(" < / > : selected value -/+ large step\r\n");
    xil_printf(" z     : restore startup FAR/NEAR ROI\r\n");
    xil_printf(" o / O : ROI status / help\r\n");
    xil_printf(" Y step=4/16, width step=8/32, height code 5=40 rows\r\n");
}
