#ifndef X_AXIS_H
#define X_AXIS_H

#include <stdbool.h>
#include <stdint.h>

#include "xil_types.h"
#include "xparameters.h"

/* Dual-ROI x_axis_finder register map.
 *
 * 0x00 CR0: EN, IRQ_EN, DONE_CLEAR, FAR height/start/width
 * 0x04 CR1: NEAR height/start/width
 * 0x08 SR read: DONE [0], FAR center [11:1], NEAR center [27:17]
 *      CENTER write: FAR center [10:0], NEAR center [26:16]
 * 0x0C DR : FAR X [10:0], FAR_VALID [15], NEAR X [26:16], NEAR_VALID [31]
 */
typedef struct
{
    volatile uint32_t CR0;
    volatile uint32_t CR1;
    volatile uint32_t SR;
    volatile uint32_t DR;
} X_AXIS_TypeDef;

#if defined(XPAR_AXI_X_AXIS_FINDER_V1_0_0_BASEADDR)
#define X_AXIS_BASEADDR XPAR_AXI_X_AXIS_FINDER_V1_0_0_BASEADDR
#elif defined(XPAR_AXI_X_AXIS_FINDER_V1_0_BASEADDR)
#define X_AXIS_BASEADDR XPAR_AXI_X_AXIS_FINDER_V1_0_BASEADDR
#elif defined(XPAR_AXI_X_AXIS_FINDER_8_0_BASEADDR)
/* Build-time bridge for the current BSP.  The resulting ELF must only be
 * used after the dual-ROI hardware replaces the legacy IP at this address. */
#define X_AXIS_BASEADDR XPAR_AXI_X_AXIS_FINDER_8_0_BASEADDR
#else
#error "dual-ROI x_axis_finder base address is missing from xparameters.h"
#endif

#define X_AXIS0 ((X_AXIS_TypeDef *)(UINTPTR)X_AXIS_BASEADDR)

#define X_AXIS_CR0_EN_MASK          (0x00000001UL)
#define X_AXIS_CR0_IRQ_EN_MASK      (0x00000002UL)
#define X_AXIS_CR0_DONE_CLEAR_MASK  (0x00000004UL)
#define X_AXIS_CR0_FAR_SIZE_SHIFT   (3U)
#define X_AXIS_CR0_FAR_SIZE_MASK    (0x00000038UL)
#define X_AXIS_CR0_FAR_START_SHIFT  (6U)
#define X_AXIS_CR0_FAR_START_MASK   (0x0000FFC0UL)
#define X_AXIS_CR0_FAR_WIDTH_SHIFT  (16U)
#define X_AXIS_CR0_FAR_WIDTH_MASK   (0x07FF0000UL)

#define X_AXIS_CR1_NEAR_SIZE_SHIFT  (0U)
#define X_AXIS_CR1_NEAR_SIZE_MASK   (0x00000007UL)
#define X_AXIS_CR1_NEAR_START_SHIFT (3U)
#define X_AXIS_CR1_NEAR_START_MASK  (0x00001FF8UL)
#define X_AXIS_CR1_NEAR_WIDTH_SHIFT (13U)
#define X_AXIS_CR1_NEAR_WIDTH_MASK  (0x00FFE000UL)

#define X_AXIS_SR_DONE_MASK         (0x00000001UL)
#define X_AXIS_DR_FAR_MASK          (0x000007FFUL)
#define X_AXIS_DR_FAR_VALID_MASK    (0x00008000UL)
#define X_AXIS_DR_NEAR_MASK         (0x07FF0000UL)
#define X_AXIS_DR_NEAR_VALID_MASK   (0x80000000UL)
#define X_AXIS_DR_NEAR_SHIFT        (16U)

#define X_AXIS_CENTER_FAR_MASK        (0x000007FFUL)
#define X_AXIS_CENTER_NEAR_MASK       (0x07FF0000UL)
#define X_AXIS_CENTER_NEAR_SHIFT      (16U)
#define X_AXIS_SR_FAR_CENTER_SHIFT    (1U)
#define X_AXIS_SR_NEAR_CENTER_SHIFT   (17U)

/* Adaptive tracking tuning.  All updates use integer arithmetic. */
#define X_AXIS_ROI_LPF_OLD_WEIGHT   (8L)
#define X_AXIS_ROI_LPF_NEW_WEIGHT   (2L)
#define X_AXIS_ROI_LPF_WEIGHT_SUM   (10L)
#define X_AXIS_ROI_MAX_MOVE         (12L)
#define X_AXIS_ROI_RECOVER_DELAY    (8U)
#define X_AXIS_ROI_RECOVER_STEP     (8L)
#define X_AXIS_FAR_PRED_GAIN        (0L)

/* NEAR always observes the wide bottom band. FAR alone tracks center/width
 * so background edges are rejected without sacrificing lateral recovery. */
#define X_AXIS_NEAR_WIDTH_FIXED       (1240U)
#define X_AXIS_FAR_WIDTH_MIN          (800U)
#define X_AXIS_FAR_WIDTH_MAX          (980U)
#define X_AXIS_WIDTH_SHRINK_DELAY     (12U)
#define X_AXIS_WIDTH_EXPAND_DELAY     (2U)
#define X_AXIS_FAR_WIDTH_SHRINK_STEP  (4U)
#define X_AXIS_FAR_WIDTH_EDGE_STEP    (16U)
#define X_AXIS_FAR_WIDTH_MISS_STEP    (24U)
#define X_AXIS_WIDTH_EDGE_GUARD       (96U)

/* FAR temporal trust gate. A one-frame jump is withheld from both ROI
 * tracking and VCU control. Persistent clustered detections are accepted as
 * a real curve and approached with the same 12 px/frame bound. */
#define X_AXIS_FAR_NORMAL_DELTA        (60U)
#define X_AXIS_FAR_LARGE_JUMP_DELTA    (140U)
#define X_AXIS_FAR_CANDIDATE_CLUSTER   (25U)
#define X_AXIS_FAR_MODERATE_CONFIRM    (2U)
#define X_AXIS_FAR_LARGE_CONFIRM       (3U)

/* NEAR stays full-width, but its detected coordinate is filtered separately.
 * Ordinary motion uses the same 80/20 LPF and 12 px/frame slew limit. A
 * larger change must repeat before it can affect the combined control value. */
#define X_AXIS_NEAR_NORMAL_DELTA        (45U)
#define X_AXIS_NEAR_LARGE_JUMP_DELTA    (140U)
#define X_AXIS_NEAR_CANDIDATE_CLUSTER   (30U)
#define X_AXIS_NEAR_MODERATE_CONFIRM    (2U)
#define X_AXIS_NEAR_LARGE_CONFIRM       (3U)

typedef struct
{
    uint16_t far_width;
    uint16_t near_width;
    uint16_t far_center;
    uint16_t near_center;
    uint16_t far_start;
    uint16_t near_start;
    uint8_t far_size_log2;
    uint8_t near_size_log2;
} XAxisDualRoiConfig;

typedef struct
{
    uint16_t far_x;
    uint16_t near_x;
    uint16_t combined_x;
    bool far_valid;
    bool near_valid;
} XAxisDualResult;

typedef struct
{
    uint16_t far_center;
    uint16_t near_center;
    uint16_t far_left;
    uint16_t far_right;
    uint16_t near_left;
    uint16_t near_right;
    uint16_t far_miss_count;
    uint16_t near_miss_count;
} XAxisTrackingStatus;

/* Programs the startup FAR/NEAR layout (maxima are exclusive):
 * FAR  x=[150,1130), y=[420,460)
 * NEAR x=[20,1260), y=[530,594)
 * FAR size code 5 is the dual-ROI 40-row mode; NEAR code 6 is 64 rows.
 */
int x_axis_apply_fixed_roi(void);

void x_axis_enable(X_AXIS_TypeDef *instance);
void x_axis_disable(X_AXIS_TypeDef *instance);
void x_axis_irq_enable(X_AXIS_TypeDef *instance);
void x_axis_irq_disable(X_AXIS_TypeDef *instance);
void x_axis_done_clear(X_AXIS_TypeDef *instance);
bool x_axis_read_done(const X_AXIS_TypeDef *instance);
bool x_axis_read_dual_result(X_AXIS_TypeDef *instance,
                             XAxisDualResult *result);
/* Updates next-frame ROI tracking and replaces FAR in result with the trusted
 * temporal value. Unconfirmed FAR jumps are returned with far_valid=false. */
void x_axis_tracking_update(XAxisDualResult *result);
bool x_axis_tracking_get_status(XAxisTrackingStatus *status);

/* Non-conflicting UART ROI controls:
 * r select, ,/. small -/+, </> large -/+, z reset, o status, O help. */
bool x_axis_roi_uart_handle(uint8_t command);
void x_axis_roi_print_status(void);
void x_axis_roi_print_help(void);

#endif /* X_AXIS_H */
