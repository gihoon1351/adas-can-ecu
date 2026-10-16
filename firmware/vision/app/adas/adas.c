/*
 * adas.c
 *
 *  Created on: 2026. 9. 29.
 *      Author: kccistc
 */

#include <stddef.h>

#include "adas.h"

#include "../cnn/cnn_box_config.h"
#include "../cnn/cnn_policy.h"
#include "../uart_filter_control.h"
#include "../../driver/cnn_box/cnn_box_driver.h"
#include "../../hal/irq/irq_hal.h"
#include "../../hal/uart_log/uart_log_hal.h"
#include "adas_irq_config.h"
#define xil_printf UartLogHal_Printf

uint32_t s_cnn_output[APP_CNN_OUTPUT_CAPACITY_WORDS]
    __attribute__((aligned(64)));
CnnDriver s_cnn_driver;
CanHw s_can;
AdasCanApp s_adas;
bool s_can_initialized = false;

static CnnPolicy s_cnn_policy;
static CnnBoxDriver s_box_driver;
static bool s_cnn_ready = false;
static IrqHal s_irq;
static XAxisDriver s_lane_driver;
static uint32_t s_cnn_frame_number;
static uint32_t s_cnn_completed_count;
static uint32_t s_cnn_next_start_ms;
static uint32_t s_rate_ms;
static uint32_t s_rate_lane_count;
static uint32_t s_rate_cnn_irq_count;
static uint32_t s_rate_cnn_completed_count;
static uint32_t s_rate_tx_count;
static int8_t i = 0;
static int8_t preview_i = 0;

#define SOBEL_PREVIEW_AVG (5)

static const CanHwTiming s_can_timing = {10U, 11U, 2U, 2U};
static const AdasCanAppConfig s_can_config =
{
    1U, /* minimum spacing only; pending lane IRQ samples drive 60 Hz */
    100U,
    6000U,
    1000U
};

static int Sobel_queue[SOBEL_AVG];
static int Sobel_far_queue[SOBEL_PREVIEW_AVG];
static int Sobel_near_queue[SOBEL_PREVIEW_AVG];
static XAxisDualResult s_lane_roi_result;
static XAxisDualResult s_raw_lane_roi_result;
static uint16_t s_last_far_x = 640U;
static uint16_t s_last_near_x = 640U;

#define ADAS_CAN_ID_YCBCR_COMMAND (0x302U)
#define ADAS_CAN_ID_YCBCR_ACK     (0x303U)
#define YCBCR_COMMAND_SET         (1U)
#define YCBCR_COMMAND_GET         (2U)

static CanFrame s_ycbcr_ack;
static bool s_ycbcr_ack_pending;

static bool App_ShowHighestCnnBox(
    const CnnDetection detections[CNN_PP_NUM_CLASSES])
{
    CnnBoxCandidate candidates[CNN_PP_NUM_CLASSES] = {0};
    CnnBoxSelection selection = {0};
    CnnBoxDriverStatus status;
    uint32_t index;

    if (detections == NULL)
    {
        (void)CnnBoxDriver_Hide(&s_box_driver);
        return false;
    }

    /* CnnPostprocess_Run() leaves one threshold-passing result per class.
     * The box driver selects the single highest score among those five. */
    for (index = 0U; index < CNN_PP_NUM_CLASSES; ++index)
    {
        candidates[index].valid = (detections[index].valid != 0U);
        candidates[index].class_id = detections[index].class_id;
        candidates[index].score = detections[index].score;
        candidates[index].x0 = detections[index].x0;
        candidates[index].y0 = detections[index].y0;
        candidates[index].x1 = detections[index].x1;
        candidates[index].y1 = detections[index].y1;
    }

    status = CnnBoxDriver_ShowHighest(&s_box_driver,
                                      candidates,
                                      CNN_PP_NUM_CLASSES,
                                      &selection);
    if (status == CNN_BOX_DRIVER_OK)
    {
#if (APP_CNN_VERBOSE_LOG_ENABLE != 0U)
        const uint32_t percent =
            (uint32_t)(selection.score * 10000.0F + 0.5F);

        xil_printf("[BOX] class=%u(%s) score=%u.%02u%% "
                   "CNN=(%u,%u)-(%u,%u)\r\n",
                   (unsigned int)selection.class_id,
                   CnnPostprocess_ClassName(selection.class_id),
                   (unsigned int)(percent / 100U),
                   (unsigned int)(percent % 100U),
                   (unsigned int)selection.coordinates.x0,
                   (unsigned int)selection.coordinates.y0,
                   (unsigned int)selection.coordinates.x1,
                   (unsigned int)selection.coordinates.y1);
#endif
        return true;
    }

    if (status == CNN_BOX_DRIVER_NO_DETECTION)
    {
        /* This also removes a box left by the preceding frame. */
#if (APP_CNN_VERBOSE_LOG_ENABLE != 0U)
        xil_printf("[BOX] no threshold detection: hidden\r\n");
#endif
        return true;
    }

    (void)CnnBoxDriver_Hide(&s_box_driver);
    xil_printf("BOX_UPDATE_ERROR=%d\r\n", (int)status);
    return false;
}

static void App_PollYcbcrCanControl(void)
{
    CanFrame frame;
    uint8_t actual[6] = {0U};
    bool accepted = false;

    if (CanHw_Receive(&s_can, &frame) && frame.id == ADAS_CAN_ID_YCBCR_COMMAND &&
        frame.dlc == 8U) {
        if (frame.data[0] == YCBCR_COMMAND_SET) {
            accepted = UartFilterControl_SetThresholds(frame.data[1], frame.data[2],
                                                        frame.data[3], frame.data[4],
                                                        frame.data[5], frame.data[6]);
        } else if (frame.data[0] == YCBCR_COMMAND_GET) {
            accepted = true;
        }
        if (!UartFilterControl_GetThresholds(actual)) accepted = false;
        s_ycbcr_ack.id = ADAS_CAN_ID_YCBCR_ACK;
        s_ycbcr_ack.dlc = 8U;
        s_ycbcr_ack.data[0] = accepted ? 0U : 1U;
        s_ycbcr_ack.data[1] = actual[0];
        s_ycbcr_ack.data[2] = actual[1];
        s_ycbcr_ack.data[3] = actual[2];
        s_ycbcr_ack.data[4] = actual[3];
        s_ycbcr_ack.data[5] = actual[4];
        s_ycbcr_ack.data[6] = actual[5];
        s_ycbcr_ack.data[7] = frame.data[7];
        s_ycbcr_ack_pending = true;
        xil_printf("YCBCR CAN %s Y=[%u,%u] Cb=[%u,%u] Cr=[%u,%u] seq=%u\r\n",
                   accepted ? "APPLIED" : "REJECTED",
                   actual[0], actual[1], actual[2], actual[3], actual[4], actual[5],
                   frame.data[7]);
    }
    if (s_ycbcr_ack_pending && CanHw_Send(&s_can, &s_ycbcr_ack))
        s_ycbcr_ack_pending = false;
}

#if (APP_CNN_BENCHMARK_ENABLE != 0U)
static uint8_t s_cnn_sw_input[CNN_SW_INPUT_BYTE_COUNT]
    __attribute__((aligned(64)));
static uint32_t s_cnn_sw_output[CNN_PP_OA_WORDS]
    __attribute__((aligned(64)));
#endif

uint32_t App_NowMs(void)
{
    XTime ticks;
    uint64_t milliseconds;

    XTime_GetTime(&ticks);
    milliseconds = ((uint64_t)ticks * 1000ULL) /
                   (uint64_t)COUNTS_PER_SECOND;

    return (uint32_t)milliseconds;
}

void App_PrintCnnStatus(CnnHalSnapshot snapshot)
{
    xil_printf("CNN SR=%08x idle=%u busy=%u done=%u "
               "layer=%u out_group=%u in_group=%u param_loaded=%u "
               "param_loading=%u param_error=%u\r\n",
               (unsigned int)snapshot.raw,
               (unsigned int)snapshot.idle,
               (unsigned int)snapshot.busy,
               (unsigned int)snapshot.done,
               (unsigned int)snapshot.layer,
               (unsigned int)snapshot.output_group,
               (unsigned int)snapshot.input_group,
               (unsigned int)snapshot.params_loaded,
               (unsigned int)snapshot.params_loading,
               (unsigned int)snapshot.params_error);
}

void App_PollIo(uint32_t now_ms)
{
    static uint32_t last_log_ms = 0U;
    XAxisTrackingStatus tracking_status;
    bool tracking_status_valid;
    int32_t raw_x;
    uint32_t can_status;
    int32_t avg_x = 0;
    int32_t avg_far_x = 0;
    int32_t avg_near_x = 0;

    int8_t j = 0;
    XAxisDriverSample sample;

    UartFilterControl_Poll();
    App_PollYcbcrCanControl();

    if (XAxisDriver_Pop(&s_lane_driver, &sample))
    {
        s_lane_roi_result = sample.result;
        /* Preserve raw RTL coordinates for diagnostics. Tracking replaces a
         * FAR jump with the last trusted value and clears far_valid until a
         * stable two/three-frame cluster confirms the new trajectory. */
        s_raw_lane_roi_result = s_lane_roi_result;
        x_axis_tracking_update(&s_lane_roi_result);

        if (s_lane_roi_result.far_valid) s_last_far_x = s_lane_roi_result.far_x;
        if (s_lane_roi_result.near_valid) s_last_near_x = s_lane_roi_result.near_x;

        s_lane_roi_result.far_x = s_last_far_x;
        s_lane_roi_result.near_x = s_last_near_x;
        if (s_lane_roi_result.far_valid && s_lane_roi_result.near_valid)
        {
            s_lane_roi_result.combined_x =
                (uint16_t)((3UL * s_last_far_x +
                            5UL * s_last_near_x + 4UL) >> 3U);
        }
        else if (s_lane_roi_result.near_valid)
        {
            s_lane_roi_result.combined_x = s_last_near_x;
        }
        else if (s_lane_roi_result.far_valid)
        {
            s_lane_roi_result.combined_x = s_last_far_x;
        }
        else
        {
            s_lane_roi_result.combined_x = 640U;
        }
        Sobel_queue[i] = (int32_t)s_lane_roi_result.combined_x;
        i++;
        if (i >= SOBEL_AVG) i = 0;

        Sobel_far_queue[preview_i] = (int32_t)s_lane_roi_result.far_x;
        Sobel_near_queue[preview_i] = (int32_t)s_lane_roi_result.near_x;
        preview_i++;
        if (preview_i >= SOBEL_PREVIEW_AVG) preview_i = 0;

        for (j = 0;j < SOBEL_AVG;j++)
        {
            avg_x += Sobel_queue[j];
        }
        for (j = 0;j < SOBEL_PREVIEW_AVG;j++)
        {
            avg_far_x += Sobel_far_queue[j];
            avg_near_x += Sobel_near_queue[j];
        }


        raw_x = avg_x / SOBEL_AVG;
        avg_far_x /= SOBEL_PREVIEW_AVG;
        avg_near_x /= SOBEL_PREVIEW_AVG;
        avg_x = 0;
        if ((raw_x >= 0) && (raw_x <= (int32_t)ADAS_X_MAX) &&
            (avg_far_x >= 0) && (avg_far_x <= (int32_t)ADAS_X_MAX) &&
            (avg_near_x >= 0) && (avg_near_x <= (int32_t)ADAS_X_MAX))
        {
            if (!AdasCanApp_UpdateLane(&s_adas, (uint16_t)raw_x,
                                       (uint16_t)avg_far_x,
                                       (uint16_t)avg_near_x,
                                       s_lane_roi_result.far_valid,
                                       s_lane_roi_result.near_valid,
                                       sample.captured_ms))
            {
                xil_printf("Sobel x update failed: %d\r\n", (int)raw_x);
            }
        }
        else
        {
            AdasCanApp_ClearX(&s_adas);
            xil_printf("Invalid Sobel x: %d\r\n", (int)raw_x);
        }
    }

    AdasCanApp_Poll(&s_adas, now_ms);
    UartLogHal_Poll();

    if ((uint32_t)(now_ms - s_rate_ms) >= APP_RATE_LOG_PERIOD_MS)
    {
        xil_printf("[RATE] dt_ms=%u sobel_irq=%u cnn_irq=%u cnn_done=%u "
                   "can_tx=%u q_drop=%u coalesce=%u log_drop=%u\r\n",
                   (unsigned int)(now_ms - s_rate_ms),
                   (unsigned int)(s_lane_driver.irq_count - s_rate_lane_count),
                   (unsigned int)(s_cnn_driver.done_irq_count - s_rate_cnn_irq_count),
                   (unsigned int)(s_cnn_completed_count - s_rate_cnn_completed_count),
                   (unsigned int)(s_adas.result_sent_count - s_rate_tx_count),
                   (unsigned int)s_lane_driver.dropped_count,
                   (unsigned int)s_adas.result_replaced_count,
                   (unsigned int)UartLogHal_GetDroppedCount());
        s_rate_ms = now_ms;
        s_rate_lane_count = s_lane_driver.irq_count;
        s_rate_cnn_irq_count = s_cnn_driver.done_irq_count;
        s_rate_cnn_completed_count = s_cnn_completed_count;
        s_rate_tx_count = s_adas.result_sent_count;
    }

    if ((uint32_t)(now_ms - last_log_ms) >= APP_CAN_LOG_PERIOD_MS)
    {
        can_status = CanHw_Status(&s_can);
        tracking_status_valid = x_axis_tracking_get_status(&tracking_status);
        xil_printf("CAN SR=%08x ready=%u busoff=%u txerr=%u "
                   "hb=%u x_valid=%u x=%u far=%u near=%u fv=%u nv=%u "
                   "detect_f=%u detect_n=%u roi_fc=%u roi_nc=%u "
                   "roi_f=%u~%u roi_n=%u~%u miss_f=%u miss_n=%u result_at=%u\r\n",
                   (unsigned int)can_status,
                   (unsigned int)((can_status >> 3U) & 1U),
                   (unsigned int)(can_status & 1U),
                   (unsigned int)((can_status >> 1U) & 1U),
                   (unsigned int)s_adas.heartbeat_counter,
                   (unsigned int)s_adas.x_valid,
                   (unsigned int)s_adas.x,
                   (unsigned int)s_lane_roi_result.far_x,
                   (unsigned int)s_lane_roi_result.near_x,
                   (unsigned int)s_lane_roi_result.far_valid,
                   (unsigned int)s_lane_roi_result.near_valid,
                   (unsigned int)s_raw_lane_roi_result.far_x,
                   (unsigned int)s_raw_lane_roi_result.near_x,
                   (unsigned int)(tracking_status_valid ? tracking_status.far_center : 0U),
                   (unsigned int)(tracking_status_valid ? tracking_status.near_center : 0U),
                   (unsigned int)(tracking_status_valid ? tracking_status.far_left : 0U),
                   (unsigned int)(tracking_status_valid ? tracking_status.far_right : 0U),
                   (unsigned int)(tracking_status_valid ? tracking_status.near_left : 0U),
                   (unsigned int)(tracking_status_valid ? tracking_status.near_right : 0U),
                   (unsigned int)(tracking_status_valid ? tracking_status.far_miss_count : 0U),
                   (unsigned int)(tracking_status_valid ? tracking_status.near_miss_count : 0U),
                   (unsigned int)s_adas.result_sent_ms);
        last_log_ms = now_ms;
    }
}

static bool App_ProcessCnnResult(uint32_t frame_number,
                                 const CnnDriverResult *result)
{
    CnnPolicyDecision decision;
    CnnResultSelection selection;
    CnnResultAppStatus can_result;
    const CnnDetection *detections;
    uint32_t candidate_count;
    uint32_t class_id;

    CnnPostprocess_Run(
        (const u32 *)CnnDriver_GetOutput(&s_cnn_driver));
    detections = CnnPostprocess_GetDetections();

    if (!App_ShowHighestCnnBox(detections))
    {
        AdasCanApp_ClearDetection(&s_adas);
        CnnPolicy_Reset(&s_cnn_policy);
        return false;
    }

    candidate_count = 0U;
    for (class_id = 0U; class_id < CNN_PP_NUM_CLASSES; ++class_id)
    {
        if (detections[class_id].valid != 0U)
        {
            ++candidate_count;
        }
    }
    decision = CnnPolicy_Update(&s_cnn_policy,
                                detections,
                                CNN_PP_NUM_CLASSES);

    xil_printf("CNN frame=%u inference=%u us IRQ=%u us OA=%08x candidates=%u\r\n",
               (unsigned int)frame_number,
               (unsigned int)result->elapsed_us,
               (unsigned int)result->irq_elapsed_us,
               (unsigned int)result->output_checksum,
               (unsigned int)candidate_count);
#if (APP_CNN_VERBOSE_LOG_ENABLE != 0U)
    CnnPostprocess_Print(detections);
#endif

    if (!decision.confirmed)
    {
        /* A weak, missing or unconfirmed result is transmitted as lane only. */
        AdasCanApp_ClearDetection(&s_adas);
        xil_printf("CNN -> CAN: LANE_ONLY (%s)\r\n",
                   decision.uncertain ? "unconfirmed candidate" : "no candidate");
        return true;
    }

    can_result = CnnResultApp_UpdateCan(&s_adas,
                                        &decision.detection,
                                        1U,
                                        App_NowMs(),
                                        &selection);
    if (can_result != CNN_RESULT_APP_UPDATED)
    {
        AdasCanApp_ClearDetection(&s_adas);
        xil_printf("CNN -> CAN update failed: %d\r\n", (int)can_result);
        return false;
    }

    xil_printf("CNN -> CAN class=%u(%s) score=%u.%02u%% y=(%u,%u)\r\n",
               (unsigned int)selection.model_class,
               CnnPostprocess_ClassName(selection.model_class),
               (unsigned int)(selection.score * 10000.0f + 0.5f) / 100U,
               (unsigned int)(selection.score * 10000.0f + 0.5f) % 100U,
               (unsigned int)selection.y0,
               (unsigned int)selection.y1);
    return true;
}

static void App_StopCnn(CnnDriverStatus status)
{
    /* A CNN fault must NOT stop lane IRQ capture or CAN heartbeat/result. */
    IrqHal_Disable(&s_irq, APP_CNN_IRQ_ID);
    s_cnn_ready = false;
    (void)CnnBoxDriver_Hide(&s_box_driver);
    AdasCanApp_ClearDetection(&s_adas);
    CnnPolicy_Reset(&s_cnn_policy);
    xil_printf("CNN frame %u FAILED: driver=%d dma=%d; lane/CAN continue\r\n",
               (unsigned int)s_cnn_frame_number, (int)status,
               (int)CnnDriver_GetLastDmaStatus(&s_cnn_driver));
    App_PrintCnnStatus(CnnDriver_GetHardwareStatus(&s_cnn_driver));
}

void App_ServiceCnn(uint32_t now_ms)
{
    CnnDriverStatus status;
    CnnDriverResult result;
    uint32_t late_ms;
    if (!s_cnn_ready) return;
    if (CnnDriver_IsActive(&s_cnn_driver))
    {
        status = CnnDriver_TryFinish(&s_cnn_driver, &result);
        if (status == CNN_DRIVER_STATUS_IN_PROGRESS) return;
        if (status != CNN_DRIVER_STATUS_OK)
        {
            App_StopCnn(status);
            return;
        }
        ++s_cnn_completed_count;
        if (!App_ProcessCnnResult(s_cnn_frame_number, &result))
        {
            App_StopCnn(CNN_DRIVER_STATUS_INVALID_ARGUMENT);
            return;
        }
        /* Service a lane IRQ that arrived during OA decoding BEFORE restart. */
        App_PollIo(App_NowMs());
        now_ms = App_NowMs();
    }
    /* Unsigned deadline comparison is wrap-safe for intervals < 2^31 ms. */
    late_ms = (uint32_t)(now_ms - s_cnn_next_start_ms);
    if (late_ms >= 0x80000000UL) return;
    status = CnnDriver_StartAsync(&s_cnn_driver);
    if (status == CNN_DRIVER_STATUS_BUSY)
    {
        if (late_ms >= APP_CNN_INFERENCE_TIMEOUT_MS)
            App_StopCnn(CNN_DRIVER_STATUS_IDLE_TIMEOUT);
        return;
    }
    if (status != CNN_DRIVER_STATUS_OK)
    {
        App_StopCnn(status);
        return;
    }
    ++s_cnn_frame_number;
    /* Phase-based target cadence; skip missed slots rather than burst.
     * If inference+postprocessing exceeds 50 ms, actual frequency is lower. */
    s_cnn_next_start_ms += ((late_ms / APP_CNN_PERIOD_MS) + 1U) *
                           APP_CNN_PERIOD_MS;
}

static bool App_InitInterrupts(void)
{
    if (APP_X_AXIS_IRQ_ID == APP_CNN_IRQ_ID ||
        !IrqHal_Init(&s_irq, (uint16_t)APP_GIC_DEVICE_ID) ||
        !XAxisDriver_Init(&s_lane_driver, X_AXIS0) ||
        !IrqHal_Connect(&s_irq, APP_X_AXIS_IRQ_ID,
                       XAxisDriver_IrqHandler, &s_lane_driver,
                       APP_X_AXIS_IRQ_PRIORITY, APP_X_AXIS_IRQ_TRIGGER))
        return false;
    if (s_cnn_ready && !IrqHal_Connect(&s_irq, APP_CNN_IRQ_ID,
                                      CnnDriver_IrqHandler, &s_cnn_driver,
                                      APP_CNN_IRQ_PRIORITY, APP_CNN_IRQ_TRIGGER))
        return false;
    xil_printf("IRQ initialized: x_axis=%u trigger=%u CNN=%u trigger=%u\r\n",
               (unsigned int)APP_X_AXIS_IRQ_ID, (unsigned int)APP_X_AXIS_IRQ_TRIGGER,
               (unsigned int)APP_CNN_IRQ_ID, (unsigned int)APP_CNN_IRQ_TRIGGER);
    xil_printf("Sobel: one fresh 0x301/DLC8 per lane IRQ (60 Hz video); "
               "CNN: 50 ms target; heartbeat: 100 ms\r\n");
    s_cnn_next_start_ms = App_NowMs();
    s_rate_ms = s_cnn_next_start_ms;
    UartLogHal_SetAsync(true);
    /* Connect and clear peripherals BEFORE enabling their GIC sources/CPU. */
    XAxisDriver_Start(&s_lane_driver);
    IrqHal_Enable(&s_irq, APP_X_AXIS_IRQ_ID);
    if (s_cnn_ready)
    {
        CnnHal_ClearDone(&s_cnn_driver.cnn);
        IrqHal_Enable(&s_irq, APP_CNN_IRQ_ID);
    }
    IrqHal_EnableCpu();
    return true;
}

bool App_Init(void)
{
    CnnDriverConfig cnn_config = {0};
    CnnDriverStatus cnn_status;
    CnnBoxDriverStatus box_status;
#if (APP_CNN_BENCHMARK_ENABLE != 0U)
    CnnBenchmarkStats hw_benchmark_stats;
    CnnBenchmarkStats sw_benchmark_stats;
    CnnBenchmarkStatus hw_benchmark_status;
    CnnBenchmarkStatus sw_benchmark_status;
    CnnSwReferenceContext sw_context;
#endif
    bool success = false;

    uint8_t queue_index;

    s_cnn_ready = false;
    s_cnn_frame_number = 0U;
    s_cnn_completed_count = 0U;
    CnnPolicy_Reset(&s_cnn_policy);

    for (queue_index = 0U; queue_index < SOBEL_AVG; ++queue_index)
    {
        Sobel_queue[queue_index] = 0;
    }
    for (preview_i = 0; preview_i < SOBEL_PREVIEW_AVG; preview_i++)
    {
        Sobel_far_queue[preview_i] = 0;
        Sobel_near_queue[preview_i] = 0;
    }
    /* The initialization loops leave their indices at the array length. */
    i = 0;
    preview_i = 0;

    /* Program all filter settings before video produces its first SOF. */
    if (UartFilterControl_Init() != XST_SUCCESS)
    {
        xil_printf("FILTER_INIT_ERROR\r\n");
    }
    else if (pcam_init() != XST_SUCCESS)
    {
        xil_printf("PCAM_INIT_ERROR\r\n");
    }
    else if ((box_status = CnnBoxDriver_Init(
                  &s_box_driver,
                  (UINTPTR)APP_CNN_BOX_BASEADDR,
                  APP_CNN_BOX_COLOR,
                  (uint8_t)APP_CNN_BOX_THICKNESS)) != CNN_BOX_DRIVER_OK)
    {
        xil_printf("BOX_INIT_ERROR=%d base=%08x; "
                   "check bitstream/XSA/video path\r\n",
                   (int)box_status,
                   (unsigned int)APP_CNN_BOX_BASEADDR);
    }
    else
    {
        xil_printf("BOX base=%08x: highest threshold detection, "
                   "red %u-pixel outline\r\n",
                   (unsigned int)APP_CNN_BOX_BASEADDR,
                   (unsigned int)APP_CNN_BOX_THICKNESS);

        /* Sobel �썑�떒 x醫뚰몴 寃�異� ROI �꽕�젙 */
        if (x_axis_apply_fixed_roi() != XST_SUCCESS)
        {
            xil_printf("X_AXIS_ROI_ERROR\r\n");

        }

        /* ROI �쟻�슜怨� �궡遺� �씠�젰 珥덇린�솕媛� �걹�궃 �떎�쓬 �솢�꽦�솕 */
        x_axis_enable(X_AXIS0);

        if (!CanHw_Init(&s_can,
                       XPAR_CAN_CONTROLLER_V1_0_0_BASEADDR,
                       s_can_timing))
        {
            xil_printf("CAN_HW_INIT_ERROR\r\n");
        }
        else if (!AdasCanApp_Init(&s_adas,
                                  &s_can_config,
                                  CanHw_Send,
                                  &s_can,
                                  App_NowMs()))
        {
            xil_printf("CAN_APP_INIT_ERROR\r\n");
        }
        else
        {
            s_can_initialized = true;
            (void)AdasCanApp_SetState(&s_adas, ADAS_ECU_RUN);

            /* Lane/CAN remain operational if optional CNN initialization fails. */
            success = true;

            cnn_config.cnn_base_address =
                (UINTPTR)XPAR_CNN_V1_0_0_BASEADDR;
            cnn_config.dma_device_id =
                (uint16_t)APP_CNN_DMA_DEVICE_ID;
            cnn_config.param_dma_device_id =
                (uint16_t)APP_CNN_PARAM_DMA_DEVICE_ID;
            cnn_config.param_words = cnn_param_data;
            cnn_config.param_byte_count = CNN_PARAM_BYTE_COUNT;
            cnn_config.param_timeout_ms = APP_CNN_PARAM_TIMEOUT_MS;
            cnn_config.weight_words =
                (const uint32_t *)g_cnn_weight_words;
            cnn_config.weight_byte_count = CNN_WEIGHT_BYTE_COUNT;
            cnn_config.output_word_count = CNN_PP_OA_WORDS;
            cnn_config.idle_timeout_ms = APP_CNN_IDLE_TIMEOUT_MS;
            cnn_config.inference_timeout_ms =
                APP_CNN_INFERENCE_TIMEOUT_MS;

            cnn_status = CnnDriver_Init(&s_cnn_driver,
                                        &cnn_config,
                                        s_cnn_output,
                                        APP_CNN_OUTPUT_CAPACITY_WORDS);
            if (cnn_status != CNN_DRIVER_STATUS_OK)
            {
                xil_printf("CNN_INIT_ERROR: driver=%d dma=%d "
                           "param_dma_sr=%08x param_cnn_sr=%08x\r\n",
                           (int)cnn_status,
                           (int)CnnDriver_GetLastDmaStatus(&s_cnn_driver),
                           (unsigned int)CnnDriver_GetParameterDmaStatus(&s_cnn_driver),
                           (unsigned int)CnnDriver_GetParameterCnnStatus(&s_cnn_driver));
            }
            else
            {
                xil_printf("CNN_PARAM_INIT_OK: DMA1 bias+scale %u bytes loaded once\r\n",
                           (unsigned int)CNN_PARAM_BYTE_COUNT);
                CnnPostprocess_SetLayout(CNN_PP_LAYOUT_GROUP_MAJOR);
                CnnPolicy_Reset(&s_cnn_policy);
                s_cnn_ready = true;
                App_PrintCnnStatus(
                    CnnDriver_GetHardwareStatus(&s_cnn_driver));
#if (APP_CNN_BENCHMARK_ENABLE != 0U)
                hw_benchmark_status = CnnBenchmark_RunHardware(
                    &s_cnn_driver,
                    APP_CNN_HW_BENCHMARK_WARMUP_RUNS,
                    APP_CNN_HW_BENCHMARK_RUNS,
                    &hw_benchmark_stats);
                CnnBenchmark_PrintStats("HW", &hw_benchmark_stats);
                if (hw_benchmark_status != CNN_BENCHMARK_STATUS_OK)
                {
                    xil_printf("CNN HW benchmark failed: %d\r\n",
                               (int)hw_benchmark_status);
                }

                CnnSwReference_FillTestInput(
                    s_cnn_sw_input, CNN_SW_INPUT_BYTE_COUNT);
                (void)CnnSwReference_Init();
                sw_context.input = s_cnn_sw_input;
                sw_context.input_byte_count = CNN_SW_INPUT_BYTE_COUNT;

                sw_benchmark_status = CnnBenchmark_RunSoftware(
                    CnnSwReference_BenchmarkRun,
                    &sw_context,
                    s_cnn_sw_output,
                    CNN_PP_OA_WORDS,
                    APP_CNN_SW_BENCHMARK_WARMUP_RUNS,
                    APP_CNN_SW_BENCHMARK_RUNS,
                    &sw_benchmark_stats);
                CnnBenchmark_PrintStats("SW", &sw_benchmark_stats);
                if (sw_benchmark_status != CNN_BENCHMARK_STATUS_OK)
                {
                    xil_printf("CNN SW benchmark failed: %d\r\n",
                               (int)sw_benchmark_status);
                }

                if ((hw_benchmark_status == CNN_BENCHMARK_STATUS_OK) &&
                    (sw_benchmark_status == CNN_BENCHMARK_STATUS_OK))
                {
                    CnnBenchmark_PrintSpeedup(&sw_benchmark_stats,
                                              &hw_benchmark_stats);
                }
#endif
                xil_printf("CAN result ID=0x301 DLC=8 "
                           "[xL,xH,class,y0,y1,farL,nearL,farH|nearH]\r\n");
                success = true;
            }
        }
    }

    if (success && !App_InitInterrupts())
    {
        xil_printf("IRQ_INIT_ERROR: verify GIC IDs/trigger settings\r\n");
        success = false;
    }
    return success;
}
