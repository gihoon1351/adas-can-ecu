#ifndef SRC_DRIVER_VCU_DRIVER_VCU_H_
#define SRC_DRIVER_VCU_DRIVER_VCU_H_

#include <stdint.h>
#include <stdbool.h>

#define LANE_GAIN_SCALE (100)
/* D-only diagnostic gains. Values are fixed-point gains divided by
 * LANE_GAIN_SCALE. This setting is not intended as final lane tracking. */
// #define KP_100RPM_SCALED (15) /* Kp = 0.00 */
// #define KD_100RPM_SCALED (0) /* Kd = 0.15 */
// #define KP_150RPM_SCALED (15) /* Kp = 0.00 */
// #define KD_150RPM_SCALED (0) /* Kd = 0.15 */
// #define KP_200RPM_SCALED (15) /* Kp = 0.00 */
// #define KD_200RPM_SCALED (0) /* Kd = 0.15; also used above 200 RPM */

#define KP_100RPM_SCALED (15) /* Kp = 0.15 */
#define KD_100RPM_SCALED (20) /* Kd = 0.20, unwind damping only */
#define KP_150RPM_SCALED (25) /* Kp = 0.25 */
#define KD_150RPM_SCALED (20) /* Kd = 0.20 */
#define KP_200RPM_SCALED (50) /* Kp = 0.50 */
#define KD_200RPM_SCALED (20) /* Kd = 0.20; also used above 200 RPM */


#define LANE_CENTER_X (646U)
#define LANE_ERROR_DEADBAND_PIXELS (15)
#define VEHICLE_MAX_RPM (200U)

/* Time-normalized lane motion used by the D term.  Expressing motion in
 * pixels/second makes the controller independent of ADAS frame interval. */
#define LANE_RATE_FILTER_DIVISOR (4)
#define LANE_RATE_MAX_PIXELS_PER_SEC (1500)
#define LANE_RATE_MAX_VALID_DT_MS (250U)
#define LANE_DERIVATIVE_REFERENCE_MS (100U)
#define LANE_UNWIND_D_MAX_RPM (8)

/* Reject a one-frame jump to a different Sobel line. A persistent new lane
 * must form a stable cluster before the controller follows it gradually. */
#define LANE_JUMP_MARGIN_PIXELS (80U)
#define LANE_REACQUIRE_CLUSTER_PIXELS (15U)
#define LANE_REACQUIRE_CONFIRM_FRAMES (4U)
#define LANE_REACQUIRE_STEP_PIXELS (20U)
#define LANE_DIRECTION_SWITCH_MARGIN_PIXELS (35U)

/* A sudden jump from a strong curve to image center is treated as a lost
 * lane, not as an immediate straight command. */
#define LANE_DROPOUT_PREVIOUS_ERROR_PIXELS (80U)
#define LANE_DROPOUT_JUMP_PIXELS (120U)
#define LANE_DROPOUT_HOLD_MS (300U)
#define LANE_DROPOUT_DECAY_MS (700U)
#define LANE_DROPOUT_STOP_MS (1500U)
#define LANE_DROPOUT_RECOVERY_FRAMES (2U)
#define LANE_DROPOUT_BASE_RPM (80U)
/* During a short lane dropout, retain the filtered NEAR correction together
 * with distance-indexed FAR feed-forward. Keep the combined remembered
 * steering below the normal 35 RPM authority. */
#define LANE_MEMORY_MAX_CORRECTION_RPM (30)
/* While the lane is missing, progressively steer toward the side where the
 * last trusted lane center was seen. This active-search bias stays below the
 * normal MEMORY steering limit and is removed as soon as RECOVER begins. */
#define LANE_MEMORY_SEARCH_START_RPM (10)
#define LANE_MEMORY_SEARCH_MAX_RPM (26)
#define LANE_CURVE_ERROR_PIXELS (20)
#define LANE_CURVE_BASE_RPM (50U)
#define LANE_FAR_CURVE_ENTER_PIXELS (45)
#define LANE_FAR_CURVE_EXIT_PIXELS (25)
#define LANE_FAR_NEAR_ENTER_PIXELS (60)
#define LANE_FAR_NEAR_EXIT_PIXELS (35)
#define LANE_CURVE_RELEASE_FRAMES (3U)
#define LANE_CURVE_MIN_CORRECTION_RPM (30)
#define LANE_DIRECTION_CONFIRM_FRAMES (2U)
#define LANE_UNTRUSTED_BASE_RPM (70U)

/* NEAR is the primary lateral-position signal. FAR supplies preview, but its
 * contribution is bounded when it locks onto a background edge on a curve. */
#define LANE_PREVIEW_FAR_WEIGHT (3U)
#define LANE_PREVIEW_NEAR_WEIGHT (5U)
#define LANE_FAR_NEAR_MAX_FUSION_DELTA_PIXELS (120)

/* Distance-indexed path memory. The camera starts seeing the road roughly
 * 150 mm ahead of the vehicle, so FAR curvature is written five 30 mm slots
 * into the future. Runtime tuning can refine these installation-dependent
 * distances without rebuilding the firmware. */
#define PATH_MEM_SIZE (16U)
#define PATH_STEP_MM_DEFAULT (30U)
#define NEAR_LOOKAHEAD_MM_DEFAULT (60U)
#define FAR_LOOKAHEAD_MM_DEFAULT (150U)
#define LANE_KFF_SCALED_DEFAULT (8)
#define FAR_NEAR_WARN_DIFF_DEFAULT (180U)
#define FAR_NEAR_REJECT_DIFF_DEFAULT (380U)
#define LANE_CONF_LOW_DEFAULT (45U)
#define LANE_CONF_HIGH_DEFAULT (70U)
#define CURVE_SPEED_GAIN_DEFAULT (15U)
#define MEMORY_MAX_DISTANCE_MM_DEFAULT (240U)
#define MEMORY_MAX_TIME_MS_DEFAULT (1200U)
#define RECOVER_BLEND_RATE_DEFAULT (20U)
#define LANE_BAD_CONFIRM_FRAMES (2U)
#define LANE_GOOD_CONFIRM_FRAMES (3U)
#define LANE_ROI_JUMP_WARN_PIXELS (80U)
#define LANE_ROI_JUMP_REJECT_PIXELS (180U)
#define LANE_CURVE_FLIP_MARGIN_PIXELS (60)
#define LANE_CURVATURE_FILTER_DIVISOR (4)
#define LANE_CURVATURE_MAX_PIXELS (220)
#define LANE_WEAK_CURVE_DIRECTION_PIXELS (15)
#define LANE_WEAK_CURVE_DIRECTION_CONFIRM_FRAMES (2U)
#define LANE_CURVE_DIRECTION_PRIORITY_PIXELS (45)
#define LANE_CURVE_DIRECTION_MIN_CORRECTION_RPM (5)
#define LANE_IMAGE_MAX_X (1279U)
#define LANE_VIRTUAL_CENTER_MAX_SHIFT_PIXELS (260)
#define LANE_SPEED_SCALE_MIN_PERCENT (70U)
#define LANE_100RPM_CURVE_MIN_BASE_RPM (85U)
#define LANE_SPEED_SCALE_SLEW_PERCENT (5U)
#define LANE_STARTUP_STEERING_GUARD_MS (500U)
#define LANE_STARTUP_MAX_CORRECTION_RPM (5)
#define LANE_LARGE_ERROR_PIXELS (180)
#define LANE_LARGE_ERROR_MAX_CORRECTION_RPM (25)
#define LANE_MEDIUM_ERROR_PIXELS (100)
#define LANE_MEDIUM_ERROR_SPEED_PERCENT (80U)
#define LANE_LARGE_ERROR_SPEED_PERCENT (70U)
#define LANE_STRAIGHT_RECOVERY_CURVE_PIXELS (25)
#define LANE_STRAIGHT_RECOVERY_ERROR_PIXELS (60)
#define LANE_STRAIGHT_RECOVERY_MAX_CORRECTION_RPM (15)
#define LANE_STRAIGHT_RECOVERY_SPEED_PERCENT (70U)
#define LANE_CENTER_SETTLE_MS (500U)
#define LANE_CENTER_SETTLE_CANCEL_PIXELS (60)
#define LANE_FAR_LOSS_SPEED_PERCENT (80U)
#define LANE_FAR_LOSS_CONFIRM_FRAMES (2U)
#define LANE_FAR_RECOVER_CONFIRM_FRAMES (3U)
#define WHEEL_CIRCUMFERENCE_MM (204U) /* pi * 65 mm wheel diameter */

/* Rollback modes: 0=original response, 1=frame step, 2=time-based ramp. */
#define LANE_PD_SLEW_MODE_NONE (0U)
#define LANE_PD_SLEW_MODE_FRAME (1U)
#define LANE_PD_SLEW_MODE_TIME (2U)
#define LANE_PD_SLEW_MODE LANE_PD_SLEW_MODE_TIME
#define LANE_PD_SLEW_FRAME_STEP_RPM (3)
#define LANE_PD_SLEW_100RPM_MS (100U)
#define LANE_PD_SLEW_150RPM_MS (200U)
#define LANE_PD_SLEW_200RPM_MS (190U)
#define AUTO_LEFT_LATCH_MIN_CORRECTION_RPM (12)
#define AUTO_FAR_LOSS_HOLD_MIN_CORRECTION_RPM (10)
#define AUTO_STRONG_STEERING_MEMORY_RPM (8)
#define AUTO_TRUSTED_STRAIGHT_CLEAR_FRAMES (3U)
/* Clear stale bend history only after sustained, centered straight geometry. */
#define AUTO_STRAIGHT_STATE_RESET_FRAMES (8U)
#define AUTO_STRAIGHT_STATE_RESET_CURVE_PIXELS (15U)
#define AUTO_STRAIGHT_STATE_RESET_CENTER_PIXELS (35U)
#define AUTO_DIRECTION_HOLD_MS (500U)
#define AUTO_DIRECTION_DECAY_STAGE_MS (300U)
#define AUTO_DIRECTION_DECAY1_RPM (5)
#define AUTO_DIRECTION_DECAY2_RPM (2)
#define AUTO_RAW_OPPOSITE_CONFIRM_FRAMES (2U)
#define AUTO_WEAK_LEFT_RELEASE_CORRECTION_RPM (3)
#define AUTO_WEAK_LEFT_MAX_CORRECTION_RPM (18)
#define AUTO_WEAK_LEFT_FULL_HOLD_PIXELS (60)
#define AUTO_WEAK_LEFT_RELEASE_PIXELS (35)
#define AUTO_STRONG_LEFT_CURVE_PIXELS (140)
#define AUTO_STRONG_LEFT_CONFIRM_FRAMES (2U)
#define AUTO_LEFT_ENTRY_CONFIRM_FRAMES (1U)
#define AUTO_LEFT_ENTRY_NEAR_PIXELS (15)
#define AUTO_LEFT_PREVIEW_MAX_CORRECTION_RPM (3)
#define AUTO_EXIT_TREND_CURVE_PIXELS (110)
#define AUTO_EXIT_TREND_CONFIRM_FRAMES (1U)
#define AUTO_EXIT_DROP_MIN_PIXELS (10)
#define AUTO_EXIT_NEUTRAL_MS (300U)
#define AUTO_EXIT_NEUTRAL_CANCEL_ERROR_PIXELS (90)
#define AUTO_EXIT_REENTRY_MARGIN_PIXELS (15)
#define AUTO_NEW_CURVE_CONFIRM_FRAMES (2U)
#define AUTO_LEFT_LATCH_RELEASE_FRAMES (3U)
#define AUTO_OPPOSITE_CONFIRM_FRAMES (3U)
#define AUTO_STRONG_OPPOSITE_CURVE_PIXELS (45)
#define AUTO_STRAIGHT_KP_SCALED (25)
#define AUTO_STRAIGHT_DEADBAND_PIXELS (15U)
#define AUTO_STRAIGHT_FINE_ERROR_PIXELS (35U)
#define AUTO_STRAIGHT_MEDIUM_ERROR_PIXELS (60U)
#define AUTO_STRAIGHT_FINE_CORRECTION_RPM (1)
#define AUTO_STRAIGHT_MEDIUM_CORRECTION_RPM (3)
#define AUTO_STRAIGHT_MAX_CORRECTION_RPM (5)
#define AUTO_STRAIGHT_SLEW_MS (80U)
#define AUTO_STRAIGHT_REVERSAL_CONFIRM_FRAMES (2U)
#define AUTO_STRAIGHT_APPROACH_CORRECTION_RPM (1)
#define AUTO_STRAIGHT_REVERSAL_FIRST_RPM (1)
#define AUTO_STRAIGHT_REVERSAL_SECOND_RPM (1)
#define AUTO_STRAIGHT_STAGE2_FRAMES (3U)
#define AUTO_STRAIGHT_STAGE3_FRAMES (5U)
#define AUTO_STRAIGHT_ERROR_DROP_PIXELS (3U)
#define AUTO_CURVE_PAIR_CONFIRM_FRAMES (2U)
#define AUTO_CURVE_FULL_CONFIRM_FRAMES (5U)
#define AUTO_CURVE_ENTRY_JUMP_PIXELS (80U)
#define AUTO_FAR_EDGE_SWITCH_PIXELS (220U)
#define AUTO_CURVE_PREVIEW_RPM (15)
#define AUTO_CURVE_STAGE1_RPM (20)
#define AUTO_CURVE_STAGE2_RPM (25)
#define AUTO_CURVE_DROPOUT_MIN_RPM (20)
#define AUTO_CURVE_EXIT_CENTER_CONFIRM_FRAMES (2U)
#define AUTO_CURVE_EXIT_HOLD_RPM (12)
#define LANE_CENTER_RETURN_RELEASE_PIXELS (45)
#define AUTO_LEFT_BASE_GAIN_PERCENT (106)
#define AUTO_LEFT_CONFIRMED_GAIN_PERCENT (106)
#define AUTO_LEFT_MIDCURVE_EXTRA_RPM (0)

/* High-speed steering authority limiter.
 * Rollback: change LANE_PD_LIMIT_MODE to LANE_PD_LIMIT_MODE_NONE. */
#define LANE_PD_LIMIT_MODE_NONE (0U)
#define LANE_PD_LIMIT_MODE_HIGH_SPEED (1U)
#define LANE_PD_LIMIT_MODE_ALL_SPEED (2U)
#define LANE_PD_LIMIT_MODE LANE_PD_LIMIT_MODE_ALL_SPEED
#define LANE_PD_LIMIT_MIN_BASE_RPM (200U)
#define LANE_PD_HIGH_SPEED_MAX_RPM (35)

#define LANE_FILTER_TRACK (0U)
#define LANE_FILTER_HOLD (1U)
#define LANE_FILTER_REACQUIRE (2U)

#define EVENT_QUEUE_SIZE (32U)
#define EVENT_QUEUE_MASK (EVENT_QUEUE_SIZE - 1U)

#define THRESHOLD_LUT_LOW_INDEX (0U)
#define THRESHOLD_LUT_MID_INDEX (1U)
#define THRESHOLD_LUT_HIGH_INDEX (2U)

#define RPM_LOW_MID_BOUNDARY (125U)
#define RPM_MID_HIGH_BOUNDARY (175U)

#define TRAFFIC_GREEN_CONFIRM_COUNT  (1U)
#define TRAFFIC_RED_CONFIRM_COUNT    (2U)
#define TRAFFIC_RED_STOP_MIN_HEIGHT   (40U)
#define SPEED_BUMP_SLOWDOWN_MS       (3000U)
#define SPEED_BUMP_REARM_MS          (500U)
#define ROAD_SPEED_30_RPM            (100U)
#define ROAD_SPEED_50_RPM            (150U)
#define ROAD_SPEED_70_RPM            (200U)
#define ROAD_SPEED_70_CURVE_CAP_RPM  (150U)
#define ROAD_SPEED_70_SLEW_INTERVAL_MS (20U)
#define ROAD_SPEED_70_STEP1_RPM      (165U)
#define ROAD_SPEED_70_INTERMEDIATE_RPM (175U)
#define ROAD_SPEED_70_STEP3_RPM      (185U)
#define ROAD_SPEED_70_STEP_HOLD_MS   (200U)
#define ROAD_SPEED_70_CONFIRM_FRAMES (3U)
#define ROAD_SPEED_70_STRICT_CONFIRM_FRAMES (5U)
#define ROAD_SPEED_70_STABLE_MS      (200U)
#define ROAD_SPEED_70_CURVE_PIXELS   (30U)
#define ROAD_SPEED_70_CENTER_PIXELS  (100U)
#define ROAD_SPEED_70_STRICT_CURVE_PIXELS (20U)
#define ROAD_SPEED_70_STRICT_CENTER_PIXELS (50U)
#define ROAD_SPEED_70_FALLBACK_CENTER_PIXELS (100U)
#define ROAD_SPEED_70_CURVE_FALLBACK_FRAMES (2U)

typedef enum
{
	ECU_VCU = 0U,
	ECU_MOTOR1,
	ECU_MOTOR2,
	ECU_ADAS1,
	ECU_ADAS2
} ECU_t;

typedef enum
{
	OBJECT_SPEED_BUMP = 0x00U,
	OBJECT_TRAFFIC_GREEN = 0x01U,
	OBJECT_TRAFFIC_RED = 0x02U,
	OBJECT_SIGN_30 = 0x03U,
	OBJECT_SIGN_70 = 0x04U,
	OBJECT_LANE = 0x05U
} ObjectType_t;

typedef struct
{
	uint16_t lanex;
	uint16_t far_x;
	uint16_t near_x;
	ObjectType_t object_type;
	uint8_t y0;
	uint8_t y1;
	bool dual_lane_valid;
	bool far_valid;
	bool near_valid;
} ADASResult_t;

typedef enum
{
	CAN_MOTOR_STATE_READY = 0x00U,
	CAN_MOTOR_STATE_ALIGNING = 0x01U,
	CAN_MOTOR_STATE_ALIGNED = 0x02U,
	CAN_MOTOR_STATE_RUNNING = 0x03U,
	CAN_MOTOR_STATE_STOPPING = 0x04U,
	CAN_MOTOR_STATE_FAULT = 0x05U
} CAN_MotorState;

typedef struct
{
	int8_t temp;          // temp
	int16_t rpm;          // rpm
	uint16_t voltage;     // voltage
	CAN_MotorState state; // Current Motor ECU operating state.
	int16_t target_rpm;   // Applied signed target RPM.
} MotorStatus_t;

typedef enum
{
	MOTOR_CMD_STOP = 0x00U,
	MOTOR_CMD_RUN = 0x01U,
	MOTOR_CMD_ALIGN = 0x02U,
	MOTOR_CMD_CLEAR_ERROR = 0x03U,
	MOTOR_CMD_SET_PARAM = 0x10U
} MotorCommandType_t;

typedef enum
{
	MOTOR_MODE_SPEED = 0x00U,
	MOTOR_MODE_POSITION = 0x01U
} MotorMode_t;

typedef enum
{
	MOTOR_DIR_FORWARD = 0x00U,
	MOTOR_DIR_REVERSE = 0x01U

} MotorDirection_t;

typedef struct
{
	uint16_t left_rpm;
	uint16_t right_rpm;
} MotorTargetrpm_t;

typedef struct
{
	int16_t center_x;
	int16_t curvature;
	uint8_t confidence;
	uint8_t valid;
} PathPoint;

typedef enum
{
	LANE_TRACK = 0U,
	LANE_PREVIEW,
	LANE_MEMORY,
	LANE_RECOVER
} lane_control_state_t;

typedef struct
{
	MotorCommandType_t command;
	MotorDirection_t direction;

	uint16_t target_rpm;
} MotorCommand_t;

typedef enum
{
	EVENT_NONE = 0U,
	EVENT_MOTOR_STOP,
	EVENT_MOTOR_RUN,
	EVENT_MOTOR_SLOW,
	EVENT_MOTOR_30,
	EVENT_MOTOR_70,
	EVENT_MOTOR_ALIGN,
	EVENT_CLEAR_ERROR,
	EVENT_SET_PARAM,
	EVENT_ESTOP_COMMAND,
	EVENT_FAULT
} EventType_t;

typedef struct
{
	EventType_t type;
	ECU_t source;
	ECU_t target;
	ADASResult_t adas_result;
	int32_t pd_output;
	MotorCommand_t motor_command;
} Event_t;

typedef struct
{
	uint16_t rpm;
	uint8_t red_height;
	uint8_t sign30_height;
	uint8_t sign70_height;
	uint8_t bump_y1;

} ADAS_ThresholdLUT_t;

typedef enum
{
	LANE_TUNE_SLEW_100_MS = 0U,
	LANE_TUNE_SLEW_150_MS,
	LANE_TUNE_SLEW_200_MS,
	LANE_TUNE_MAX_CORRECTION_RPM,
	LANE_TUNE_CENTER_X,
	LANE_TUNE_DEADBAND_PIXELS,
	LANE_TUNE_JUMP_PIXELS,
	LANE_TUNE_CLUSTER_PIXELS,
	LANE_TUNE_CONFIRM_FRAMES,
	LANE_TUNE_REACQUIRE_STEP_PIXELS,
	LANE_TUNE_RATE_FILTER_DIVISOR,
	LANE_TUNE_RATE_MAX_PIXELS_PER_SEC,
	LANE_TUNE_RATE_MAX_DT_MS,
	LANE_TUNE_DERIVATIVE_REFERENCE_MS,
	LANE_TUNE_SLEW_MODE,
	LANE_TUNE_LIMIT_MODE,
	LANE_TUNE_KFF_SCALED,
	LANE_TUNE_PATH_STEP_MM,
	LANE_TUNE_NEAR_LOOKAHEAD_MM,
	LANE_TUNE_FAR_LOOKAHEAD_MM,
	LANE_TUNE_FAR_NEAR_WARN_DIFF,
	LANE_TUNE_FAR_NEAR_REJECT_DIFF,
	LANE_TUNE_CONF_LOW,
	LANE_TUNE_CONF_HIGH,
	LANE_TUNE_CURVE_ENTER,
	LANE_TUNE_CURVE_EXIT,
	LANE_TUNE_CURVE_SPEED_GAIN,
	LANE_TUNE_MEMORY_MAX_DISTANCE_MM,
	LANE_TUNE_MEMORY_MAX_TIME_MS,
	LANE_TUNE_RECOVER_BLEND_RATE
} LaneTuningKey_t;

typedef struct
{
	uint32_t slew_100_ms;
	uint32_t slew_150_ms;
	uint32_t slew_200_ms;
	uint32_t max_correction_rpm;
	uint32_t center_x;
	uint32_t deadband_pixels;
	uint32_t jump_pixels;
	uint32_t cluster_pixels;
	uint32_t confirm_frames;
	uint32_t reacquire_step_pixels;
	uint32_t rate_filter_divisor;
	uint32_t rate_max_pixels_per_sec;
	uint32_t rate_max_dt_ms;
	uint32_t derivative_reference_ms;
	uint32_t slew_mode;
	uint32_t limit_mode;
	uint32_t kff_scaled;
	uint32_t path_step_mm;
	uint32_t near_lookahead_mm;
	uint32_t far_lookahead_mm;
	uint32_t far_near_warn_diff;
	uint32_t far_near_reject_diff;
	uint32_t confidence_low;
	uint32_t confidence_high;
	uint32_t curve_enter_pixels;
	uint32_t curve_exit_pixels;
	uint32_t curve_speed_gain;
	uint32_t memory_max_distance_mm;
	uint32_t memory_max_time_ms;
	uint32_t recover_blend_rate;
} LaneTuning_t;

typedef struct
{
	lane_control_state_t state;
	uint8_t confidence;
	int16_t far_near_diff;
	int16_t curve_raw;
	int16_t curve_filtered;
	int16_t path_curve;
	int16_t steer_feedback;
	int16_t steer_feedforward;
	int16_t steer_total;
	uint8_t speed_scale_percent;
	uint32_t distance_mm;
	uint8_t path_index;
	uint8_t recover_blend_percent;
	bool memory_timed_out;
} LaneDiagnostics_t;

bool QueueEvent(const Event_t *event);
bool GetNextEvent(Event_t *event);
void APP_UpdateADAS(void);
MotorTargetrpm_t APP_ProcessEvent(Event_t *event);
void APP_SetVehicleEnabled(bool enabled);
bool APP_SetBaseRPM(uint16_t rpm);
void APP_ResetLaneControl(uint16_t lane_x, bool lane_valid);
bool APP_SetLaneGain(uint16_t rpm_band, bool set_kp, int32_t gain_scaled);
bool APP_GetLaneGains(uint16_t rpm_band, int32_t *kp_scaled,
		int32_t *kd_scaled);
bool APP_SetLaneTuning(LaneTuningKey_t key, uint32_t value);
void APP_GetLaneTuning(LaneTuning_t *tuning);
void APP_GetLaneControlState(uint16_t *base_rpm, int32_t *lane_error,
		int32_t *pd_output, int32_t *kp_scaled, int32_t *kd_scaled);
void APP_GetLaneFilterState(uint16_t *raw_lane_x, uint16_t *used_lane_x,
		uint8_t *filter_state, uint8_t *confirm_count);
void APP_GetLaneSlewState(int32_t *requested_pd, int32_t *applied_pd);
int32_t APP_GetLaneRate(void);
void APP_GetLanePreviewState(uint16_t *far_x, uint16_t *near_x,
		bool *curve_preview_active);
void APP_GetLaneDiagnostics(LaneDiagnostics_t *diagnostics);
bool APP_IsEventQueueFaulted(void);
bool APP_IsRedLightStopActive(void);

#endif /* SRC_DRIVER_VCU_DRIVER_VCU_H_ */
