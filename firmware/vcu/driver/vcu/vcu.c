#include "vcu.h"
#include "../can/can.h"
#include "../../common/delay/delay.h"

#include <stddef.h>

/* =========================================================
 * Event Queue
 * ========================================================= */

static Event_t event_queue[EVENT_QUEUE_SIZE];

static uint8_t event_head = 0U;
static uint8_t event_tail = 0U;
static bool event_queue_faulted = false;

/* =========================================================
 * Vehicle State
 * ========================================================= */

static bool vehicle_enabled = false;
static uint32_t vehicle_enable_tick = 0U;

static uint16_t current_base_rpm = 0U;
static uint16_t current_effective_base_rpm = 0U;
static uint16_t road_speed_rpm = ROAD_SPEED_50_RPM;
/* 70-sign acceleration gate: 0=inactive, 1=150 RPM confirmation,
 * 2=165 RPM, 3=175 RPM, 4=185 RPM, 5=supervised 200 RPM. */
static uint8_t road_speed_70_stage = 0U;
static uint8_t road_speed_70_straight_count = 0U;
static uint8_t road_speed_70_curve_bad_count = 0U;
static uint32_t road_speed_70_stable_tick = 0U;
/* Continuous 70-zone speed slew.  This replaces the audible 165/175/185 RPM
 * staircase and prevents short lane-geometry changes from causing large,
 * repeated speed jumps. */
static uint16_t road_speed_70_slewed_rpm = ROAD_SPEED_70_CURVE_CAP_RPM;
static uint32_t road_speed_70_slew_tick = 0U;
static bool bump_active = false;
static bool bump_armed = true;
static uint32_t bump_start_tick = 0U;
static uint32_t bump_last_detection_tick = 0U;
static bool red_light_stop = false;
static uint8_t red_confirm_count = 0U;
/* ADAS-confirmed green is accepted immediately to release a red-light stop. */
static uint8_t green_confirm_count = 0U;

/* Lane steering is sourced only from ADAS1. */
static int32_t current_lane_pd = 0;
static int32_t requested_lane_pd = 0;
static int32_t current_lane_error = 0;
static int32_t filtered_lane_rate = 0;
static int32_t previous_lane_rate_error = 0;
static uint32_t previous_lane_rate_tick = 0U;
static bool lane_rate_initialized = false;
static int32_t current_kp_scaled = KP_150RPM_SCALED;
static int32_t current_kd_scaled = KD_150RPM_SCALED;
static int32_t runtime_kp_100_scaled = KP_100RPM_SCALED;
static int32_t runtime_kd_100_scaled = KD_100RPM_SCALED;
static int32_t runtime_kp_150_scaled = KP_150RPM_SCALED;
static int32_t runtime_kd_150_scaled = KD_150RPM_SCALED;
static int32_t runtime_kp_200_scaled = KP_200RPM_SCALED;
static int32_t runtime_kd_200_scaled = KD_200RPM_SCALED;
static LaneTuning_t lane_tuning = {
	.slew_100_ms = LANE_PD_SLEW_100RPM_MS,
	.slew_150_ms = LANE_PD_SLEW_150RPM_MS,
	.slew_200_ms = LANE_PD_SLEW_200RPM_MS,
	.max_correction_rpm = LANE_PD_HIGH_SPEED_MAX_RPM,
	.center_x = LANE_CENTER_X,
	.deadband_pixels = LANE_ERROR_DEADBAND_PIXELS,
	.jump_pixels = LANE_JUMP_MARGIN_PIXELS,
	.cluster_pixels = LANE_REACQUIRE_CLUSTER_PIXELS,
	.confirm_frames = LANE_REACQUIRE_CONFIRM_FRAMES,
	.reacquire_step_pixels = LANE_REACQUIRE_STEP_PIXELS,
	.rate_filter_divisor = LANE_RATE_FILTER_DIVISOR,
	.rate_max_pixels_per_sec = LANE_RATE_MAX_PIXELS_PER_SEC,
	.rate_max_dt_ms = LANE_RATE_MAX_VALID_DT_MS,
	.derivative_reference_ms = LANE_DERIVATIVE_REFERENCE_MS,
	.slew_mode = LANE_PD_SLEW_MODE,
	.limit_mode = LANE_PD_LIMIT_MODE,
	.kff_scaled = LANE_KFF_SCALED_DEFAULT,
	.path_step_mm = PATH_STEP_MM_DEFAULT,
	.near_lookahead_mm = NEAR_LOOKAHEAD_MM_DEFAULT,
	.far_lookahead_mm = FAR_LOOKAHEAD_MM_DEFAULT,
	.far_near_warn_diff = FAR_NEAR_WARN_DIFF_DEFAULT,
	.far_near_reject_diff = FAR_NEAR_REJECT_DIFF_DEFAULT,
	.confidence_low = LANE_CONF_LOW_DEFAULT,
	.confidence_high = LANE_CONF_HIGH_DEFAULT,
	.curve_enter_pixels = LANE_FAR_NEAR_ENTER_PIXELS,
	.curve_exit_pixels = LANE_FAR_NEAR_EXIT_PIXELS,
	.curve_speed_gain = CURVE_SPEED_GAIN_DEFAULT,
	.memory_max_distance_mm = MEMORY_MAX_DISTANCE_MM_DEFAULT,
	.memory_max_time_ms = MEMORY_MAX_TIME_MS_DEFAULT,
	.recover_blend_rate = RECOVER_BLEND_RATE_DEFAULT
};
static uint16_t current_raw_lane_x = LANE_CENTER_X;
static uint16_t trusted_lane_x = LANE_CENTER_X;
static uint16_t current_used_lane_x = LANE_CENTER_X;
static uint16_t current_far_lane_x = LANE_CENTER_X;
static uint16_t current_near_lane_x = LANE_CENTER_X;
static bool curve_preview_active = false;
static uint8_t curve_preview_release_count = 0U;
static uint16_t candidate_lane_x = LANE_CENTER_X;
static int8_t lane_candidate_side = 0;
static uint8_t lane_filter_state = LANE_FILTER_TRACK;
static uint8_t lane_candidate_count = 0U;
static bool lane_filter_initialized = false;
static bool lane_dropout_active = false;
static bool lane_dropout_timed_out = false;
static uint32_t lane_dropout_start_tick = 0U;
static uint16_t lane_dropout_start_x = LANE_CENTER_X;
static uint8_t lane_dropout_recovery_count = 0U;
static int32_t lane_pd_ramp_start = 0;
static int32_t lane_pd_ramp_target = 0;
static uint32_t lane_pd_ramp_start_tick = 0U;
static uint32_t lane_pd_ramp_duration_ms = LANE_PD_SLEW_100RPM_MS;
static bool lane_pd_ramp_active = false;
static int8_t lane_steering_sign = 0;
static int8_t lane_pending_sign = 0;
static uint8_t lane_pending_sign_count = 0U;
static int8_t auto_straight_error_direction = 0;
static uint8_t auto_straight_error_count = 0U;
static int32_t auto_straight_previous_abs_error = 0;
static int8_t auto_raw_straight_direction = 0;
static uint8_t auto_raw_straight_count = 0U;
static int32_t auto_raw_straight_previous_abs_error = 0;

/* Near provides present lateral feedback. Far-minus-Near is stored as a
 * future path and becomes feed-forward only after the vehicle travels to the
 * corresponding distance slot. */
static PathPoint path_memory[PATH_MEM_SIZE];
static lane_control_state_t lane_control_state = LANE_TRACK;
static uint8_t lane_confidence = 0U;
static uint8_t lane_bad_count = 0U;
static uint8_t lane_good_count = 0U;
static uint8_t lane_recover_blend_percent = 100U;
static int16_t lane_curve_raw = 0;
static int16_t lane_curve_filtered = 0;
static int8_t lane_weak_curve_direction = 0;
static int8_t lane_weak_curve_candidate = 0;
static uint8_t lane_weak_curve_count = 0U;
/* AUTO-only signed curve latch: -1=left, +1=right, 0=none. */
static int8_t auto_curve_latched_direction = 0;
/* Last direction proven by a stable FAR/NEAR pair. Retain it only across a
 * short FAR dropout so a bend does not collapse to a 2-3 RPM correction. */
static int8_t auto_last_trusted_curve_direction = 0;
static int8_t auto_last_strong_steering_direction = 0;
static uint8_t auto_trusted_straight_count = 0U;
static uint8_t auto_straight_state_reset_count = 0U;
static uint32_t auto_last_strong_steering_tick = 0U;
static uint8_t auto_raw_opposite_count = 0U;
static bool lane_geometry_untrusted = true;
static uint8_t auto_curve_release_count = 0U;
static uint8_t auto_opposite_evidence_count = 0U;
static uint8_t auto_strong_curve_count = 0U;
static int8_t auto_entry_candidate_direction = 0;
static uint8_t auto_entry_count = 0U;
static uint8_t auto_exit_trend_count = 0U;
static int32_t auto_previous_abs_curve = 0;
static int32_t auto_exit_reference_curve = 0;
static bool auto_exit_neutral_active = false;
static uint32_t auto_exit_neutral_tick = 0U;
static bool auto_post_exit_track_active = false;
static int8_t auto_new_curve_candidate_direction = 0;
static uint8_t auto_new_curve_count = 0U;
static bool auto_pair_history_valid = false;
static uint16_t auto_pair_previous_far_x = LANE_CENTER_X;
static uint16_t auto_pair_previous_near_x = LANE_CENTER_X;
static uint8_t auto_pair_stable_count = 0U;
static int8_t auto_curve_authority_direction = 0;
static uint8_t auto_curve_authority_count = 0U;
static bool auto_near_measurement_trusted = false;
static uint8_t auto_curve_exit_center_count = 0U;
static bool lane_center_settle_active = false;
static uint32_t lane_center_settle_tick = 0U;
static int16_t lane_path_curve = 0;
static int16_t lane_steer_feedback = 0;
static int16_t lane_steer_feedforward = 0;
static int16_t lane_steer_total = 0;
static uint8_t lane_speed_scale_percent = 100U;
static bool lane_far_loss_active = false;
static uint8_t lane_far_loss_count = 0U;
static uint8_t lane_far_recover_count = 0U;
static uint32_t lane_distance_mm = 0U;
static uint32_t lane_distance_remainder = 0U;
static uint32_t lane_path_step_accum_mm = 0U;
static uint32_t lane_distance_last_tick = 0U;
static uint32_t lane_memory_start_distance_mm = 0U;
static uint32_t lane_memory_start_tick = 0U;
static bool lane_memory_timed_out = false;
static bool lane_roi_history_valid = false;
static uint16_t lane_previous_far_x = LANE_CENTER_X;
static uint16_t lane_previous_near_x = LANE_CENTER_X;
static int16_t lane_previous_curve_raw = 0;
static uint8_t lane_path_write_index = 0U;
static uint16_t lane_virtual_anchor_near_x = LANE_CENTER_X;
static int16_t lane_virtual_anchor_curve = 0;
static uint32_t lane_virtual_anchor_distance_mm = 0U;
static bool lane_virtual_anchor_valid = false;
static int8_t lane_memory_search_direction = 0;
static int16_t lane_memory_search_correction = 0;

static void APP_ResetLanePDSlew(void);
static void Lane_ResetPathMemory(void);
static int32_t Lane_Abs32(int32_t value);

/* Keep the currently tracked lane when Sobel suddenly reports a distant
 * line. A consistent new position is accepted only after several frames and
 * is approached with a bounded step so the PD derivative cannot kick. */
static uint16_t APP_FilterLaneX(uint16_t raw_lane_x, bool measurement_valid)
{
	uint16_t center_delta;
	uint16_t previous_delta;
	uint16_t raw_jump;
	uint32_t elapsed;
	int32_t start_error;
	int32_t decayed_error;
	uint32_t now = millis();

	/* ADAS already supplies the virtual lane-center coordinate.  A second
	 * stateful virtual-center/reacquisition filter in the VCU delayed real
	 * curves and could preserve steering in the previous direction.  Use the
	 * newest valid ADAS result directly; physical steering remains bounded by
	 * the RPM correction limit and the time-based motor-command slew. */
	current_raw_lane_x = raw_lane_x;
	center_delta = (raw_lane_x >= lane_tuning.center_x) ?
			(uint16_t)(raw_lane_x - lane_tuning.center_x) :
			(uint16_t)(lane_tuning.center_x - raw_lane_x);

	/* Explicit RTL validity is authoritative. Preserve the last trusted
	 * trajectory briefly, then decay it and stop if neither ROI recovers. */
	if ((measurement_valid == false) && (lane_dropout_active == false)) {
		lane_dropout_active = true;
		lane_dropout_timed_out = false;
		lane_dropout_start_tick = now;
		lane_dropout_start_x = current_used_lane_x;
		lane_dropout_recovery_count = 0U;
		lane_filter_initialized = true;
	}

	if (lane_filter_initialized == false) {
		trusted_lane_x = raw_lane_x;
		current_used_lane_x = raw_lane_x;
		candidate_lane_x = raw_lane_x;
		lane_filter_initialized = true;
		lane_filter_state = LANE_FILTER_TRACK;
		return current_used_lane_x;
	}

	if (lane_dropout_active != false) {
		if (measurement_valid != false) {
			if (lane_dropout_recovery_count < LANE_DROPOUT_RECOVERY_FRAMES) {
				lane_dropout_recovery_count++;
			}
		} else {
			lane_dropout_recovery_count = 0U;
		}

		if (lane_dropout_recovery_count >= LANE_DROPOUT_RECOVERY_FRAMES) {
			lane_dropout_active = false;
			lane_dropout_timed_out = false;
			lane_dropout_recovery_count = 0U;
			trusted_lane_x = raw_lane_x;
			current_used_lane_x = raw_lane_x;
			lane_filter_state = LANE_FILTER_TRACK;
			return current_used_lane_x;
		}

		elapsed = (uint32_t)(now - lane_dropout_start_tick);
		if (elapsed >= LANE_DROPOUT_STOP_MS) {
			lane_dropout_timed_out = true;
			current_used_lane_x = (uint16_t)lane_tuning.center_x;
		} else if (elapsed <= LANE_DROPOUT_HOLD_MS) {
			current_used_lane_x = lane_dropout_start_x;
		} else {
			uint32_t decay_elapsed = elapsed - LANE_DROPOUT_HOLD_MS;
			if (decay_elapsed >= LANE_DROPOUT_DECAY_MS) {
				current_used_lane_x = (uint16_t)lane_tuning.center_x;
			} else {
				start_error = (int32_t)lane_dropout_start_x -
						(int32_t)lane_tuning.center_x;
				decayed_error = (int32_t)(((int64_t)start_error *
						(int64_t)(LANE_DROPOUT_DECAY_MS - decay_elapsed)) /
						(int64_t)LANE_DROPOUT_DECAY_MS);
				current_used_lane_x = (uint16_t)((int32_t)lane_tuning.center_x +
						decayed_error);
			}
		}
		lane_filter_state = LANE_FILTER_HOLD;
		lane_candidate_count = lane_dropout_recovery_count;
		return current_used_lane_x;
	}

	previous_delta = (trusted_lane_x >= lane_tuning.center_x) ?
			(uint16_t)(trusted_lane_x - lane_tuning.center_x) :
			(uint16_t)(lane_tuning.center_x - trusted_lane_x);
	raw_jump = (raw_lane_x >= trusted_lane_x) ?
			(uint16_t)(raw_lane_x - trusted_lane_x) :
			(uint16_t)(trusted_lane_x - raw_lane_x);

	if ((previous_delta >= LANE_DROPOUT_PREVIOUS_ERROR_PIXELS) &&
	    (center_delta <= lane_tuning.deadband_pixels) &&
	    (raw_jump >= LANE_DROPOUT_JUMP_PIXELS)) {
		lane_dropout_active = true;
		lane_dropout_timed_out = false;
		lane_dropout_start_tick = now;
		lane_dropout_start_x = trusted_lane_x;
		lane_dropout_recovery_count = 0U;
		current_used_lane_x = trusted_lane_x;
		lane_filter_state = LANE_FILTER_HOLD;
		return current_used_lane_x;
	}

	/* Follow ordinary lane motion immediately. A large displacement is not
	 * trusted until three consecutive samples form a stable cluster. During
	 * confirmation, command neutral steering rather than retaining the old
	 * curve direction. This rejects alternating Sobel lines without adding
	 * continuous low-pass delay to genuine curves. */
	if (raw_jump <= lane_tuning.jump_pixels) {
		trusted_lane_x = raw_lane_x;
		current_used_lane_x = raw_lane_x;
		candidate_lane_x = raw_lane_x;
		lane_candidate_side = 0;
		lane_candidate_count = 0U;
		lane_filter_state = LANE_FILTER_TRACK;
		return current_used_lane_x;
	}

	if (lane_candidate_count == 0U) {
		candidate_lane_x = raw_lane_x;
		lane_candidate_count = 1U;
	} else {
		uint16_t candidate_delta = (raw_lane_x >= candidate_lane_x) ?
				(uint16_t)(raw_lane_x - candidate_lane_x) :
				(uint16_t)(candidate_lane_x - raw_lane_x);
		if (candidate_delta <= lane_tuning.cluster_pixels) {
			candidate_lane_x = raw_lane_x;
			if (lane_candidate_count < lane_tuning.confirm_frames) {
				lane_candidate_count++;
			}
		} else {
			candidate_lane_x = raw_lane_x;
			lane_candidate_count = 1U;
		}
	}

	lane_filter_state = LANE_FILTER_HOLD;
	current_used_lane_x = (uint16_t)lane_tuning.center_x;
	if (lane_candidate_count >= lane_tuning.confirm_frames) {
		trusted_lane_x = candidate_lane_x;
		current_used_lane_x = trusted_lane_x;
		lane_candidate_count = 0U;
		lane_filter_state = LANE_FILTER_REACQUIRE;
	}
	return current_used_lane_x;

#if 0 /* Previous stateful pixel filter retained only for rollback history. */
	uint16_t trusted_delta;
	uint16_t center_delta;
	uint16_t trusted_center_delta;
	int8_t raw_side;
	int8_t trusted_side;

	current_raw_lane_x = raw_lane_x;

	if (lane_filter_initialized == false) {
		trusted_lane_x = raw_lane_x;
		current_used_lane_x = raw_lane_x;
		candidate_lane_x = raw_lane_x;
		lane_candidate_side = 0;
		lane_candidate_count = 0U;
		lane_filter_state = LANE_FILTER_TRACK;
		lane_filter_initialized = true;
		return current_used_lane_x;
	}

	/* A value back near the image center is always safe to accept.  This
	 * releases a stale left/right correction immediately after a curve. */
	center_delta = (raw_lane_x >= lane_tuning.center_x) ?
			(uint16_t)(raw_lane_x - lane_tuning.center_x) :
			(uint16_t)(lane_tuning.center_x - raw_lane_x);
	if (center_delta <= lane_tuning.deadband_pixels) {
		trusted_lane_x = raw_lane_x;
		current_used_lane_x = raw_lane_x;
		candidate_lane_x = raw_lane_x;
		lane_candidate_side = 0;
		lane_candidate_count = 0U;
		lane_filter_state = LANE_FILTER_TRACK;
		return current_used_lane_x;
	}

	/* If the measurement moves clearly to the opposite side of the image,
	 * stop using the previous curve immediately.  Keep steering neutral while
	 * the new direction is confirmed, then reacquire from image center rather
	 * than walking all the way across from the stale virtual center. */
	trusted_side = 0;
	if (trusted_lane_x > (lane_tuning.center_x + lane_tuning.deadband_pixels)) {
		trusted_side = 1;
	} else if ((trusted_lane_x + lane_tuning.deadband_pixels) < lane_tuning.center_x) {
		trusted_side = -1;
	}

	raw_side = 0;
	if (raw_lane_x > (lane_tuning.center_x + LANE_DIRECTION_SWITCH_MARGIN_PIXELS)) {
		raw_side = 1;
	} else if ((raw_lane_x + LANE_DIRECTION_SWITCH_MARGIN_PIXELS) < lane_tuning.center_x) {
		raw_side = -1;
	}

	if ((trusted_side != 0) && (raw_side != 0) &&
	    (trusted_side != raw_side)) {
		if ((lane_candidate_count == 0U) ||
		    (lane_candidate_side != raw_side)) {
			lane_candidate_count = 1U;
			lane_candidate_side = raw_side;
		} else if (lane_candidate_count < lane_tuning.confirm_frames) {
			lane_candidate_count++;
		}

		candidate_lane_x = raw_lane_x;
		lane_filter_state = LANE_FILTER_HOLD;
		current_used_lane_x = (uint16_t)lane_tuning.center_x;

		if (lane_candidate_count < lane_tuning.confirm_frames) {
			return current_used_lane_x;
		}

		/* Direction switch confirmed: discard the old curve history.  The
		 * motor-side RPM slew already bounds physical steering, so retaining a
		 * second pixel-domain slew here only delays the real lane response. */
		trusted_lane_x = candidate_lane_x;
		current_used_lane_x = trusted_lane_x;
		lane_filter_state = LANE_FILTER_REACQUIRE;
		return current_used_lane_x;
	}

	/* On the same side, a measurement closer to image center means the curve
	 * is opening.  Accept it immediately so an old tight-turn value can never
	 * command more steering while RAW is already unwinding. */
	trusted_center_delta = (trusted_lane_x >= lane_tuning.center_x) ?
			(uint16_t)(trusted_lane_x - lane_tuning.center_x) :
			(uint16_t)(lane_tuning.center_x - trusted_lane_x);
	if ((trusted_side != 0) &&
	    ((raw_side == trusted_side) || (raw_side == 0)) &&
	    (center_delta < trusted_center_delta)) {
		trusted_lane_x = raw_lane_x;
		current_used_lane_x = raw_lane_x;
		candidate_lane_x = raw_lane_x;
		lane_candidate_side = 0;
		lane_candidate_count = 0U;
		lane_filter_state = LANE_FILTER_TRACK;
		return current_used_lane_x;
	}

	trusted_delta = (raw_lane_x >= trusted_lane_x) ?
			(raw_lane_x - trusted_lane_x) : (trusted_lane_x - raw_lane_x);

	if (trusted_delta <= lane_tuning.jump_pixels) {
		trusted_lane_x = raw_lane_x;
		current_used_lane_x = raw_lane_x;
		candidate_lane_x = raw_lane_x;
		lane_candidate_side = 0;
		lane_candidate_count = 0U;
		lane_filter_state = LANE_FILTER_TRACK;
		return current_used_lane_x;
	}

	raw_side = (raw_lane_x > lane_tuning.center_x) ? 1 : -1;

	/* A real curve moves across the image, so requiring every sample to stay
	 * inside a small pixel cluster can lock the filter forever.  Confirm a
	 * new lane by consecutive samples on the same side of image center. */
	if ((lane_candidate_count == 0U) || (raw_side != lane_candidate_side)) {
		candidate_lane_x = raw_lane_x;
		lane_candidate_side = raw_side;
		lane_candidate_count = 1U;
		lane_filter_state = LANE_FILTER_HOLD;
		current_used_lane_x = (uint16_t)lane_tuning.center_x;
		return current_used_lane_x;
	}

	/* Retain the newest curve position while confirming its direction. */
	candidate_lane_x = raw_lane_x;
	if (lane_candidate_count < lane_tuning.confirm_frames) {
		lane_candidate_count++;
	}

	if (lane_candidate_count < lane_tuning.confirm_frames) {
		lane_filter_state = LANE_FILTER_HOLD;
		current_used_lane_x = (uint16_t)lane_tuning.center_x;
		return current_used_lane_x;
	}

	lane_filter_state = LANE_FILTER_REACQUIRE;
	trusted_lane_x = candidate_lane_x;
	current_used_lane_x = trusted_lane_x;
	return current_used_lane_x;
#endif
}

static int32_t APP_InterpolateSigned(int32_t low_value, int32_t high_value,
		uint16_t rpm, uint16_t low_rpm, uint16_t high_rpm)
{
	int64_t delta;

	if (rpm <= low_rpm) return low_value;
	if (rpm >= high_rpm) return high_value;
	delta = (int64_t)(high_value - low_value) * (int64_t)(rpm - low_rpm);
	return low_value + (int32_t)(delta / (int64_t)(high_rpm - low_rpm));
}

static int32_t APP_ScaleCorrectionForRPM(int32_t reference_200_rpm,
		uint16_t rpm)
{
	int32_t scaled;

	if (rpm < 100U) rpm = 100U;
	if (rpm > 200U) rpm = 200U;
	scaled = (reference_200_rpm * (int32_t)rpm + 100) / 200;
	if ((reference_200_rpm > 0) && (scaled < 1)) scaled = 1;
	return scaled;
}

static uint16_t APP_GetCurrentRPM(void);

static uint16_t APP_GetSteeringControlRPM(void)
{
	uint16_t measured_rpm = APP_GetCurrentRPM();

	/* Tune steering from measured wheel speed, independent of which AUTO
	 * command started the run. During launch, measured RPM is still zero, so
	 * use the effective/requested value until fresh motor motion is visible. */
	if ((vehicle_enabled != false) && (measured_rpm != 0U))
		return measured_rpm;
	return (current_effective_base_rpm != 0U) ?
		current_effective_base_rpm : current_base_rpm;
}

static int32_t APP_InterpolateGain(int32_t gain100, int32_t gain150,
		int32_t gain200, uint16_t rpm)
{
	if (rpm <= 100U) return gain100;
	if (rpm < 150U)
		return APP_InterpolateSigned(gain100, gain150, rpm, 100U, 150U);
	if (rpm < 200U)
		return APP_InterpolateSigned(gain150, gain200, rpm, 150U, 200U);
	return gain200;
}

static uint32_t APP_GetLanePDSlewDurationMs(void)
{
	uint16_t rpm = APP_GetSteeringControlRPM();

	return (uint32_t)APP_InterpolateGain((int32_t)lane_tuning.slew_100_ms,
		(int32_t)lane_tuning.slew_150_ms,
		(int32_t)lane_tuning.slew_200_ms, rpm);
}

static void APP_ResetLanePDSlew(void)
{
	current_lane_pd = 0;
	requested_lane_pd = 0;
	lane_pd_ramp_start = 0;
	lane_pd_ramp_target = 0;
	lane_pd_ramp_start_tick = 0U;
	lane_pd_ramp_duration_ms = lane_tuning.slew_100_ms;
	lane_pd_ramp_active = false;
	filtered_lane_rate = 0;
	previous_lane_rate_error = 0;
	previous_lane_rate_tick = 0U;
	lane_rate_initialized = false;
	lane_steering_sign = 0;
	lane_pending_sign = 0;
	lane_pending_sign_count = 0U;
	auto_straight_error_direction = 0;
	auto_straight_error_count = 0U;
	auto_straight_previous_abs_error = 0;
	auto_raw_straight_direction = 0;
	auto_raw_straight_count = 0U;
	auto_raw_straight_previous_abs_error = 0;
}

/* Reset transient steering state at the start of an automatic run.  When a
 * fresh lane is already being tracked, preserve the filtered FAR/NEAR and
 * path-preview state; clearing those values made the first NEAR sample alone
 * decide the launch direction.  With no fresh lane, perform the full reset
 * so the next valid frame becomes a clean reference. */
void APP_ResetLaneControl(uint16_t lane_x, bool lane_valid)
{
	APP_ResetLanePDSlew();
	if ((lane_valid != false) && (lane_filter_initialized != false)) {
		lane_candidate_count = 0U;
		lane_candidate_side = 0;
		lane_dropout_active = false;
		lane_dropout_timed_out = false;
		lane_dropout_start_tick = 0U;
		lane_dropout_recovery_count = 0U;
		return;
	}
	Lane_ResetPathMemory();
	current_lane_error = 0;
	lane_candidate_count = 0U;
	lane_candidate_side = 0;
	lane_filter_state = LANE_FILTER_TRACK;
	lane_dropout_active = false;
	lane_dropout_timed_out = false;
	lane_dropout_start_tick = 0U;
	lane_dropout_start_x = (uint16_t)lane_tuning.center_x;
	lane_dropout_recovery_count = 0U;
	current_far_lane_x = lane_x;
	current_near_lane_x = lane_x;
	curve_preview_active = false;
	curve_preview_release_count = 0U;

	if (lane_valid != false) {
		current_raw_lane_x = lane_x;
		trusted_lane_x = lane_x;
		current_used_lane_x = lane_x;
		candidate_lane_x = lane_x;
		lane_filter_initialized = true;
	} else {
		current_raw_lane_x = (uint16_t)lane_tuning.center_x;
		trusted_lane_x = (uint16_t)lane_tuning.center_x;
		current_used_lane_x = (uint16_t)lane_tuning.center_x;
		candidate_lane_x = (uint16_t)lane_tuning.center_x;
		lane_filter_initialized = false;
	}
}

/* FAR sees an approaching bend before NEAR reaches it. Keep the preview
 * active for a few frames on exit so the vehicle does not accelerate in the
 * middle of a noisy curve transition. */
static void APP_UpdateCurvePreview(uint16_t far_x, uint16_t near_x,
		bool dual_lane_valid, uint8_t confidence)
{
	int32_t far_error;
	int32_t far_near_delta;

	current_far_lane_x = far_x;
	current_near_lane_x = near_x;
	if ((dual_lane_valid == false) ||
	    (confidence < (uint8_t)lane_tuning.confidence_low)) {
		curve_preview_active = false;
		curve_preview_release_count = 0U;
		return;
	}

	far_error = (int32_t)lane_tuning.center_x - (int32_t)far_x;
	if (far_error < 0) far_error = -far_error;
	far_near_delta = (int32_t)far_x - (int32_t)near_x;
	if (far_near_delta < 0) far_near_delta = -far_near_delta;

	if (curve_preview_active == false) {
		if ((confidence >= (uint8_t)lane_tuning.confidence_low) &&
		    ((far_error >= (int32_t)lane_tuning.curve_enter_pixels) ||
		     (far_near_delta >= (int32_t)lane_tuning.curve_enter_pixels))) {
			curve_preview_active = true;
			curve_preview_release_count = 0U;
		}
	} else if ((far_error <= (int32_t)lane_tuning.curve_exit_pixels) &&
	           (far_near_delta <= (int32_t)lane_tuning.curve_exit_pixels)) {
		if (curve_preview_release_count < LANE_CURVE_RELEASE_FRAMES) {
			curve_preview_release_count++;
		}
		if (curve_preview_release_count >= LANE_CURVE_RELEASE_FRAMES) {
			curve_preview_active = false;
			curve_preview_release_count = 0U;
		}
	} else {
		curve_preview_release_count = 0U;
	}
}

static int32_t APP_UpdateTimeBasedLanePD(int32_t desired_pd,
		uint32_t requested_duration_ms)
{
	uint32_t now = millis();

	if (lane_pd_ramp_active != false) {
		uint32_t elapsed = (uint32_t)(now - lane_pd_ramp_start_tick);

		if (elapsed >= lane_pd_ramp_duration_ms) {
			current_lane_pd = lane_pd_ramp_target;
			lane_pd_ramp_active = false;
		} else {
			int32_t delta = lane_pd_ramp_target - lane_pd_ramp_start;
			current_lane_pd = lane_pd_ramp_start +
					(int32_t)(((int64_t)delta * (int64_t)elapsed) /
					(int64_t)lane_pd_ramp_duration_ms);
		}
	}

	if (desired_pd != lane_pd_ramp_target) {
		lane_pd_ramp_start = current_lane_pd;
		lane_pd_ramp_target = desired_pd;
		lane_pd_ramp_start_tick = now;
		lane_pd_ramp_duration_ms = requested_duration_ms;
		lane_pd_ramp_active = (lane_pd_ramp_start != lane_pd_ramp_target);
	}

	return current_lane_pd;
}

/* =========================================================
 * QueueEvent
 *
 * Description:
 * Pushes one application event into the event queue.
 * The queue uses a circular buffer and EVENT_QUEUE_MASK for
 * index wrapping.
 *
 * Parameter:
 * event - Pointer to the event to be stored.
 *
 * Return:
 * true  - Event was successfully queued.
 * false - Input pointer is NULL or the queue is full.
 * ========================================================= */

bool QueueEvent(const Event_t *event)
{
	uint8_t next_head;

	if (event == NULL) {
		return false;
	}

	next_head = (uint8_t)((event_head + 1U) & (uint8_t)EVENT_QUEUE_MASK);

	if (next_head == event_tail) {
		event_queue_faulted = true;
		return false;
	}

	event_queue[event_head] = *event;

	event_head = next_head;

	return true;
}

bool APP_IsEventQueueFaulted(void)
{
	return event_queue_faulted;
}

/* =========================================================
 * GetNextEvent
 *
 * Description:
 * Removes the oldest event from the event queue and copies it
 * to the caller-provided buffer.
 *
 * Parameter:
 * event - Pointer to the destination event structure.
 *
 * Return:
 * true  - One event was successfully read from the queue.
 * false - Input pointer is NULL or the queue is empty.
 * ========================================================= */

bool GetNextEvent(Event_t *event)
{
	if (event == NULL) {
		return false;
	}

	if (event_head == event_tail) {
		return false;
	}

	*event = event_queue[event_tail];

	event_tail = (uint8_t)((event_tail + 1U) & (uint8_t)EVENT_QUEUE_MASK);

	return true;
}

/* =========================================================
 * APP_LanePD
 *
 * Description:
 * Calculates the lane tracking correction using a PD
 * controller. The error is calculated from the configured
 * lane center and the detected lane x-coordinate.
 *
 * Parameter:
 * lane_x - Detected lane x-coordinate.
 *
 * Return:
 * Signed PD correction value used to create differential
 * left and right motor RPM commands.
 * ========================================================= */

static int32_t APP_LanePD(uint16_t lane_x)
{
	int32_t error;
	int32_t derivative;
	int32_t raw_lane_rate;
	int32_t pd_output;
	int32_t p_output;
	int32_t unwind_d_output;
	int32_t kp_scaled;
	int32_t kd_scaled;
	int32_t abs_error;
	int32_t correction_limit;
	uint16_t steering_base_rpm;
	uint16_t control_rpm;
	uint32_t reversal_confirm_frames;
	uint32_t active_deadband_pixels;
	int8_t requested_sign;
	bool auto_straight_mode;
	uint32_t now;
	uint32_t dt_ms;

	control_rpm = APP_GetSteeringControlRPM();
	auto_straight_mode =
		((auto_post_exit_track_active != false) ||
		 ((auto_curve_latched_direction == 0) &&
		  ((Lane_Abs32(lane_curve_raw) < LANE_STRAIGHT_RECOVERY_CURVE_PIXELS) ||
		   (auto_curve_authority_count < 3U))));
	active_deadband_pixels = (auto_straight_mode != false) ?
		AUTO_STRAIGHT_DEADBAND_PIXELS : lane_tuning.deadband_pixels;
	error = (int32_t)lane_tuning.center_x - (int32_t)lane_x;
	if ((error >= -(int32_t)active_deadband_pixels) &&
	    (error <= (int32_t)active_deadband_pixels)) {
		 error = 0;
	}
	abs_error = (error < 0) ? -error : error;
	if ((auto_straight_mode != false) && (error != 0)) {
		int8_t error_direction = (error > 0) ? 1 : -1;
		bool error_not_recovering =
			(abs_error + (int32_t)AUTO_STRAIGHT_ERROR_DROP_PIXELS) >=
			auto_straight_previous_abs_error;

		if ((error_direction == auto_straight_error_direction) &&
		    (error_not_recovering != false)) {
			if (auto_straight_error_count < AUTO_STRAIGHT_STAGE3_FRAMES)
				auto_straight_error_count++;
		} else {
			auto_straight_error_direction = error_direction;
			auto_straight_error_count = 1U;
		}
		auto_straight_previous_abs_error = abs_error;
	} else {
		auto_straight_error_direction = 0;
		auto_straight_error_count = 0U;
		auto_straight_previous_abs_error = 0;
	}

	now = millis();
	if (lane_rate_initialized == false) {
		filtered_lane_rate = 0;
		lane_rate_initialized = true;
	} else {
		dt_ms = (uint32_t)(now - previous_lane_rate_tick);
		if ((dt_ms > 0U) && (dt_ms <= lane_tuning.rate_max_dt_ms)) {
			raw_lane_rate = (int32_t)(((int64_t)(error - previous_lane_rate_error) *
			                          1000LL) / (int64_t)dt_ms);
			if (raw_lane_rate > (int32_t)lane_tuning.rate_max_pixels_per_sec) {
				raw_lane_rate = (int32_t)lane_tuning.rate_max_pixels_per_sec;
			} else if (raw_lane_rate < -(int32_t)lane_tuning.rate_max_pixels_per_sec) {
				raw_lane_rate = -(int32_t)lane_tuning.rate_max_pixels_per_sec;
			}
			filtered_lane_rate += (raw_lane_rate - filtered_lane_rate) /
			                      (int32_t)lane_tuning.rate_filter_divisor;
		} else {
			filtered_lane_rate = 0;
		}
	}
	previous_lane_rate_error = error;
	previous_lane_rate_tick = now;
	/* Convert pixels/second to the equivalent error change over a 100 ms
	 * reference interval so the existing fixed-point Kd values remain safe. */
	derivative = (filtered_lane_rate * (int32_t)lane_tuning.derivative_reference_ms) /
	             1000;

	kp_scaled = APP_InterpolateGain(runtime_kp_100_scaled,
		runtime_kp_150_scaled, runtime_kp_200_scaled, control_rpm);
	kd_scaled = APP_InterpolateGain(runtime_kd_100_scaled,
		runtime_kd_150_scaled, runtime_kd_200_scaled, control_rpm);
	p_output = (kp_scaled * error) / LANE_GAIN_SCALE;
	unwind_d_output = 0;
	/* Use D only as braking while the lateral error magnitude is shrinking.
	 * It may release at most four RPM of existing steering and is never
	 * allowed to create steering in the opposite direction. This provides
	 * phase lead at an S-curve transition without amplifying RAW lane noise. */
	if (((error > 0) && (derivative < 0)) ||
	    ((error < 0) && (derivative > 0))) {
		unwind_d_output = (kd_scaled * derivative) / LANE_GAIN_SCALE;
		if (unwind_d_output > LANE_UNWIND_D_MAX_RPM) {
			unwind_d_output = LANE_UNWIND_D_MAX_RPM;
		} else if (unwind_d_output < -LANE_UNWIND_D_MAX_RPM) {
			unwind_d_output = -LANE_UNWIND_D_MAX_RPM;
		}
	}
	pd_output = p_output + unwind_d_output;
	if (((p_output > 0) && (pd_output < 0)) ||
	    ((p_output < 0) && (pd_output > 0))) {
		pd_output = 0;
	}
	requested_lane_pd = pd_output;

	/* Limit only the high-speed (200 RPM and above) steering correction.
	 * PDR keeps the unclamped request so saturation remains visible in logs. */
	if ((lane_tuning.limit_mode == LANE_PD_LIMIT_MODE_ALL_SPEED) ||
	    ((lane_tuning.limit_mode == LANE_PD_LIMIT_MODE_HIGH_SPEED) &&
	     (control_rpm >= RPM_MID_HIGH_BOUNDARY))) {
		/* max_correction_rpm is the steering allowance at 100 RPM. Scale
		 * it with the commanded average so curvature remains consistent. */
		steering_base_rpm = control_rpm;
		if ((abs_error >= LANE_CURVE_ERROR_PIXELS) &&
		    (steering_base_rpm > LANE_CURVE_BASE_RPM)) {
			steering_base_rpm = LANE_CURVE_BASE_RPM;
		}
		if ((lane_dropout_active != false) &&
		    (steering_base_rpm > LANE_DROPOUT_BASE_RPM)) {
			steering_base_rpm = LANE_DROPOUT_BASE_RPM;
		}
		correction_limit = (int32_t)(((uint32_t)lane_tuning.max_correction_rpm *
				(uint32_t)steering_base_rpm + 50U) / 100U);
		/* The previous proportional scaling reduced a 35 RPM allowance to
		 * only 21 RPM at curve speed, which caused visible understeer. */
		if (((curve_preview_active != false) ||
		     (abs_error >= LANE_CURVE_ERROR_PIXELS)) &&
		    (correction_limit < LANE_CURVE_MIN_CORRECTION_RPM)) {
			correction_limit = LANE_CURVE_MIN_CORRECTION_RPM;
		}
		if ((steering_base_rpm != 0U) && (correction_limit < 1)) {
			correction_limit = 1;
		}
		if (pd_output > correction_limit) {
			pd_output = correction_limit;
		} else if (pd_output < -correction_limit) {
			pd_output = -correction_limit;
		}
	}
	/* AUTO retains full curve authority. On a confirmed straight, increase
	 * correction in small bands so a modest offset does not immediately hit
	 * the full differential and cross the lane center. */
	if (auto_straight_mode != false) {
		int32_t straight_limit;
		int32_t straight_medium_limit = APP_InterpolateGain(2, 3, 3,
			control_rpm);
		int32_t straight_max_limit = APP_InterpolateGain(5, 6, 6,
			control_rpm);

		if ((abs_error < AUTO_STRAIGHT_FINE_ERROR_PIXELS) ||
		    (auto_straight_error_count < AUTO_STRAIGHT_STAGE2_FRAMES))
			straight_limit = AUTO_STRAIGHT_FINE_CORRECTION_RPM;
		else if ((abs_error < AUTO_STRAIGHT_MEDIUM_ERROR_PIXELS) ||
		         (auto_straight_error_count < AUTO_STRAIGHT_STAGE3_FRAMES))
			straight_limit = straight_medium_limit;
		else
			straight_limit = straight_max_limit;

		if (pd_output > straight_limit) {
			pd_output = straight_limit;
		} else if (pd_output < -straight_limit) {
			pd_output = -straight_limit;
		}
	}

	/* On an AUTO straight, react to an opposite error immediately but keep
	 * the first response small.  Persistent evidence raises the authority from
	 * 1 to 3 RPM, then releases the normal (at most 5 RPM) straight correction.
	 * Curves and lower speed modes retain the existing neutral confirmation. */
	requested_sign = (pd_output > 0) ? 1 : ((pd_output < 0) ? -1 : 0);
	reversal_confirm_frames = (auto_straight_mode != false) ?
		AUTO_STRAIGHT_REVERSAL_CONFIRM_FRAMES : LANE_DIRECTION_CONFIRM_FRAMES;
	if ((requested_sign != 0) && (lane_steering_sign != 0) &&
	    (requested_sign != lane_steering_sign)) {
		if (lane_pending_sign == requested_sign) {
			if (lane_pending_sign_count < reversal_confirm_frames) {
				lane_pending_sign_count++;
			}
		} else {
			lane_pending_sign = requested_sign;
			lane_pending_sign_count = 1U;
		}
		if (lane_pending_sign_count < reversal_confirm_frames) {
			if (auto_straight_mode != false) {
				int32_t reversal_limit =
					(lane_pending_sign_count <= 1U) ?
					AUTO_STRAIGHT_REVERSAL_FIRST_RPM :
					AUTO_STRAIGHT_REVERSAL_SECOND_RPM;

				if (pd_output > reversal_limit)
					pd_output = reversal_limit;
				else if (pd_output < -reversal_limit)
					pd_output = -reversal_limit;
			} else {
				pd_output = 0;
			}
		} else {
			lane_steering_sign = requested_sign;
			lane_pending_sign = 0;
			lane_pending_sign_count = 0U;
		}
	} else if (requested_sign != 0) {
		lane_steering_sign = requested_sign;
		lane_pending_sign = 0;
		lane_pending_sign_count = 0U;
	} else {
		lane_pending_sign = 0;
		lane_pending_sign_count = 0U;
	}

	if (vehicle_enabled == false) {
		APP_ResetLanePDSlew();
	}
	else if (error == 0) {
		/* Do not carry a previous turn through a centered lane sample. */
		if (Lane_Abs32(current_lane_error) >
		    LANE_CENTER_RETURN_RELEASE_PIXELS) {
			lane_center_settle_active = true;
			lane_center_settle_tick = millis();
		}
		APP_ResetLanePDSlew();
	}
	else if (lane_tuning.slew_mode == LANE_PD_SLEW_MODE_FRAME) {
		if (pd_output > current_lane_pd) {
			current_lane_pd += LANE_PD_SLEW_FRAME_STEP_RPM;
			if (current_lane_pd > pd_output) current_lane_pd = pd_output;
		} else if (pd_output < current_lane_pd) {
			current_lane_pd -= LANE_PD_SLEW_FRAME_STEP_RPM;
			if (current_lane_pd < pd_output) current_lane_pd = pd_output;
		}
	}
	else if (lane_tuning.slew_mode == LANE_PD_SLEW_MODE_TIME) {
		/* Release an existing turn immediately as the lane approaches center.
		 * This only reduces correction in its current direction; an opposite
		 * command still passes through direction confirmation and normal slew. */
		if ((abs_error <= LANE_CENTER_RETURN_RELEASE_PIXELS) &&
		    (((current_lane_pd > 0) && (pd_output >= 0) &&
		      (pd_output < current_lane_pd)) ||
		     ((current_lane_pd < 0) && (pd_output <= 0) &&
		      (pd_output > current_lane_pd)))) {
			current_lane_pd = pd_output;
			lane_pd_ramp_active = false;
		} else {
			current_lane_pd = APP_UpdateTimeBasedLanePD(pd_output,
				APP_GetLanePDSlewDurationMs());
		}
	}
	else {
		current_lane_pd = pd_output;
	}

	current_lane_error = error;
	current_kp_scaled = kp_scaled;
	current_kd_scaled = kd_scaled;

	return current_lane_pd;
}

/* =========================================================
 * APP_GetCurrentRPM
 *
 * Description:
 * Reads the latest Motor1 and Motor2 status values and
 * estimates the current vehicle RPM.
 *
 * Behavior:
 * - If both motor status values are valid, returns the
 *   average absolute RPM.
 * - If only one motor status is valid, returns that motor's
 *   absolute RPM.
 * - If neither status is valid, returns current_base_rpm.
 *
 * Return:
 * Current estimated vehicle RPM.
 * ========================================================= */

static uint16_t APP_GetCurrentRPM(void)
{
	MotorStatus_t motor1_status = {0};
	MotorStatus_t motor2_status = {0};

	bool motor1_valid;
	bool motor2_valid;

	int32_t motor1_rpm_abs = 0;
	int32_t motor2_rpm_abs = 0;
	uint32_t rpm_sum;

	motor1_valid = GetMotorStatus(ECU_MOTOR1, &motor1_status);

	motor2_valid = GetMotorStatus(ECU_MOTOR2, &motor2_status);

	if ((motor1_valid != false) && (motor2_valid != false)) {
		motor1_rpm_abs = (int32_t)motor1_status.rpm;

		motor2_rpm_abs = (int32_t)motor2_status.rpm;

		if (motor1_rpm_abs < 0) {
			motor1_rpm_abs = -motor1_rpm_abs;
		}

		if (motor2_rpm_abs < 0) {
			motor2_rpm_abs = -motor2_rpm_abs;
		}

		rpm_sum = (uint32_t)motor1_rpm_abs + (uint32_t)motor2_rpm_abs;

		return (uint16_t)(rpm_sum >> 1U);
	}

	if (motor1_valid != false) {
		motor1_rpm_abs = (int32_t)motor1_status.rpm;

		if (motor1_rpm_abs < 0) {
			motor1_rpm_abs = -motor1_rpm_abs;
		}

		return (uint16_t)motor1_rpm_abs;
	}

	if (motor2_valid != false) {
		motor2_rpm_abs = (int32_t)motor2_status.rpm;

		if (motor2_rpm_abs < 0) {
			motor2_rpm_abs = -motor2_rpm_abs;
		}

		return (uint16_t)motor2_rpm_abs;
	}

	return current_base_rpm;
}

static int32_t Lane_Abs32(int32_t value)
{
	return (value < 0) ? -value : value;
}

/* Reconstruct the current NEAR center while either ROI is untrustworthy.
 * FAR-to-NEAR curvature describes where the previously observed road will
 * arrive as the vehicle covers the look-ahead separation.  Advancing the last
 * trusted NEAR by that distance produces a bounded virtual center instead of
 * freezing on a stale pixel or accepting a newly exposed lane boundary.
 * Lane_UpdateState still enters MEMORY and enforces MEMDIST/MEMTIME, so this
 * estimate cannot extend driving beyond the existing safety limits. */
static uint16_t Lane_GetVirtualNearX(uint16_t measured_near_x,
		bool measurement_trusted)
{
	uint32_t lookahead_span_mm;
	uint32_t travelled_mm;
	int32_t shift;
	int32_t virtual_x;

	if (measurement_trusted != false) {
		lane_virtual_anchor_near_x = measured_near_x;
		lane_virtual_anchor_curve = lane_curve_filtered;
		lane_virtual_anchor_distance_mm = lane_distance_mm;
		lane_virtual_anchor_valid = true;
		return measured_near_x;
	}

	if (lane_virtual_anchor_valid == false) return current_used_lane_x;

	lookahead_span_mm = (lane_tuning.far_lookahead_mm >
			lane_tuning.near_lookahead_mm) ?
			(lane_tuning.far_lookahead_mm -
			 lane_tuning.near_lookahead_mm) : 1U;
	travelled_mm = lane_distance_mm - lane_virtual_anchor_distance_mm;
	if (travelled_mm > lookahead_span_mm) travelled_mm = lookahead_span_mm;

	shift = ((int32_t)lane_virtual_anchor_curve *
			(int32_t)travelled_mm) / (int32_t)lookahead_span_mm;
	if (shift > LANE_VIRTUAL_CENTER_MAX_SHIFT_PIXELS)
		shift = LANE_VIRTUAL_CENTER_MAX_SHIFT_PIXELS;
	else if (shift < -LANE_VIRTUAL_CENTER_MAX_SHIFT_PIXELS)
		shift = -LANE_VIRTUAL_CENTER_MAX_SHIFT_PIXELS;

	virtual_x = (int32_t)lane_virtual_anchor_near_x + shift;
	if (virtual_x < 0) virtual_x = 0;
	else if (virtual_x > (int32_t)LANE_IMAGE_MAX_X)
		virtual_x = (int32_t)LANE_IMAGE_MAX_X;
	return (uint16_t)virtual_x;
}

static void Lane_ResetPathMemory(void)
{
	uint8_t index;

	for (index = 0U; index < PATH_MEM_SIZE; index++) {
		path_memory[index].center_x = (int16_t)lane_tuning.center_x;
		path_memory[index].curvature = 0;
		path_memory[index].confidence = 0U;
		path_memory[index].valid = 0U;
	}
	lane_control_state = LANE_TRACK;
	lane_confidence = 0U;
	lane_bad_count = 0U;
	lane_good_count = 0U;
	lane_recover_blend_percent = 100U;
	lane_curve_raw = 0;
	lane_curve_filtered = 0;
	lane_weak_curve_direction = 0;
	lane_weak_curve_candidate = 0;
	lane_weak_curve_count = 0U;
	lane_center_settle_active = false;
	lane_center_settle_tick = 0U;
	lane_path_curve = 0;
	lane_steer_feedback = 0;
	lane_steer_feedforward = 0;
	lane_steer_total = 0;
	lane_speed_scale_percent = 100U;
	lane_far_loss_active = false;
	lane_far_loss_count = 0U;
	lane_far_recover_count = 0U;
	lane_distance_mm = 0U;
	lane_distance_remainder = 0U;
	lane_path_step_accum_mm = 0U;
	lane_distance_last_tick = millis();
	lane_memory_start_distance_mm = 0U;
	lane_memory_start_tick = 0U;
	lane_memory_timed_out = false;
	lane_roi_history_valid = false;
	lane_previous_far_x = (uint16_t)lane_tuning.center_x;
	lane_previous_near_x = (uint16_t)lane_tuning.center_x;
	lane_previous_curve_raw = 0;
	lane_path_write_index = 0U;
	lane_virtual_anchor_near_x = (uint16_t)lane_tuning.center_x;
	lane_virtual_anchor_curve = 0;
	lane_virtual_anchor_distance_mm = 0U;
	lane_virtual_anchor_valid = false;
	lane_memory_search_direction = 0;
	lane_memory_search_correction = 0;
	auto_straight_state_reset_count = 0U;
}

static void Lane_UpdateFarLossSpeed(bool far_valid)
{
	if (far_valid == false) {
		lane_far_recover_count = 0U;
		if (lane_far_loss_count < LANE_FAR_LOSS_CONFIRM_FRAMES)
			lane_far_loss_count++;
		if (lane_far_loss_count >= LANE_FAR_LOSS_CONFIRM_FRAMES)
			lane_far_loss_active = true;
	} else {
		lane_far_loss_count = 0U;
		if (lane_far_loss_active != false) {
			if (lane_far_recover_count < LANE_FAR_RECOVER_CONFIRM_FRAMES)
				lane_far_recover_count++;
			if (lane_far_recover_count >= LANE_FAR_RECOVER_CONFIRM_FRAMES) {
				lane_far_loss_active = false;
				lane_far_recover_count = 0U;
			}
		} else {
			lane_far_recover_count = 0U;
		}
	}
}

static void Lane_ShiftPathMemory(void)
{
	uint8_t index;

	for (index = 0U; index < (PATH_MEM_SIZE - 1U); index++) {
		path_memory[index] = path_memory[index + 1U];
	}
	path_memory[PATH_MEM_SIZE - 1U].center_x =
			(int16_t)lane_tuning.center_x;
	path_memory[PATH_MEM_SIZE - 1U].curvature = 0;
	path_memory[PATH_MEM_SIZE - 1U].confidence = 0U;
	path_memory[PATH_MEM_SIZE - 1U].valid = 0U;
}

/* Integrate measured wheel RPM into a short-range distance estimate. There is
 * no encoder count in the current CAN protocol; actual motor-status RPM is
 * preferred and APP_GetCurrentRPM falls back to commanded RPM only when both
 * status frames are unavailable. */
static void Lane_UpdateDistance(void)
{
	uint32_t now = millis();
	uint32_t dt_ms;
	uint32_t numerator;
	uint32_t distance_delta;
	uint16_t rpm;

	if (lane_distance_last_tick == 0U) {
		lane_distance_last_tick = now;
		return;
	}
	dt_ms = (uint32_t)(now - lane_distance_last_tick);
	lane_distance_last_tick = now;
	if ((vehicle_enabled == false) || (dt_ms == 0U)) return;
	if (dt_ms > 250U) dt_ms = 250U;

	rpm = APP_GetCurrentRPM();
	numerator = ((uint32_t)rpm * WHEEL_CIRCUMFERENCE_MM * dt_ms) +
			lane_distance_remainder;
	distance_delta = numerator / 60000U;
	lane_distance_remainder = numerator % 60000U;
	lane_distance_mm += distance_delta;
	lane_path_step_accum_mm += distance_delta;

	while ((lane_tuning.path_step_mm != 0U) &&
	       (lane_path_step_accum_mm >= lane_tuning.path_step_mm)) {
		lane_path_step_accum_mm -= lane_tuning.path_step_mm;
		Lane_ShiftPathMemory();
	}
}

static uint8_t Lane_UpdateConfidence(uint16_t far_x, uint16_t near_x,
		bool far_valid, bool near_valid, uint32_t age_ms)
{
	int32_t score = 100;
	int32_t diff = (int32_t)far_x - (int32_t)near_x;
	int32_t abs_diff = Lane_Abs32(diff);
	int32_t far_jump = 0;
	int32_t near_jump = 0;
	int32_t curve_for_filter;

	lane_curve_raw = (int16_t)diff;
	if (near_valid == false) score -= 60;
	if (far_valid == false) score -= 30;
	if (age_ms >= 1000U) {
		score = 0;
	} else if (age_ms > 250U) {
		score -= (int32_t)(((age_ms - 250U) * 40U) / 750U);
	}

	if ((far_valid != false) && (near_valid != false)) {
		if (abs_diff >= (int32_t)lane_tuning.far_near_reject_diff) {
			score -= 70;
		} else if (abs_diff > (int32_t)lane_tuning.far_near_warn_diff) {
			uint32_t span = lane_tuning.far_near_reject_diff -
					lane_tuning.far_near_warn_diff;
			uint32_t excess = (uint32_t)abs_diff -
					lane_tuning.far_near_warn_diff;
			score -= 20 + (int32_t)((excess * 30U) / span);
		}

		if (lane_roi_history_valid != false) {
			far_jump = Lane_Abs32((int32_t)far_x -
					(int32_t)lane_previous_far_x);
			near_jump = Lane_Abs32((int32_t)near_x -
					(int32_t)lane_previous_near_x);
			if ((far_jump >= LANE_ROI_JUMP_REJECT_PIXELS) ||
			    (near_jump >= LANE_ROI_JUMP_REJECT_PIXELS)) {
				score -= 50;
			} else if ((far_jump >= LANE_ROI_JUMP_WARN_PIXELS) ||
			           (near_jump >= LANE_ROI_JUMP_WARN_PIXELS)) {
				score -= 20;
			}
			if ((Lane_Abs32(lane_previous_curve_raw) >=
			     LANE_CURVE_FLIP_MARGIN_PIXELS) &&
			    (abs_diff >= LANE_CURVE_FLIP_MARGIN_PIXELS) &&
			    (((lane_previous_curve_raw > 0) && (diff < 0)) ||
			     ((lane_previous_curve_raw < 0) && (diff > 0)))) {
				score -= 25;
			}
		}
	}

	if (score < 0) score = 0;
	if (score > 100) score = 100;
	lane_confidence = (uint8_t)score;

	/* Establish weak curve direction from two consecutive trusted raw ROI
	 * differences.  The filtered curve intentionally moves slowly and was too
	 * late to protect launch steering on a shallow bend. */
	if ((far_valid != false) && (near_valid != false) &&
	    (lane_confidence >= (uint8_t)lane_tuning.confidence_low) &&
	    (abs_diff < (int32_t)lane_tuning.far_near_reject_diff) &&
	    (abs_diff >= LANE_WEAK_CURVE_DIRECTION_PIXELS)) {
		int8_t direction = (diff < 0) ? -1 : 1;
		if (direction == lane_weak_curve_candidate) {
			if (lane_weak_curve_count < LANE_WEAK_CURVE_DIRECTION_CONFIRM_FRAMES)
				lane_weak_curve_count++;
		} else {
			lane_weak_curve_candidate = direction;
			lane_weak_curve_count = 1U;
		}
		if (lane_weak_curve_count >= LANE_WEAK_CURVE_DIRECTION_CONFIRM_FRAMES)
			lane_weak_curve_direction = direction;
	} else {
		lane_weak_curve_candidate = 0;
		lane_weak_curve_count = 0U;
		lane_weak_curve_direction = 0;
	}

	if ((far_valid != false) && (near_valid != false) &&
	    (lane_confidence >= (uint8_t)lane_tuning.confidence_low) &&
	    (abs_diff < (int32_t)lane_tuning.far_near_reject_diff)) {
		curve_for_filter = diff;
		if (curve_for_filter > LANE_CURVATURE_MAX_PIXELS)
			curve_for_filter = LANE_CURVATURE_MAX_PIXELS;
		else if (curve_for_filter < -LANE_CURVATURE_MAX_PIXELS)
			curve_for_filter = -LANE_CURVATURE_MAX_PIXELS;
		lane_curve_filtered += (int16_t)((curve_for_filter -
				lane_curve_filtered) / LANE_CURVATURE_FILTER_DIVISOR);
	}

	if ((far_valid != false) && (near_valid != false)) {
		lane_previous_far_x = far_x;
		lane_previous_near_x = near_x;
		lane_previous_curve_raw = (int16_t)diff;
		lane_roi_history_valid = true;
	}
	return lane_confidence;
}

/* AUTO can outrun a weak preview correction and expose the opposite lane
 * boundary. Keep either established bend through that transient. Direction
 * is derived from consecutive FAR-minus-NEAR geometry, not from the absolute
 * NEAR position, so starting on either side of the lane behaves identically. */
static void Lane_ResetAutoState(void)
{
	auto_curve_latched_direction = 0;
	auto_last_trusted_curve_direction = 0;
	auto_last_strong_steering_direction = 0;
	auto_trusted_straight_count = 0U;
	auto_last_strong_steering_tick = 0U;
	auto_raw_opposite_count = 0U;
	lane_geometry_untrusted = true;
	auto_curve_release_count = 0U;
	auto_opposite_evidence_count = 0U;
	auto_strong_curve_count = 0U;
	auto_entry_candidate_direction = 0;
	auto_entry_count = 0U;
	auto_exit_trend_count = 0U;
	auto_previous_abs_curve = 0;
	auto_exit_reference_curve = 0;
	auto_exit_neutral_active = false;
	auto_exit_neutral_tick = 0U;
	auto_post_exit_track_active = false;
	auto_new_curve_candidate_direction = 0;
	auto_new_curve_count = 0U;
	auto_pair_history_valid = false;
	auto_pair_previous_far_x = (uint16_t)lane_tuning.center_x;
	auto_pair_previous_near_x = (uint16_t)lane_tuning.center_x;
	auto_pair_stable_count = 0U;
	auto_curve_authority_direction = 0;
	auto_curve_authority_count = 0U;
	auto_near_measurement_trusted = false;
	auto_curve_exit_center_count = 0U;
}

static void Lane_ClearStaleCurveStateOnStraight(uint16_t near_x)
{
	uint8_t index;

	Lane_ResetAutoState();
	for (index = 0U; index < PATH_MEM_SIZE; index++) {
		path_memory[index].center_x = (int16_t)lane_tuning.center_x;
		path_memory[index].curvature = 0;
		path_memory[index].confidence = 0U;
		path_memory[index].valid = 0U;
	}
	lane_curve_raw = 0;
	lane_curve_filtered = 0;
	lane_path_curve = 0;
	lane_steer_feedforward = 0;
	lane_virtual_anchor_near_x = near_x;
	lane_virtual_anchor_curve = 0;
	lane_virtual_anchor_distance_mm = lane_distance_mm;
	lane_virtual_anchor_valid = true;
	lane_memory_search_direction = 0;
	lane_memory_search_correction = 0;
}

static void Lane_UpdateStraightStateReset(uint16_t far_x, uint16_t near_x,
		bool far_trusted)
{
	bool confirmed_geometry = (far_trusted != false) &&
		(Lane_Abs32((int32_t)far_x - (int32_t)near_x) <=
		 AUTO_STRAIGHT_STATE_RESET_CURVE_PIXELS) &&
		(Lane_Abs32((int32_t)lane_tuning.center_x - (int32_t)near_x) <=
		 AUTO_STRAIGHT_STATE_RESET_CENTER_PIXELS);

	if (confirmed_geometry == false) {
		auto_straight_state_reset_count = 0U;
		return;
	}

	if (auto_straight_state_reset_count < AUTO_STRAIGHT_STATE_RESET_FRAMES)
		auto_straight_state_reset_count++;
	if (auto_straight_state_reset_count == AUTO_STRAIGHT_STATE_RESET_FRAMES) {
		Lane_ClearStaleCurveStateOnStraight(near_x);
		/* Keep the counter above the trigger so this straight clears once. */
		auto_straight_state_reset_count = AUTO_STRAIGHT_STATE_RESET_FRAMES + 1U;
	}
}

static void Lane_UpdateAutoCurveLatch(uint16_t far_x, uint16_t near_x,
		bool far_valid, bool near_valid)
{
	int32_t diff = (int32_t)far_x - (int32_t)near_x;
	int32_t abs_diff = Lane_Abs32(diff);
	int32_t previous_abs_curve = auto_previous_abs_curve;
	int8_t observed_direction = (diff < 0) ? -1 : ((diff > 0) ? 1 : 0);
	int32_t near_error = (int32_t)lane_tuning.center_x - (int32_t)near_x;
	int32_t entry_jump_pixels = APP_InterpolateGain(
		AUTO_CURVE_ENTRY_JUMP_PIXELS,
		AUTO_CURVE_ENTRY_JUMP_PIXELS + 40,
		AUTO_CURVE_ENTRY_JUMP_PIXELS + 80,
		APP_GetSteeringControlRPM());
	uint8_t path_index;
	bool pair_candidate = (far_valid != false) && (near_valid != false) &&
		(lane_confidence >= (uint8_t)lane_tuning.confidence_low) &&
		(abs_diff < (int32_t)lane_tuning.far_near_reject_diff) &&
		(abs_diff <= (int32_t)AUTO_FAR_EDGE_SWITCH_PIXELS);
	bool pair_stable = pair_candidate;
	bool near_agrees;
	bool trusted_pair;

	if (vehicle_enabled == false) {
		Lane_ResetAutoState();
		return;
	}
	if ((pair_stable != false) && (auto_pair_history_valid != false) &&
	    ((Lane_Abs32((int32_t)far_x - (int32_t)auto_pair_previous_far_x) >
	      entry_jump_pixels) ||
	     (Lane_Abs32((int32_t)near_x - (int32_t)auto_pair_previous_near_x) >
	      entry_jump_pixels))) {
		pair_stable = false;
	}
	if (pair_candidate != false) {
		auto_pair_previous_far_x = far_x;
		auto_pair_previous_near_x = near_x;
		auto_pair_history_valid = true;
	} else {
		auto_pair_history_valid = false;
	}
	if (pair_stable != false) {
		if (auto_pair_stable_count < AUTO_CURVE_PAIR_CONFIRM_FRAMES)
			auto_pair_stable_count++;
	} else {
		auto_pair_stable_count = 0U;
	}
	trusted_pair = pair_stable &&
		(auto_pair_stable_count >= AUTO_CURVE_PAIR_CONFIRM_FRAMES);
	if ((trusted_pair != false) &&
	    (abs_diff >= LANE_WEAK_CURVE_DIRECTION_PIXELS)) {
		auto_last_trusted_curve_direction = observed_direction;
	} else if ((trusted_pair != false) &&
	           (abs_diff <= (int32_t)lane_tuning.curve_exit_pixels)) {
		/* A confirmed straight, unlike an invalid FAR sample, safely clears
		 * the remembered bend direction. */
		auto_last_trusted_curve_direction = 0;
	}
	near_agrees = ((observed_direction < 0) &&
		(near_error > (int32_t)AUTO_STRAIGHT_DEADBAND_PIXELS)) ||
		((observed_direction > 0) &&
		 (near_error < -(int32_t)AUTO_STRAIGHT_DEADBAND_PIXELS));
	if ((trusted_pair != false) &&
	    (abs_diff >= LANE_WEAK_CURVE_DIRECTION_PIXELS)) {
		if (auto_curve_authority_direction == observed_direction) {
			if (auto_curve_authority_count < AUTO_CURVE_FULL_CONFIRM_FRAMES)
				auto_curve_authority_count++;
		} else {
			auto_curve_authority_direction = observed_direction;
			auto_curve_authority_count = 1U;
		}
		if ((near_agrees != false) && (auto_curve_authority_count < 3U))
			auto_curve_authority_count = 3U;
	} else if ((auto_curve_latched_direction == 0) ||
	           ((observed_direction != 0) &&
	            (observed_direction != auto_curve_latched_direction))) {
		/* Lateral NEAR error is handled only by the bounded straight recovery.
		 * It must never grant curve authority by itself. A brief same-direction
		 * ROI dropout, however, preserves the already proven entry stage. */
		auto_curve_authority_direction = 0;
		auto_curve_authority_count = 0U;
	}
	if ((trusted_pair != false) &&
	    (abs_diff >= AUTO_STRONG_LEFT_CURVE_PIXELS) &&
	    ((near_agrees != false) || (auto_curve_authority_count >= 3U))) {
		if (auto_strong_curve_count < AUTO_STRONG_LEFT_CONFIRM_FRAMES)
			auto_strong_curve_count++;
	} else {
		auto_strong_curve_count = 0U;
	}
	if ((trusted_pair != false) &&
	    (abs_diff >= LANE_WEAK_CURVE_DIRECTION_PIXELS)) {
		if (auto_entry_candidate_direction == observed_direction) {
			if (auto_entry_count < AUTO_LEFT_ENTRY_CONFIRM_FRAMES)
				auto_entry_count++;
		} else {
			auto_entry_candidate_direction = observed_direction;
			auto_entry_count = 1U;
		}
	} else {
		auto_entry_candidate_direction = 0;
		auto_entry_count = 0U;
	}
	/* After leaving a bend, use geometry rather than a fixed straight timer.
	 * Ignore old FAR/path prediction until a new signed curve grows for two
	 * consecutive frames. This gate is independent of lateral start offset. */
	if (auto_post_exit_track_active != false) {
		if ((trusted_pair != false) &&
		    (abs_diff >= LANE_WEAK_CURVE_DIRECTION_PIXELS) &&
		    (near_agrees != false) &&
		    (previous_abs_curve > 0) && (abs_diff > previous_abs_curve)) {
			if (auto_new_curve_candidate_direction == observed_direction) {
				if (auto_new_curve_count < AUTO_NEW_CURVE_CONFIRM_FRAMES)
					auto_new_curve_count++;
			} else {
				auto_new_curve_candidate_direction = observed_direction;
				auto_new_curve_count = 1U;
			}
			if (auto_new_curve_count >= AUTO_NEW_CURVE_CONFIRM_FRAMES) {
				auto_post_exit_track_active = false;
				auto_exit_neutral_active = false;
				auto_new_curve_candidate_direction = 0;
				auto_new_curve_count = 0U;
			}
		} else {
			auto_new_curve_candidate_direction = 0;
			auto_new_curve_count = 0U;
		}
	}

	/* Enter post-exit tracking as soon as the established signed curvature
	 * drops materially without changing direction. */
	if ((auto_curve_latched_direction != 0) &&
	    (trusted_pair != false) &&
	    (observed_direction == auto_curve_latched_direction) &&
	    (abs_diff < AUTO_EXIT_TREND_CURVE_PIXELS) &&
	    (auto_previous_abs_curve > 0) &&
	    ((auto_previous_abs_curve - abs_diff) >=
	     AUTO_EXIT_DROP_MIN_PIXELS)) {
		if (auto_exit_trend_count < AUTO_EXIT_TREND_CONFIRM_FRAMES)
			auto_exit_trend_count++;
	} else {
		auto_exit_trend_count = 0U;
	}
	if (trusted_pair != false) {
		auto_previous_abs_curve = abs_diff;
	}
	if (auto_exit_trend_count >= AUTO_EXIT_TREND_CONFIRM_FRAMES) {
		auto_curve_latched_direction = 0;
		auto_curve_release_count = 0U;
		auto_opposite_evidence_count = 0U;
		auto_exit_trend_count = 0U;
		auto_exit_reference_curve = abs_diff;
		auto_exit_neutral_active = true;
		auto_exit_neutral_tick = millis();
		auto_post_exit_track_active = true;
		auto_new_curve_candidate_direction = 0;
		auto_new_curve_count = 0U;
		for (path_index = 0U; path_index < PATH_MEM_SIZE; path_index++) {
			path_memory[path_index].curvature = 0;
			path_memory[path_index].confidence = 0U;
			path_memory[path_index].valid = 0U;
		}
		lane_path_curve = 0;
		lane_steer_feedforward = 0;
		return;
	}

	if ((auto_exit_neutral_active == false) &&
	    (auto_post_exit_track_active == false) &&
	    (auto_curve_latched_direction == 0) &&
	    (trusted_pair != false) &&
	    (abs_diff >= LANE_WEAK_CURVE_DIRECTION_PIXELS) &&
	    (auto_entry_count >= AUTO_LEFT_ENTRY_CONFIRM_FRAMES)) {
		auto_curve_latched_direction = auto_entry_candidate_direction;
		auto_curve_release_count = 0U;
		auto_opposite_evidence_count = 0U;
		return;
	}

	if (auto_curve_latched_direction == 0) return;

	if ((trusted_pair != false) &&
	    (observed_direction == -auto_curve_latched_direction) &&
	    (abs_diff >= AUTO_STRONG_OPPOSITE_CURVE_PIXELS)) {
		auto_curve_release_count = 0U;
		if (auto_opposite_evidence_count < AUTO_OPPOSITE_CONFIRM_FRAMES)
			auto_opposite_evidence_count++;
		if (auto_opposite_evidence_count >= AUTO_OPPOSITE_CONFIRM_FRAMES) {
			auto_curve_latched_direction = observed_direction;
			auto_opposite_evidence_count = 0U;
		}
		return;
	}

	auto_opposite_evidence_count = 0U;
	if ((trusted_pair != false) &&
	    (abs_diff <= (int32_t)lane_tuning.curve_exit_pixels)) {
		if (auto_curve_release_count < AUTO_LEFT_LATCH_RELEASE_FRAMES)
			auto_curve_release_count++;
		if (auto_curve_release_count >= AUTO_LEFT_LATCH_RELEASE_FRAMES) {
			auto_curve_latched_direction = 0;
			auto_curve_release_count = 0U;
		}
	} else {
		/* Invalid, jumping, or one-sided ROI data must not look like a
		 * confirmed curve exit. */
		auto_curve_release_count = 0U;
	}
}

static void Lane_UpdatePathMemory(uint16_t near_x, bool dual_lane_valid)
{
	uint32_t index;
	PathPoint *point;

	if ((dual_lane_valid == false) ||
	    (lane_confidence < (uint8_t)lane_tuning.confidence_high) ||
	    (Lane_Abs32(lane_curve_raw) >=
	     (int32_t)lane_tuning.far_near_reject_diff) ||
	    (lane_tuning.path_step_mm == 0U)) return;

	index = (lane_tuning.far_lookahead_mm +
			(lane_tuning.path_step_mm / 2U)) / lane_tuning.path_step_mm;
	if (index >= PATH_MEM_SIZE) index = PATH_MEM_SIZE - 1U;
	lane_path_write_index = (uint8_t)index;
	point = &path_memory[index];
	if (point->valid != 0U) {
		point->center_x = (int16_t)(((int32_t)point->center_x +
				(int32_t)near_x) / 2);
		point->curvature = (int16_t)(((int32_t)point->curvature * 3 +
				(int32_t)lane_curve_filtered) / 4);
		if (lane_confidence > point->confidence)
			point->confidence = lane_confidence;
	} else {
		point->center_x = (int16_t)near_x;
		point->curvature = lane_curve_filtered;
		point->confidence = lane_confidence;
		point->valid = 1U;
	}
}

static void Lane_EnterMemory(void)
{
	int32_t last_error = (int32_t)lane_tuning.center_x -
			(int32_t)current_used_lane_x;

	lane_control_state = LANE_MEMORY;
	lane_memory_start_distance_mm = lane_distance_mm;
	lane_memory_start_tick = millis();
	lane_memory_timed_out = false;
	lane_recover_blend_percent = 0U;
	/* Continue the actual steering direction that was being commanded just
	 * before lane loss. Lane_UpdateState runs before APP_FilterLaneX processes
	 * the current sample, so current_used_lane_x alone can describe a stale
	 * opposite-side sample and reverse the search direction at MEMORY entry. */
	if (lane_steer_total > 0)
		lane_memory_search_direction = 1;
	else if (lane_steer_total < 0)
		lane_memory_search_direction = -1;
	else if (last_error > (int32_t)lane_tuning.deadband_pixels)
		lane_memory_search_direction = 1;
	else if (last_error < -(int32_t)lane_tuning.deadband_pixels)
		lane_memory_search_direction = -1;
	else if (lane_curve_filtered > 0)
		lane_memory_search_direction = -1;
	else if (lane_curve_filtered < 0)
		lane_memory_search_direction = 1;
	else
		lane_memory_search_direction = 0;
	lane_memory_search_correction = 0;
}

static void Lane_UpdateState(bool near_valid)
{
	/* NEAR is the present-position signal and alone controls entry to MEMORY.
	 * FAR confidence must not turn a still-valid NEAR measurement into a lane
	 * loss; an implausible FAR is handled separately as lost preview. */
	bool good = (near_valid != false);
	bool bad = (near_valid == false);

	/* Confidence may be observed while parked, but lane-loss distance/time
	 * supervision is meaningful only during an enabled drive. Each automatic
	 * start performs a full lane/path reset before enabling the vehicle. */
	if (vehicle_enabled == false) {
		lane_control_state = (curve_preview_active != false) ?
				LANE_PREVIEW : LANE_TRACK;
		lane_bad_count = 0U;
		lane_good_count = 0U;
		lane_recover_blend_percent = 100U;
		lane_memory_timed_out = false;
		return;
	}

	if (good != false) {
		if (lane_good_count < LANE_GOOD_CONFIRM_FRAMES) lane_good_count++;
		lane_bad_count = 0U;
	} else if (bad != false) {
		if (lane_bad_count < LANE_BAD_CONFIRM_FRAMES) lane_bad_count++;
		lane_good_count = 0U;
	} else {
		lane_good_count = 0U;
		lane_bad_count = 0U;
	}

	switch (lane_control_state) {
	case LANE_TRACK:
	case LANE_PREVIEW:
		if (lane_bad_count >= LANE_BAD_CONFIRM_FRAMES) {
			Lane_EnterMemory();
		} else {
			lane_control_state = (curve_preview_active != false) ?
					LANE_PREVIEW : LANE_TRACK;
		}
		break;
	case LANE_MEMORY:
		/* Prefer a confirmed reacquisition on the boundary frame. Previously
		 * timeout was latched first, permanently blocking RECOVER. */
		if (lane_good_count >= LANE_GOOD_CONFIRM_FRAMES) {
			lane_control_state = LANE_RECOVER;
			lane_recover_blend_percent = 0U;
			lane_memory_timed_out = false;
			lane_memory_search_correction = 0;
		} else if (((lane_distance_mm - lane_memory_start_distance_mm) >=
		     lane_tuning.memory_max_distance_mm) ||
		    ((uint32_t)(millis() - lane_memory_start_tick) >=
		     lane_tuning.memory_max_time_ms)) {
			lane_memory_timed_out = true;
		}
		break;
	case LANE_RECOVER:
		if (lane_bad_count >= LANE_BAD_CONFIRM_FRAMES) {
			Lane_EnterMemory();
		} else if (good != false) {
			uint32_t blend = (uint32_t)lane_recover_blend_percent +
					lane_tuning.recover_blend_rate;
			lane_recover_blend_percent = (blend >= 100U) ? 100U :
					(uint8_t)blend;
			if (lane_recover_blend_percent >= 100U) {
				lane_control_state = (curve_preview_active != false) ?
						LANE_PREVIEW : LANE_TRACK;
			}
		}
		break;
	default:
		lane_control_state = LANE_TRACK;
		break;
	}
}

static int32_t Lane_CalcFeedForward(void)
{
	int32_t feedforward;

	if ((path_memory[0].valid == 0U) ||
	    (path_memory[0].confidence < (uint8_t)lane_tuning.confidence_low)) {
		lane_path_curve = 0;
		lane_steer_feedforward = 0;
		return 0;
	}
	lane_path_curve = path_memory[0].curvature;
	/* Positive image curvature means the future lane moves right. Existing
	 * motor sign convention needs a negative differential for a right turn. */
	feedforward = -((int32_t)lane_tuning.kff_scaled *
			(int32_t)lane_path_curve) / LANE_GAIN_SCALE;
	if (feedforward > (int32_t)lane_tuning.max_correction_rpm)
		feedforward = (int32_t)lane_tuning.max_correction_rpm;
	else if (feedforward < -(int32_t)lane_tuning.max_correction_rpm)
		feedforward = -(int32_t)lane_tuning.max_correction_rpm;
	lane_steer_feedforward = (int16_t)feedforward;
	return feedforward;
}

static int32_t Lane_CombineSteering(int32_t feedback, int32_t feedforward)
{
	int32_t total;
	int32_t limit = (int32_t)lane_tuning.max_correction_rpm;
	int32_t abs_curve = Lane_Abs32(lane_curve_raw);
	int32_t latch_floor = 0;
	uint16_t control_rpm = APP_GetSteeringControlRPM();
	int32_t preview_limit = APP_ScaleCorrectionForRPM(
		AUTO_LEFT_PREVIEW_MAX_CORRECTION_RPM, control_rpm);
	int32_t weak_max = APP_ScaleCorrectionForRPM(
		AUTO_WEAK_LEFT_MAX_CORRECTION_RPM, control_rpm);
	int32_t latch_min = APP_ScaleCorrectionForRPM(
		AUTO_LEFT_LATCH_MIN_CORRECTION_RPM, control_rpm);
	int32_t release_limit = APP_ScaleCorrectionForRPM(
		AUTO_WEAK_LEFT_RELEASE_CORRECTION_RPM, control_rpm);
	int32_t far_loss_hold_min = APP_ScaleCorrectionForRPM(
		AUTO_FAR_LOSS_HOLD_MIN_CORRECTION_RPM, control_rpm);
	int32_t far_loss_decay1 = APP_ScaleCorrectionForRPM(
		AUTO_DIRECTION_DECAY1_RPM, control_rpm);
	int32_t far_loss_decay2 = APP_ScaleCorrectionForRPM(
		AUTO_DIRECTION_DECAY2_RPM, control_rpm);

	if (lane_control_state == LANE_MEMORY) {
		uint32_t elapsed = (uint32_t)(millis() - lane_memory_start_tick);
		uint32_t search = LANE_MEMORY_SEARCH_START_RPM;
		if ((lane_memory_search_direction != 0) &&
		    (lane_tuning.memory_max_time_ms != 0U)) {
			uint32_t span = LANE_MEMORY_SEARCH_MAX_RPM -
					LANE_MEMORY_SEARCH_START_RPM;
			if (elapsed > lane_tuning.memory_max_time_ms)
				elapsed = lane_tuning.memory_max_time_ms;
			search += (span * elapsed) / lane_tuning.memory_max_time_ms;
			lane_memory_search_correction = (int16_t)(
					(int32_t)search * lane_memory_search_direction);
		} else {
			lane_memory_search_correction = 0;
		}
		/* APP_FilterLaneX holds the last trusted NEAR position for 300 ms,
		 * then decays it toward center for 700 ms. Preserve that bounded
		 * correction while the distance-indexed FAR path continues to provide
		 * feed-forward. This avoids an abrupt steering release at lane loss. */
		total = feedforward + feedback + lane_memory_search_correction;
		if (limit > LANE_MEMORY_MAX_CORRECTION_RPM)
			limit = LANE_MEMORY_MAX_CORRECTION_RPM;
	} else if (lane_control_state == LANE_RECOVER) {
		lane_memory_search_correction = 0;
		total = feedforward + (feedback *
				(int32_t)lane_recover_blend_percent) / 100;
	} else {
		total = feedback + feedforward;
	}
	/* In post-exit tracking the prior FAR/path sample belongs to the completed
	 * bend.  Hold the lane with current NEAR only until new geometry is
	 * confirmed; this naturally supports both short and long straights. */
	if (auto_post_exit_track_active != false) {
		feedforward = 0;
		total = feedback;
		lane_steer_feedforward = 0;
	}
	/* FAR may see the next left bend while the vehicle is still centered on
	 * the preceding straight.  Keep only a small preview until two FAR frames
	 * and matching NEAR motion confirm entry.  A strong bend may lead after its
	 * own two-frame confirmation. */
	if ((lane_curve_raw <= -LANE_WEAK_CURVE_DIRECTION_PIXELS) &&
	    (auto_entry_count < AUTO_LEFT_ENTRY_CONFIRM_FRAMES)) {
		if (total > preview_limit)
			total = preview_limit;
		else if (total < -preview_limit)
			total = -preview_limit;
	}
	/* After a real return from a curve into the center deadband, hold neutral
	 * steering briefly so NEAR can settle. FAR can momentarily lock onto a
	 * boundary/background edge during this transition and must not immediately
	 * launch another turn. Cancel the hold if NEAR leaves a safe center band. */
	if (lane_center_settle_active != false) {
		uint32_t elapsed = (uint32_t)(millis() - lane_center_settle_tick);
		if (Lane_Abs32(current_lane_error) >=
		    LANE_CENTER_SETTLE_CANCEL_PIXELS) {
			lane_center_settle_active = false;
		} else if (elapsed < LANE_CENTER_SETTLE_MS) {
			total = 0;
			lane_weak_curve_direction = 0;
			lane_weak_curve_candidate = 0;
			lane_weak_curve_count = 0U;
		} else {
			lane_center_settle_active = false;
		}
	}
	/* When trusted FAR/NEAR geometry already establishes the bend direction,
	 * do not let absolute NEAR position command the opposite turn.  This is
	 * especially important at launch: camera mounting offset can put both ROI
	 * centers on one side of center_x even though FAR-to-NEAR motion correctly
	 * shows a weak bend.  Negative curvature is a left bend (positive motor
	 * differential); positive curvature is a right bend. */
	if (lane_weak_curve_direction < 0) {
		if ((lane_curve_raw <= -LANE_CURVE_DIRECTION_PRIORITY_PIXELS) &&
		    (total < LANE_CURVE_DIRECTION_MIN_CORRECTION_RPM))
			total = LANE_CURVE_DIRECTION_MIN_CORRECTION_RPM;
		else if (total < 0) {
			/* Preserve the proven AUTO150 behavior exactly. AUTO may
			 * retain an opposite NEAR correction only when the vehicle is
			 * already displaced beyond the center deadband. */
			if (current_lane_error >=
			    -(int32_t)lane_tuning.deadband_pixels)
				total = 0;
		}
	} else if (lane_weak_curve_direction > 0) {
		if ((lane_curve_raw >= LANE_CURVE_DIRECTION_PRIORITY_PIXELS) &&
		    (total > -LANE_CURVE_DIRECTION_MIN_CORRECTION_RPM))
			total = -LANE_CURVE_DIRECTION_MIN_CORRECTION_RPM;
		else if (total > 0) {
			if (current_lane_error <=
			    (int32_t)lane_tuning.deadband_pixels)
				total = 0;
		}
	}
	/* At AUTO, either weak bend keeps a symmetric direction latch without
	 * inheriting the normal 30 RPM curve minimum. As curvature unwinds, taper
	 * the contribution toward zero so the vehicle can enter the straight. */
	if (auto_curve_latched_direction != 0) {
		if (abs_curve >= AUTO_WEAK_LEFT_FULL_HOLD_PIXELS)
			latch_floor = latch_min;
		else if (abs_curve >= AUTO_WEAK_LEFT_RELEASE_PIXELS)
			latch_floor = release_limit;

		if ((auto_strong_curve_count < AUTO_STRONG_LEFT_CONFIRM_FRAMES)) {
			if (total > weak_max)
				total = weak_max;
			else if (total < -weak_max)
				total = -weak_max;
		}
		/* Start unwinding on the first confirmed downward curvature sample;
		 * the second sample enters the 300 ms neutral hold. */
		if ((auto_exit_trend_count > 0U) &&
		    (auto_strong_curve_count < AUTO_STRONG_LEFT_CONFIRM_FRAMES)) {
			latch_floor = 0;
			if ((auto_curve_latched_direction < 0) &&
			    (total > release_limit))
				total = release_limit;
			else if ((auto_curve_latched_direction > 0) &&
			         (total < -release_limit))
				total = -release_limit;
		}
		/* If NEAR already asks to unwind the latched turn, remove the latch floor
		 * immediately and leave only a small FAR preview while approaching the
		 * straight. */
		if ((((auto_curve_latched_direction < 0) && (feedback <= 0)) ||
		     ((auto_curve_latched_direction > 0) && (feedback >= 0))) &&
		    (auto_strong_curve_count < AUTO_STRONG_LEFT_CONFIRM_FRAMES)) {
			latch_floor = 0;
			if ((auto_curve_latched_direction < 0) &&
			    (total > release_limit))
				total = release_limit;
			else if ((auto_curve_latched_direction > 0) &&
			         (total < -release_limit))
				total = -release_limit;
		}
		if ((auto_curve_latched_direction < 0) && (total < latch_floor))
			total = latch_floor;
		else if ((auto_curve_latched_direction > 0) &&
		         (total > -latch_floor))
			total = -latch_floor;
	}
	/* When FAR disappears in the middle of a proven bend, preserve a modest
	 * speed-scaled correction in the last trusted direction. NEAR feedback may
	 * still request a stronger command, but cannot neutralize or reverse the
	 * turn until FAR geometry recovers. Positive geometry is a right bend and
	 * therefore requires a negative motor differential. */
	if ((lane_far_loss_active != false) &&
	    (auto_last_trusted_curve_direction != 0)) {
		if ((auto_last_trusted_curve_direction < 0) &&
		    (total < far_loss_hold_min))
			total = far_loss_hold_min;
		else if ((auto_last_trusted_curve_direction > 0) &&
		         (total > -far_loss_hold_min))
			total = -far_loss_hold_min;
	}
	/* A new AUTO curve cannot jump directly from straight correction to
	 * full authority. Invalid, one-sided, jumping, or direction-flipping ROI
	 * samples reset this ramp, so a false FAR edge remains a small preview. */
	if (abs_curve >= LANE_WEAK_CURVE_DIRECTION_PIXELS) {
		int32_t entry_limit;

		/* Once the two-frame FAR/NEAR pair check has passed, build useful
		 * steering authority immediately: 5 -> 10 -> 15 RPM at 200 RPM.
		 * Retain the fifth-frame full-authority gate for genuinely strong
		 * curves so a weak preview cannot jump straight to 35 RPM. */
		if (auto_curve_authority_count < 2U)
			entry_limit = APP_ScaleCorrectionForRPM(
				AUTO_CURVE_PREVIEW_RPM, control_rpm);
		else if (auto_curve_authority_count == 2U)
			entry_limit = APP_ScaleCorrectionForRPM(
				AUTO_CURVE_STAGE1_RPM, control_rpm);
		else if (auto_curve_authority_count <
		         AUTO_CURVE_FULL_CONFIRM_FRAMES)
			entry_limit = APP_ScaleCorrectionForRPM(
				AUTO_CURVE_STAGE2_RPM, control_rpm);
		else
			entry_limit = limit;

		if (total > entry_limit)
			total = entry_limit;
		else if (total < -entry_limit)
			total = -entry_limit;
	}
	/* A proven bend must not collapse back to a 3--7 RPM preview merely because
	 * one ROI flickers or the temporal pair check misses a frame. Preserve only
	 * a modest speed-scaled floor while the curve latch remains active; full
	 * authority still requires trusted pairs. The latch itself is released only
	 * by three trusted straight frames or confirmed opposite geometry. */
	if ((lane_geometry_untrusted != false) &&
	    (auto_curve_latched_direction != 0)) {
		int32_t dropout_floor;

		/* At a hairpin apex NEAR can return to image center even though the bend
		 * is not finished. After two centered frames reduce authority, but keep
		 * 12 RPM in the proven direction until a trusted FAR/NEAR straight pair
		 * releases the latch. */
		if ((auto_near_measurement_trusted != false) &&
		    (Lane_Abs32(current_lane_error) <=
		     LANE_CENTER_RETURN_RELEASE_PIXELS)) {
			if (auto_curve_exit_center_count <
			    AUTO_CURVE_EXIT_CENTER_CONFIRM_FRAMES)
				auto_curve_exit_center_count++;
		} else {
			auto_curve_exit_center_count = 0U;
		}

		if (auto_curve_exit_center_count <
		    AUTO_CURVE_EXIT_CENTER_CONFIRM_FRAMES)
			dropout_floor = APP_ScaleCorrectionForRPM(
				AUTO_CURVE_DROPOUT_MIN_RPM, control_rpm);
		else
			dropout_floor = APP_ScaleCorrectionForRPM(
				AUTO_CURVE_EXIT_HOLD_RPM, control_rpm);

		if ((auto_curve_latched_direction < 0) &&
		    (total < dropout_floor))
			total = dropout_floor;
		else if ((auto_curve_latched_direction > 0) &&
		         (total > -dropout_floor))
			total = -dropout_floor;
	} else {
		auto_curve_exit_center_count = 0U;
	}
	/* A very large lateral error usually means the vehicle is already near or
	 * beyond a lane boundary.  Keep recovery authoritative but avoid a full
	 * differential command that can carry it across the opposite boundary. */
	if ((Lane_Abs32(current_lane_error) >= LANE_LARGE_ERROR_PIXELS) &&
	    (limit > LANE_LARGE_ERROR_MAX_CORRECTION_RPM))
		limit = LANE_LARGE_ERROR_MAX_CORRECTION_RPM;
	/* Once FAR and NEAR are nearly parallel, the road is effectively straight.
	 * A large NEAR offset then needs lateral recovery, but a full correction
	 * causes the vehicle to cross the center and oscillate. */
	if ((Lane_Abs32(lane_curve_raw) < LANE_STRAIGHT_RECOVERY_CURVE_PIXELS) &&
	    (Lane_Abs32(current_lane_error) >= LANE_STRAIGHT_RECOVERY_ERROR_PIXELS) &&
	    (limit > LANE_STRAIGHT_RECOVERY_MAX_CORRECTION_RPM))
		limit = LANE_STRAIGHT_RECOVERY_MAX_CORRECTION_RPM;
	/* Neutralize steering briefly at the predicted curve exit.  Cancel the
	 * hold immediately if lateral error grows or curvature clearly returns. */
	if (auto_exit_neutral_active != false) {
		uint32_t elapsed = (uint32_t)(millis() - auto_exit_neutral_tick);
		if ((Lane_Abs32(current_lane_error) >=
		     AUTO_EXIT_NEUTRAL_CANCEL_ERROR_PIXELS) ||
		    (abs_curve >= AUTO_EXIT_TREND_CURVE_PIXELS) ||
		    (abs_curve > (auto_exit_reference_curve +
		     AUTO_EXIT_REENTRY_MARGIN_PIXELS))) {
			auto_exit_neutral_active = false;
		} else if (elapsed < AUTO_EXIT_NEUTRAL_MS) {
			total = 0;
		} else {
			auto_exit_neutral_active = false;
		}
	}
	/* Remember the direction of a steering command that was actually strong
	 * enough to enter a bend. During invalid or temporally unstable geometry,
	 * hold that direction instead of allowing a one-frame false boundary to
	 * neutralize or reverse the vehicle. Clear it only after three trusted,
	 * centered straight frames. */
	if (lane_geometry_untrusted == false) {
		if (Lane_Abs32(total) >= AUTO_STRONG_STEERING_MEMORY_RPM) {
			auto_last_strong_steering_direction = (total > 0) ? 1 : -1;
			auto_last_strong_steering_tick = millis();
			auto_trusted_straight_count = 0U;
		} else if ((Lane_Abs32(lane_curve_raw) <
		            LANE_STRAIGHT_RECOVERY_CURVE_PIXELS) &&
		           (Lane_Abs32(current_lane_error) <
		            AUTO_STRAIGHT_FINE_ERROR_PIXELS)) {
			if (auto_trusted_straight_count <
			    AUTO_TRUSTED_STRAIGHT_CLEAR_FRAMES)
				auto_trusted_straight_count++;
			if (auto_trusted_straight_count >=
			    AUTO_TRUSTED_STRAIGHT_CLEAR_FRAMES)
				auto_last_strong_steering_direction = 0;
		} else {
			auto_trusted_straight_count = 0U;
		}
	} else if (auto_last_strong_steering_direction != 0) {
		uint32_t memory_elapsed = (uint32_t)(millis() -
			auto_last_strong_steering_tick);
		int32_t memory_floor;

		if (memory_elapsed < AUTO_DIRECTION_HOLD_MS)
			memory_floor = far_loss_hold_min;
		else if (memory_elapsed < (AUTO_DIRECTION_HOLD_MS +
		         AUTO_DIRECTION_DECAY_STAGE_MS))
			memory_floor = far_loss_decay1;
		else if (memory_elapsed < (AUTO_DIRECTION_HOLD_MS +
		         (2U * AUTO_DIRECTION_DECAY_STAGE_MS)))
			memory_floor = far_loss_decay2;
		else {
			memory_floor = 0;
			auto_last_strong_steering_direction = 0;
		}
		if ((auto_last_strong_steering_direction > 0) &&
		    (total < memory_floor))
			total = memory_floor;
		else if ((auto_last_strong_steering_direction < 0) &&
		         (total > -memory_floor))
			total = -memory_floor;
	}
	/* The measured optical/lane center remains 646. Compensate only the
	 * confirmed left-bend response; straight recovery and right bends retain
	 * their existing symmetric tuning. A negative curve direction requires a
	 * positive motor differential with this drivetrain convention. */
	if ((auto_curve_latched_direction < 0) && (total > 0)) {
		int32_t left_gain = AUTO_LEFT_BASE_GAIN_PERCENT;

		/* Use 110% only while trusted FAR/NEAR geometry shows a sustained
		 * strong bend. Weak curvature, an exit trend, or FAR/geometry loss
		 * immediately falls back to the proven 108% response so additional
		 * steering cannot be carried into the next section. */
		if ((lane_geometry_untrusted == false) &&
		    (lane_far_loss_active == false) &&
		    (auto_exit_trend_count == 0U) &&
		    (abs_curve >= AUTO_WEAK_LEFT_FULL_HOLD_PIXELS)) {
			left_gain = AUTO_LEFT_CONFIRMED_GAIN_PERCENT;
		}
		total = (total * left_gain + 50) / 100;
		/* Add a small amount only after curve authority is established and
		 * while curvature is being sustained. Entry preview and the detected
		 * exit trend keep their existing response. */
		if ((auto_curve_authority_count >= 3U) &&
		    (auto_exit_trend_count == 0U) &&
		    (abs_curve >= AUTO_WEAK_LEFT_FULL_HOLD_PIXELS)) {
			total += AUTO_LEFT_MIDCURVE_EXTRA_RPM;
		}
	}
	if (total > limit) total = limit;
	else if (total < -limit) total = -limit;
	lane_steer_feedback = (int16_t)feedback;
	lane_steer_total = (int16_t)total;
	return total;
}

static void Lane_UpdateSpeedScale(void)
{
	uint32_t target = 100U;
	uint32_t strength = (uint32_t)Lane_Abs32(lane_curve_filtered);
	uint32_t reduction;

	if ((lane_control_state == LANE_MEMORY) ||
	    (lane_control_state == LANE_RECOVER)) {
		strength = (uint32_t)Lane_Abs32(lane_path_curve);
	}
	if ((curve_preview_active != false) ||
	    (lane_control_state == LANE_MEMORY) ||
	    (lane_control_state == LANE_RECOVER)) {
		reduction = (strength * lane_tuning.curve_speed_gain) / 100U;
		if (reduction > (100U - LANE_SPEED_SCALE_MIN_PERCENT))
			reduction = 100U - LANE_SPEED_SCALE_MIN_PERCENT;
		target = 100U - reduction;
		if (((lane_control_state == LANE_MEMORY) ||
		     (lane_control_state == LANE_RECOVER)) && (target > 80U))
			target = 80U;
	}
	/* FAR supplies curve preview. If it is lost while NEAR still permits
	 * steering, keep tracking but reduce speed until FAR is stably restored. */
	if ((lane_far_loss_active != false) &&
	    (target > LANE_FAR_LOSS_SPEED_PERCENT))
		target = LANE_FAR_LOSS_SPEED_PERCENT;
	/* Curvature can look small when FAR and NEAR move together even though the
	 * vehicle is far from the lane center.  Reduce speed from lateral error as
	 * well so recovery has time to settle without crossing the other side. */
	if ((Lane_Abs32(current_lane_error) >= LANE_LARGE_ERROR_PIXELS) &&
	    (target > LANE_LARGE_ERROR_SPEED_PERCENT)) {
		target = LANE_LARGE_ERROR_SPEED_PERCENT;
	} else if ((Lane_Abs32(current_lane_error) >= LANE_MEDIUM_ERROR_PIXELS) &&
	           (target > LANE_MEDIUM_ERROR_SPEED_PERCENT)) {
		target = LANE_MEDIUM_ERROR_SPEED_PERCENT;
	}
	if ((Lane_Abs32(lane_curve_raw) < LANE_STRAIGHT_RECOVERY_CURVE_PIXELS) &&
	    (Lane_Abs32(current_lane_error) >= LANE_STRAIGHT_RECOVERY_ERROR_PIXELS) &&
	    (target > LANE_STRAIGHT_RECOVERY_SPEED_PERCENT))
		target = LANE_STRAIGHT_RECOVERY_SPEED_PERCENT;

	if (target > lane_speed_scale_percent) {
		uint32_t next = lane_speed_scale_percent +
				LANE_SPEED_SCALE_SLEW_PERCENT;
		lane_speed_scale_percent = (next > target) ? (uint8_t)target :
				(uint8_t)next;
	} else if (target < lane_speed_scale_percent) {
		uint32_t step = LANE_SPEED_SCALE_SLEW_PERCENT;
		lane_speed_scale_percent = ((uint32_t)lane_speed_scale_percent >
				target + step) ?
				(uint8_t)((uint32_t)lane_speed_scale_percent - step) :
				(uint8_t)target;
	}
}

/* =========================================================
 * APP_GetThresholdLUT
 *
 * Description:
 * Selects the ADAS detection threshold table according to
 * the current vehicle RPM.
 *
 * Range:
 * 0 to 124 RPM   -> Low-speed threshold table.
 * 125 to 174 RPM -> Mid-speed threshold table.
 * 175 RPM or more -> High-speed threshold table.
 *
 * Parameter:
 * current_rpm - Current estimated vehicle RPM.
 *
 * Return:
 * Pointer to the selected ADAS threshold table entry.
 * ========================================================= */

/* =========================================================
 * APP_UpdateADAS
 *
 * Description:
 * Reads the latest ADAS1 and ADAS2 results, selects the
 * RPM-dependent detection threshold, and converts detected
 * objects into application events.
 *
 * Processing Flow:
 * CAN parser
 * -> ADAS result structure
 * -> Current motor RPM
 * -> RPM-based threshold selection
 * -> Object-specific threshold check
 * -> Event queue
 *
 * Event Mapping:
 * SPEED_BUMP    -> EVENT_MOTOR_SLOW
 * TRAFFIC_GREEN -> EVENT_MOTOR_RUN
 * TRAFFIC_RED   -> EVENT_MOTOR_STOP
 * SIGN_30       -> EVENT_MOTOR_30
 * SIGN_70       -> EVENT_MOTOR_70
 * LANE          -> EVENT_NONE with PD output
 *
 * Notes:
 * - Lane PD is calculated only from ADAS1.
 * - Bump, signs, and red-light events are accepted from either ADAS.
 * - A run event requires valid green detections from both ADAS ECUs.
 * - A speed bump changes the base RPM to 100 RPM after the
 *   threshold is satisfied.
 * - The reduced speed is kept until another speed-changing
 *   event updates current_base_rpm.
 * ========================================================= */

static void APP_UpdateOneADAS(ECU_t source, const ADASResult_t *result, uint32_t result_tick)
{
	Event_t event = {0};

	uint8_t object_height = 0U;

	if (result == NULL) {
		return;
	}
	/* ADAS2 CAN IDs remain decodable for compatibility/diagnostics, but only
	 * ADAS1 is allowed to affect vehicle control. */
	if (source != ECU_ADAS1) {
		return;
	}
	event.source = source;
	event.target = ECU_VCU;

	event.adas_result = *result;

	event.pd_output = current_lane_pd;

	if (result->y1 >= result->y0) {
		object_height = (uint8_t)(result->y1 - result->y0);
	} else {
		object_height = 0U;
	}
	/* A red-light stop requires consecutive qualifying red frames. Any other
	 * class breaks the sequence so a one-frame CNN class jump cannot stop the
	 * vehicle. */
	if (result->object_type != OBJECT_TRAFFIC_RED) {
		red_confirm_count = 0U;
	}

	switch (result->object_type) {
		/* =============================================
		 * Speed Bump
		 *
		 * Uses y1 instead of bounding-box height.
		 * Generates a SLOW event when the speed bump is
		 * close enough according to the selected LUT.
		 * ============================================= */

	case OBJECT_SPEED_BUMP:
		bump_last_detection_tick = result_tick;
		if ((bump_armed != false) && (bump_active == false)) {
			event.type = EVENT_MOTOR_SLOW;
		} else {
			event.type = EVENT_NONE;
		}

		break;

		/* =============================================
		 * Traffic Green
		 *
		 * Generates a RUN event when the detected object
		 * height is greater than the minimum threshold.
		 * ============================================= */

	case OBJECT_TRAFFIC_GREEN:
		/* ADAS already applies the CNN score and temporal confirmation policy.
		 * Do not reject its confirmed green class again based on box height. */
		if (green_confirm_count < TRAFFIC_GREEN_CONFIRM_COUNT)
			green_confirm_count++;
		if (green_confirm_count >= TRAFFIC_GREEN_CONFIRM_COUNT) {
			event.type = EVENT_MOTOR_RUN;
			green_confirm_count = 0U;
		} else {
			event.type = EVENT_NONE;
		}

		break;

		/* =============================================
		 * Traffic Red
		 *
		 * Generates a STOP event when the detected red-light
		 * object height exceeds the RPM-dependent threshold.
		 * ============================================= */

	case OBJECT_TRAFFIC_RED:
		green_confirm_count = 0U;

		if (object_height >= TRAFFIC_RED_STOP_MIN_HEIGHT) {
			if (red_confirm_count < TRAFFIC_RED_CONFIRM_COUNT)
				red_confirm_count++;
			if (red_confirm_count >= TRAFFIC_RED_CONFIRM_COUNT)
				event.type = EVENT_MOTOR_STOP;
			else
				event.type = EVENT_NONE;
		} else {
			red_confirm_count = 0U;
			event.type = EVENT_NONE;
		}

		break;

		/* =============================================
		 * Sign 30
		 *
		 * Generates a 30-zone event when the detected sign
		 * height exceeds the RPM-dependent threshold.
		 * ============================================= */

	case OBJECT_SIGN_30:
		event.type = EVENT_MOTOR_30;

		break;

		/* =============================================
		 * Sign 70
		 *
		 * Generates a 70-zone event when the detected sign
		 * height exceeds the RPM-dependent threshold.
		 * ============================================= */

	case OBJECT_SIGN_70:
		event.type = EVENT_MOTOR_70;

		break;

		/* =============================================
		 * Lane
		 *
		 * Lane information does not create a speed-state
		 * event. The lane correction is carried in pd_output.
		 * ============================================= */

	case OBJECT_LANE:

		event.type = EVENT_NONE;

		break;

	default:

		event.type = EVENT_NONE;

		break;
	}

	(void)QueueEvent(&event);
}

void APP_UpdateADAS(void)
{
	ADASResult_t result = {0};
	uint32_t result_tick = 0U;
	uint16_t lane_x = 0U;
	uint16_t far_x = 0U;
	uint16_t near_x = 0U;
	bool far_valid = false;
	bool near_valid = false;
	ECU_t source = ECU_VCU;
	uint32_t latest_lane_tick = 0U;
	uint32_t now;

	Lane_UpdateDistance();
	/* A held CNN bump detection may emit many frames. Rearm only after the
	 * class has been absent long enough, never merely because its 2 s timer
	 * expired. */
	if ((bump_armed == false) &&
	    ((uint32_t)(millis() - bump_last_detection_tick) >=
	     SPEED_BUMP_REARM_MS)) {
		bump_armed = true;
	}

	/* Lane is a latest-value mailbox because old steering samples should
	 * never be replayed. Every non-lane object is drained from the ordered
	 * ADAS event queue so transient RED/sign/bump detections are not lost. */
	if (CAN_GetLatestADAS1Lane(&lane_x, &far_x, &near_x,
			&far_valid, &near_valid) != false) {
		Event_t lane_event = {0};
		uint16_t filtered_lane_x;
		int32_t feedback;
		int32_t feedforward;
		int32_t total_steering;
		uint8_t confidence;
		uint32_t age_ms = 0U;
		bool far_trusted;
		bool near_trusted;
		bool raw_near_trusted;
		uint16_t control_near_x;

		now = millis();
		if (CAN_GetADASResultLastTick(ECU_ADAS1, &latest_lane_tick) != false)
			age_ms = (uint32_t)(now - latest_lane_tick);
		else
			age_ms = 1000U;
		confidence = Lane_UpdateConfidence(far_x, near_x, far_valid,
				near_valid, age_ms);
		Lane_UpdateAutoCurveLatch(far_x, near_x, far_valid, near_valid);
		far_trusted = (far_valid != false) && (near_valid != false) &&
				(Lane_Abs32((int32_t)far_x - (int32_t)near_x) <
				 (int32_t)lane_tuning.far_near_reject_diff);
		/* At the selected 200-RPM profile, a very large FAR/NEAR split is
		 * normally FAR switching to another Sobel boundary.  Do not let that
		 * sample feed preview, path memory, or speed curvature; NEAR remains
		 * available for bounded center recovery. */
		if (Lane_Abs32((int32_t)far_x - (int32_t)near_x) >
		    (int32_t)AUTO_FAR_EDGE_SWITCH_PIXELS) {
			far_trusted = false;
		}
		/* FV/NV alone are insufficient: both repeated runs contained valid
		 * flags while FAR jumped to another edge. Curve-latch pair stability
		 * already applies a speed-scaled temporal jump limit, so require its
		 * confirmation before FAR may influence steering or path memory. */
		if (auto_pair_stable_count < AUTO_CURVE_PAIR_CONFIRM_FRAMES)
			far_trusted = false;
		Lane_UpdateStraightStateReset(far_x, near_x, far_trusted);
		lane_geometry_untrusted = (far_trusted == false) ||
			(near_valid == false) || (age_ms > 250U);
		/* Even when temporal stability rejects the pair, two bounded raw
		 * samples in the opposite signed direction are enough to release the
		 * old memory and begin the next S-bend. This prevents a remembered
		 * right turn from blocking the following left hairpin. */
		if ((lane_geometry_untrusted != false) &&
		    (auto_last_strong_steering_direction != 0) &&
		    (far_valid != false) && (near_valid != false) &&
		    (Lane_Abs32((int32_t)far_x - (int32_t)near_x) >=
		     AUTO_STRONG_OPPOSITE_CURVE_PIXELS) &&
		    (Lane_Abs32((int32_t)far_x - (int32_t)near_x) <=
		     AUTO_FAR_EDGE_SWITCH_PIXELS)) {
			int8_t raw_steering_direction =
				(((int32_t)far_x - (int32_t)near_x) < 0) ? 1 : -1;
			if (raw_steering_direction ==
			    -auto_last_strong_steering_direction) {
				if (auto_raw_opposite_count <
				    AUTO_RAW_OPPOSITE_CONFIRM_FRAMES)
					auto_raw_opposite_count++;
				if (auto_raw_opposite_count >=
				    AUTO_RAW_OPPOSITE_CONFIRM_FRAMES) {
					auto_last_strong_steering_direction =
						raw_steering_direction;
					auto_last_strong_steering_tick = now;
					auto_raw_opposite_count = 0U;
				}
			} else {
				auto_raw_opposite_count = 0U;
			}
		} else {
			auto_raw_opposite_count = 0U;
		}
		Lane_UpdateFarLossSpeed(far_trusted);
		APP_UpdateCurvePreview(far_x, near_x, far_trusted, confidence);
		Lane_UpdatePathMemory(near_x, far_trusted);
		Lane_UpdateState(near_valid);
		raw_near_trusted = (near_valid != false) && (age_ms <= 250U);
		auto_near_measurement_trusted = raw_near_trusted;
		near_trusted = raw_near_trusted;
		/* During a latched AUTO bend, a low-confidence or unconfirmed
		 * opposite pair is more likely a newly exposed boundary than the lane
		 * center.  Reuse the bounded path-memory virtual center instead. */
		if ((auto_curve_latched_direction != 0) &&
		    ((confidence < (uint8_t)lane_tuning.confidence_low) ||
		     ((((lane_curve_raw > 0) ? 1 : ((lane_curve_raw < 0) ? -1 : 0)) ==
		       -auto_curve_latched_direction) &&
		      (auto_opposite_evidence_count < AUTO_OPPOSITE_CONFIRM_FRAMES)))) {
			near_trusted = false;
		}
		control_near_x = Lane_GetVirtualNearX(near_x, near_trusted);
		filtered_lane_x = APP_FilterLaneX(control_near_x, near_trusted);

		/* In MEMORY, filtered_lane_x is not a raw invalid measurement. It is
		 * the last trusted NEAR value held briefly and then decayed to center
		 * by APP_FilterLaneX. Continue computing its bounded feedback so lane
		 * loss does not immediately cancel the steering that entered a curve. */
		feedback = APP_LanePD(filtered_lane_x);
		feedforward = Lane_CalcFeedForward();
		total_steering = Lane_CombineSteering(feedback, feedforward);
		/* Give a real, mutually consistent FAR/NEAR straight pair final authority
		 * over stale curve-latch, settle, and feed-forward state.  Use raw NEAR
		 * here so the hold/virtual-center filter cannot hide a growing departure.
		 * Authority still rises only 1 -> 3 -> 5 RPM on persistent evidence. */
		if ((raw_near_trusted != false) &&
		    (((far_trusted != false) &&
		      (Lane_Abs32((int32_t)far_x - (int32_t)near_x) <
		       LANE_STRAIGHT_RECOVERY_CURVE_PIXELS)) ||
		     (Lane_Abs32((int32_t)lane_tuning.center_x -
		                 (int32_t)near_x) >=
		      LANE_STRAIGHT_RECOVERY_ERROR_PIXELS))) {
			int32_t raw_error = (int32_t)lane_tuning.center_x -
				(int32_t)near_x;
			int32_t raw_abs_error = Lane_Abs32(raw_error);
			int8_t raw_direction = (raw_error > 0) ? 1 :
				((raw_error < 0) ? -1 : 0);

			if (raw_abs_error > (int32_t)AUTO_STRAIGHT_DEADBAND_PIXELS) {
				int32_t recovery_limit;
				uint16_t recovery_rpm = APP_GetSteeringControlRPM();
				bool raw_error_recovering =
					(auto_raw_straight_previous_abs_error > 0) &&
					((raw_abs_error +
					  (int32_t)AUTO_STRAIGHT_ERROR_DROP_PIXELS) <
					  auto_raw_straight_previous_abs_error);
				int32_t recovery_medium_limit = APP_InterpolateGain(2, 3, 3,
					recovery_rpm);
				int32_t recovery_max_limit = APP_InterpolateGain(4, 5, 6,
					recovery_rpm);

				if ((raw_direction == auto_raw_straight_direction) &&
				    (raw_error_recovering == false)) {
					if (auto_raw_straight_count <
					    AUTO_STRAIGHT_STAGE3_FRAMES)
						auto_raw_straight_count++;
				} else {
					auto_raw_straight_direction = raw_direction;
					auto_raw_straight_count = 1U;
				}
				auto_raw_straight_previous_abs_error = raw_abs_error;

				if (raw_error_recovering != false)
					recovery_limit = AUTO_STRAIGHT_APPROACH_CORRECTION_RPM;
				else if ((raw_abs_error < (int32_t)AUTO_STRAIGHT_FINE_ERROR_PIXELS) ||
				    (auto_raw_straight_count < AUTO_STRAIGHT_STAGE2_FRAMES))
					recovery_limit = AUTO_STRAIGHT_FINE_CORRECTION_RPM;
				else if ((raw_abs_error <
				          (int32_t)AUTO_STRAIGHT_MEDIUM_ERROR_PIXELS) ||
				         (auto_raw_straight_count < AUTO_STRAIGHT_STAGE3_FRAMES))
					recovery_limit = recovery_medium_limit;
				else
					recovery_limit = recovery_max_limit;

				{
					int32_t recovery_command = (raw_direction > 0) ?
						recovery_limit : -recovery_limit;
					int8_t latched_steering_direction =
						(auto_curve_latched_direction != 0) ?
						-auto_curve_latched_direction : 0;

					/* A confirmed curve already supplies at least the speed-scaled
					 * curve floor. When NEAR asks for the same direction, never
					 * replace that stronger command with the small straight-recovery
					 * value. Opposite NEAR evidence may still request bounded center
					 * recovery so the vehicle cannot be held outside the lane. */
					if ((latched_steering_direction == 0) ||
					    (raw_direction != latched_steering_direction) ||
					    (Lane_Abs32(total_steering) < recovery_limit))
						total_steering = recovery_command;
				}
				lane_steer_feedback = (int16_t)total_steering;
				lane_steer_feedforward = 0;
				lane_steer_total = (int16_t)total_steering;
			} else {
				auto_raw_straight_direction = 0;
				auto_raw_straight_count = 0U;
				auto_raw_straight_previous_abs_error = 0;
				/* A trusted straight pair inside the deadband has final authority:
				 * release all residual steering immediately instead of carrying it
				 * across the center and starting another oscillation. */
				if ((far_trusted != false) &&
				    (Lane_Abs32((int32_t)far_x - (int32_t)near_x) <
				     LANE_STRAIGHT_RECOVERY_CURVE_PIXELS)) {
					total_steering = 0;
					lane_steer_feedback = 0;
					lane_steer_feedforward = 0;
					lane_steer_total = 0;
				}
			}
		} else {
			auto_raw_straight_direction = 0;
			auto_raw_straight_count = 0U;
			auto_raw_straight_previous_abs_error = 0;
		}
		Lane_UpdateSpeedScale();
		lane_event.type = EVENT_NONE;
		lane_event.source = ECU_ADAS1;
		lane_event.target = ECU_VCU;
		lane_event.pd_output = total_steering;
		(void)QueueEvent(&lane_event);
	}

	while (CAN_GetNextADASEvent(&source, &result, &result_tick) != false) {
		if (source == ECU_ADAS1)
			APP_UpdateOneADAS(source, &result, result_tick);
	}
}

void APP_SetVehicleEnabled(bool enabled)
{
	if ((enabled != false) && (vehicle_enabled == false)) {
		vehicle_enable_tick = millis();
	}
	vehicle_enabled = enabled;
	if (enabled == false) {
		current_base_rpm = 0U;
		Lane_ResetAutoState();
		APP_ResetLanePDSlew();
	}
}

bool APP_SetBaseRPM(uint16_t rpm)
{
	if ((vehicle_enabled == false) ||
	    ((rpm != 100U) && (rpm != 150U) && (rpm != 200U) &&
	     (rpm != 250U))) {
		return false;
	}

	current_base_rpm = rpm;
	road_speed_rpm = rpm;
	return true;
}

bool APP_IsRedLightStopActive(void)
{
	return red_light_stop;
}

bool APP_SetLaneGain(uint16_t rpm_band, bool set_kp, int32_t gain_scaled)
{
	int32_t *gain;

	if ((gain_scaled < 0) || (gain_scaled > 200)) {
		return false;
	}

	if (rpm_band == 100U) {
		gain = (set_kp != false) ? &runtime_kp_100_scaled : &runtime_kd_100_scaled;
	} else if (rpm_band == 150U) {
		gain = (set_kp != false) ? &runtime_kp_150_scaled : &runtime_kd_150_scaled;
	} else if (rpm_band == 200U) {
		gain = (set_kp != false) ? &runtime_kp_200_scaled : &runtime_kd_200_scaled;
	} else {
		return false;
	}

	*gain = gain_scaled;
	return true;
}

bool APP_GetLaneGains(uint16_t rpm_band, int32_t *kp_scaled,
		int32_t *kd_scaled)
{
	if ((kp_scaled == NULL) || (kd_scaled == NULL)) {
		return false;
	}

	if (rpm_band == 100U) {
		*kp_scaled = runtime_kp_100_scaled;
		*kd_scaled = runtime_kd_100_scaled;
	} else if (rpm_band == 150U) {
		*kp_scaled = runtime_kp_150_scaled;
		*kd_scaled = runtime_kd_150_scaled;
	} else if (rpm_band == 200U) {
		*kp_scaled = runtime_kp_200_scaled;
		*kd_scaled = runtime_kd_200_scaled;
	} else {
		return false;
	}

	return true;
}

bool APP_SetLaneTuning(LaneTuningKey_t key, uint32_t value)
{
	switch (key) {
	case LANE_TUNE_SLEW_100_MS:
	case LANE_TUNE_SLEW_150_MS:
	case LANE_TUNE_SLEW_200_MS:
		if ((value < 1U) || (value > 5000U)) return false;
		if (key == LANE_TUNE_SLEW_100_MS) lane_tuning.slew_100_ms = value;
		else if (key == LANE_TUNE_SLEW_150_MS) lane_tuning.slew_150_ms = value;
		else lane_tuning.slew_200_ms = value;
		break;
	case LANE_TUNE_MAX_CORRECTION_RPM:
		if (value > VEHICLE_MAX_RPM) return false;
		lane_tuning.max_correction_rpm = value;
		break;
	case LANE_TUNE_CENTER_X:
		if (value > 1279U) return false;
		lane_tuning.center_x = value;
		break;
	case LANE_TUNE_DEADBAND_PIXELS:
		if (value > 100U) return false;
		lane_tuning.deadband_pixels = value;
		break;
	case LANE_TUNE_JUMP_PIXELS:
		if ((value < 1U) || (value > 640U)) return false;
		lane_tuning.jump_pixels = value;
		break;
	case LANE_TUNE_CLUSTER_PIXELS:
		if ((value < 1U) || (value > 640U)) return false;
		lane_tuning.cluster_pixels = value;
		break;
	case LANE_TUNE_CONFIRM_FRAMES:
		if ((value < 1U) || (value > 20U)) return false;
		lane_tuning.confirm_frames = value;
		break;
	case LANE_TUNE_REACQUIRE_STEP_PIXELS:
		if ((value < 1U) || (value > 100U)) return false;
		lane_tuning.reacquire_step_pixels = value;
		break;
	case LANE_TUNE_RATE_FILTER_DIVISOR:
		if ((value < 1U) || (value > 32U)) return false;
		lane_tuning.rate_filter_divisor = value;
		break;
	case LANE_TUNE_RATE_MAX_PIXELS_PER_SEC:
		if ((value < 100U) || (value > 10000U)) return false;
		lane_tuning.rate_max_pixels_per_sec = value;
		break;
	case LANE_TUNE_RATE_MAX_DT_MS:
		if ((value < 10U) || (value > 2000U)) return false;
		lane_tuning.rate_max_dt_ms = value;
		break;
	case LANE_TUNE_DERIVATIVE_REFERENCE_MS:
		if ((value < 1U) || (value > 1000U)) return false;
		lane_tuning.derivative_reference_ms = value;
		break;
	case LANE_TUNE_SLEW_MODE:
		if (value > LANE_PD_SLEW_MODE_TIME) return false;
		lane_tuning.slew_mode = value;
		APP_ResetLanePDSlew();
		break;
	case LANE_TUNE_LIMIT_MODE:
		if (value > LANE_PD_LIMIT_MODE_ALL_SPEED) return false;
		lane_tuning.limit_mode = value;
		break;
	case LANE_TUNE_KFF_SCALED:
		if (value > 100U) return false;
		lane_tuning.kff_scaled = value;
		break;
	case LANE_TUNE_PATH_STEP_MM:
		if ((value < 10U) || (value > 100U)) return false;
		lane_tuning.path_step_mm = value;
		Lane_ResetPathMemory();
		break;
	case LANE_TUNE_NEAR_LOOKAHEAD_MM:
		if (value > 1000U) return false;
		lane_tuning.near_lookahead_mm = value;
		break;
	case LANE_TUNE_FAR_LOOKAHEAD_MM:
		if ((value < 30U) || (value > 1500U)) return false;
		lane_tuning.far_lookahead_mm = value;
		Lane_ResetPathMemory();
		break;
	case LANE_TUNE_FAR_NEAR_WARN_DIFF:
		if ((value < 20U) || (value >= lane_tuning.far_near_reject_diff))
			return false;
		lane_tuning.far_near_warn_diff = value;
		break;
	case LANE_TUNE_FAR_NEAR_REJECT_DIFF:
		if ((value <= lane_tuning.far_near_warn_diff) || (value > 800U))
			return false;
		lane_tuning.far_near_reject_diff = value;
		break;
	case LANE_TUNE_CONF_LOW:
		if ((value > 100U) || (value >= lane_tuning.confidence_high))
			return false;
		lane_tuning.confidence_low = value;
		break;
	case LANE_TUNE_CONF_HIGH:
		if ((value > 100U) || (value <= lane_tuning.confidence_low))
			return false;
		lane_tuning.confidence_high = value;
		break;
	case LANE_TUNE_CURVE_ENTER:
		if ((value < 5U) || (value > 300U) ||
		    (value <= lane_tuning.curve_exit_pixels)) return false;
		lane_tuning.curve_enter_pixels = value;
		break;
	case LANE_TUNE_CURVE_EXIT:
		if ((value > 200U) || (value >= lane_tuning.curve_enter_pixels))
			return false;
		lane_tuning.curve_exit_pixels = value;
		break;
	case LANE_TUNE_CURVE_SPEED_GAIN:
		if (value > 100U) return false;
		lane_tuning.curve_speed_gain = value;
		break;
	case LANE_TUNE_MEMORY_MAX_DISTANCE_MM:
		if ((value < 30U) || (value > 2000U)) return false;
		lane_tuning.memory_max_distance_mm = value;
		break;
	case LANE_TUNE_MEMORY_MAX_TIME_MS:
		if ((value < 100U) || (value > 5000U)) return false;
		lane_tuning.memory_max_time_ms = value;
		break;
	case LANE_TUNE_RECOVER_BLEND_RATE:
		if ((value < 1U) || (value > 100U)) return false;
		lane_tuning.recover_blend_rate = value;
		break;
	default:
		return false;
	}

	return true;
}

void APP_GetLaneTuning(LaneTuning_t *tuning)
{
	if (tuning != NULL) {
		*tuning = lane_tuning;
	}
}

void APP_GetLaneControlState(uint16_t *base_rpm, int32_t *lane_error,
		int32_t *pd_output, int32_t *kp_scaled, int32_t *kd_scaled)
{
	if (base_rpm != NULL) {
		*base_rpm = current_effective_base_rpm;
	}
	if (lane_error != NULL) {
		*lane_error = current_lane_error;
	}
	if (pd_output != NULL) {
		*pd_output = current_lane_pd;
	}
	if (kp_scaled != NULL) {
		*kp_scaled = current_kp_scaled;
	}
	if (kd_scaled != NULL) {
		*kd_scaled = current_kd_scaled;
	}
}

void APP_GetLaneFilterState(uint16_t *raw_lane_x, uint16_t *used_lane_x,
		uint8_t *filter_state, uint8_t *confirm_count)
{
	if (raw_lane_x != NULL) {
		*raw_lane_x = current_raw_lane_x;
	}
	if (used_lane_x != NULL) {
		*used_lane_x = current_used_lane_x;
	}
	if (filter_state != NULL) {
		*filter_state = lane_filter_state;
	}
	if (confirm_count != NULL) {
		*confirm_count = lane_candidate_count;
	}
}

void APP_GetLaneSlewState(int32_t *requested_pd, int32_t *applied_pd)
{
	if (requested_pd != NULL) {
		*requested_pd = requested_lane_pd;
	}
	if (applied_pd != NULL) {
		*applied_pd = current_lane_pd;
	}
}

int32_t APP_GetLaneRate(void)
{
	return filtered_lane_rate;
}

void APP_GetLanePreviewState(uint16_t *far_x, uint16_t *near_x,
		bool *preview_active)
{
	if (far_x != NULL) *far_x = current_far_lane_x;
	if (near_x != NULL) *near_x = current_near_lane_x;
	if (preview_active != NULL) *preview_active = curve_preview_active;
}

void APP_GetLaneDiagnostics(LaneDiagnostics_t *diagnostics)
{
	if (diagnostics == NULL) return;
	diagnostics->state = lane_control_state;
	diagnostics->confidence = lane_confidence;
	diagnostics->far_near_diff = lane_curve_raw;
	diagnostics->curve_raw = lane_curve_raw;
	diagnostics->curve_filtered = lane_curve_filtered;
	diagnostics->path_curve = lane_path_curve;
	diagnostics->steer_feedback = lane_steer_feedback;
	diagnostics->steer_feedforward = lane_steer_feedforward;
	diagnostics->steer_total = lane_steer_total;
	diagnostics->speed_scale_percent = lane_speed_scale_percent;
	diagnostics->distance_mm = lane_distance_mm;
	diagnostics->path_index = lane_path_write_index;
	diagnostics->recover_blend_percent = lane_recover_blend_percent;
	diagnostics->memory_timed_out = lane_memory_timed_out;
}

/* =========================================================
 * APP_ProcessEvent
 *
 * Description:
 * Applies one application event to the vehicle state and
 * calculates the left and right target motor RPM values.
 *
 * Speed-State Behavior:
 * EVENT_MOTOR_RUN  -> Enable vehicle and set 150 RPM only
 *                     when starting from the disabled state.
 * EVENT_MOTOR_70   -> Set base RPM to 200 RPM.
 * EVENT_MOTOR_30   -> Set base RPM to 100 RPM.
 * EVENT_MOTOR_SLOW -> Set base RPM to 100 RPM.
 * EVENT_MOTOR_STOP -> Disable vehicle and set base RPM to 0.
 * EVENT_NONE       -> Keep the current base RPM.
 *
 * Lane Control:
 * The PD output is applied as a differential correction:
 * left_rpm  = base_rpm - pd_output
 * right_rpm = base_rpm + pd_output
 *
 * The final left and right RPM commands are clamped to
 * the range from 0 to VEHICLE_MAX_RPM.
 *
 * Parameter:
 * event - Pointer to the event to be processed.
 *
 * Return:
 * Target RPM command for the left and right motors.
 * ========================================================= */

MotorTargetrpm_t APP_ProcessEvent(Event_t *event)
{
	MotorTargetrpm_t target_rpm = {0U, 0U};
	uint16_t command_base_rpm;

	int32_t current_pd_output;

	int32_t left_rpm;
	int32_t right_rpm;

	if (event == NULL) {
		return target_rpm;
	}

	current_pd_output = event->pd_output;
	/* Expire temporary bump slowdown from real VCU elapsed time. Repeated
	 * detections do not touch bump_start_tick. */
	if ((bump_active != false) &&
	    ((uint32_t)(millis() - bump_start_tick) >= SPEED_BUMP_SLOWDOWN_MS)) {
		bump_active = false;
		/* Do not restore a previously latched 70-zone command after the
		 * bump. Return to the normal 150-RPM road speed instead. Preserve
		 * the safer 100-RPM school-zone limit when it is already active. */
		if (road_speed_rpm != ROAD_SPEED_30_RPM) {
			road_speed_rpm = ROAD_SPEED_50_RPM;
			road_speed_70_stage = 0U;
			road_speed_70_straight_count = 0U;
		}
	}

	switch (event->type) {
		/* =====================================
		 * Speed Bump
		 *
		 * Changes the base RPM to 100 RPM for
		 * three seconds. It then returns to
		 * 150 RPM, except in a 100-RPM zone.
		 * ===================================== */

	case EVENT_MOTOR_SLOW:
		road_speed_70_stage = 0U;
		road_speed_70_straight_count = 0U;
		if ((vehicle_enabled != false) && (red_light_stop == false) &&
		    (bump_active == false) && (bump_armed != false)) {
			bump_active = true;
			bump_armed = false;
			bump_start_tick = millis();
		}

		break;

		/* =====================================
		 * Traffic Green
		 *
		 * Starts the vehicle at 150 RPM when
		 * the vehicle is currently disabled.
		 * If already running, the current speed
		 * state is preserved.
		 * ===================================== */

	case EVENT_MOTOR_RUN:

		if (vehicle_enabled == false) {
			red_light_stop = false;
			bump_active = false;
			road_speed_rpm = ROAD_SPEED_50_RPM;
			road_speed_70_stage = 0U;
			road_speed_70_straight_count = 0U;
			vehicle_enabled = true;
			vehicle_enable_tick = millis();

			current_base_rpm = ROAD_SPEED_50_RPM;
			Lane_ResetAutoState();
		}

		break;

		/* =====================================
		 * Sign 70
		 *
		 * Changes the base RPM to 200 RPM while
		 * the vehicle is enabled.
		 * ===================================== */

	case EVENT_MOTOR_70:
		if (road_speed_rpm != ROAD_SPEED_70_RPM) {
			/* Begin the continuous 70-zone ramp at the speed that was active
			 * when the sign was seen.  In particular, a 30-zone transition now
			 * slews 100 -> 200 directly instead of jumping 100 -> 150 first. */
			road_speed_70_slewed_rpm = road_speed_rpm;
			if (road_speed_70_slewed_rpm < ROAD_SPEED_30_RPM)
				road_speed_70_slewed_rpm = ROAD_SPEED_30_RPM;
			if (road_speed_70_slewed_rpm > ROAD_SPEED_70_RPM)
				road_speed_70_slewed_rpm = ROAD_SPEED_70_RPM;
			road_speed_70_slew_tick = millis();
			road_speed_70_stage = 1U;
			road_speed_70_straight_count = 0U;
			road_speed_70_stable_tick = 0U;
		}
		road_speed_rpm = ROAD_SPEED_70_RPM;

		break;

		/* =====================================
		 * Sign 30
		 *
		 * Changes the base RPM to 100 RPM while
		 * the vehicle is enabled.
		 * ===================================== */

	case EVENT_MOTOR_30:
		road_speed_rpm = ROAD_SPEED_30_RPM;
		road_speed_70_stage = 0U;
		road_speed_70_straight_count = 0U;

		break;

		/* =====================================
		 * Traffic Red
		 *
		 * Immediately disables the vehicle and
		 * returns a zero-RPM command.
		 * ===================================== */

	case EVENT_MOTOR_STOP:

		red_light_stop = true;
		bump_active = false;
		bump_armed = false;
		bump_last_detection_tick = millis();
		vehicle_enabled = false;

		current_base_rpm = 0U;
		current_effective_base_rpm = 0U;
		road_speed_70_stage = 0U;
		road_speed_70_straight_count = 0U;
		Lane_ResetAutoState();
		APP_ResetLanePDSlew();

		return target_rpm;

		/* =====================================
		 * Lane / No Class Action
		 *
		 * Keeps the current base RPM and applies
		 * only the current lane PD correction.
		 * ===================================== */

	case EVENT_NONE:

		break;

	default:

		return target_rpm;
	}

	/*
	 * Do not generate a motor command until the vehicle has
	 * been enabled by a valid RUN event.
	 */
	if (vehicle_enabled == false) {
		current_effective_base_rpm = 0U;
		return target_rpm;
	}
	if (red_light_stop != false) {
		current_base_rpm = 0U;
		current_effective_base_rpm = 0U;
		return target_rpm;
	}

	/* Road-zone speed and temporary slowdown are independent. A 30 zone is
	 * already capped at 100; a bump temporarily caps 50/70 zones at 100. */
	current_base_rpm = road_speed_rpm;
	if ((bump_active != false) && (current_base_rpm > 100U))
		current_base_rpm = 100U;

	if (current_base_rpm == 0U) {
		current_effective_base_rpm = 0U;
		return target_rpm;
	}

	command_base_rpm = current_base_rpm;
	/* A 70 sign now selects one continuously slewed command instead of the
	 * 165/175/185 RPM staircase.  Valid straight geometry requests 200 RPM;
	 * bends or untrusted geometry request the proven 150-RPM curve profile.
	 * One RPM every 20 ms makes the transition inaudible and prevents a single
	 * noisy frame from producing an acceleration/deceleration kick. */
	if (current_base_rpm == ROAD_SPEED_70_RPM) {
		uint16_t desired_70_rpm = ROAD_SPEED_70_RPM;
		uint32_t now_70 = millis();
		if ((lane_geometry_untrusted != false) ||
		    (Lane_Abs32(lane_curve_raw) > ROAD_SPEED_70_CURVE_PIXELS) ||
		    (Lane_Abs32(current_lane_error) > ROAD_SPEED_70_CENTER_PIXELS))
			desired_70_rpm = ROAD_SPEED_70_CURVE_CAP_RPM;

		if (road_speed_70_slew_tick == 0U)
			road_speed_70_slew_tick = now_70;
		while ((uint32_t)(now_70 - road_speed_70_slew_tick) >=
		       ROAD_SPEED_70_SLEW_INTERVAL_MS) {
			if (road_speed_70_slewed_rpm < desired_70_rpm)
				road_speed_70_slewed_rpm++;
			else if (road_speed_70_slewed_rpm > desired_70_rpm)
				road_speed_70_slewed_rpm--;
			road_speed_70_slew_tick += ROAD_SPEED_70_SLEW_INTERVAL_MS;
		}
		command_base_rpm = road_speed_70_slewed_rpm;
	} else {
		road_speed_70_slewed_rpm = ROAD_SPEED_70_CURVE_CAP_RPM;
		road_speed_70_slew_tick = 0U;
	}
	/* Preview speed reduction is proportional to remembered curvature and
	 * slewed in small percentage steps. Existing motor-side RPM ramping remains
	 * the final physical slew limiter. */
	command_base_rpm = (uint16_t)(((uint32_t)command_base_rpm *
			(uint32_t)lane_speed_scale_percent + 50U) / 100U);
	/* The continuous 70-zone slew above already handles curve entry.  Do not
	 * apply a second instantaneous 150-RPM cap here; that old cap caused the
	 * audible speed kick and lateral disturbance when a sign preceded a bend. */
	/* Keep the 100 RPM road-zone response above the motor's weak low-speed
	 * region during normal curvature-based slowing.  Dropout and untrusted-lane
	 * safety caps below remain authoritative and may still reduce it further. */
	if ((current_base_rpm == ROAD_SPEED_30_RPM) &&
	    (command_base_rpm < LANE_100RPM_CURVE_MIN_BASE_RPM)) {
		command_base_rpm = LANE_100RPM_CURVE_MIN_BASE_RPM;
	}
	if (lane_memory_timed_out != false) {
		current_effective_base_rpm = 0U;
		return target_rpm;
	}
	if (lane_dropout_active != false) {
		if (lane_dropout_timed_out != false) {
			current_effective_base_rpm = 0U;
			return target_rpm;
		}
		if (command_base_rpm > LANE_DROPOUT_BASE_RPM) {
			command_base_rpm = LANE_DROPOUT_BASE_RPM;
		}
	}
	if ((lane_filter_state == LANE_FILTER_HOLD) &&
	    (command_base_rpm > LANE_UNTRUSTED_BASE_RPM)) {
		command_base_rpm = LANE_UNTRUSTED_BASE_RPM;
	}
	current_effective_base_rpm = command_base_rpm;

	/* =========================================
	 * Lane PD
	 *
	 * Inside the deadband, both motors use the
	 * same base RPM.
	 *
	 * Outside the deadband, the PD correction is
	 * applied as a left/right differential.
	 * ========================================= */

	/* Hold launch steering nearly neutral while several fresh FAR/NEAR frames
	 * establish a consistent direction.  This prevents one noisy NEAR sample
	 * from producing an immediate full-lock turn after motor start. */
	if ((uint32_t)(millis() - vehicle_enable_tick) <
	    LANE_STARTUP_STEERING_GUARD_MS) {
		if (current_pd_output > LANE_STARTUP_MAX_CORRECTION_RPM) {
			current_pd_output = LANE_STARTUP_MAX_CORRECTION_RPM;
		} else if (current_pd_output < -LANE_STARTUP_MAX_CORRECTION_RPM) {
			current_pd_output = -LANE_STARTUP_MAX_CORRECTION_RPM;
		}
	}

	/* APP_LanePD already applies deadband and optional slew limiting. */
	left_rpm = (int32_t)command_base_rpm - current_pd_output;

	right_rpm = (int32_t)command_base_rpm + current_pd_output;

	/* =========================================
	 * Left RPM Clamp
	 * ========================================= */

	if (left_rpm < 0) {
		left_rpm = 0;
	} else if (left_rpm > (int32_t)VEHICLE_MAX_RPM) {
		left_rpm = (int32_t)VEHICLE_MAX_RPM;
	} else {
		/* No action */
	}

	/* =========================================
	 * Right RPM Clamp
	 * ========================================= */

	if (right_rpm < 0) {
		right_rpm = 0;
	} else if (right_rpm > (int32_t)VEHICLE_MAX_RPM) {
		right_rpm = (int32_t)VEHICLE_MAX_RPM;
	} else {
		/* No action */
	}

	target_rpm.left_rpm = (uint16_t)left_rpm;

	target_rpm.right_rpm = (uint16_t)right_rpm;

	return target_rpm;
}
