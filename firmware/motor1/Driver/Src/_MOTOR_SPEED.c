#include "_MOTOR_SPEED.h"

#include "tim.h"
#include "gpio.h"
#include "_SIN_MAP.h"

volatile MotorSpeedTuning g_motor_speed_tuning =
{
    MOTOR_SPEED_MAX_TARGET_RPM,
    MOTOR_SPEED_MAX_PWM_AMPLITUDE,
    MOTOR_SPEED_MID_PWM_AMPLITUDE,
    MOTOR_SPEED_MIN_PWM_AMPLITUDE,
    MOTOR_SPEED_LOW_PWM_PER_RPM,
    MOTOR_SPEED_LOW_PWM_END_RPM,
    MOTOR_SPEED_HIGH_BOOST_START_RPM,
    MOTOR_SPEED_HIGH_BOOST_PWM_PER_RPM,
    MOTOR_SPEED_RAMP_RPM_PER_STEP,
    MOTOR_SPEED_LOW_RAMP_END_RPM,
    MOTOR_SPEED_LOW_RAMP_INTERVAL_TICKS,
    MOTOR_SPEED_STARTUP_RAMP_INTERVAL_TICKS,
    MOTOR_SPEED_HIGH_RAMP_INTERVAL_TICKS,
    MOTOR_SPEED_ZERO_DEADBAND_COUNTS_PER_TICK,
    MOTOR_SPEED_ALIGN_PWM_AMPLITUDE,
    MOTOR_SPEED_ALIGN_HOLD_TIME_MS,
    MOTOR_SPEED_KP_PWM_PER_RPM,
    MOTOR_SPEED_KI_DIVISOR,
    MOTOR_SPEED_INTEGRAL_LIMIT_PWM
};

typedef struct
{
    volatile int16_t target_rpm;
    volatile int16_t target_counts_per_tick;
    volatile int16_t measured_counts_per_tick;
    int32_t velocity_window_sum;
    int32_t filtered_velocity_q8;
    int32_t commanded_rpm;
    int32_t speed_error_integral;
    volatile int32_t command_electrical_step_q16;
    volatile uint32_t command_electrical_angle_q16;
    volatile uint16_t command_pwm_amplitude;
    uint16_t previous_angle_counts;
    uint16_t stall_recovery_counter;
    uint16_t recovery_boost_ticks;
    uint8_t velocity_window_count;
    uint8_t previous_angle_valid;
    volatile uint8_t command_angle_valid;
    uint8_t ramp_divider;
    volatile uint8_t is_active;
} MotorSpeedControlState;

typedef struct
{
    volatile uint8_t is_active;
    volatile uint8_t succeeded;
    uint16_t elapsed_ms;
} MotorSpeedAlignState;

typedef struct
{
    int32_t electrical_zero_offset_counts;
    int8_t encoder_direction_sign;
    uint16_t pole_pairs;
} MotorSpeedCalibration;

typedef struct
{
    volatile uint32_t sample_count;
    volatile uint16_t angle;
    volatile uint16_t previous_angle;
    volatile int16_t raw_delta;
    volatile int16_t max_abs_delta;
    volatile int16_t measured_rpm;
    volatile int16_t target_rpm;
    volatile int32_t commanded_rpm;
    volatile uint16_t pwm_amplitude;
    volatile uint16_t stall_counter;
    volatile uint8_t active;
    volatile uint8_t aligning;
} MotorSpeedDiagnostic;

volatile MotorSpeedDiagnostic g_motor1_speed_diagnostic = {0};

static MotorSpeedControlState speed_state = {0};
static MotorSpeedAlignState align_state = {0};
static MotorSpeedCalibration calibration =
{
    0,
    MOTOR_SPEED_ENCODER_DIRECTION_SIGN,
    MOTOR_SPEED_POLE_PAIRS
};
static int8_t previous_phase_a_sign = 0;
static int8_t previous_phase_b_sign = 0;

static int16_t MotorSpeed_AngleDifference(uint16_t new_angle, uint16_t old_angle);
static void MotorSpeed_UpdateVelocity(uint16_t measured_angle);
static void MotorSpeed_RunClosedLoopController(uint16_t measured_angle);
static void MotorSpeed_SyncCommandAngle(uint16_t measured_angle);
static void MotorSpeed_SyncCommandAngleWithLead(uint16_t measured_angle,
                                                int32_t target_rpm);
static int32_t MotorSpeed_Limit(int32_t value, int32_t minimum, int32_t maximum);
static void MotorSpeed_AlignStep(uint16_t measured_angle);
static void MotorSpeed_SetVoltageVector(uint16_t electrical_angle, uint16_t amplitude);
static void MotorSpeed_SetPhasePwm(int16_t phase_a_pwm, int16_t phase_b_pwm);
static void MotorSpeed_EnableOutput(void);
static void MotorSpeed_DisableOutput(void);

void MotorSpeed_SetTarget(int16_t target_counts_per_tick)
{
    speed_state.target_counts_per_tick = target_counts_per_tick;
    speed_state.target_rpm =
        MotorSpeed_CountsPerTickToRpm(target_counts_per_tick);
}

void MotorSpeed_SetTargetRpm(int32_t target_rpm)
{
    int32_t scaled_counts;
    int32_t max_target_rpm = g_motor_speed_tuning.max_target_rpm;

    if (max_target_rpm < 0) max_target_rpm = -max_target_rpm;
    if (max_target_rpm > 32767) max_target_rpm = 32767;

    target_rpm = MotorSpeed_Limit(
        target_rpm,
        -max_target_rpm,
         max_target_rpm);
    scaled_counts = target_rpm * (int32_t)MOTOR_SPEED_FULL_TURN_COUNTS;

    /* Round to the nearest encoder count per 1 ms control tick. */
    if (scaled_counts >= 0)
    {
        scaled_counts += 30000;
    }
    else
    {
        scaled_counts -= 30000;
    }

    scaled_counts /= 60000;
    scaled_counts = MotorSpeed_Limit(scaled_counts, -32768, 32767);
    speed_state.target_rpm = (int16_t)target_rpm;
    speed_state.target_counts_per_tick = (int16_t)scaled_counts;
}

bool MotorSpeed_SetRampInterval(uint8_t band, uint16_t interval_ticks)
{
    if ((interval_ticks < 5U) || (interval_ticks > 300U)) return false;
    if (band == 0U) g_motor_speed_tuning.low_ramp_interval_ticks = interval_ticks;
    else if (band == 1U) g_motor_speed_tuning.startup_ramp_interval_ticks = interval_ticks;
    else if (band == 2U) g_motor_speed_tuning.high_ramp_interval_ticks = interval_ticks;
    else return false;
    return true;
}

int16_t MotorSpeed_CountsPerTickToRpm(int16_t counts_per_tick)
{
    int32_t scaled_rpm = (int32_t)counts_per_tick * 60000;

    if (scaled_rpm >= 0)
    {
        scaled_rpm += (int32_t)MOTOR_SPEED_FULL_TURN_COUNTS / 2;
    }
    else
    {
        scaled_rpm -= (int32_t)MOTOR_SPEED_FULL_TURN_COUNTS / 2;
    }

    scaled_rpm /= (int32_t)MOTOR_SPEED_FULL_TURN_COUNTS;
    scaled_rpm = MotorSpeed_Limit(scaled_rpm, -32768, 32767);
    return (int16_t)scaled_rpm;
}

int16_t MotorSpeed_GetFeedbackRpm(void)
{
    return MotorSpeed_CountsPerTickToRpm(speed_state.measured_counts_per_tick);
}

void MotorSpeed_Start(void)
{
    /* The rotor electrical angle is unknown until alignment is complete. */
    if (!align_state.succeeded)
    {
        MotorSpeed_Stop();
        return;
    }

    speed_state.velocity_window_sum = 0;
    speed_state.filtered_velocity_q8 = 0;
    if (speed_state.target_rpm > 0)
    {
        speed_state.commanded_rpm =
            (speed_state.target_rpm < MOTOR_SPEED_STARTUP_MIN_RPM) ?
            speed_state.target_rpm : MOTOR_SPEED_STARTUP_MIN_RPM;
    }
    else if (speed_state.target_rpm < 0)
    {
        speed_state.commanded_rpm =
            (speed_state.target_rpm > -MOTOR_SPEED_STARTUP_MIN_RPM) ?
            speed_state.target_rpm : -MOTOR_SPEED_STARTUP_MIN_RPM;
    }
    else
    {
        speed_state.commanded_rpm = 0;
    }
    speed_state.speed_error_integral = 0;
    speed_state.command_electrical_step_q16 = 0;
    speed_state.command_electrical_angle_q16 = 0;
    speed_state.command_pwm_amplitude = 0;
    speed_state.stall_recovery_counter = 0;
    speed_state.recovery_boost_ticks = 0;
    speed_state.velocity_window_count = 0;
    speed_state.measured_counts_per_tick = 0;
    speed_state.previous_angle_valid = 0;
    speed_state.command_angle_valid = 0;
    speed_state.ramp_divider = 0;
    MotorSpeed_EnableOutput();
    speed_state.is_active = 1;
}

void MotorSpeed_ControlStep(uint16_t measured_angle_counts)
{
    measured_angle_counts &= MOTOR_SPEED_ANGLE_MASK;
    MotorSpeed_UpdateVelocity(measured_angle_counts);

    if (align_state.is_active)
    {
        MotorSpeed_AlignStep(measured_angle_counts);
    }
    else if (speed_state.is_active)
    {
        MotorSpeed_RunClosedLoopController(measured_angle_counts);
    }
}

static void MotorSpeed_UpdateVelocity(uint16_t measured_angle)
{
    int16_t raw_velocity = 0;
    int32_t window_velocity_q8;
    int32_t rounded_velocity;

    if (speed_state.previous_angle_valid)
    {
        raw_velocity = MotorSpeed_AngleDifference(
            measured_angle, speed_state.previous_angle_counts);
    }

    else
    {
        speed_state.previous_angle_valid = 1;
    }

    g_motor1_speed_diagnostic.sample_count++;
    g_motor1_speed_diagnostic.angle = measured_angle;
    g_motor1_speed_diagnostic.previous_angle =
        speed_state.previous_angle_counts;
    g_motor1_speed_diagnostic.raw_delta = raw_velocity;
    {
        int32_t absolute_delta = raw_velocity;
        if (absolute_delta < 0) absolute_delta = -absolute_delta;
        if (absolute_delta > g_motor1_speed_diagnostic.max_abs_delta)
        {
            g_motor1_speed_diagnostic.max_abs_delta = (int16_t)absolute_delta;
        }
    }

    speed_state.previous_angle_counts = measured_angle;

    /* A single 1 ms angle difference is dominated by encoder jitter. Average
       ten samples first, then low-pass the fixed-point counts/ms result. */
    speed_state.velocity_window_sum += (int32_t)raw_velocity;
    speed_state.velocity_window_count++;

    if (speed_state.velocity_window_count < MOTOR_SPEED_VELOCITY_WINDOW_TICKS)
    {
        return;
    }

    window_velocity_q8 =
        (speed_state.velocity_window_sum *
         (1L << MOTOR_SPEED_VELOCITY_Q_SHIFT)) /
        (int32_t)MOTOR_SPEED_VELOCITY_WINDOW_TICKS;
    speed_state.filtered_velocity_q8 +=
        (window_velocity_q8 - speed_state.filtered_velocity_q8) /
        MOTOR_SPEED_FILTER_DIVISOR;

    if (speed_state.filtered_velocity_q8 >= 0)
    {
        rounded_velocity =
            (speed_state.filtered_velocity_q8 +
             (1L << (MOTOR_SPEED_VELOCITY_Q_SHIFT - 1U))) >>
            MOTOR_SPEED_VELOCITY_Q_SHIFT;
    }
    else
    {
        rounded_velocity = -(((-speed_state.filtered_velocity_q8) +
             (1L << (MOTOR_SPEED_VELOCITY_Q_SHIFT - 1U))) >>
            MOTOR_SPEED_VELOCITY_Q_SHIFT);
    }

    rounded_velocity *= MOTOR_SPEED_FEEDBACK_DIRECTION;
    speed_state.measured_counts_per_tick = (int16_t)MotorSpeed_Limit(
        rounded_velocity, -32768, 32767);
    speed_state.velocity_window_sum = 0;
    speed_state.velocity_window_count = 0;
}

static void MotorSpeed_RunClosedLoopController(uint16_t measured_angle)
{
    int32_t target_rpm = speed_state.target_rpm;
    int32_t absolute_commanded_rpm;
    int32_t absolute_measured_rpm;
    int32_t ramp_interval_ticks;
    int32_t pwm_amplitude;
    int64_t electrical_step_q16;
    int32_t ramp_step = g_motor_speed_tuning.ramp_rpm_per_step;
    int32_t zero_deadband = g_motor_speed_tuning.zero_deadband_counts_per_tick;
    int32_t measured_rpm;
    int32_t speed_error;
    int32_t speed_correction_rpm;
    int32_t kp = g_motor_speed_tuning.kp_pwm_per_rpm;
    int32_t ki_divisor = g_motor_speed_tuning.ki_divisor;
    int32_t correction_limit_rpm = g_motor_speed_tuning.integral_limit_pwm;

    g_motor1_speed_diagnostic.target_rpm = speed_state.target_rpm;
    g_motor1_speed_diagnostic.measured_rpm =
        MotorSpeed_CountsPerTickToRpm(speed_state.measured_counts_per_tick);
    g_motor1_speed_diagnostic.commanded_rpm = speed_state.commanded_rpm;
    g_motor1_speed_diagnostic.stall_counter =
        speed_state.stall_recovery_counter;
    g_motor1_speed_diagnostic.active = speed_state.is_active;
    g_motor1_speed_diagnostic.aligning = align_state.is_active;

    if (ramp_step < 1) ramp_step = 1;
    if (zero_deadband < 0) zero_deadband = 0;
    if (kp < 0) kp = 0;
    if (ki_divisor < 1) ki_divisor = 1;
    if (correction_limit_rpm < 0) correction_limit_rpm = -correction_limit_rpm;

    /* Ramp the requested speed setpoint, then close the loop around the
       encoder-derived measured speed by slightly advancing or retarding the
       proven open-loop electrical vector speed. */
    absolute_commanded_rpm = speed_state.commanded_rpm;
    if (absolute_commanded_rpm < 0)
    {
        absolute_commanded_rpm = -absolute_commanded_rpm;
    }
    if (absolute_commanded_rpm < MOTOR_SPEED_STARTUP_FAST_END_RPM)
    {
        ramp_interval_ticks = g_motor_speed_tuning.low_ramp_interval_ticks;
    }
    else if (absolute_commanded_rpm < g_motor_speed_tuning.low_ramp_end_rpm)
    {
        ramp_interval_ticks = g_motor_speed_tuning.startup_ramp_interval_ticks;
    }
    else
    {
        ramp_interval_ticks = g_motor_speed_tuning.high_ramp_interval_ticks;
    }
    if (ramp_interval_ticks < 1) ramp_interval_ticks = 1;

    if (++speed_state.ramp_divider >= ramp_interval_ticks)
    {
        speed_state.ramp_divider = 0;
        if (speed_state.commanded_rpm < target_rpm)
        {
            speed_state.commanded_rpm += ramp_step;
            if (speed_state.commanded_rpm > target_rpm) speed_state.commanded_rpm = target_rpm;
        }
        else if (speed_state.commanded_rpm > target_rpm)
        {
            speed_state.commanded_rpm -= ramp_step;
            if (speed_state.commanded_rpm < target_rpm) speed_state.commanded_rpm = target_rpm;
        }
    }

    if ((target_rpm == 0) && (speed_state.commanded_rpm == 0) &&
        (speed_state.measured_counts_per_tick >=
            -zero_deadband) &&
        (speed_state.measured_counts_per_tick <=
             zero_deadband))
    {
        speed_state.speed_error_integral = 0;
        speed_state.command_pwm_amplitude = 0;
        speed_state.command_angle_valid = 0;
        speed_state.command_electrical_step_q16 = 0;
        MotorSpeed_DisableOutput();
        return;
    }

    if (!speed_state.command_angle_valid)
    {
        /* Use a moderate 45-degree starting torque angle.  A 90-degree jump
           caused position-dependent reverse impulses, while zero lead did
           not provide enough torque to capture the rotor reliably. */
        MotorSpeed_SyncCommandAngleWithLead(measured_angle, target_rpm);
    }

    measured_rpm =
        MotorSpeed_CountsPerTickToRpm(speed_state.measured_counts_per_tick);
    speed_error = speed_state.commanded_rpm - measured_rpm;

    speed_state.speed_error_integral += speed_error;
    {
        int32_t integral_limit = correction_limit_rpm * ki_divisor;
        speed_state.speed_error_integral = MotorSpeed_Limit(
            speed_state.speed_error_integral,
            -integral_limit,
             integral_limit);
    }

    speed_correction_rpm =
        ((speed_error * kp) / 8) +
        (speed_state.speed_error_integral / ki_divisor);
    speed_correction_rpm = MotorSpeed_Limit(
        speed_correction_rpm,
        -correction_limit_rpm,
         correction_limit_rpm);

    electrical_step_q16 =
        ((int64_t)(speed_state.commanded_rpm + speed_correction_rpm) *
         (int64_t)(-calibration.encoder_direction_sign) *
         (int64_t)calibration.pole_pairs *
         (int64_t)MOTOR_SPEED_FULL_TURN_COUNTS *
         (1LL << MOTOR_SPEED_COMMAND_Q_SHIFT)) /
        (60LL * (int64_t)MOTOR_SPEED_VECTOR_UPDATE_HZ);
    speed_state.command_electrical_step_q16 =
        (int32_t)electrical_step_q16;

    absolute_measured_rpm = measured_rpm;
    if (absolute_measured_rpm < 0)
    {
        absolute_measured_rpm = -absolute_measured_rpm;
    }
    if ((absolute_commanded_rpm >= MOTOR_SPEED_STALL_DETECT_START_RPM) &&
        (((speed_state.commanded_rpm > 0) && (measured_rpm < -20)) ||
         ((speed_state.commanded_rpm < 0) && (measured_rpm > 20)) ||
         (absolute_measured_rpm < ((absolute_commanded_rpm * 2) / 3))))
    {
        if (speed_state.stall_recovery_counter < MOTOR_SPEED_STALL_DETECT_TICKS)
        {
            speed_state.stall_recovery_counter++;
        }
    }
    else
    {
        speed_state.stall_recovery_counter = 0;
    }

    if (speed_state.stall_recovery_counter >= MOTOR_SPEED_STALL_DETECT_TICKS)
    {
        /* Never jump the electrical angle while the rotor is moving.  That
           recovery method produced a reverse torque impulse under load.
           Preserve phase continuity and temporarily raise only the voltage. */
        speed_state.speed_error_integral = 0;
        speed_state.stall_recovery_counter = 0;
        speed_state.recovery_boost_ticks = MOTOR_SPEED_RECOVERY_BOOST_TICKS;
    }

    absolute_commanded_rpm = speed_state.commanded_rpm;
    if (absolute_commanded_rpm < 0)
    {
        absolute_commanded_rpm = -absolute_commanded_rpm;
    }
    if (absolute_commanded_rpm < g_motor_speed_tuning.low_pwm_end_rpm)
    {
        pwm_amplitude = g_motor_speed_tuning.min_pwm_amplitude +
            (absolute_commanded_rpm * g_motor_speed_tuning.low_pwm_per_rpm);
    }
    else
    {
        pwm_amplitude = g_motor_speed_tuning.mid_pwm_amplitude;
        if (absolute_commanded_rpm > g_motor_speed_tuning.high_boost_start_rpm)
        {
            pwm_amplitude +=
                (absolute_commanded_rpm -
                 g_motor_speed_tuning.high_boost_start_rpm) *
                g_motor_speed_tuning.high_boost_pwm_per_rpm;
        }
    }
    if ((speed_error > 10) && (target_rpm > 0))
    {
        pwm_amplitude += speed_error * 20;
    }
    else if ((speed_error < -10) && (target_rpm < 0))
    {
        pwm_amplitude += (-speed_error) * 20;
    }
    {
        int32_t min_pwm = g_motor_speed_tuning.min_pwm_amplitude;
        int32_t max_pwm = g_motor_speed_tuning.max_pwm_amplitude;
        int32_t requested_pwm;
        int32_t current_pwm = speed_state.command_pwm_amplitude;
        if (min_pwm < 0) min_pwm = 0;
        if (min_pwm > 3599) min_pwm = 3599;
        if (max_pwm < min_pwm) max_pwm = min_pwm;
        if (max_pwm > 3599) max_pwm = 3599;
        requested_pwm = MotorSpeed_Limit(pwm_amplitude, min_pwm, max_pwm);
        if (speed_state.recovery_boost_ticks > 0U)
        {
            speed_state.recovery_boost_ticks--;
            requested_pwm = MotorSpeed_Limit(
                MOTOR_SPEED_RECOVERY_PWM_AMPLITUDE, min_pwm, max_pwm);
        }
        if (current_pwm == 0)
        {
            current_pwm = requested_pwm;
        }
        else if (requested_pwm > (current_pwm + MOTOR_SPEED_PWM_SLEW_PER_MS))
        {
            current_pwm += MOTOR_SPEED_PWM_SLEW_PER_MS;
        }
        else if (requested_pwm < (current_pwm - MOTOR_SPEED_PWM_SLEW_PER_MS))
        {
            current_pwm -= MOTOR_SPEED_PWM_SLEW_PER_MS;
        }
        else
        {
            current_pwm = requested_pwm;
        }
        speed_state.command_pwm_amplitude = (uint16_t)current_pwm;
        g_motor1_speed_diagnostic.pwm_amplitude =
            speed_state.command_pwm_amplitude;
    }
}

static void MotorSpeed_SyncCommandAngle(uint16_t measured_angle)
{
    int32_t rotor_electrical_angle =
        ((int32_t)measured_angle * calibration.encoder_direction_sign *
         (int32_t)calibration.pole_pairs)
        + calibration.electrical_zero_offset_counts;

    speed_state.command_electrical_angle_q16 =
        ((uint32_t)rotor_electrical_angle & MOTOR_SPEED_ANGLE_MASK) <<
        MOTOR_SPEED_COMMAND_Q_SHIFT;
    speed_state.command_angle_valid = 1;
}

static void MotorSpeed_SyncCommandAngleWithLead(uint16_t measured_angle,
                                                int32_t target_rpm)
{
    int32_t rotor_electrical_angle =
        ((int32_t)measured_angle * calibration.encoder_direction_sign *
         (int32_t)calibration.pole_pairs)
        + calibration.electrical_zero_offset_counts;
    int32_t lead_counts =
        (target_rpm >= 0)
        ? (int32_t)(MOTOR_SPEED_FULL_TURN_COUNTS / 8U)
        : -(int32_t)(MOTOR_SPEED_FULL_TURN_COUNTS / 8U);

    speed_state.command_electrical_angle_q16 =
        ((uint32_t)(rotor_electrical_angle + lead_counts) &
         MOTOR_SPEED_ANGLE_MASK) << MOTOR_SPEED_COMMAND_Q_SHIFT;
    speed_state.command_angle_valid = 1;
}

void MotorSpeed_FastControlStep(void)
{
    static uint8_t pwm_update_divider = 0;
    const int32_t electrical_period_q16 =
        (int32_t)(MOTOR_SPEED_FULL_TURN_COUNTS <<
                  MOTOR_SPEED_COMMAND_Q_SHIFT);
    int32_t next_electrical_angle_q16;

    if (++pwm_update_divider < MOTOR_SPEED_FAST_UPDATE_DIVIDER)
    {
        return;
    }
    pwm_update_divider = 0;

    if ((!speed_state.is_active) || (!speed_state.command_angle_valid))
    {
        return;
    }

    next_electrical_angle_q16 =
        (int32_t)speed_state.command_electrical_angle_q16 +
        speed_state.command_electrical_step_q16;

    if (next_electrical_angle_q16 < 0)
    {
        next_electrical_angle_q16 += electrical_period_q16;
    }
    else if (next_electrical_angle_q16 >= electrical_period_q16)
    {
        next_electrical_angle_q16 -= electrical_period_q16;
    }

    speed_state.command_electrical_angle_q16 =
        (uint32_t)next_electrical_angle_q16;
    MotorSpeed_SetVoltageVector(
        (uint16_t)(speed_state.command_electrical_angle_q16 >>
                   MOTOR_SPEED_COMMAND_Q_SHIFT),
        speed_state.command_pwm_amplitude);
}

static int16_t MotorSpeed_AngleDifference(uint16_t new_angle, uint16_t old_angle)
{
    int32_t difference = (int32_t)new_angle - (int32_t)old_angle;

    /* Correct the 16383 -> 0 (or 0 -> 16383) encoder wrap-around. */
    if (difference > (int32_t)(MOTOR_SPEED_FULL_TURN_COUNTS / 2U))
    {
        difference -= MOTOR_SPEED_FULL_TURN_COUNTS;
    }
    else if (difference < -(int32_t)(MOTOR_SPEED_FULL_TURN_COUNTS / 2U))
    {
        difference += MOTOR_SPEED_FULL_TURN_COUNTS;
    }

    return (int16_t)difference;
}

static int32_t MotorSpeed_Limit(int32_t value, int32_t minimum, int32_t maximum)
{
    if (value > maximum) return maximum;
    if (value < minimum) return minimum;
    return value;
}

void MotorSpeed_AlignStart(void)
{
    int16_t requested_rpm = speed_state.target_rpm;
    int16_t requested_target = speed_state.target_counts_per_tick;

    /* Alignment must stop the bridge and reset the PI state, but it must not
       discard a target RPM that was configured before the ALIGN command. */
    MotorSpeed_Stop();
    speed_state.target_rpm = requested_rpm;
    speed_state.target_counts_per_tick = requested_target;
    align_state.elapsed_ms = 0;
    align_state.succeeded = 0;
    MotorSpeed_EnableOutput();
    align_state.is_active = 1;
}

static void MotorSpeed_AlignStep(uint16_t measured_angle)
{
    /* Restore the known working static alignment. Automatic multi-turn
       calibration slipped on this motor and produced an impossible pole-pair
       estimate, which caused RUN to be rejected. */
    int32_t align_pwm = g_motor_speed_tuning.align_pwm_amplitude;
    int32_t hold_ms = g_motor_speed_tuning.align_hold_time_ms;
    if (align_pwm < 0) align_pwm = 0;
    if (align_pwm > 3599) align_pwm = 3599;
    if (hold_ms < 1) hold_ms = 1;
    MotorSpeed_SetVoltageVector(0, (uint16_t)align_pwm);
    if (++align_state.elapsed_ms < hold_ms)
    {
        return;
    }

    calibration.encoder_direction_sign = MOTOR_SPEED_ENCODER_DIRECTION_SIGN;
    calibration.pole_pairs = MOTOR_SPEED_POLE_PAIRS;
    calibration.electrical_zero_offset_counts =
        -((int32_t)measured_angle *
          (int32_t)calibration.encoder_direction_sign *
          (int32_t)calibration.pole_pairs);
    align_state.succeeded = 1;

    align_state.is_active = 0;
    MotorSpeed_DisableOutput();
}

static void MotorSpeed_SetVoltageVector(uint16_t electrical_angle,
                                        uint16_t amplitude)
{
    int16_t phase_a_sine_q12 = SinLUT_Get(electrical_angle);
    int16_t phase_b_cosine_q12 = CosLUT_Get(electrical_angle);
    int16_t phase_a_pwm = (int16_t)(
        ((int32_t)amplitude * phase_a_sine_q12) / 4096);
    int16_t phase_b_pwm = (int16_t)(
        ((int32_t)amplitude * phase_b_cosine_q12) / 4096);

    MotorSpeed_SetPhasePwm(phase_a_pwm, phase_b_pwm);
}

static int8_t MotorSpeed_GetSign(int16_t value)
{
    return (value > 0) ? 1 : ((value < 0) ? -1 : 0);
}

static uint16_t MotorSpeed_SetBridgeDirection(
    int16_t signed_pwm,
    int8_t new_sign,
    int8_t old_sign,
    uint32_t timer_channel,
    GPIO_TypeDef *positive_port,
    uint16_t positive_pin,
    GPIO_TypeDef *negative_port,
    uint16_t negative_pin)
{
    if (new_sign != old_sign)
    {
        /* Set duty to zero before changing H-bridge direction. */
        __HAL_TIM_SET_COMPARE(&htim2, timer_channel, 0);
        HAL_GPIO_WritePin(positive_port, positive_pin,
                          (new_sign > 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(negative_port, negative_pin,
                          (new_sign < 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }

    return (uint16_t)((signed_pwm >= 0) ? signed_pwm : -signed_pwm);
}

static void MotorSpeed_SetPhasePwm(int16_t phase_a_pwm, int16_t phase_b_pwm)
{
    int8_t phase_a_sign = MotorSpeed_GetSign(phase_a_pwm);
    int8_t phase_b_sign = MotorSpeed_GetSign(phase_b_pwm);
    uint16_t phase_a_duty;
    uint16_t phase_b_duty;

    phase_a_duty = MotorSpeed_SetBridgeDirection(
        phase_a_pwm, phase_a_sign, previous_phase_a_sign, TIM_CHANNEL_3,
        HW_ELEC_AP_GPIO_Port, HW_ELEC_AP_Pin,
        HW_ELEC_AM_GPIO_Port, HW_ELEC_AM_Pin);

    phase_b_duty = MotorSpeed_SetBridgeDirection(
        phase_b_pwm, phase_b_sign, previous_phase_b_sign, TIM_CHANNEL_4,
        HW_ELEC_BP_GPIO_Port, HW_ELEC_BP_Pin,
        HW_ELEC_BM_GPIO_Port, HW_ELEC_BM_Pin);

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, phase_a_duty);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, phase_b_duty);

    previous_phase_a_sign = phase_a_sign;
    previous_phase_b_sign = phase_b_sign;
}

static void MotorSpeed_EnableOutput(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    __HAL_AFIO_REMAP_TIM2_PARTIAL_2();

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
    HAL_TIM_Base_Start_IT(&htim2);
}

static void MotorSpeed_DisableOutput(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4);
    HAL_TIM_Base_Stop_IT(&htim2);

    HAL_GPIO_WritePin(HW_ELEC_AP_GPIO_Port, HW_ELEC_AP_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(HW_ELEC_AM_GPIO_Port, HW_ELEC_AM_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(HW_ELEC_BP_GPIO_Port, HW_ELEC_BP_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(HW_ELEC_BM_GPIO_Port, HW_ELEC_BM_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10 | GPIO_PIN_11, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    previous_phase_a_sign = 0;
    previous_phase_b_sign = 0;
}

int16_t MotorSpeed_GetTarget(void)
{
    return speed_state.target_counts_per_tick;
}

int16_t MotorSpeed_GetFeedback(void)
{
    return speed_state.measured_counts_per_tick;
}

int16_t MotorSpeed_GetTargetRpm(void)
{
    return speed_state.target_rpm;
}

uint8_t MotorSpeed_IsActive(void)
{
    return speed_state.is_active;
}

uint8_t MotorSpeed_AlignIsDone(void)
{
    return align_state.succeeded;
}

uint8_t MotorSpeed_AlignIsActive(void)
{
    return align_state.is_active;
}

void MotorSpeed_Stop(void)
{
    speed_state.is_active = 0;
    speed_state.target_rpm = 0;
    speed_state.target_counts_per_tick = 0;
    speed_state.measured_counts_per_tick = 0;
    speed_state.velocity_window_sum = 0;
    speed_state.filtered_velocity_q8 = 0;
    speed_state.commanded_rpm = 0;
    speed_state.speed_error_integral = 0;
    speed_state.command_electrical_step_q16 = 0;
    speed_state.command_electrical_angle_q16 = 0;
    speed_state.command_pwm_amplitude = 0;
    speed_state.stall_recovery_counter = 0;
    speed_state.recovery_boost_ticks = 0;
    speed_state.velocity_window_count = 0;
    speed_state.previous_angle_valid = 0;
    speed_state.command_angle_valid = 0;
    speed_state.ramp_divider = 0;
    align_state.is_active = 0;
    MotorSpeed_DisableOutput();
}
