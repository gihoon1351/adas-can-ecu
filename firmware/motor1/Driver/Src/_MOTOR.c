#include "_MOTOR.h"
#include "_MOTOR_SPEED.h"

volatile MotorPositionTuning g_motor_position_tuning =
{
    MOTOR_POSITION_KP_NUMERATOR,
    MOTOR_POSITION_KP_DENOMINATOR,
    MOTOR_POSITION_KD,
    MOTOR_POSITION_MAX_PWM_AMPLITUDE,
    MOTOR_POSITION_DEADBAND_COUNTS,
    MOTOR_VELOCITY_DEADBAND_COUNTS_PER_TICK,
    MOTOR_VELOCITY_FILTER_DIVISOR,
    MOTOR_ALIGN_PWM_AMPLITUDE,
    MOTOR_ALIGN_STAGE_TIME_MS,
    MOTOR_ALIGN_MIN_MOVEMENT_COUNTS
};

typedef enum
{
    MOTOR_ALIGN_STAGE_HOLD = 0,
    MOTOR_ALIGN_STAGE_ROTATE
} MotorAlignStage;

typedef struct
{
    volatile uint16_t target_mechanical_angle_counts;
    uint16_t previous_position_angle_counts;
    int32_t velocity_filter_accumulator;
    volatile uint8_t is_active;
    uint8_t previous_position_angle_valid;
} MotorPositionState;

typedef struct
{
    volatile uint16_t measured_mechanical_angle_counts;
    volatile int16_t measured_velocity_counts_per_tick;
    uint16_t previous_feedback_angle_counts;
    uint8_t previous_feedback_angle_valid;
} MotorFeedbackState;

typedef struct
{
    volatile uint8_t is_active;
    volatile uint8_t calibration_succeeded;
    MotorAlignStage stage;
    uint16_t stage_elapsed_ms;
    uint16_t rotation_start_mechanical_angle_counts;
} MotorAlignState;

typedef struct
{
    int32_t electrical_zero_offset_counts;
    int8_t encoder_to_electrical_direction_sign;
} MotorCalibration;

static MotorPositionState position_state = {0};
static MotorFeedbackState feedback_state = {0};
static MotorAlignState align_state = {0};
static MotorCalibration calibration =
{
    MOTOR_ELECTRICAL_ZERO_OFFSET_COUNTS,
    MOTOR_ENCODER_TO_ELECTRICAL_DIRECTION
};

static uint16_t open_loop_electrical_angle_counts = 0;
static int8_t previous_phase_a_sign = 0;
static int8_t previous_phase_b_sign = 0;
static volatile MotorControlMode motor_control_mode = MOTOR_CONTROL_MODE_POSITION;
static volatile MotorControlState motor_control_state = MOTOR_CONTROL_STATE_STOPPED;
static volatile uint16_t motor_control_feedback_angle = 0U;

static int16_t Motor_AngleDifferenceCounts(uint16_t target_angle_counts,
                                           uint16_t actual_angle_counts);
static void Motor_UpdateFeedback(uint16_t measured_mechanical_angle_counts);
static void Motor_AlignStep(uint16_t measured_mechanical_angle_counts);
static void Motor_AlignHoldStep(uint16_t measured_mechanical_angle_counts);
static void Motor_AlignRotateStep(uint16_t measured_mechanical_angle_counts);
static void Motor_AlignFinish(uint16_t measured_mechanical_angle_counts,
                              uint16_t command_electrical_angle_counts);
static void Motor_PositionStep(uint16_t measured_mechanical_angle_counts);
static int32_t Motor_CalculateTorqueCommand(int16_t position_error_counts,
                                            int16_t measured_velocity_counts_per_tick);
static int32_t Motor_LimitTorqueCommand(int32_t signed_torque_pwm_command);
static void Motor_ApplyTorque(uint16_t measured_mechanical_angle_counts,
                              int32_t signed_torque_pwm_command);
static void Motor_SetVoltageVector(uint16_t command_voltage_angle_counts,
                                   uint16_t pwm_amplitude);
static int8_t Motor_GetCommandSign(int16_t signed_phase_pwm_command);
static uint16_t Motor_ApplyBridgePhase(int16_t signed_phase_pwm_command,
                                       int8_t command_sign,
                                       int8_t previous_command_sign,
                                       uint32_t timer_channel,
                                       GPIO_TypeDef *positive_gpio_port,
                                       uint16_t positive_gpio_pin,
                                       GPIO_TypeDef *negative_gpio_port,
                                       uint16_t negative_gpio_pin);
static void Motor_SetPhasePwm(int16_t phase_a_signed_pwm_command,
                              int16_t phase_b_signed_pwm_command);
static void Motor_EnableOutput(void);
static void Motor_DisableOutput(void);

static int16_t Motor_AngleDifferenceCounts(uint16_t target_angle_counts,
                                           uint16_t actual_angle_counts)
{
    int32_t difference_counts = (int32_t)target_angle_counts
                              - (int32_t)actual_angle_counts;

    if (difference_counts > (int32_t)MOTOR_HALF_TURN_COUNTS)
    {
        difference_counts -= (int32_t)MOTOR_FULL_TURN_COUNTS;
    }
    else if (difference_counts < -(int32_t)MOTOR_HALF_TURN_COUNTS)
    {
        difference_counts += (int32_t)MOTOR_FULL_TURN_COUNTS;
    }

    return (int16_t)difference_counts;
}

void Motor_OpenLoopStep(int16_t electrical_step_counts, uint16_t pwm_amplitude)
{
    int32_t next_electrical_angle_counts =
        (int32_t)open_loop_electrical_angle_counts
        + (int32_t)electrical_step_counts;

    open_loop_electrical_angle_counts =
        (uint16_t)next_electrical_angle_counts & MOTOR_ANGLE_MASK;
    Motor_EnableOutput();
    Motor_SetVoltageVector(open_loop_electrical_angle_counts, pwm_amplitude);
}

void Motor_PositionSetTarget(uint16_t target_mechanical_angle_counts)
{
    position_state.target_mechanical_angle_counts = target_mechanical_angle_counts & MOTOR_ANGLE_MASK;
}

void Motor_PositionStart(void)
{
    if (!align_state.calibration_succeeded)
    {
        Motor_Stop();
        return;
    }

    position_state.previous_position_angle_valid = 0;
    position_state.velocity_filter_accumulator = 0;
    Motor_EnableOutput();
    position_state.is_active = 1;
}

uint8_t Motor_PositionIsActive(void)
{
    return position_state.is_active;
}

void Motor_AlignStart(void)
{
    position_state.is_active = 0;
    position_state.previous_position_angle_valid = 0;
    position_state.velocity_filter_accumulator = 0;

    align_state.stage_elapsed_ms = 0;
    align_state.rotation_start_mechanical_angle_counts = 0;
    align_state.stage = MOTOR_ALIGN_STAGE_HOLD;
    align_state.calibration_succeeded = 0;
    Motor_EnableOutput();
    align_state.is_active = 1;
}

uint8_t Motor_AlignIsDone(void)
{
    return align_state.calibration_succeeded;
}

uint8_t Motor_AlignIsActive(void)
{
    return align_state.is_active;
}

uint16_t Motor_GetFeedbackAngle(void)
{
    return feedback_state.measured_mechanical_angle_counts;
}

int16_t Motor_GetFeedbackVelocity(void)
{
    return feedback_state.measured_velocity_counts_per_tick;
}

void Motor_PositionControlStep(uint16_t measured_mechanical_angle_counts)
{
    measured_mechanical_angle_counts &= MOTOR_ANGLE_MASK;
    Motor_UpdateFeedback(measured_mechanical_angle_counts);

    if (align_state.is_active)
    {
        Motor_AlignStep(measured_mechanical_angle_counts);
        return;
    }

    if (position_state.is_active)
    {
        Motor_PositionStep(measured_mechanical_angle_counts);
    }
}

static void Motor_UpdateFeedback(uint16_t measured_mechanical_angle_counts)
{
    feedback_state.measured_mechanical_angle_counts =
        measured_mechanical_angle_counts;

    if (feedback_state.previous_feedback_angle_valid)
    {
        feedback_state.measured_velocity_counts_per_tick = Motor_AngleDifferenceCounts(measured_mechanical_angle_counts, feedback_state.previous_feedback_angle_counts);
    }
    else
    {
        feedback_state.measured_velocity_counts_per_tick = 0;
        feedback_state.previous_feedback_angle_valid = 1;
    }

    feedback_state.previous_feedback_angle_counts = measured_mechanical_angle_counts;
}

static void Motor_AlignStep(uint16_t measured_mechanical_angle_counts)
{
    switch (align_state.stage)
    {
        case MOTOR_ALIGN_STAGE_HOLD:
            Motor_AlignHoldStep(measured_mechanical_angle_counts);
            break;

        case MOTOR_ALIGN_STAGE_ROTATE:
            Motor_AlignRotateStep(measured_mechanical_angle_counts);
            break;

        default:
            align_state.is_active = 0;
            Motor_DisableOutput();
            break;
    }
}

static void Motor_AlignHoldStep(uint16_t measured_mechanical_angle_counts)
{
    uint16_t align_pwm = (uint16_t)((g_motor_position_tuning.align_pwm_amplitude < 0) ? 0 :
        ((g_motor_position_tuning.align_pwm_amplitude > 3599) ? 3599 : g_motor_position_tuning.align_pwm_amplitude));
    uint32_t stage_time = (uint32_t)((g_motor_position_tuning.align_stage_time_ms <= 0) ? 1 : g_motor_position_tuning.align_stage_time_ms);
    Motor_SetVoltageVector(0U, align_pwm);

    if (++align_state.stage_elapsed_ms >= stage_time)
    {
        align_state.rotation_start_mechanical_angle_counts = measured_mechanical_angle_counts;
        align_state.stage_elapsed_ms = 0;
        align_state.stage = MOTOR_ALIGN_STAGE_ROTATE;
    }
}

static void Motor_AlignRotateStep(uint16_t measured_mechanical_angle_counts)
{
    uint32_t stage_time = (uint32_t)((g_motor_position_tuning.align_stage_time_ms <= 0) ? 1 : g_motor_position_tuning.align_stage_time_ms);
    uint16_t align_pwm = (uint16_t)((g_motor_position_tuning.align_pwm_amplitude < 0) ? 0 :
        ((g_motor_position_tuning.align_pwm_amplitude > 3599) ? 3599 : g_motor_position_tuning.align_pwm_amplitude));
    uint16_t command_electrical_angle_counts = (uint16_t)(((uint32_t)align_state.stage_elapsed_ms * MOTOR_FULL_TURN_COUNTS) / stage_time) & MOTOR_ANGLE_MASK;

    Motor_SetVoltageVector(command_electrical_angle_counts, align_pwm);

    if (++align_state.stage_elapsed_ms >= stage_time)
    {
        Motor_AlignFinish(measured_mechanical_angle_counts, command_electrical_angle_counts);
    }
}

static void Motor_AlignFinish(uint16_t measured_mechanical_angle_counts, uint16_t command_electrical_angle_counts)
{
    int16_t mechanical_displacement_counts = Motor_AngleDifferenceCounts( measured_mechanical_angle_counts, align_state.rotation_start_mechanical_angle_counts);

    int32_t min_movement = (g_motor_position_tuning.align_min_movement_counts < 0) ? 0 : g_motor_position_tuning.align_min_movement_counts;
    if ((mechanical_displacement_counts >= min_movement) || (mechanical_displacement_counts <= -min_movement))
    {
        calibration.encoder_to_electrical_direction_sign =
            (mechanical_displacement_counts > 0) ? 1 : -1;

        calibration.electrical_zero_offset_counts =
            (int32_t)command_electrical_angle_counts
            - ((int32_t)measured_mechanical_angle_counts
               * calibration.encoder_to_electrical_direction_sign
               * (int32_t)MOTOR_POLE_PAIRS);

        align_state.calibration_succeeded = 1;
    }

    align_state.is_active = 0;
    Motor_DisableOutput();
}

static void Motor_PositionStep(uint16_t measured_mechanical_angle_counts)
{
    int16_t raw_velocity_counts_per_tick = 0;
    int16_t filtered_velocity_counts_per_tick;
    int16_t position_error_counts;
    int32_t signed_torque_pwm_command;

    if (position_state.previous_position_angle_valid)
    {
        raw_velocity_counts_per_tick = Motor_AngleDifferenceCounts( measured_mechanical_angle_counts, position_state.previous_position_angle_counts);
    }
    else
    {
        position_state.previous_position_angle_valid = 1;
    }

    position_state.previous_position_angle_counts = measured_mechanical_angle_counts;

    /* Fixed-point first-order low-pass filter. Keeping the fractional part
       in the accumulator lets small residual velocities decay back to zero. */
    int32_t filter_divisor = (g_motor_position_tuning.velocity_filter_divisor <= 0) ? 1 : g_motor_position_tuning.velocity_filter_divisor;
    position_state.velocity_filter_accumulator += (int32_t)raw_velocity_counts_per_tick - (position_state.velocity_filter_accumulator / filter_divisor);
    filtered_velocity_counts_per_tick = (int16_t)(position_state.velocity_filter_accumulator / filter_divisor);

    position_error_counts = Motor_AngleDifferenceCounts( position_state.target_mechanical_angle_counts, measured_mechanical_angle_counts);

    /* Stop switching at the target to avoid idle noise and hunting. */
    int32_t position_deadband = (g_motor_position_tuning.position_deadband_counts < 0) ? 0 : g_motor_position_tuning.position_deadband_counts;
    int32_t velocity_deadband = (g_motor_position_tuning.velocity_deadband_counts_per_tick < 0) ? 0 : g_motor_position_tuning.velocity_deadband_counts_per_tick;
    if ((position_error_counts >= -position_deadband) && (position_error_counts <= position_deadband) && (filtered_velocity_counts_per_tick >= -velocity_deadband) && (filtered_velocity_counts_per_tick <= velocity_deadband))
    {
        Motor_DisableOutput();
        return;
    }

    signed_torque_pwm_command = Motor_CalculateTorqueCommand(position_error_counts,filtered_velocity_counts_per_tick);
    signed_torque_pwm_command = Motor_LimitTorqueCommand(signed_torque_pwm_command);

    Motor_ApplyTorque(measured_mechanical_angle_counts, signed_torque_pwm_command);
}

static int32_t Motor_CalculateTorqueCommand(int16_t position_error_counts, int16_t measured_velocity_counts_per_tick)
{
    int32_t proportional_term;
    int32_t derivative_term;

    /* P: larger position error produces a larger restoring command. */
    int32_t kp_denominator = (g_motor_position_tuning.kp_denominator == 0) ? 1 : g_motor_position_tuning.kp_denominator;
    proportional_term = ((int32_t)position_error_counts * g_motor_position_tuning.kp_numerator) / kp_denominator;

    /* D: measured motion opposes the P command and damps oscillation. */
    derivative_term = (int32_t)measured_velocity_counts_per_tick * g_motor_position_tuning.kd;

    return proportional_term - derivative_term;
}

static int32_t Motor_LimitTorqueCommand(int32_t signed_torque_pwm_command)
{
    int32_t max_pwm = g_motor_position_tuning.max_pwm_amplitude;
    if (max_pwm < 0) max_pwm = 0;
    if (max_pwm > 3599) max_pwm = 3599;
    if (signed_torque_pwm_command > max_pwm)
    {
        return max_pwm;
    }

    if (signed_torque_pwm_command < -max_pwm)
    {
        return -max_pwm;
    }

    return signed_torque_pwm_command;
}

static void Motor_ApplyTorque(uint16_t measured_mechanical_angle_counts,
                              int32_t signed_torque_pwm_command)
{
    int32_t rotor_electrical_angle_counts;
    int32_t electrical_direction_torque_pwm_command;
    int32_t command_voltage_angle_counts;
    uint16_t pwm_amplitude;

    rotor_electrical_angle_counts = ((int32_t)measured_mechanical_angle_counts * calibration.encoder_to_electrical_direction_sign * (int32_t)MOTOR_POLE_PAIRS) + calibration.electrical_zero_offset_counts;

    electrical_direction_torque_pwm_command = signed_torque_pwm_command * calibration.encoder_to_electrical_direction_sign;

    if (electrical_direction_torque_pwm_command >= 0)
    {
        command_voltage_angle_counts = rotor_electrical_angle_counts + (int32_t)MOTOR_QUARTER_TURN_COUNTS;
    }
    else
    {
        command_voltage_angle_counts = rotor_electrical_angle_counts - (int32_t)MOTOR_QUARTER_TURN_COUNTS;
    }

    pwm_amplitude = (uint16_t)((signed_torque_pwm_command >= 0) ? signed_torque_pwm_command : -signed_torque_pwm_command);

    Motor_SetVoltageVector((uint16_t)command_voltage_angle_counts & MOTOR_ANGLE_MASK, pwm_amplitude);
}

static void Motor_SetVoltageVector(uint16_t command_voltage_angle_counts, uint16_t pwm_amplitude)
{
    int16_t phase_a_sine_q12 = SinLUT_Get(command_voltage_angle_counts);
    int16_t phase_b_cosine_q12 = CosLUT_Get(command_voltage_angle_counts);
    int16_t phase_a_signed_pwm_command = (int16_t)(((int32_t)pwm_amplitude * phase_a_sine_q12) / 4096);
    int16_t phase_b_signed_pwm_command = (int16_t)(((int32_t)pwm_amplitude * phase_b_cosine_q12) / 4096);

    Motor_SetPhasePwm(phase_a_signed_pwm_command, phase_b_signed_pwm_command);
}

static int8_t Motor_GetCommandSign(int16_t signed_phase_pwm_command)
{
    return (signed_phase_pwm_command > 0) ? 1 : ((signed_phase_pwm_command < 0) ? -1 : 0);
}

static uint16_t Motor_ApplyBridgePhase(int16_t signed_phase_pwm_command,
                                       int8_t command_sign,
                                       int8_t previous_command_sign,
                                       uint32_t timer_channel,
                                       GPIO_TypeDef *positive_gpio_port,
                                       uint16_t positive_gpio_pin,
                                       GPIO_TypeDef *negative_gpio_port,
                                       uint16_t negative_gpio_pin)
{
    if (command_sign != previous_command_sign)
    {
        __HAL_TIM_SET_COMPARE(&htim2, timer_channel, 0);

        HAL_GPIO_WritePin(positive_gpio_port, positive_gpio_pin, (command_sign > 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(negative_gpio_port, negative_gpio_pin, (command_sign < 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }

    return (uint16_t)((signed_phase_pwm_command >= 0) ? signed_phase_pwm_command : -signed_phase_pwm_command);
}

static void Motor_SetPhasePwm(int16_t phase_a_signed_pwm_command, int16_t phase_b_signed_pwm_command)
{
    int8_t phase_a_sign = Motor_GetCommandSign(phase_a_signed_pwm_command);
    int8_t phase_b_sign = Motor_GetCommandSign(phase_b_signed_pwm_command);
    uint16_t phase_a_pwm_duty_counts;
    uint16_t phase_b_pwm_duty_counts;

    phase_a_pwm_duty_counts = Motor_ApplyBridgePhase(
        phase_a_signed_pwm_command,
        phase_a_sign,
        previous_phase_a_sign,
        TIM_CHANNEL_3,
        HW_ELEC_AP_GPIO_Port,
        HW_ELEC_AP_Pin,
        HW_ELEC_AM_GPIO_Port,
        HW_ELEC_AM_Pin);

    phase_b_pwm_duty_counts = Motor_ApplyBridgePhase(
        phase_b_signed_pwm_command,
        phase_b_sign,
        previous_phase_b_sign,
        TIM_CHANNEL_4,
        HW_ELEC_BP_GPIO_Port,
        HW_ELEC_BP_Pin,
        HW_ELEC_BM_GPIO_Port,
        HW_ELEC_BM_Pin);

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, phase_a_pwm_duty_counts);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, phase_b_pwm_duty_counts);

    previous_phase_a_sign = phase_a_sign;
    previous_phase_b_sign = phase_b_sign;
}

static void Motor_EnableOutput(void)
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

static void Motor_DisableOutput(void)
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

void Motor_Stop(void)
{
    position_state.is_active = 0;
    position_state.previous_position_angle_valid = 0;
    position_state.velocity_filter_accumulator = 0;
    align_state.is_active = 0;

    Motor_DisableOutput();
}

void MotorControl_SetMode(MotorControlMode mode)
{
    if ((mode != MOTOR_CONTROL_MODE_SPEED) &&
        (mode != MOTOR_CONTROL_MODE_POSITION))
    {
        return;
    }

    if (mode == motor_control_mode)
    {
        return;
    }

    /* Stop the controller that currently owns the bridge. Do not stop the
       newly selected speed controller here because that would erase its RPM
       target before RUN. */
    if (motor_control_mode == MOTOR_CONTROL_MODE_SPEED)
    {
        MotorSpeed_Stop();
    }
    else
    {
        Motor_Stop();
    }

    motor_control_state = MOTOR_CONTROL_STATE_STOPPED;
    motor_control_mode = mode;
}

MotorControlMode MotorControl_GetMode(void)
{
    return motor_control_mode;
}

void MotorControl_SetTargetRpm(int32_t target_rpm)
{
    MotorSpeed_SetTargetRpm(target_rpm);
}

void MotorControl_SetTargetDegree(uint16_t target_degree)
{
    target_degree %= 360U;
    Motor_PositionSetTarget((uint16_t)(
        ((uint32_t)target_degree * MOTOR_ANGLE_COUNTS) / 360U));
}

void MotorControl_Start(void)
{
    if (motor_control_mode == MOTOR_CONTROL_MODE_SPEED)
    {
        Motor_Stop();
        MotorSpeed_Start();
        motor_control_state = MotorSpeed_IsActive()
            ? MOTOR_CONTROL_STATE_RUNNING
            : MOTOR_CONTROL_STATE_STOPPED;
    }
    else
    {
        MotorSpeed_Stop();
        Motor_PositionStart();
        motor_control_state = Motor_PositionIsActive()
            ? MOTOR_CONTROL_STATE_RUNNING
            : MOTOR_CONTROL_STATE_STOPPED;
    }
}

void MotorControl_AlignStart(void)
{
    Motor_Stop();

    if (motor_control_mode == MOTOR_CONTROL_MODE_SPEED)
    {
        /* MotorSpeed_AlignStart() resets the speed loop while preserving the
           RPM target configured by SET_PARAM. */
        MotorSpeed_AlignStart();
    }
    else
    {
        MotorSpeed_Stop();
        Motor_AlignStart();
    }

    motor_control_state = MOTOR_CONTROL_STATE_ALIGNING;
}

void MotorControl_Stop(void)
{
    Motor_Stop();
    MotorSpeed_Stop();
    motor_control_state = MOTOR_CONTROL_STATE_STOPPED;
}

void MotorControl_ControlStep(uint16_t measured_angle_counts)
{
    measured_angle_counts &= MOTOR_ANGLE_MASK;
    motor_control_feedback_angle = measured_angle_counts;

    if (motor_control_mode == MOTOR_CONTROL_MODE_SPEED)
    {
        MotorSpeed_ControlStep(measured_angle_counts);
    }
    else
    {
        Motor_PositionControlStep(measured_angle_counts);
    }

    if ((motor_control_state == MOTOR_CONTROL_STATE_ALIGNING) &&
        (((motor_control_mode == MOTOR_CONTROL_MODE_SPEED) &&
          !MotorSpeed_AlignIsActive()) ||
         ((motor_control_mode == MOTOR_CONTROL_MODE_POSITION) &&
          !Motor_AlignIsActive())))
    {
        motor_control_state = MOTOR_CONTROL_STATE_STOPPED;
    }
}

MotorControlState MotorControl_GetState(void)
{
    return motor_control_state;
}

uint8_t MotorControl_AlignIsDone(void)
{
    return (motor_control_mode == MOTOR_CONTROL_MODE_SPEED)
        ? MotorSpeed_AlignIsDone()
        : Motor_AlignIsDone();
}

uint16_t MotorControl_GetFeedbackAngle(void)
{
    return motor_control_feedback_angle;
}

int16_t MotorControl_GetFeedbackRpm(void)
{
    return (motor_control_mode == MOTOR_CONTROL_MODE_SPEED)
        ? MotorSpeed_GetFeedbackRpm()
        : MotorSpeed_CountsPerTickToRpm(Motor_GetFeedbackVelocity());
}

int16_t MotorControl_GetTargetRpm(void)
{
    int16_t target_rpm = 0;

    if (motor_control_mode == MOTOR_CONTROL_MODE_SPEED)
    {
        target_rpm = MotorSpeed_GetTargetRpm();
    }

    return target_rpm;
}