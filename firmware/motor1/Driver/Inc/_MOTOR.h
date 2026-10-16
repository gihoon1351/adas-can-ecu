#ifndef INC__MOTOR_H_
#define INC__MOTOR_H_

#include "stm32f1xx_hal.h"
#include "tim.h"
#include "gpio.h"
#include "_SIN_MAP.h"

/* All angle values use encoder/LUT counts: 0..16383 represents one turn. */
#define MOTOR_FULL_TURN_COUNTS                      16384U
#define MOTOR_HALF_TURN_COUNTS                      (MOTOR_FULL_TURN_COUNTS / 2U)
#define MOTOR_QUARTER_TURN_COUNTS                   (MOTOR_FULL_TURN_COUNTS / 4U)
#define MOTOR_ANGLE_MASK                            (MOTOR_FULL_TURN_COUNTS - 1U)
#define MOTOR_POLE_PAIRS                            50U

#define MOTOR_ELECTRICAL_ZERO_OFFSET_COUNTS         0
#define MOTOR_ENCODER_TO_ELECTRICAL_DIRECTION       1

#define MOTOR_POSITION_KP_NUMERATOR                 1
#define MOTOR_POSITION_KP_DENOMINATOR               4
#define MOTOR_POSITION_KD                           2
#define MOTOR_POSITION_MAX_PWM_AMPLITUDE            720
#define MOTOR_POSITION_DEADBAND_COUNTS              8 // 1 count = 360 / 16384 ≈ 0.02197 => 0.02197 * 8 = 0.17576
#define MOTOR_VELOCITY_DEADBAND_COUNTS_PER_TICK     2
#define MOTOR_VELOCITY_FILTER_DIVISOR               4

#define MOTOR_ALIGN_PWM_AMPLITUDE                    720
#define MOTOR_ALIGN_STAGE_TIME_MS                    500U
#define MOTOR_ALIGN_MIN_MOVEMENT_COUNTS              32

/* Edit g_motor_position_tuning in STM32CubeIDE Live Expressions.
 * Values are read on every 1 ms control step. PWM values are clamped to the
 * TIM2 ARR range and zero divisors are replaced with 1 at the point of use. */
typedef struct
{
    volatile int32_t kp_numerator;
    volatile int32_t kp_denominator;
    volatile int32_t kd;
    volatile int32_t max_pwm_amplitude;
    volatile int32_t position_deadband_counts;
    volatile int32_t velocity_deadband_counts_per_tick;
    volatile int32_t velocity_filter_divisor;
    volatile int32_t align_pwm_amplitude;
    volatile int32_t align_stage_time_ms;
    volatile int32_t align_min_movement_counts;
} MotorPositionTuning;

extern volatile MotorPositionTuning g_motor_position_tuning;

/* Compatibility alias used by CAN angle conversion. */
#define MOTOR_ANGLE_COUNTS                          MOTOR_FULL_TURN_COUNTS

typedef enum
{
    MOTOR_CONTROL_MODE_SPEED = 0,
    MOTOR_CONTROL_MODE_POSITION = 1
} MotorControlMode;

typedef enum
{
    MOTOR_CONTROL_STATE_STOPPED = 0,
    MOTOR_CONTROL_STATE_RUNNING = 1,
    MOTOR_CONTROL_STATE_ALIGNING = 2
} MotorControlState;

/* Unified motor-control interface used by the application layer. */
void MotorControl_SetMode(MotorControlMode mode);
MotorControlMode MotorControl_GetMode(void);
void MotorControl_SetTargetRpm(int32_t target_rpm);
void MotorControl_SetTargetDegree(uint16_t target_degree);
void MotorControl_Start(void);
void MotorControl_AlignStart(void);
void MotorControl_Stop(void);
void MotorControl_ControlStep(uint16_t measured_angle_counts);
MotorControlState MotorControl_GetState(void);
uint8_t MotorControl_AlignIsDone(void);
uint16_t MotorControl_GetFeedbackAngle(void);
int16_t MotorControl_GetFeedbackRpm(void);
int16_t MotorControl_GetTargetRpm(void);

void Motor_OpenLoopStep(int16_t electrical_step_counts, uint16_t pwm_amplitude);

void Motor_PositionSetTarget(uint16_t target_mechanical_angle_counts);
void Motor_PositionStart(void);
void Motor_PositionControlStep(uint16_t measured_mechanical_angle_counts);
uint8_t Motor_PositionIsActive(void);

void Motor_AlignStart(void);
uint8_t Motor_AlignIsDone(void);
uint8_t Motor_AlignIsActive(void);

uint16_t Motor_GetFeedbackAngle(void);
int16_t Motor_GetFeedbackVelocity(void);

void Motor_Stop(void);

#endif /* INC__MOTOR_H_ */
