#ifndef INC__MOTOR_SPEED_H_
#define INC__MOTOR_SPEED_H_

#include "stm32f1xx_hal.h"
#include <stdbool.h>

/*
 * Speed-control motor module
 *
 * This project calls the motor control function every 1 ms.
 * Therefore, speed is expressed as encoder counts per 1 ms.
 *
 * Examples (encoder resolution = 16384 counts/revolution):
 *   1 count/ms   = about   3.66 RPM
 *  10 counts/ms  = about  36.62 RPM
 * -10 counts/ms  = about -36.62 RPM
 */
#define MOTOR_SPEED_FULL_TURN_COUNTS              16384U
#define MOTOR_SPEED_ANGLE_MASK                    (MOTOR_SPEED_FULL_TURN_COUNTS - 1U)
#define MOTOR_SPEED_POLE_PAIRS                    50U
/* Set to -1 when the physical positive rotation direction must be reversed
   from the encoder's raw increasing-count direction. */
#define MOTOR_SPEED_FEEDBACK_DIRECTION            (-1)
#define MOTOR_SPEED_ENCODER_DIRECTION_SIGN        (-1)

#define MOTOR_SPEED_MAX_TARGET_RPM                200
/* Motor 2 initial 100--200 RPM tuning profile.
   The controller keeps the proven voltage-vector rotation path and uses
   encoder feedback to detect loss of synchronism. When the measured speed
   collapses or reverses under load, it re-synchronizes the command angle to
   the current rotor angle and applies a short PWM boost so the motor can
   recover without being spun by hand. */
#define MOTOR_SPEED_MAX_PWM_AMPLITUDE             3599
#define MOTOR_SPEED_MID_PWM_AMPLITUDE             3500
#define MOTOR_SPEED_MIN_PWM_AMPLITUDE             1200
#define MOTOR_SPEED_LOW_PWM_PER_RPM               10
#define MOTOR_SPEED_LOW_PWM_END_RPM               201
#define MOTOR_SPEED_HIGH_BOOST_START_RPM          280
#define MOTOR_SPEED_HIGH_BOOST_PWM_PER_RPM        6
#define MOTOR_SPEED_RAMP_RPM_PER_STEP             1
#define MOTOR_SPEED_LOW_RAMP_END_RPM              101
/* Motor 1 validated startup profile. */
#define MOTOR_SPEED_STARTUP_FAST_END_RPM          40
#define MOTOR_SPEED_STARTUP_MIN_RPM               100
#define MOTOR_SPEED_LOW_RAMP_INTERVAL_TICKS       10U
#define MOTOR_SPEED_STARTUP_RAMP_INTERVAL_TICKS   20U
#define MOTOR_SPEED_HIGH_RAMP_INTERVAL_TICKS      6U
#define MOTOR_SPEED_STALL_DETECT_START_RPM        70
#define MOTOR_SPEED_STALL_DETECT_TICKS            20U
#define MOTOR_SPEED_RECOVERY_BOOST_TICKS          300U
#define MOTOR_SPEED_RECOVERY_PWM_AMPLITUDE         3000
#define MOTOR_SPEED_PWM_SLEW_PER_MS                20
#define MOTOR_SPEED_COMMAND_Q_SHIFT               16U
#define MOTOR_SPEED_PWM_UPDATE_HZ                 20000U
#define MOTOR_SPEED_FAST_UPDATE_DIVIDER           2U
#define MOTOR_SPEED_VECTOR_UPDATE_HZ              \
    (MOTOR_SPEED_PWM_UPDATE_HZ / MOTOR_SPEED_FAST_UPDATE_DIVIDER)
#define MOTOR_SPEED_VELOCITY_WINDOW_TICKS         10U
#define MOTOR_SPEED_VELOCITY_Q_SHIFT              8U
#define MOTOR_SPEED_FILTER_DIVISOR                4
#define MOTOR_SPEED_ZERO_DEADBAND_COUNTS_PER_TICK 1
#define MOTOR_SPEED_KP_PWM_PER_RPM                0
#define MOTOR_SPEED_KI_DIVISOR                    100
#define MOTOR_SPEED_INTEGRAL_LIMIT_PWM            0
#define MOTOR_SPEED_ENCODER_SYNC_ENABLE           0U
#define MOTOR_SPEED_SYNC_BASE_LEAD_COUNTS         \
    (MOTOR_SPEED_FULL_TURN_COUNTS / 128U)
#define MOTOR_SPEED_SYNC_LEAD_PER_RPM             8
#define MOTOR_SPEED_SYNC_MAX_LEAD_COUNTS          \
    (MOTOR_SPEED_FULL_TURN_COUNTS / 12U)

#define MOTOR_SPEED_ALIGN_PWM_AMPLITUDE           1800
#define MOTOR_SPEED_ALIGN_HOLD_TIME_MS            700U
/* Edit g_motor_speed_tuning in STM32CubeIDE Live Expressions. */
typedef struct
{
    volatile int32_t max_target_rpm;
    volatile int32_t max_pwm_amplitude;
    volatile int32_t mid_pwm_amplitude;
    volatile int32_t min_pwm_amplitude;
    volatile int32_t low_pwm_per_rpm;
    volatile int32_t low_pwm_end_rpm;
    volatile int32_t high_boost_start_rpm;
    volatile int32_t high_boost_pwm_per_rpm;
    volatile int32_t ramp_rpm_per_step;
    volatile int32_t low_ramp_end_rpm;
    volatile int32_t low_ramp_interval_ticks;
    volatile int32_t startup_ramp_interval_ticks;
    volatile int32_t high_ramp_interval_ticks;
    volatile int32_t zero_deadband_counts_per_tick;
    volatile int32_t align_pwm_amplitude;
    volatile int32_t align_hold_time_ms;
    volatile int32_t kp_pwm_per_rpm;
    volatile int32_t ki_divisor;
    volatile int32_t integral_limit_pwm;
} MotorSpeedTuning;

extern volatile MotorSpeedTuning g_motor_speed_tuning;

/* Set the desired speed. Positive and negative values reverse direction. */
void MotorSpeed_SetTarget(int16_t target_counts_per_tick);

/* CAN-facing helpers. RPM is mechanical shaft speed. */
void MotorSpeed_SetTargetRpm(int32_t target_rpm);
bool MotorSpeed_SetRampInterval(uint8_t band, uint16_t interval_ticks);
int16_t MotorSpeed_CountsPerTickToRpm(int16_t counts_per_tick);
int16_t MotorSpeed_GetFeedbackRpm(void);
int16_t MotorSpeed_GetTargetRpm(void);

/* Start after alignment has completed successfully. */
void MotorSpeed_Start(void);

/* Call once every 1 ms with the latest 14-bit encoder angle. */
void MotorSpeed_ControlStep(uint16_t measured_angle_counts);

/* Call from the TIM2 PWM update interrupt to interpolate the voltage vector. */
void MotorSpeed_FastControlStep(void);

/* Run once after power-up. Keep calling ControlStep() during alignment. */
void MotorSpeed_AlignStart(void);
uint8_t MotorSpeed_AlignIsDone(void);
uint8_t MotorSpeed_AlignIsActive(void);

int16_t MotorSpeed_GetTarget(void);
int16_t MotorSpeed_GetFeedback(void);
uint8_t MotorSpeed_IsActive(void);
void MotorSpeed_Stop(void);

#endif /* INC__MOTOR_SPEED_H_ */
