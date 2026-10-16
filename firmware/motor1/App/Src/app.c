#include "app.h"

#include <stdbool.h>

#include "adc.h"
#include "main.h"
#include "_BTN.h"
#include "_CAN.h"
#include "_LED.h"
#include "_MOTOR.h"
#include "_MOTOR_SPEED.h"
#include "_MT6816.h"
#include "_POWER_VOLT.h"
#include "_SIN_MAP.h"
#include "_TEMPERATURE.h"

volatile bool led_flag = false;

static volatile bool updateAngle_flag = false;
static volatile bool Button_Debounce1_flag = false;
static volatile bool Button_Debounce2_flag = false;
static volatile bool motor_flag = false;
static volatile bool emergency_stop_flag = false;
static volatile bool sensor_update_flag = false;
static volatile bool heartbeat_flag = false;
static volatile bool vcu_fault_flag = false;
static volatile bool vcu_heartbeat_timeout_flag = false;

static bool motor_run_after_align_pending = false;
static bool motor_stopping = false;
static uint32_t motor_stop_started_ms = 0U;
static uint16_t load_stall_counter_ms = 0U;
static uint16_t reverse_rotation_counter_ms = 0U;
static uint16_t critical_stall_counter_ms = 0U;
static uint16_t load_stall_run_time_ms = 0U;
static bool can_debug_led_state = false;
static uint32_t can_debug_led_last_toggle_ms = 0U;
static uint32_t can_debug_led_last_activity_ms = 0U;

typedef enum
{
	APP_DEBUG_STARTUP_DISABLED = 0,
	APP_DEBUG_STARTUP_START_STEP,
	APP_DEBUG_STARTUP_WAIT_ALIGN,
	APP_DEBUG_STARTUP_HOLD_STEP,
	APP_DEBUG_STARTUP_DONE
} AppDebugStartupState;

typedef enum
{
	APP_DEBUG_DIRECTION_RIGHT = 0,
	APP_DEBUG_DIRECTION_LEFT
} AppDebugDirection;

#define APP_DEBUG_STARTUP_STEP_COUNT    5U
#define APP_STOP_RPM_THRESHOLD      5
#define APP_STOP_OBSERVE_TIME_MS    30U
#define APP_LOAD_STALL_TRIP_MS      300U
#define APP_REVERSE_TRIP_MS          50U
#define APP_CRITICAL_STALL_TRIP_MS   20U
#define APP_LOAD_STALL_GRACE_MS    1000U
/* Temporary floor-test override: keep CAN/E-stop protection active, but do
 * not latch a motor ECU fault from load stall or reverse-rotation detection. */
#define APP_LOAD_MOTION_FAULT_ENABLE 0U

typedef struct
{
	uint16_t rpm;
	uint32_t duration_ms;
	AppDebugDirection direction;
} AppDebugStartupStep;

static const AppDebugStartupStep app_debug_startup_steps[APP_DEBUG_STARTUP_STEP_COUNT] =
{
	{ 100U, 10000U, APP_DEBUG_DIRECTION_RIGHT },
	{ 150U, 10000U, APP_DEBUG_DIRECTION_RIGHT },
	{ 200U, 10000U, APP_DEBUG_DIRECTION_RIGHT },
	{ 100U,  5000U, APP_DEBUG_DIRECTION_RIGHT },
	{ 200U,  5000U, APP_DEBUG_DIRECTION_RIGHT }
};

static AppDebugStartupState debug_startup_state = APP_DEBUG_STARTUP_DISABLED;
static uint8_t debug_startup_step_index = 0U;
static uint32_t debug_startup_step_started_ms = 0U;

static void App_HandleCanEvent(const CAN_Event *event);
static void App_ApplyMotorCommandTarget(const CAN_Event *event, bool allow_zero_rpm);
static void App_MarkCanActivity(void);
static void App_UpdateDebugLeds(void);
static void App_UpdateDebugStartupSequence(void);
static void App_CancelDebugStartupSequence(void);
static int32_t App_GetDebugStartupTargetRpm(const AppDebugStartupStep *step);
static bool App_IsCanRunAllowed(void);
static CAN_ECUState App_GetHeartbeatState(void);
static CAN_MotorState App_GetMotorStatusState(void);
static void App_StartControlledStop(void);
static void App_UpdateStoppingState(void);
static void App_CheckLoadStall(void);

void App_Init(void)
{
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
	HAL_TIM_Base_Start_IT(&htim2);

	MotorControl_Stop();
	MotorControl_SetMode(MOTOR_CONTROL_MODE_SPEED);

	CAN_Config();

	motor_flag = false;
	buttons[0].flag = false;
	buttons[1].flag = false;

	HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_Base_Start_IT(&htim4);

	if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_ADCEx_Calibration_Start(&hadc2) != HAL_OK)
	{
		Error_Handler();
	}

	Temperature_Update();
	PowerVolt_Update();

	if (MOTOR_TUNING_MODE != 0U)
	{
		debug_startup_step_index = 0U;
		debug_startup_step_started_ms = 0U;
		debug_startup_state = APP_DEBUG_STARTUP_START_STEP;
	}
}

void App_Run(void)
{
	static uint16_t angle_temp = 0x00;
	CAN_Event can_event;

	while (CAN_GetNextEvent(&can_event))
	{
		App_MarkCanActivity();
		App_HandleCanEvent(&can_event);
	}

	CAN_ProcessPendingTx();

	if (CAN_IsVcuHeartbeatTimedOut(HAL_GetTick()))
	{
		if (!vcu_heartbeat_timeout_flag)
		{
			vcu_heartbeat_timeout_flag = true;
			if (MOTOR_TUNING_MODE == 0U)
			{
				MotorControl_Stop();
				led_flag = true;
			}
		}
	}
	else
	{
		vcu_heartbeat_timeout_flag = false;
	}

	App_UpdateDebugStartupSequence();
	App_UpdateDebugLeds();

	if (Button_Debounce1_flag)
	{
		Button_Debounce1_flag = false;
		Button_DebounceTask(&buttons[0]);
	}
	if (Button_Debounce2_flag)
	{
		Button_Debounce2_flag = false;
		Button_DebounceTask(&buttons[1]);
	}
	if (buttons[0].flag)
	{
		buttons[0].flag = false;
		emergency_stop_flag = true;
		led_flag = true;
		App_CancelDebugStartupSequence();
		MotorControl_Stop();
		CAN_SendEmergencyStopRequest();
	}
	else if (buttons[1].flag)
	{
		buttons[1].flag = false;
		App_CancelDebugStartupSequence();
		MotorControl_Stop();
		NVIC_SystemReset();
	}
	if (motor_flag)
	{
		motor_flag = false;

		angle_temp = (UpdateAngle() >> 2) & MOTOR_ANGLE_MASK;

		if (App_IsCanRunAllowed())
		{
			MotorControl_ControlStep(angle_temp);
			App_UpdateStoppingState();
			App_CheckLoadStall();
			
			if (motor_run_after_align_pending &&
				(MotorControl_GetState() == MOTOR_CONTROL_STATE_STOPPED) &&
				MotorControl_AlignIsDone())
			{
				motor_run_after_align_pending = false;
				MotorControl_Start();
			}
		}
	}
	if (sensor_update_flag)
	{
		sensor_update_flag = false;

		Temperature_Update();
		PowerVolt_Update();
	}
	if (updateAngle_flag)
	{
		CAN_MotorStatus status = {0};

		updateAngle_flag = false;

		status.temperature_c = Temperature_GetCelsius();
		status.rpm = MotorControl_GetFeedbackRpm();
		status.power_10mv = PowerVolt_Get10mV();
		status.state = App_GetMotorStatusState();
		status.target_rpm = MotorControl_GetTargetRpm();

		CAN_SendMotorStatus(&status);
		App_MarkCanActivity();
	}
	if (heartbeat_flag)
	{
		heartbeat_flag = false;
		CAN_SendHeartbeat(App_GetHeartbeatState());
		App_MarkCanActivity();
	}
}

void App_OnTimerElapsed(TIM_HandleTypeDef *htim)
{
	static uint8_t TIM1_1ms = 0;
	static uint8_t TIM4_10ms = 0;
	static uint8_t heartbeat_10ms_count = 0U;

	if (htim->Instance == TIM1)
	{
		TIM1_1ms++;
		if (TIM1_1ms == 5)
		{
			Button_Debounce1_flag = true;
		}
		else if (TIM1_1ms == 6)
		{
			Button_Debounce2_flag = true;
			TIM1_1ms = 0;
		}
	}
	if (htim->Instance == TIM3)
	{
		motor_flag = true;
	}
	if (htim->Instance == TIM4)
	{
		updateAngle_flag = true;
		TIM4_10ms++;
		if (TIM4_10ms >= 20U)
		{
			TIM4_10ms = 0;
			sensor_update_flag = true;
		}
		heartbeat_10ms_count++;
		if (heartbeat_10ms_count >= 10U)
		{
			heartbeat_10ms_count = 0U;
			heartbeat_flag = true;
		}
	}
}

static void App_HandleCanEvent(const CAN_Event *event)
{
	if (event->type != CAN_EVENT_NONE)
	{
		App_CancelDebugStartupSequence();
	}

	switch (event->type)
	{
	case CAN_EVENT_EMERGENCY_STOP:
		emergency_stop_flag = true;
		led_flag = true;
		motor_run_after_align_pending = false;
		MotorControl_Stop();
		break;

	case CAN_EVENT_VCU_FAULT:
		vcu_fault_flag = true;
		led_flag = true;
		motor_run_after_align_pending = false;
		MotorControl_Stop();
		break;

	case CAN_EVENT_GLOBAL_SYSTEM_RESET:
		motor_run_after_align_pending = false;
		MotorControl_Stop();
		NVIC_SystemReset();
		break;

	case CAN_EVENT_GLOBAL_START:
		if (App_IsCanRunAllowed() && !motor_stopping)
		{
			if (MotorControl_GetState() != MOTOR_CONTROL_STATE_RUNNING)
			{
				MotorControl_SetMode(MOTOR_CONTROL_MODE_SPEED);
				if (MotorControl_AlignIsDone())
				{
					motor_run_after_align_pending = false;
					MotorControl_Start();
				}
				else
				{
					motor_run_after_align_pending = true;
					MotorControl_AlignStart();
				}
			}
		}
		break;

	case CAN_EVENT_GLOBAL_STOP:
	case CAN_EVENT_GLOBAL_DISABLE:
	case CAN_EVENT_GLOBAL_SAFE_STATE:
		motor_run_after_align_pending = false;
		App_StartControlledStop();
		break;

	case CAN_EVENT_GLOBAL_ENABLE:
		break;

	case CAN_EVENT_GLOBAL_FAULT_RESET:
		emergency_stop_flag = false;
		vcu_fault_flag = false;
		led_flag = false;
		break;

	case CAN_EVENT_MOTOR_STOP:
		motor_run_after_align_pending = false;
		App_StartControlledStop();
		break;

	case CAN_EVENT_MOTOR_RUN:
		if (App_IsCanRunAllowed() && !motor_stopping)
		{
			App_ApplyMotorCommandTarget(event, false);
			MotorControl_SetMode(MOTOR_CONTROL_MODE_SPEED);
			if (MotorControl_GetState() != MOTOR_CONTROL_STATE_RUNNING)
			{
				/* Preserve the absolute encoder electrical offset across STOP.
				   Re-aligning every RUN can pull toward the opposite pole. */
				if (MotorControl_AlignIsDone())
				{
					motor_run_after_align_pending = false;
					MotorControl_Start();
				}
				else
				{
					motor_run_after_align_pending = true;
					MotorControl_AlignStart();
				}
			}
		}
		break;

	case CAN_EVENT_MOTOR_ALIGN:
		if (App_IsCanRunAllowed() && !motor_stopping)
		{
			motor_run_after_align_pending = false;
			App_ApplyMotorCommandTarget(event, false);
			MotorControl_SetMode(MOTOR_CONTROL_MODE_SPEED);
			MotorControl_AlignStart();
		}
		break;

	case CAN_EVENT_CLEAR_ERROR:
		emergency_stop_flag = false;
		vcu_fault_flag = false;
		led_flag = false;
		break;

	case CAN_EVENT_SET_PARAM:
		if (!motor_stopping)
    	{
        	App_ApplyMotorCommandTarget(event, true);
    	}
    	break;

	case CAN_EVENT_SET_RAMP_INTERVAL:
#if MOTOR_REMOTE_TUNING_ENABLE
		{
			bool accepted = (!motor_flag && !motor_stopping) &&
				MotorSpeed_SetRampInterval(event->direction, event->target_rpm);
			CAN_SendRampTuningAck(event->direction, event->target_rpm, accepted);
		}
#endif
		break;

	case CAN_EVENT_NONE:
	default:
		break;
	}
}

static void App_ApplyMotorCommandTarget(const CAN_Event *event, bool allow_zero_rpm)
{
	int32_t signed_target_rpm;

	if (event == NULL)
	{
		return;
	}
	if ((!allow_zero_rpm) && (event->target_rpm == 0U))
	{
		return;
	}

	signed_target_rpm = (event->direction == CAN_MOTOR_DIR_REVERSE)
		? -(int32_t)event->target_rpm
		:  (int32_t)event->target_rpm;
	MotorControl_SetTargetRpm(signed_target_rpm);
}

static void App_MarkCanActivity(void)
{
	uint32_t now = HAL_GetTick();

	can_debug_led_last_activity_ms = now;
	if ((now - can_debug_led_last_toggle_ms) >= 50U)
	{
		can_debug_led_last_toggle_ms = now;
		can_debug_led_state = !can_debug_led_state;
	}
}

static void App_UpdateDebugLeds(void)
{
	uint32_t now = HAL_GetTick();

	if (led_flag || emergency_stop_flag || vcu_fault_flag ||
		((MOTOR_TUNING_MODE == 0U) && vcu_heartbeat_timeout_flag))
	{
		LED1_On();
	}
	else
	{
		LED1_Off();
	}

	if ((now - can_debug_led_last_activity_ms) > 250U)
	{
		can_debug_led_state = false;
	}

	if (can_debug_led_state)
	{
		LED2_On();
	}
	else
	{
		LED2_Off();
	}
}

static void App_UpdateDebugStartupSequence(void)
{
	uint32_t now;

	if ((MOTOR_TUNING_MODE == 0U) ||
		(debug_startup_state == APP_DEBUG_STARTUP_DISABLED) ||
		(debug_startup_state == APP_DEBUG_STARTUP_DONE))
	{
		return;
	}

	if (!App_IsCanRunAllowed())
	{
		App_CancelDebugStartupSequence();
		MotorControl_Stop();
		return;
	}

	now = HAL_GetTick();

	switch (debug_startup_state)
	{
	case APP_DEBUG_STARTUP_START_STEP:
		if (debug_startup_step_index >= APP_DEBUG_STARTUP_STEP_COUNT)
		{
			MotorControl_Stop();
			debug_startup_state = APP_DEBUG_STARTUP_DONE;
			return;
		}

		MotorControl_SetMode(MOTOR_CONTROL_MODE_SPEED);
		MotorControl_SetTargetRpm(
			App_GetDebugStartupTargetRpm(&app_debug_startup_steps[debug_startup_step_index]));

		if (MotorControl_GetState() == MOTOR_CONTROL_STATE_RUNNING)
		{
			debug_startup_step_started_ms = now;
			debug_startup_state = APP_DEBUG_STARTUP_HOLD_STEP;
		}
		else if (MotorControl_AlignIsDone())
		{
			motor_run_after_align_pending = false;
			MotorControl_Start();
			debug_startup_step_started_ms = now;
			debug_startup_state = APP_DEBUG_STARTUP_HOLD_STEP;
		}
		else
		{
			motor_run_after_align_pending = true;
			MotorControl_AlignStart();
			debug_startup_state = APP_DEBUG_STARTUP_WAIT_ALIGN;
		}
		break;

	case APP_DEBUG_STARTUP_WAIT_ALIGN:
		if (MotorControl_GetState() == MOTOR_CONTROL_STATE_RUNNING)
		{
			debug_startup_step_started_ms = now;
			debug_startup_state = APP_DEBUG_STARTUP_HOLD_STEP;
		}
		else if ((MotorControl_GetState() == MOTOR_CONTROL_STATE_STOPPED) &&
				 MotorControl_AlignIsDone())
		{
			motor_run_after_align_pending = false;
			MotorControl_Start();
			debug_startup_step_started_ms = now;
			debug_startup_state = APP_DEBUG_STARTUP_HOLD_STEP;
		}
		break;

	case APP_DEBUG_STARTUP_HOLD_STEP:
		if ((now - debug_startup_step_started_ms) >=
			app_debug_startup_steps[debug_startup_step_index].duration_ms)
		{
			debug_startup_step_index++;
			if (debug_startup_step_index >= APP_DEBUG_STARTUP_STEP_COUNT)
			{
				motor_run_after_align_pending = false;
				MotorControl_Stop();
				debug_startup_state = APP_DEBUG_STARTUP_DONE;
			}
			else
			{
				debug_startup_state = APP_DEBUG_STARTUP_START_STEP;
			}
		}
		break;

	case APP_DEBUG_STARTUP_DISABLED:
	case APP_DEBUG_STARTUP_DONE:
	default:
		break;
	}
}

static void App_CancelDebugStartupSequence(void)
{
	debug_startup_state = APP_DEBUG_STARTUP_DISABLED;
	debug_startup_step_index = 0U;
	debug_startup_step_started_ms = 0U;
	motor_run_after_align_pending = false;
}

static int32_t App_GetDebugStartupTargetRpm(const AppDebugStartupStep *step)
{
	if (step == NULL)
	{
		return 0;
	}

	(void)step->direction;

	return (int32_t)step->rpm;
}

static bool App_IsCanRunAllowed(void)
{
	return (!emergency_stop_flag &&
			!vcu_fault_flag &&
			((MOTOR_TUNING_MODE != 0U) || !vcu_heartbeat_timeout_flag));
}

static CAN_ECUState App_GetHeartbeatState(void)
{
	if (emergency_stop_flag || vcu_fault_flag)
	{
		return CAN_ECU_STATE_FAULT;
	}

	if (MotorControl_GetState() == MOTOR_CONTROL_STATE_STOPPED)
	{
		return CAN_ECU_STATE_READY;
	}

	return CAN_ECU_STATE_RUN;
}

static CAN_MotorState App_GetMotorStatusState(void)
{
    CAN_MotorState state;
    MotorControlState control_state;

    if (emergency_stop_flag ||
        vcu_fault_flag ||
        ((MOTOR_TUNING_MODE == 0U) && vcu_heartbeat_timeout_flag))
    {
        state = CAN_MOTOR_STATE_FAULT;
    }
    else if (motor_stopping)
    {
        state = CAN_MOTOR_STATE_STOPPING;
    }
    else
    {
        control_state = MotorControl_GetState();

        switch (control_state)
        {
        case MOTOR_CONTROL_STATE_ALIGNING:
            state = CAN_MOTOR_STATE_ALIGNING;
            break;

        case MOTOR_CONTROL_STATE_RUNNING:
            state = CAN_MOTOR_STATE_RUNNING;
            break;

        case MOTOR_CONTROL_STATE_STOPPED:
            if (MotorControl_AlignIsDone() != 0U)
            {
                state = CAN_MOTOR_STATE_ALIGNED;
            }
            else
            {
                state = CAN_MOTOR_STATE_READY;
            }
            break;

        default:
            state = CAN_MOTOR_STATE_FAULT;
            break;
        }
    }

    return state;
}

static void App_StartControlledStop(void)
{
    motor_stopping = true;
    motor_stop_started_ms = HAL_GetTick();

    MotorControl_Stop();
}

static void App_UpdateStoppingState(void)
{
    int32_t rpm;

    if (!motor_stopping)
    {
        return;
    }

    if ((HAL_GetTick() - motor_stop_started_ms) <
        APP_STOP_OBSERVE_TIME_MS)
    {
        return;
    }

    rpm = (int32_t)MotorControl_GetFeedbackRpm();

    if (rpm < 0)
    {
        rpm = -rpm;
    }

    if (rpm <= APP_STOP_RPM_THRESHOLD)
    {
        motor_stopping = false;
    }
}

static void App_CheckLoadStall(void)
{
    int32_t target;
    int32_t feedback;
    bool too_slow;
    bool critical_stall;
    bool reverse_rotation;

	if (APP_LOAD_MOTION_FAULT_ENABLE == 0U)
	{
		load_stall_counter_ms = 0U;
		reverse_rotation_counter_ms = 0U;
		critical_stall_counter_ms = 0U;
		load_stall_run_time_ms = 0U;
		return;
	}

    if (MotorControl_GetState() != MOTOR_CONTROL_STATE_RUNNING)
    {
        load_stall_counter_ms = 0U;
        reverse_rotation_counter_ms = 0U;
        critical_stall_counter_ms = 0U;
        load_stall_run_time_ms = 0U;
        return;
    }

    if (load_stall_run_time_ms < APP_LOAD_STALL_GRACE_MS)
    {
        load_stall_run_time_ms++;
        load_stall_counter_ms = 0U;
        reverse_rotation_counter_ms = 0U;
        critical_stall_counter_ms = 0U;
        return;
    }

    target = MotorControl_GetTargetRpm();
    feedback = MotorControl_GetFeedbackRpm();
    too_slow = ((target >= MOTOR_SPEED_STARTUP_MIN_RPM) &&
                (feedback < ((target * 4) / 5))) ||
               ((target <= -MOTOR_SPEED_STARTUP_MIN_RPM) &&
                (feedback > ((target * 4) / 5)));
    reverse_rotation = ((target >= MOTOR_SPEED_STARTUP_MIN_RPM) && (feedback < 0)) ||
                       ((target <= -MOTOR_SPEED_STARTUP_MIN_RPM) && (feedback > 0));
    critical_stall = ((target >= MOTOR_SPEED_STARTUP_MIN_RPM) &&
                      (feedback >= 0) && (feedback < (target / 2))) ||
                     ((target <= -MOTOR_SPEED_STARTUP_MIN_RPM) &&
                      (feedback <= 0) && (feedback > (target / 2)));

    if (critical_stall)
    {
        if (critical_stall_counter_ms < APP_CRITICAL_STALL_TRIP_MS)
        {
            critical_stall_counter_ms++;
        }
    }
    else
    {
        critical_stall_counter_ms = 0U;
    }

    if (reverse_rotation)
    {
        if (reverse_rotation_counter_ms < APP_REVERSE_TRIP_MS)
        {
            reverse_rotation_counter_ms++;
        }
    }
    else
    {
        reverse_rotation_counter_ms = 0U;
    }

    if (!too_slow)
    {
        load_stall_counter_ms = 0U;
        return;
    }

    if (load_stall_counter_ms < APP_LOAD_STALL_TRIP_MS)
    {
        load_stall_counter_ms++;
    }
    if ((load_stall_counter_ms >= APP_LOAD_STALL_TRIP_MS) ||
        (critical_stall_counter_ms >= APP_CRITICAL_STALL_TRIP_MS) ||
        (reverse_rotation_counter_ms >= APP_REVERSE_TRIP_MS))
    {
        load_stall_counter_ms = 0U;
        reverse_rotation_counter_ms = 0U;
        critical_stall_counter_ms = 0U;
        emergency_stop_flag = true;
        led_flag = true;
        motor_run_after_align_pending = false;
        MotorControl_Stop();
        CAN_SendFault();
    }
}
