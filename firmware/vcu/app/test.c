#include "test.h"

/* Debug-only UART and synthetic-ADAS scenarios are disabled in the
 * production control build. Set to 1 only for a dedicated bench build. */
#define ENABLE_DEBUG_TEST_CODE (0U)

#if (ENABLE_DEBUG_TEST_CODE != 0U)

#include <stddef.h>

#include "xil_printf.h"
#include "../common/delay/delay.h"
#include "../driver/motor/motor.h"

#define UART_TEST_RPM (150U)
#define UART_ALIGN_WAIT_MS (1200U)
#define SCENARIO_STEP_MS (10000U)
#define SIM_OBJECT_Y0 (20U)
#define SIM_SIGN_Y1 (75U)
#define SIM_TRAFFIC_Y1 (70U)
#define SIM_BUMP_NEAR_Y1 (190U)

typedef enum
{
	UART_SEQUENCE_IDLE = 0,
	UART_SEQUENCE_WAIT_ALIGN
} UartSequenceState_t;

typedef enum
{
	ADAS_SCENARIO_IDLE = 0,
	ADAS_SCENARIO_WAIT_ALIGN,
	ADAS_SCENARIO_SEND_GREEN,
	ADAS_SCENARIO_HOLD_GREEN,
	ADAS_SCENARIO_HOLD_70,
	ADAS_SCENARIO_HOLD_BUMP,
	ADAS_SCENARIO_HOLD_CLEAR,
	ADAS_SCENARIO_HOLD_30
} AdasScenarioState_t;

static UartSequenceState_t uart_state = UART_SEQUENCE_IDLE;
static uint32_t uart_due_tick;
static bool motor_alignment_ready;
static AdasScenarioState_t adas_state = ADAS_SCENARIO_IDLE;
static uint32_t adas_due_tick;

static bool InjectAdas(ObjectType_t object_type, uint8_t y0, uint8_t y1)
{
	CAN_Frame_t frame = {0};

	frame.id = CAN_ID_ADAS1_RESULT;
	frame.dlc = 5U;
	frame.data[0] = (uint8_t)(LANE_CENTER_X & 0xFFU);
	frame.data[1] = (uint8_t)((LANE_CENTER_X >> 8U) & 0xFFU);
	frame.data[2] = (uint8_t)object_type;
	frame.data[3] = y0;
	frame.data[4] = y1;

	if (CAN_InjectRxFrameForTest(&frame) == false) {
		return false;
	}
	xil_printf("[SIM ADAS] ID=0x%x CLASS=%d LANE_X=%d Y0=%d Y1=%d\r\n", (unsigned int)frame.id, (int)object_type, (int)LANE_CENTER_X, (int)y0, (int)y1);
	return true;
}

void TEST_CancelSequences(void)
{
	uart_state = UART_SEQUENCE_IDLE;
	adas_state = ADAS_SCENARIO_IDLE;
	motor_alignment_ready = false;
}

bool TEST_StartAdasScenario(MotorTargetrpm_t *target_rpm, bool *motors_started, Heartbeat_t *heartbeat, bool emergency_stop_latched, bool manual_estop_latched)
{
	if ((target_rpm == NULL) || (motors_started == NULL) || (heartbeat == NULL)) {
		return false;
	}
	if ((emergency_stop_latched != false) || (manual_estop_latched != false)) {
		xil_printf("[SIM ADAS] START BLOCKED BY SAFETY\r\n");
		return false;
	}
	target_rpm->left_rpm = UART_TEST_RPM;
	target_rpm->right_rpm = UART_TEST_RPM;
	if ((MOTOR_RequestTargetRPM(target_rpm) == false) || (MOTOR_RequestAlign() == false)) {
		return false;
	}
	*motors_started = false;
	motor_alignment_ready = false;
	heartbeat->state = ECU_STATE_READY;
	adas_due_tick = millis() + UART_ALIGN_WAIT_MS;
	adas_state = ADAS_SCENARIO_WAIT_ALIGN;
	xil_printf("[SIM ADAS] AUTO START : SET_PARAM + ALIGN\r\n");
	xil_printf("[SIM ADAS] GREEN -> 70 -> BUMP -> CLEAR -> 30 -> RED\r\n");
	return true;
}

void TEST_ProcessAdasScenario(MotorTargetrpm_t *target_rpm, bool *motors_started, Heartbeat_t *heartbeat, bool emergency_stop_latched, bool manual_estop_latched)
{
	uint32_t now;
	bool injected = false;

	if ((target_rpm == NULL) || (motors_started == NULL) || (heartbeat == NULL) || (adas_state == ADAS_SCENARIO_IDLE)) {
		return;
	}
	if ((emergency_stop_latched != false) || (manual_estop_latched != false)) {
		adas_state = ADAS_SCENARIO_IDLE;
		xil_printf("[SIM ADAS] SCENARIO CANCELED BY SAFETY\r\n");
		return;
	}
	now = millis();
	if ((int32_t)(now - adas_due_tick) < 0) {
		return;
	}

	switch (adas_state) {
	case ADAS_SCENARIO_WAIT_ALIGN:
		if (MOTOR_RequestRun() != false) {
			*motors_started = true;
			motor_alignment_ready = true;
			heartbeat->state = ECU_STATE_RUN;
			adas_state = ADAS_SCENARIO_SEND_GREEN;
			adas_due_tick = now;
			xil_printf("[SIM ADAS] ALIGN COMPLETE / MOTOR RUN\r\n");
		}
		break;
	case ADAS_SCENARIO_SEND_GREEN:
		injected = InjectAdas(OBJECT_TRAFFIC_GREEN, SIM_OBJECT_Y0, SIM_TRAFFIC_Y1);
		if (injected) {
			adas_state = ADAS_SCENARIO_HOLD_GREEN;
		}
		break;
	case ADAS_SCENARIO_HOLD_GREEN:
		injected = InjectAdas(OBJECT_SIGN_70, SIM_OBJECT_Y0, SIM_SIGN_Y1);
		if (injected) {
			adas_state = ADAS_SCENARIO_HOLD_70;
		}
		break;
	case ADAS_SCENARIO_HOLD_70:
		injected = InjectAdas(OBJECT_SPEED_BUMP, 0U, SIM_BUMP_NEAR_Y1);
		if (injected) {
			adas_state = ADAS_SCENARIO_HOLD_BUMP;
		}
		break;
	case ADAS_SCENARIO_HOLD_BUMP:
		injected = InjectAdas(OBJECT_LANE, 0U, 0U);
		if (injected) {
			adas_state = ADAS_SCENARIO_HOLD_CLEAR;
		}
		break;
	case ADAS_SCENARIO_HOLD_CLEAR:
		injected = InjectAdas(OBJECT_SIGN_30, SIM_OBJECT_Y0, SIM_SIGN_Y1);
		if (injected) {
			adas_state = ADAS_SCENARIO_HOLD_30;
		}
		break;
	case ADAS_SCENARIO_HOLD_30:
		injected = InjectAdas(OBJECT_TRAFFIC_RED, SIM_OBJECT_Y0, SIM_TRAFFIC_Y1);
		if (injected) {
			adas_state = ADAS_SCENARIO_IDLE;
			xil_printf("[SIM ADAS] RED / SCENARIO COMPLETE\r\n");
		}
		break;
	default:
		adas_state = ADAS_SCENARIO_IDLE;
		break;
	}
	if ((injected != false) && (adas_state != ADAS_SCENARIO_IDLE)) {
		adas_due_tick = now + SCENARIO_STEP_MS;
	}
}

void TEST_ProcessAdasControl(MotorTargetrpm_t *target_rpm, bool *motors_started, Heartbeat_t *heartbeat, bool emergency_stop_latched)
{
	Event_t event;
	if ((target_rpm == NULL) || (motors_started == NULL) || (heartbeat == NULL))
		return;
	APP_UpdateADAS();
	while (GetNextEvent(&event) != false) {
		*target_rpm = APP_ProcessEvent(&event);
		xil_printf("[APP] EVENT=%d PD=%d TARGET L=%d R=%d\r\n", (int)event.type, (int)event.pd_output, (int)target_rpm->left_rpm, (int)target_rpm->right_rpm);
		if (emergency_stop_latched != false)
			continue;
		if (event.type == EVENT_MOTOR_STOP) {
			if (MOTOR_RequestStopBoth() != false) {
				*motors_started = false;
				heartbeat->state = ECU_STATE_READY;
				adas_state = ADAS_SCENARIO_IDLE;
			} else {
				xil_printf("[ERROR] ADAS MOTOR STOP PAIR QUEUE FAIL\r\n");
			}
			continue;
		} else if (*motors_started != false) {
			if (MOTOR_RequestTargetRPM(target_rpm) == false)
				xil_printf("[ERROR] ADAS MOTOR PARAM QUEUE FAIL\r\n");
		} else if ((event.type == EVENT_MOTOR_RUN) && (motor_alignment_ready != false)) {
			if ((MOTOR_RequestTargetRPM(target_rpm) != false) && (MOTOR_RequestRun() != false)) {
				*motors_started = true;
				heartbeat->state = ECU_STATE_RUN;
			} else
				xil_printf("[ERROR] GREEN RESTART QUEUE FAIL\r\n");
		}
	}
}

void TEST_PrintUartHelp(void)
{
	xil_printf("\r\n=== UART MOTOR TEST ===\r\n");
	xil_printf("g: RUN  s: STOP  e: E-STOP  c: CLEAR  i: INFO  h: HELP\r\n");
	xil_printf("a: ALIGN  G: GREEN  7: SIGN70  b: BUMP  3: SIGN30  r: RED  t: AUTO\r\n");
}

void TEST_ProcessUartCommand(uint8_t command, MotorTargetrpm_t *target_rpm, bool *motors_started, bool *manual_estop_latched, Heartbeat_t *heartbeat, bool emergency_stop_latched)
{
	if ((target_rpm == NULL) || (motors_started == NULL) || (manual_estop_latched == NULL) || (heartbeat == NULL))
		return;
	switch (command) {
	case 'g':
		adas_state = ADAS_SCENARIO_IDLE;
		if ((uart_state != UART_SEQUENCE_IDLE) || emergency_stop_latched || *manual_estop_latched) {
			xil_printf("[UART] RUN BLOCKED\r\n");
			break;
		}
		target_rpm->left_rpm = UART_TEST_RPM;
		target_rpm->right_rpm = UART_TEST_RPM;
		if ((MOTOR_RequestTargetRPM(target_rpm) == false) || (MOTOR_RequestAlign() == false)) {
			xil_printf("[ERROR] MOTOR SETUP QUEUE FAIL\r\n");
			break;
		}
		motor_alignment_ready = false;
		uart_due_tick = millis() + UART_ALIGN_WAIT_MS;
		uart_state = UART_SEQUENCE_WAIT_ALIGN;
		*motors_started = false;
		heartbeat->state = ECU_STATE_READY;
		break;
	case 's': {
		bool motor_stop_ok;

		TEST_CancelSequences();

		motor_stop_ok = MOTOR_RequestStopBoth();

		if (motor_stop_ok == false) {
			xil_printf("[ERROR] MOTOR STOP PAIR QUEUE FAIL\r\n");
		} else {
			*motors_started = false;
			target_rpm->left_rpm = 0U;
			target_rpm->right_rpm = 0U;
			heartbeat->state = ECU_STATE_READY;
		}

		break;
	}
	case 'e':
		TEST_CancelSequences();
		if (SendEmergencyStopCommand() != false) {
			*manual_estop_latched = true;
			*motors_started = false;
			target_rpm->left_rpm = 0U;
			target_rpm->right_rpm = 0U;
			heartbeat->state = ECU_STATE_READY;
		} else
			xil_printf("[ERROR] E-STOP SEND FAIL\r\n");
		break;
	case 'c':
		TEST_CancelSequences();
		if (MOTOR_RequestClearError() != false) {
			*manual_estop_latched = false;
			*motors_started = false;
			heartbeat->state = ECU_STATE_READY;
		} else
			xil_printf("[ERROR] CLEAR_ERROR QUEUE FAIL\r\n");
		break;
	case 'i':
		xil_printf("[UART] TARGET L=%d R=%d STARTED=%d ALIGNED=%d ESTOP=%d SAFE=%d\r\n", (int)target_rpm->left_rpm, (int)target_rpm->right_rpm, (int)*motors_started, (int)motor_alignment_ready, (int)*manual_estop_latched, (int)emergency_stop_latched);
		break;
	case 'h':
		TEST_PrintUartHelp();
		break;
	case 'a':
		if (!emergency_stop_latched && !*manual_estop_latched && MOTOR_RequestAlign())
			xil_printf("[UART] MOTOR ALIGN START\r\n");
		else
			xil_printf("[UART] ALIGN BLOCKED OR QUEUE FAIL\r\n");
		break;
	case 'G':
		(void)InjectAdas(OBJECT_TRAFFIC_GREEN, SIM_OBJECT_Y0, SIM_TRAFFIC_Y1);
		break;
	case '7':
		(void)InjectAdas(OBJECT_SIGN_70, SIM_OBJECT_Y0, SIM_SIGN_Y1);
		break;
	case 'b':
		(void)InjectAdas(OBJECT_SPEED_BUMP, 0U, SIM_BUMP_NEAR_Y1);
		break;
	case '3':
		(void)InjectAdas(OBJECT_SIGN_30, SIM_OBJECT_Y0, SIM_SIGN_Y1);
		break;
	case 'r':
		(void)InjectAdas(OBJECT_TRAFFIC_RED, SIM_OBJECT_Y0, SIM_TRAFFIC_Y1);
		break;
	case 't':
		(void)TEST_StartAdasScenario(target_rpm, motors_started, heartbeat, emergency_stop_latched, *manual_estop_latched);
		break;
	case '\r':
	case '\n':
		break;
	default:
		xil_printf("[UART] UNKNOWN CMD=%c\r\n", command);
		break;
	}
}

void TEST_ProcessPendingUartRun(bool *motors_started, bool *manual_estop_latched, Heartbeat_t *heartbeat, bool emergency_stop_latched)
{
	if ((motors_started == NULL) || (manual_estop_latched == NULL) || (heartbeat == NULL) || (uart_state != UART_SEQUENCE_WAIT_ALIGN))
		return;
	if (emergency_stop_latched || *manual_estop_latched) {
		uart_state = UART_SEQUENCE_IDLE;
		return;
	}
	if ((int32_t)(millis() - uart_due_tick) < 0)
		return;
	uart_state = UART_SEQUENCE_IDLE;
	if (MOTOR_RequestRun() != false) {
		*motors_started = true;
		motor_alignment_ready = true;
		heartbeat->state = ECU_STATE_RUN;
		(void)InjectAdas(OBJECT_TRAFFIC_GREEN, SIM_OBJECT_Y0, SIM_TRAFFIC_Y1);
	} else {
		*motors_started = false;
		motor_alignment_ready = false;
		heartbeat->state = ECU_STATE_READY;
		xil_printf("[ERROR] MOTOR RUN QUEUE FAIL\r\n");
	}
}

#endif /* ENABLE_DEBUG_TEST_CODE */
