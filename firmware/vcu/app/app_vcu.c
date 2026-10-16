
/*=========================================================
 * Development notes
 * sehun merge
 * gihun bypass
 * safety -> fault manager
 * E.S. -> bypass route (Gihoon)
 * Investigate queue-full behavior if Motor1 succeeds but Motor2 fails.
 *=========================================================*/

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "xil_printf.h"

#include "../common/delay/delay.h"
#include "../common/interrupt/interrupt.h"
#include "app_vcu.h"
#include "../driver/vcu/vcu.h"
#include "../driver/can/can.h"
#include "../driver/motor/motor.h"
#include "../driver/bluetooth/bluetooth.h"
#include "../hal/gpio/gpio.h"
#include "../hal/tim/tim.h"
#include "../hal/can/can.h"
#include "../hal/safety_domain/safety_domain.h"

#define ADAS1_ECU_CONNECTED (1U)
#define ADAS2_ECU_CONNECTED (0U)
#define MOTOR1_ECU_CONNECTED (1U)
#define MOTOR2_ECU_CONNECTED (1U)
#define TIMER_1MS_PSC (79U)
#define TIMER_1MS_ARR (999U)
#define TIMER_10MS_PSC (79U)
#define TIMER_10MS_ARR (9999U)
#define BUS_OFF_RESET_DELAY_MS (1000U)
#define ESTOP_CONFIRM_TIMEOUT_MS (1000U)
#define ESTOP_RETRY_INTERVAL_MS (100U)
#define ADAS_RESULT_TIMEOUT_MS (1000U)
#define BLUETOOTH_COMMAND_BUFFER_SIZE (32U)
#define BLUETOOTH_COMMAND_IDLE_MS (50U)
#define BLUETOOTH_MIN_RPM (100U)
#define BLUETOOTH_MAX_RPM (VEHICLE_MAX_RPM)
#define BLUETOOTH_TURN_INNER_RPM (100U)
#define BLUETOOTH_TURN_OUTER_RPM (150U)
#define AUTO_MOTOR_COMMAND_MIN_INTERVAL_MS (100U)
#define AUTO_MOTOR_COMMAND_MIN_DELTA_RPM (5U)
#define AUTO_STEERING_REVERSAL_MIN_MS (200U)
#define AUTO_STEERING_REVERSAL_MAX_MS (400U)
#define AUTO_STEERING_SETTLED_DELTA_RPM (8U)
#define MOTOR_REMOTE_TUNING_ENABLE (1U)

#define REQUIRED_HEARTBEAT_MASK \
	(((MOTOR1_ECU_CONNECTED != 0U) ? CAN_TIMEOUT_MOTOR1 : 0U) | \
	 ((MOTOR2_ECU_CONNECTED != 0U) ? CAN_TIMEOUT_MOTOR2 : 0U) | \
	 ((ADAS1_ECU_CONNECTED != 0U) ? CAN_TIMEOUT_ADAS1 : 0U) | \
	 ((ADAS2_ECU_CONNECTED != 0U) ? CAN_TIMEOUT_ADAS2 : 0U))

/*=========================================================
 * GPIO 0-3: CAN faults by ECU
 * GPIO 4-7: heartbeat timeouts in the same ECU order
 *=========================================================*/
#define LED_CAN_FAULT_MOTOR1 GPIO_PIN_0
#define LED_CAN_FAULT_MOTOR2 GPIO_PIN_1
#define LED_CAN_FAULT_ADAS1 GPIO_PIN_2
#define LED_CAN_FAULT_ADAS2 GPIO_PIN_3
#define LED_HEARTBEAT_TIMEOUT_MOTOR1 GPIO_PIN_4
#define LED_HEARTBEAT_TIMEOUT_MOTOR2 GPIO_PIN_5
#define LED_HEARTBEAT_TIMEOUT_ADAS1 GPIO_PIN_6
#define LED_HEARTBEAT_TIMEOUT_ADAS2 GPIO_PIN_7

static uint32_t GetCanFaultLedMask(void)
{
	uint32_t led_mask = 0U;

	if ((MOTOR1_ECU_CONNECTED != 0U) && CAN_IsFaultDetected(ECU_MOTOR1))
		led_mask |= LED_CAN_FAULT_MOTOR1;
	if ((MOTOR2_ECU_CONNECTED != 0U) && CAN_IsFaultDetected(ECU_MOTOR2))
		led_mask |= LED_CAN_FAULT_MOTOR2;
	if ((ADAS1_ECU_CONNECTED != 0U) && CAN_IsFaultDetected(ECU_ADAS1))
		led_mask |= LED_CAN_FAULT_ADAS1;
	if ((ADAS2_ECU_CONNECTED != 0U) && CAN_IsFaultDetected(ECU_ADAS2))
		led_mask |= LED_CAN_FAULT_ADAS2;

	return led_mask;
}

static uint32_t GetHeartbeatTimeoutLedMask(uint32_t timeout_mask)
{
	uint32_t led_mask = 0U;

	if (timeout_mask & CAN_TIMEOUT_MOTOR1)
		led_mask |= LED_HEARTBEAT_TIMEOUT_MOTOR1;
	if (timeout_mask & CAN_TIMEOUT_MOTOR2)
		led_mask |= LED_HEARTBEAT_TIMEOUT_MOTOR2;
	if (timeout_mask & CAN_TIMEOUT_ADAS1)
		led_mask |= LED_HEARTBEAT_TIMEOUT_ADAS1;
	if (timeout_mask & CAN_TIMEOUT_ADAS2)
		led_mask |= LED_HEARTBEAT_TIMEOUT_ADAS2;

	return led_mask;
}

typedef enum
{
	MOTOR_START_IDLE = 0U,
	MOTOR_START_WAIT_PARAM,
	MOTOR_START_WAIT_RUN
} MotorStartState_t;

typedef enum
{
	VCU_FAULT_NONE = 0U,
	VCU_FAULT_CAN_BUS_OFF,
	VCU_FAULT_MOTOR_COMMAND_TIMEOUT,
	VCU_FAULT_HEARTBEAT_INVALID,
	VCU_FAULT_CAN_RX_QUEUE_OVERFLOW,
	VCU_FAULT_ADAS_RESULT_TIMEOUT,
	VCU_FAULT_CAN_ECU,
	VCU_FAULT_ADAS_EVENT_QUEUE_OVERFLOW,
	VCU_FAULT_MOTOR_RUN_QUEUE,
	VCU_FAULT_MOTOR_STOP_QUEUE,
	VCU_FAULT_ESTOP_CONFIRM_TIMEOUT
} VCU_FaultReason_t;

static void CancelMotorStartSequence(void);

static void LatchEmergencyStop(bool *latched, Heartbeat_t *heartbeat)
{
	if ((latched == NULL) || (heartbeat == NULL))
		return;
	if (*latched == false) {
		CAN_DiscardPendingTx();
		APP_SetVehicleEnabled(false);
	}
	CancelMotorStartSequence();
	*latched = true;
	heartbeat->state = ECU_STATE_FAULT;
}

typedef struct
{
	uint32_t timeout_mask;
	uint32_t debug_count;

	uint32_t can_fault_led_mask;
	uint32_t heartbeat_timeout_led_mask;

	uint32_t received_heartbeat_mask;
	uint32_t healthy_heartbeat_mask;

	MotorStatus_t motor1_status;
	MotorStatus_t motor2_status;
	MotorTargetrpm_t target_rpm;

	Heartbeat_t heartbeat;

	bool emergency_stop_latched;
	bool drive_gate_opened;
	bool startup_fault_reset_sent;
	bool startup_align_requested;
	uint32_t startup_align_motor1_baseline;
	uint32_t startup_align_motor2_baseline;
	MotorStartState_t motor_start_state;

	bool controlled_stop_pending;
	bool motors_started;
	bool bluetooth_manual_control;
	bool fault_clear_pending;

	bool bus_off_recovery_active;
	bool bus_off_estop_owned;
	bool watchdog_reset_pending;

	uint32_t bus_off_start_tick;

	bool estop_confirmation_started;
	bool estop_confirmed;

	uint32_t estop_start_tick;
	uint32_t estop_last_retry_tick;

	uint32_t estop_motor1_status_baseline;
	uint32_t estop_motor2_status_baseline;

	uint32_t control_gate_open_tick;

	uint32_t adas1_result_last_tick;
	uint32_t adas2_result_last_tick;
	VCU_FaultReason_t reported_fault;

} VCU_Context_t;

static VCU_Context_t vcu;

static uint8_t bluetooth_command[BLUETOOTH_COMMAND_BUFFER_SIZE];
static uint8_t bluetooth_command_length;
static MotorTargetrpm_t last_auto_motor_command;
static uint32_t last_auto_motor_command_tick;
static bool last_auto_motor_command_valid;
static int8_t last_auto_steering_sign;
static int8_t pending_auto_steering_sign;
static uint32_t pending_auto_steering_tick;
static bool auto_steering_reversal_pending;
static uint16_t can_diag_peak_tec;
static uint8_t can_diag_last_error_state;
static uint32_t can_diag_last_error_tick;
static uint32_t can_diag_last_error_sr;

static const char *CanErrorStateName(uint8_t error_state)
{
	switch (error_state) {
	case 1U:  return "IDLE";
	case 2U:  return "SOF_BIT";
	case 3U:  return "ARBITRATION_BIT";
	case 4U:  return "CONTROL_BIT";
	case 5U:  return "DATA_BIT";
	case 6U:  return "CRC_BIT";
	case 7U:  return "CRC_DELIMITER";
	case 8U:  return "ACK_MISSING";
	case 9U:  return "ACK_DELIMITER";
	case 10U: return "EOF_FORM";
	case 12U: return "ERROR_FLAG";
	case 15U: return "STUFF_BIT";
	default:  return "UNKNOWN";
	}
}

static void CaptureCanTxDiagnostics(void)
{
	uint32_t status = CAN0->CAN_SR;
	uint16_t tec = (uint16_t)((status & CAN_TEC_MASK) >> CAN_TEC_POS);
	uint8_t error_state =
		(uint8_t)((status & CAN_LAST_ERROR_STATE_MASK) >>
		          CAN_LAST_ERROR_STATE_POS);

	if (tec > can_diag_peak_tec)
		can_diag_peak_tec = tec;
	if ((tec != 0U) || ((status & CAN_ERROR) != 0U)) {
		can_diag_last_error_state = error_state;
		can_diag_last_error_tick = millis();
		can_diag_last_error_sr = status;
	}
}

static uint16_t MotorRpmDelta(uint16_t first, uint16_t second)
{
	return (first >= second) ? (uint16_t)(first - second) :
			(uint16_t)(second - first);
}

static bool MotorReadyForAutoStart(ECU_t ecu)
{
	MotorStatus_t status = {0};
	uint16_t actual_rpm;

	if (GetMotorStatus(ecu, &status) == false)
		return false;
	actual_rpm = (uint16_t)((status.rpm < 0) ?
		-(int32_t)status.rpm : status.rpm);
	return ((status.state == CAN_MOTOR_STATE_ALIGNED) ||
	        (status.state == CAN_MOTOR_STATE_READY)) &&
	       (status.target_rpm == 0) && (actual_rpm <= 5U);
}

static bool CanAcceptGreenAutoStart(void)
{
	return (vcu.drive_gate_opened != false) &&
	       (vcu.emergency_stop_latched == false) &&
	       (vcu.controlled_stop_pending == false) &&
	       (vcu.motors_started == false) &&
	       (vcu.motor_start_state == MOTOR_START_IDLE) &&
	       ((vcu.healthy_heartbeat_mask & REQUIRED_HEARTBEAT_MASK) ==
	        REQUIRED_HEARTBEAT_MASK) &&
	       (vcu.can_fault_led_mask == 0U) &&
	       (CAN_IsBusOff(CAN0) == false) &&
	       (SAFETY_DOMAIN_IsFaultIrq(SAFETY_DOMAIN0) == false) &&
	       MotorReadyForAutoStart(ECU_MOTOR1) &&
	       MotorReadyForAutoStart(ECU_MOTOR2);
}

static int8_t AutoSteeringSign(const MotorTargetrpm_t *target)
{
	if (target->right_rpm > target->left_rpm) return 1;
	if (target->left_rpm > target->right_rpm) return -1;
	return 0;
}

/* Lane control still runs every 10 ms, but the two motor SET_PARAM frames are
 * rate limited. Repeated near-identical commands previously exercised the
 * custom CAN transmitter continuously and drove its TEC to bus-off. Returning
 * true when a frame is intentionally deferred prevents a false queue fault;
 * the newest vcu.target_rpm is reconsidered on the next control pass. */
static bool RequestRateLimitedAutoTarget(const MotorTargetrpm_t *target)
{
	uint32_t now;
	int8_t requested_sign;
	bool reversal_confirmed = false;

	if (target == NULL) return false;
	now = millis();
	requested_sign = AutoSteeringSign(target);

	if (auto_steering_reversal_pending != false) {
		if ((requested_sign == 0) ||
		    (requested_sign == last_auto_steering_sign)) {
			auto_steering_reversal_pending = false;
		} else if (requested_sign != pending_auto_steering_sign) {
			pending_auto_steering_sign = requested_sign;
			pending_auto_steering_tick = now;
			return true;
		} else {
			uint32_t reversal_age =
				(uint32_t)(now - pending_auto_steering_tick);
			bool actual_steering_settled = false;

			if (reversal_age >= AUTO_STEERING_REVERSAL_MIN_MS) {
				MotorStatus_t motor1_status;
				MotorStatus_t motor2_status;

				if ((GetMotorStatus(ECU_MOTOR1, &motor1_status) != false) &&
				    (GetMotorStatus(ECU_MOTOR2, &motor2_status) != false)) {
					uint16_t motor1_rpm = (uint16_t)((motor1_status.rpm < 0) ?
						-(int32_t)motor1_status.rpm : motor1_status.rpm);
					uint16_t motor2_rpm = (uint16_t)((motor2_status.rpm < 0) ?
						-(int32_t)motor2_status.rpm : motor2_status.rpm);

					actual_steering_settled =
						(MotorRpmDelta(motor1_rpm, motor2_rpm) <=
						 AUTO_STEERING_SETTLED_DELTA_RPM);
				}
			}

			if ((reversal_age < AUTO_STEERING_REVERSAL_MIN_MS) ||
			    ((reversal_age < AUTO_STEERING_REVERSAL_MAX_MS) &&
			     (actual_steering_settled == false))) {
				return true;
			}

			auto_steering_reversal_pending = false;
			reversal_confirmed = true;
		}
	}

	if ((reversal_confirmed == false) &&
	    (auto_steering_reversal_pending == false) &&
	    (requested_sign != 0) &&
	    (last_auto_steering_sign != 0) &&
	    (requested_sign != last_auto_steering_sign)) {
		MotorTargetrpm_t neutral_target;
		uint32_t average_rpm = ((uint32_t)target->left_rpm +
		                        (uint32_t)target->right_rpm + 1U) / 2U;

		neutral_target.left_rpm = (uint16_t)average_rpm;
		neutral_target.right_rpm = (uint16_t)average_rpm;
		if (MOTOR_RequestTargetRPM(&neutral_target) == false) return false;
		last_auto_motor_command = neutral_target;
		last_auto_motor_command_tick = now;
		last_auto_motor_command_valid = true;
		pending_auto_steering_sign = requested_sign;
		pending_auto_steering_tick = now;
		auto_steering_reversal_pending = true;
		return true;
	}

	if ((last_auto_motor_command_valid != false) &&
	    ((uint32_t)(now - last_auto_motor_command_tick) <
	     AUTO_MOTOR_COMMAND_MIN_INTERVAL_MS))
		return true;
	if ((last_auto_motor_command_valid != false) &&
	    (MotorRpmDelta(target->left_rpm,
	                   last_auto_motor_command.left_rpm) <
	     AUTO_MOTOR_COMMAND_MIN_DELTA_RPM) &&
	    (MotorRpmDelta(target->right_rpm,
	                   last_auto_motor_command.right_rpm) <
	     AUTO_MOTOR_COMMAND_MIN_DELTA_RPM))
		return true;
	if (MOTOR_RequestTargetRPM(target) == false) return false;
	last_auto_motor_command = *target;
	last_auto_motor_command_tick = now;
	last_auto_motor_command_valid = true;
	last_auto_steering_sign = requested_sign;
	return true;
}
static uint32_t bluetooth_last_rx_tick;

/* Report only the first cause that latched the current fault.  xil_printf is
 * mirrored to Bluetooth, so this produces one concise diagnostic there too. */
static void ReportFaultOnce(VCU_FaultReason_t reason, uint32_t detail1, uint32_t detail2)
{
	if ((reason == VCU_FAULT_NONE) || (vcu.reported_fault != VCU_FAULT_NONE)) {
		return;
	}

	vcu.reported_fault = reason;
	switch (reason) {
	case VCU_FAULT_CAN_BUS_OFF:
	{
		uint32_t sticky_diag = CAN0->CAN_DIAG;
		xil_printf("[BT] FAULT: CAN_BUS_OFF CAUSE=%s(%d) PEAK_TEC=%d AGE_MS=%d SAFETY_SR=0x%x CAN_SR=0x%x PREV_SR=0x%x HW_PEAK_TEC=%d HW_CAUSE=%d HW_BO=%d DIAG=0x%x\r\n",
		           CanErrorStateName(can_diag_last_error_state),
		           (int)can_diag_last_error_state,
		           (int)can_diag_peak_tec,
		           (int)((uint32_t)(millis() - can_diag_last_error_tick)),
		           (unsigned int)detail1, (unsigned int)detail2,
		           (unsigned int)can_diag_last_error_sr,
		           (int)((sticky_diag & CAN_DIAG_PEAK_TEC_MASK) >> CAN_DIAG_PEAK_TEC_POS),
		           (int)((sticky_diag & CAN_DIAG_ERROR_STATE_MASK) >> CAN_DIAG_ERROR_STATE_POS),
		           (int)((sticky_diag & CAN_DIAG_BUS_OFF) != 0U),
		           (unsigned int)sticky_diag);
		break;
	}
	case VCU_FAULT_MOTOR_COMMAND_TIMEOUT:
		xil_printf("[BT] FAULT: MOTOR_COMMAND_TIMEOUT M1=%d M2=%d\r\n",
		           (int)detail1, (int)detail2);
		break;
	case VCU_FAULT_HEARTBEAT_INVALID:
		xil_printf("[BT] FAULT: HEARTBEAT_LOST INVALID_MASK=0x%x HEALTHY=0x%x M1=%d M2=%d ADAS1=%d ADAS2=%d\r\n",
		           (unsigned int)detail1, (unsigned int)detail2,
		           (int)((detail1 & CAN_TIMEOUT_MOTOR1) != 0U),
		           (int)((detail1 & CAN_TIMEOUT_MOTOR2) != 0U),
		           (int)((detail1 & CAN_TIMEOUT_ADAS1) != 0U),
		           (int)((detail1 & CAN_TIMEOUT_ADAS2) != 0U));
		break;
	case VCU_FAULT_CAN_RX_QUEUE_OVERFLOW:
		xil_printf("[BT] FAULT: CAN_RX_QUEUE_OVERFLOW CAUSE=RX_PROCESSING_DELAY\r\n");
		break;
	case VCU_FAULT_ADAS_RESULT_TIMEOUT:
		xil_printf("[BT] FAULT: ADAS_RESULT_TIMEOUT ADAS1=%d ADAS2=%d\r\n",
		           (int)detail1, (int)detail2);
		break;
	case VCU_FAULT_CAN_ECU:
		xil_printf("[BT] FAULT: ECU_FAULT MASK=0x%x M1=%d M2=%d ADAS1=%d ADAS2=%d\r\n",
		           (unsigned int)detail1,
		           (int)((detail1 & LED_CAN_FAULT_MOTOR1) != 0U),
		           (int)((detail1 & LED_CAN_FAULT_MOTOR2) != 0U),
		           (int)((detail1 & LED_CAN_FAULT_ADAS1) != 0U),
		           (int)((detail1 & LED_CAN_FAULT_ADAS2) != 0U));
		break;
	case VCU_FAULT_ADAS_EVENT_QUEUE_OVERFLOW:
		xil_printf("[BT] FAULT: ADAS_EVENT_QUEUE_OVERFLOW APP=%d CAN=%d\r\n",
		           (int)detail1, (int)detail2);
		break;
	case VCU_FAULT_MOTOR_RUN_QUEUE:
		xil_printf("[BT] FAULT: MOTOR_RUN_QUEUE_FAIL CAUSE=CAN_TX_QUEUE\r\n");
		break;
	case VCU_FAULT_MOTOR_STOP_QUEUE:
		xil_printf("[BT] FAULT: MOTOR_STOP_QUEUE_FAIL CAUSE=CAN_TX_QUEUE\r\n");
		break;
	case VCU_FAULT_ESTOP_CONFIRM_TIMEOUT:
		xil_printf("[BT] FAULT: ESTOP_CONFIRM_TIMEOUT M1_STATE=%d M2_STATE=%d\r\n",
		           (int)detail1, (int)detail2);
		break;
	default:
		break;
	}
}

static void ProcessFaultClear(void)
{
	MotorConfirmationState_t motor1_confirmation;
	MotorConfirmationState_t motor2_confirmation;

	if (vcu.fault_clear_pending == false) {
		return;
	}

	motor1_confirmation = MOTOR_GetConfirmationState(ECU_MOTOR1);
	motor2_confirmation = MOTOR_GetConfirmationState(ECU_MOTOR2);
	if ((motor1_confirmation != MOTOR_CONFIRMATION_CONFIRMED) ||
	    (motor2_confirmation != MOTOR_CONFIRMATION_CONFIRMED)) {
		return;
	}

	MOTOR_ClearConfirmation(ECU_MOTOR1);
	MOTOR_ClearConfirmation(ECU_MOTOR2);
	(void)CAN_ClearFaultFlag(ECU_MOTOR1);
	(void)CAN_ClearFaultFlag(ECU_MOTOR2);
	CAN_ClearRxQueueFault();

	vcu.fault_clear_pending = false;
	vcu.emergency_stop_latched = false;
	vcu.estop_confirmation_started = false;
	vcu.estop_confirmed = false;
	vcu.drive_gate_opened = false;
	vcu.startup_fault_reset_sent = false;
	vcu.startup_align_requested = false;
	vcu.can_fault_led_mask = 0U;
	vcu.heartbeat_timeout_led_mask = 0U;
	vcu.reported_fault = VCU_FAULT_NONE;
	can_diag_peak_tec = 0U;
	can_diag_last_error_state = 0U;
	can_diag_last_error_tick = millis();
	can_diag_last_error_sr = 0U;
	vcu.heartbeat.state = ECU_STATE_READY;
	CancelMotorStartSequence();
	GPIO_WritePort(GPIO, 0U);
	xil_printf("[BT] FAULT CLEARED - STARTUP RECHECK\r\n");
}

static bool BluetoothCommandEqualsIgnoreCase(const char *text)
{
	uint8_t index = 0U;

	if (text == NULL) {
		return false;
	}

	while ((index < bluetooth_command_length) && (text[index] != '\0')) {
		uint8_t received = bluetooth_command[index];
		uint8_t expected = (uint8_t)text[index];

		if ((received >= (uint8_t)'A') && (received <= (uint8_t)'Z')) {
			received = (uint8_t)(received + ((uint8_t)'a' - (uint8_t)'A'));
		}
		if ((expected >= (uint8_t)'A') && (expected <= (uint8_t)'Z')) {
			expected = (uint8_t)(expected + ((uint8_t)'a' - (uint8_t)'A'));
		}
		if (received != expected) {
			return false;
		}
		index++;
	}

	return ((index == bluetooth_command_length) && (text[index] == '\0'));
}

static bool BluetoothCommandIsStop(void)
{
	/* UTF-8 byte sequence for the Korean command "stop" (jeongji). */
	static const uint8_t korean_stop[] = {0xECU, 0xA0U, 0x95U, 0xECU, 0xA7U, 0x80U};
	uint8_t index;

	if (BluetoothCommandEqualsIgnoreCase("stop") ||
	    BluetoothCommandEqualsIgnoreCase("s") ||
	    BluetoothCommandEqualsIgnoreCase("0")) {
		return true;
	}

	if (bluetooth_command_length != (uint8_t)sizeof(korean_stop)) {
		return false;
	}
	for (index = 0U; index < (uint8_t)sizeof(korean_stop); index++) {
		if (bluetooth_command[index] != korean_stop[index]) {
			return false;
		}
	}
	return true;
}

/* Keep the application vehicle state identical to an ADAS green-light RUN.
 * Bluetooth supplies its own exact left/right RPM after this state update. */
static void ApplyBluetoothRunEnable(void)
{
	Event_t run_event = {0};

	run_event.type = EVENT_MOTOR_RUN;
	run_event.pd_output = 0;
	(void)APP_ProcessEvent(&run_event);
}

static void ResetLaneControlForADASRun(void)
{
	ADASResult_t result = {0};
	uint32_t result_tick = 0U;
	uint32_t now = millis();
	bool lane_valid = false;

	if (CAN_PeekADASResult(ECU_ADAS1, &result, &result_tick) != false) {
		lane_valid = (((uint32_t)(now - result_tick) < ADAS_RESULT_TIMEOUT_MS) &&
		              (result.near_valid != false) &&
		              (result.near_x <= 1279U));
	}
	APP_ResetLaneControl(result.near_x, lane_valid);
	last_auto_motor_command_valid = false;
	last_auto_steering_sign = 0;
	pending_auto_steering_sign = 0;
	pending_auto_steering_tick = 0U;
	auto_steering_reversal_pending = false;
	xil_printf("[BT] LANE RESET VALID=%d RAW=%d\r\n",
	           (int)lane_valid, (int)result.near_x);
}

static bool BluetoothCommandHasKey(const char *key, uint8_t key_length)
{
	uint8_t index;

	if ((key == NULL) || (bluetooth_command_length <= key_length) ||
	    (bluetooth_command[key_length] != '=')) {
		return false;
	}
	for (index = 0U; index < key_length; index++) {
		uint8_t received = bluetooth_command[index];
		uint8_t expected = (uint8_t)key[index];
		if ((received >= 'A') && (received <= 'Z')) received += 'a' - 'A';
		if (received != expected) return false;
	}
	return true;
}

static uint8_t QueueYcbcrThresholdBurst(const uint8_t values[6],
	                                    bool query,
	                                    uint8_t sequence)
{
	uint8_t queued = 0U;
	while ((queued < 3U) &&
	       (SendYcbcrThresholdCommand(values, query, sequence) != false)) {
		++queued;
	}
	return queued;
}

static void ProcessBluetoothCommand(void)
{
	MotorTargetrpm_t requested_rpm = {0U, 0U};
	uint32_t rpm = 0U;
	uint8_t index;
	bool numeric = (bluetooth_command_length != 0U);
	bool run_command = false;
	static uint8_t ycbcr_sequence = 0U;

	/* Runtime Zynq colour-filter tuning over CAN.
	 * Examples: YC=80,255,0,110,110,170  or  YC? */
	if ((bluetooth_command_length == 3U) &&
	    ((bluetooth_command[0] | 0x20U) == 'y') &&
	    ((bluetooth_command[1] | 0x20U) == 'c') &&
	    (bluetooth_command[2] == '?')) {
		uint8_t queued = QueueYcbcrThresholdBurst(NULL, true, ++ycbcr_sequence);
		if (queued != 0U)
			xil_printf("[BT] YC QUERY SENT SEQ=%d BURST=%d\r\n",
			           (int)ycbcr_sequence, (int)queued);
		else xil_printf("[BT] YC ERROR: CAN QUEUE FULL\r\n");
		return;
	}
	if ((bluetooth_command_length >= 14U) &&
	    ((bluetooth_command[0] | 0x20U) == 'y') &&
	    ((bluetooth_command[1] | 0x20U) == 'c') &&
	    (bluetooth_command[2] == '=')) {
		uint8_t values[6] = {0U};
		uint8_t field = 0U;
		uint16_t value = 0U;
		bool have_digit = false;
		for (index = 3U; index <= bluetooth_command_length; ++index) {
			uint8_t ch = (index < bluetooth_command_length) ? bluetooth_command[index] : ',';
			if ((ch >= '0') && (ch <= '9')) {
				have_digit = true;
				value = (uint16_t)(value * 10U + (uint16_t)(ch - '0'));
				if (value > 255U) { field = 7U; break; }
			} else if (ch == ',') {
				if (!have_digit || field >= 6U) { field = 7U; break; }
				values[field++] = (uint8_t)value;
				value = 0U; have_digit = false;
			} else { field = 7U; break; }
		}
		if ((field != 6U) || values[0] > values[1] ||
		    values[2] > values[3] || values[4] > values[5]) {
			xil_printf("[BT] YC ERROR: YC=YL,YH,CBL,CBH,CRL,CRH (0-255)\r\n");
			return;
		}
		{
			uint8_t queued = QueueYcbcrThresholdBurst(
				values, false, ++ycbcr_sequence);
			if (queued == 0U) {
			xil_printf("[BT] YC ERROR: CAN QUEUE FULL\r\n"); return;
			}
			xil_printf("[BT] YC SENT Y=%d,%d CB=%d,%d CR=%d,%d SEQ=%d BURST=%d\r\n",
			           values[0], values[1], values[2], values[3],
			           values[4], values[5], (int)ycbcr_sequence, (int)queued);
		}
		return;
	}

	if (BluetoothCommandEqualsIgnoreCase("tune") != false) {
		LaneTuning_t tuning;
		APP_GetLaneTuning(&tuning);
		xil_printf("[BT] TUNE SLMODE=%d SL100=%d SL150=%d SL200=%d LIMMODE=%d LIMIT=%d CENTER=%d DEAD=%d\r\n",
		           (int)tuning.slew_mode,
		           (int)tuning.slew_100_ms, (int)tuning.slew_150_ms,
		           (int)tuning.slew_200_ms, (int)tuning.limit_mode,
		           (int)tuning.max_correction_rpm,
		           (int)tuning.center_x, (int)tuning.deadband_pixels);
		xil_printf("[BT] FILTER JUMP=%d CLUSTER=%d CONFIRM=%d STEP=%d\r\n",
		           (int)tuning.jump_pixels, (int)tuning.cluster_pixels,
		           (int)tuning.confirm_frames, (int)tuning.reacquire_step_pixels);
		xil_printf("[BT] RATE RATEF=%d RATEMAX=%d RATEDT=%d DREF=%d\r\n",
		           (int)tuning.rate_filter_divisor,
		           (int)tuning.rate_max_pixels_per_sec,
		           (int)tuning.rate_max_dt_ms,
		           (int)tuning.derivative_reference_ms);
		xil_printf("[BT] PATH KFF=%d STEP=%d NLOOK=%d FLOOK=%d MEMDIST=%d MEMTIME=%d\r\n",
		           (int)tuning.kff_scaled, (int)tuning.path_step_mm,
		           (int)tuning.near_lookahead_mm,
		           (int)tuning.far_lookahead_mm,
		           (int)tuning.memory_max_distance_mm,
		           (int)tuning.memory_max_time_ms);
		xil_printf("[BT] CONF FNWARN=%d FNREJECT=%d LOW=%d HIGH=%d CURVENT=%d CURVEXIT=%d CSGAIN=%d RECBLEND=%d\r\n",
		           (int)tuning.far_near_warn_diff,
		           (int)tuning.far_near_reject_diff,
		           (int)tuning.confidence_low,
		           (int)tuning.confidence_high,
		           (int)tuning.curve_enter_pixels,
		           (int)tuning.curve_exit_pixels,
		           (int)tuning.curve_speed_gain,
		           (int)tuning.recover_blend_rate);
		return;
	}

	{
		LaneTuningKey_t tuning_key = LANE_TUNE_SLEW_100_MS;
		const char *tuning_name = NULL;
		uint8_t value_index = 0U;
		uint32_t value = 0U;

		if (BluetoothCommandHasKey("sl100", 5U)) { tuning_key = LANE_TUNE_SLEW_100_MS; tuning_name = "SL100"; value_index = 6U; }
		else if (BluetoothCommandHasKey("sl150", 5U)) { tuning_key = LANE_TUNE_SLEW_150_MS; tuning_name = "SL150"; value_index = 6U; }
		else if (BluetoothCommandHasKey("sl200", 5U)) { tuning_key = LANE_TUNE_SLEW_200_MS; tuning_name = "SL200"; value_index = 6U; }
		else if (BluetoothCommandHasKey("slmode", 6U)) { tuning_key = LANE_TUNE_SLEW_MODE; tuning_name = "SLMODE"; value_index = 7U; }
		else if (BluetoothCommandHasKey("limmode", 7U)) { tuning_key = LANE_TUNE_LIMIT_MODE; tuning_name = "LIMMODE"; value_index = 8U; }
		else if (BluetoothCommandHasKey("limit", 5U)) { tuning_key = LANE_TUNE_MAX_CORRECTION_RPM; tuning_name = "LIMIT"; value_index = 6U; }
		else if (BluetoothCommandHasKey("center", 6U)) { tuning_key = LANE_TUNE_CENTER_X; tuning_name = "CENTER"; value_index = 7U; }
		else if (BluetoothCommandHasKey("dead", 4U)) { tuning_key = LANE_TUNE_DEADBAND_PIXELS; tuning_name = "DEAD"; value_index = 5U; }
		else if (BluetoothCommandHasKey("jump", 4U)) { tuning_key = LANE_TUNE_JUMP_PIXELS; tuning_name = "JUMP"; value_index = 5U; }
		else if (BluetoothCommandHasKey("cluster", 7U)) { tuning_key = LANE_TUNE_CLUSTER_PIXELS; tuning_name = "CLUSTER"; value_index = 8U; }
		else if (BluetoothCommandHasKey("confirm", 7U)) { tuning_key = LANE_TUNE_CONFIRM_FRAMES; tuning_name = "CONFIRM"; value_index = 8U; }
		else if (BluetoothCommandHasKey("step", 4U)) { tuning_key = LANE_TUNE_REACQUIRE_STEP_PIXELS; tuning_name = "STEP"; value_index = 5U; }
		else if (BluetoothCommandHasKey("ratef", 5U)) { tuning_key = LANE_TUNE_RATE_FILTER_DIVISOR; tuning_name = "RATEF"; value_index = 6U; }
		else if (BluetoothCommandHasKey("ratemax", 7U)) { tuning_key = LANE_TUNE_RATE_MAX_PIXELS_PER_SEC; tuning_name = "RATEMAX"; value_index = 8U; }
		else if (BluetoothCommandHasKey("ratedt", 6U)) { tuning_key = LANE_TUNE_RATE_MAX_DT_MS; tuning_name = "RATEDT"; value_index = 7U; }
		else if (BluetoothCommandHasKey("dref", 4U)) { tuning_key = LANE_TUNE_DERIVATIVE_REFERENCE_MS; tuning_name = "DREF"; value_index = 5U; }
		else if (BluetoothCommandHasKey("kff", 3U)) { tuning_key = LANE_TUNE_KFF_SCALED; tuning_name = "KFF"; value_index = 4U; }
		else if (BluetoothCommandHasKey("pstep", 5U)) { tuning_key = LANE_TUNE_PATH_STEP_MM; tuning_name = "PSTEP"; value_index = 6U; }
		else if (BluetoothCommandHasKey("nlook", 5U)) { tuning_key = LANE_TUNE_NEAR_LOOKAHEAD_MM; tuning_name = "NLOOK"; value_index = 6U; }
		else if (BluetoothCommandHasKey("flook", 5U)) { tuning_key = LANE_TUNE_FAR_LOOKAHEAD_MM; tuning_name = "FLOOK"; value_index = 6U; }
		else if (BluetoothCommandHasKey("fnwarn", 6U)) { tuning_key = LANE_TUNE_FAR_NEAR_WARN_DIFF; tuning_name = "FNWARN"; value_index = 7U; }
		else if (BluetoothCommandHasKey("fnreject", 8U)) { tuning_key = LANE_TUNE_FAR_NEAR_REJECT_DIFF; tuning_name = "FNREJECT"; value_index = 9U; }
		else if (BluetoothCommandHasKey("conflow", 7U)) { tuning_key = LANE_TUNE_CONF_LOW; tuning_name = "CONFLOW"; value_index = 8U; }
		else if (BluetoothCommandHasKey("confhigh", 8U)) { tuning_key = LANE_TUNE_CONF_HIGH; tuning_name = "CONFHIGH"; value_index = 9U; }
		else if (BluetoothCommandHasKey("curvent", 7U)) { tuning_key = LANE_TUNE_CURVE_ENTER; tuning_name = "CURVENT"; value_index = 8U; }
		else if (BluetoothCommandHasKey("curvexit", 8U)) { tuning_key = LANE_TUNE_CURVE_EXIT; tuning_name = "CURVEXIT"; value_index = 9U; }
		else if (BluetoothCommandHasKey("csgain", 6U)) { tuning_key = LANE_TUNE_CURVE_SPEED_GAIN; tuning_name = "CSGAIN"; value_index = 7U; }
		else if (BluetoothCommandHasKey("memdist", 7U)) { tuning_key = LANE_TUNE_MEMORY_MAX_DISTANCE_MM; tuning_name = "MEMDIST"; value_index = 8U; }
		else if (BluetoothCommandHasKey("memtime", 7U)) { tuning_key = LANE_TUNE_MEMORY_MAX_TIME_MS; tuning_name = "MEMTIME"; value_index = 8U; }
		else if (BluetoothCommandHasKey("recblend", 8U)) { tuning_key = LANE_TUNE_RECOVER_BLEND_RATE; tuning_name = "RECBLEND"; value_index = 9U; }

		if (tuning_name != NULL) {
			if (value_index >= bluetooth_command_length) {
				xil_printf("[BT] TUNE ERROR: VALUE REQUIRED\r\n");
				return;
			}
			for (index = value_index; index < bluetooth_command_length; index++) {
				if ((bluetooth_command[index] < '0') || (bluetooth_command[index] > '9') ||
				    (value > 10000U)) {
					xil_printf("[BT] TUNE ERROR: POSITIVE INTEGER REQUIRED\r\n");
					return;
				}
				value = value * 10U + (uint32_t)(bluetooth_command[index] - '0');
			}
			if (!APP_SetLaneTuning(tuning_key, value)) {
				xil_printf("[BT] TUNE ERROR: %s VALUE OUT OF RANGE\r\n", tuning_name);
				return;
			}
			xil_printf("[BT] %s=%d APPLIED\r\n", tuning_name, (int)value);
			return;
		}
	}

	if (BluetoothCommandEqualsIgnoreCase("gains") != false) {
		int32_t kp100, kd100, kp150, kd150, kp200, kd200;
		(void)APP_GetLaneGains(100U, &kp100, &kd100);
		(void)APP_GetLaneGains(150U, &kp150, &kd150);
		(void)APP_GetLaneGains(200U, &kp200, &kd200);
		xil_printf("[BT] GAINS 100:KP=%d KD=%d 150:KP=%d KD=%d 200:KP=%d KD=%d SCALE=%d\r\n",
		           (int)kp100, (int)kd100, (int)kp150, (int)kd150,
		           (int)kp200, (int)kd200, (int)LANE_GAIN_SCALE);
		return;
	}

	/* Runtime lane-gain tuning. Examples: KP100=35, KD150=20.
	 * Values are fixed-point /100 and intentionally remain RAM-only. */
	if ((bluetooth_command_length >= 7U) &&
	    ((bluetooth_command[0] | 0x20U) == 'k') &&
	    (((bluetooth_command[1] | 0x20U) == 'p') ||
	     ((bluetooth_command[1] | 0x20U) == 'd')) &&
	    (bluetooth_command[5] == '=')) {
		bool set_kp = ((bluetooth_command[1] | 0x20U) == 'p');
		uint16_t rpm_band;
		uint32_t gain = 0U;

		if ((bluetooth_command[2] == '1') && (bluetooth_command[3] == '0') &&
		    (bluetooth_command[4] == '0')) {
			rpm_band = 100U;
		} else if ((bluetooth_command[2] == '1') && (bluetooth_command[3] == '5') &&
		           (bluetooth_command[4] == '0')) {
			rpm_band = 150U;
		} else if ((bluetooth_command[2] == '2') && (bluetooth_command[3] == '0') &&
		           (bluetooth_command[4] == '0')) {
			rpm_band = 200U;
		} else {
			xil_printf("[BT] GAIN ERROR: BAND MUST BE 100/150/200\r\n");
			return;
		}

		for (index = 6U; index < bluetooth_command_length; index++) {
			if ((bluetooth_command[index] < '0') || (bluetooth_command[index] > '9')) {
				xil_printf("[BT] GAIN ERROR: USE KP100=35 OR KD100=15\r\n");
				return;
			}
			gain = gain * 10U + (uint32_t)(bluetooth_command[index] - '0');
			if (gain > 200U) {
				xil_printf("[BT] GAIN ERROR: RANGE 0-200\r\n");
				return;
			}
		}
		if (!APP_SetLaneGain(rpm_band, set_kp, (int32_t)gain)) {
			xil_printf("[BT] GAIN ERROR: RANGE 0-200\r\n");
			return;
		}
		xil_printf("[BT] %s%d=%d SCALE=100 APPLIED\r\n",
		           set_kp ? "KP" : "KD", (int)rpm_band, (int)gain);
		return;
	}

#if MOTOR_REMOTE_TUNING_ENABLE
	/* Compact aliases avoid command splitting on slow Bluetooth links:
	 * rl10 = low, rs100 = startup, rh6 = high. */
	if ((bluetooth_command_length >= 3U) &&
	    ((bluetooth_command[0] | 0x20U) == 'r') &&
	    (((bluetooth_command[1] | 0x20U) == 'l') ||
	     ((bluetooth_command[1] | 0x20U) == 's') ||
	     ((bluetooth_command[1] | 0x20U) == 'h'))) {
		uint8_t band = ((bluetooth_command[1] | 0x20U) == 'l') ? 0U :
		               (((bluetooth_command[1] | 0x20U) == 's') ? 1U : 2U);
		uint32_t ticks = 0U;
		for (index = 2U; index < bluetooth_command_length; index++) {
			if ((bluetooth_command[index] < '0') || (bluetooth_command[index] > '9')) {
				xil_printf("[BT] RAMP ERROR: USE RL/RS/RH + 5-300\r\n"); return;
			}
			ticks = ticks * 10U + (uint32_t)(bluetooth_command[index] - '0');
		}
		if (vcu.motors_started || (vcu.motor_start_state != MOTOR_START_IDLE) ||
		    vcu.controlled_stop_pending) {
			xil_printf("[BT] RAMP ERROR: STOP MOTORS FIRST\r\n"); return;
		}
		if ((ticks < 5U) || (ticks > 300U) ||
		    !SendMotorRampTuningPair(band, (uint16_t)ticks)) {
			xil_printf("[BT] RAMP ERROR: RANGE 5-300 OR QUEUE\r\n"); return;
		}
		xil_printf("[BT] RAMP SENT BAND=%d TICKS=%d; WAIT FOR M1/M2 ACK\r\n",
		           (int)band, (int)ticks);
		return;
	}

	if ((bluetooth_command_length >= 10U) &&
	    ((bluetooth_command[0] | 0x20U) == 'r') &&
	    ((bluetooth_command[1] | 0x20U) == 'a') &&
	    ((bluetooth_command[2] | 0x20U) == 'm') &&
	    ((bluetooth_command[3] | 0x20U) == 'p') &&
	    (bluetooth_command[4] == ' ')) {
		uint8_t band;
		uint8_t value_index;
		uint32_t ticks = 0U;
		if ((bluetooth_command[5] | 0x20U) == 'l') { band = 0U; value_index = 9U; }
		else if ((bluetooth_command[5] | 0x20U) == 's') { band = 1U; value_index = 13U; }
		else if ((bluetooth_command[5] | 0x20U) == 'h') { band = 2U; value_index = 10U; }
		else { xil_printf("[BT] RAMP ERROR: USE LOW/STARTUP/HIGH\r\n"); return; }
		if ((value_index >= bluetooth_command_length) ||
		    (bluetooth_command[value_index - 1U] != ' ')) {
			xil_printf("[BT] RAMP ERROR: FORMAT\r\n"); return;
		}
		for (index = value_index; index < bluetooth_command_length; index++) {
			if ((bluetooth_command[index] < '0') || (bluetooth_command[index] > '9')) {
				xil_printf("[BT] RAMP ERROR: FORMAT\r\n"); return;
			}
			ticks = ticks * 10U + (uint32_t)(bluetooth_command[index] - '0');
		}
		if (vcu.motors_started || (vcu.motor_start_state != MOTOR_START_IDLE) ||
		    vcu.controlled_stop_pending) {
			xil_printf("[BT] RAMP ERROR: STOP MOTORS FIRST\r\n"); return;
		}
		if ((ticks < 5U) || (ticks > 300U) ||
		    !SendMotorRampTuningPair(band, (uint16_t)ticks)) {
			xil_printf("[BT] RAMP ERROR: RANGE 5-300 OR QUEUE\r\n"); return;
		}
		xil_printf("[BT] RAMP SENT BAND=%d TICKS=%d; WAIT FOR M1/M2 ACK\r\n",
		           (int)band, (int)ticks);
		return;
	}
#endif

	if (BluetoothCommandEqualsIgnoreCase("tec") != false) {
		uint32_t can_status = CAN0->CAN_SR;
		uint32_t sticky_diag = CAN0->CAN_DIAG;
		uint8_t error_state = CAN_GetLastErrorState(CAN0);
		const char *error_reason = "UNKNOWN";
		switch (error_state) {
		case 1U:  error_reason = "IDLE"; break;
		case 2U:  error_reason = "SOF_BIT"; break;
		case 3U:  error_reason = "ARBITRATION_BIT"; break;
		case 4U:  error_reason = "CONTROL_BIT"; break;
		case 5U:  error_reason = "DATA_BIT"; break;
		case 6U:  error_reason = "CRC_BIT"; break;
		case 7U:  error_reason = "CRC_DELIMITER"; break;
		case 8U:  error_reason = "ACK_MISSING"; break;
		case 9U:  error_reason = "ACK_DELIMITER"; break;
		case 10U: error_reason = "EOF_FORM"; break;
		case 12U: error_reason = "ERROR_FLAG"; break;
		case 15U: error_reason = "STUFF_BIT"; break;
		default: break;
		}
		xil_printf("[CAN] TEC=%d CAUSE=%s(%d) ERROR=%d BUS_OFF=%d SR=0x%x HW_PEAK_TEC=%d HW_CAUSE=%d HW_BO=%d DIAG=0x%x\r\n",
		           (int)CAN_GetTEC(CAN0),
		           error_reason,
		           (int)error_state,
		           (int)((can_status & CAN_ERROR) != 0U),
		           (int)((can_status & CAN_BUS_OFF) != 0U),
		           (unsigned int)can_status,
		           (int)((sticky_diag & CAN_DIAG_PEAK_TEC_MASK) >> CAN_DIAG_PEAK_TEC_POS),
		           (int)((sticky_diag & CAN_DIAG_ERROR_STATE_MASK) >> CAN_DIAG_ERROR_STATE_POS),
		           (int)((sticky_diag & CAN_DIAG_BUS_OFF) != 0U),
		           (unsigned int)sticky_diag);
		return;
	}
	bool adas_run = false;
	bool turn_left = false;
	bool turn_right = false;

	if (bluetooth_command_length == 0U) {
		return;
	}

	if (BluetoothCommandIsStop() != false) {
		APP_SetVehicleEnabled(false);
		if (((vcu.motors_started != false) ||
		     (vcu.motor_start_state != MOTOR_START_IDLE)) &&
		    (vcu.controlled_stop_pending == false)) {
			if (MOTOR_RequestStopBoth() != false) {
				vcu.motor_start_state = MOTOR_START_IDLE;
				vcu.controlled_stop_pending = true;
				xil_printf("[BT] STOP REQUESTED\r\n");
			} else {
				xil_printf("[BT] ERROR: STOP QUEUE FAIL\r\n");
			}
		} else {
			xil_printf("[BT] ALREADY STOPPED\r\n");
		}
		return;
	}
	if (BluetoothCommandEqualsIgnoreCase("faultclear") ||
	    BluetoothCommandEqualsIgnoreCase("clear") ||
	    BluetoothCommandEqualsIgnoreCase("c")) {
		if ((vcu.bus_off_recovery_active != false) ||
		    (vcu.watchdog_reset_pending != false)) {
			xil_printf("[BT] ERROR: FAULT CLEAR BLOCKED BY SAFETY FAULT\r\n");
		} else if (vcu.fault_clear_pending != false) {
			xil_printf("[BT] FAULT CLEAR IN PROGRESS\r\n");
		} else if (MOTOR_RequestClearError() != false) {
			vcu.fault_clear_pending = true;
			vcu.estop_confirmed = true;
			xil_printf("[BT] FAULT CLEAR REQUESTED\r\n");
		} else {
			xil_printf("[BT] ERROR: FAULT CLEAR QUEUE FAIL\r\n");
		}
		return;
	}
	if (BluetoothCommandEqualsIgnoreCase("auto") != false) {
		vcu.bluetooth_manual_control = false;
		xil_printf("[BT] ADAS AUTO CONTROL ENABLED\r\n");
		return;
	}
	if (BluetoothCommandEqualsIgnoreCase("run") != false) {
		rpm = 150U;
		run_command = true;
		adas_run = true;
	} else if (BluetoothCommandEqualsIgnoreCase("auto100") != false) {
		rpm = 100U;
		run_command = true;
		adas_run = true;
	} else if (BluetoothCommandEqualsIgnoreCase("auto150") != false) {
		rpm = 150U;
		run_command = true;
		adas_run = true;
	} else if (BluetoothCommandEqualsIgnoreCase("auto200") != false ||
	           BluetoothCommandEqualsIgnoreCase("a") != false) {
		rpm = 200U;
		run_command = true;
		adas_run = true;
	} else if (BluetoothCommandEqualsIgnoreCase("auto250") != false) {
		rpm = 250U;
		run_command = true;
		adas_run = true;
	} else if (BluetoothCommandEqualsIgnoreCase("left") ||
	           BluetoothCommandEqualsIgnoreCase("l")) {
		rpm = BLUETOOTH_TURN_OUTER_RPM;
		run_command = true;
		turn_left = true;
	} else if (BluetoothCommandEqualsIgnoreCase("right") ||
	           BluetoothCommandEqualsIgnoreCase("r")) {
		rpm = BLUETOOTH_TURN_OUTER_RPM;
		run_command = true;
		turn_right = true;
	}

	for (index = 0U; (run_command == false) && (index < bluetooth_command_length); index++) {
		if ((bluetooth_command[index] < (uint8_t)'0') ||
		    (bluetooth_command[index] > (uint8_t)'9')) {
			numeric = false;
			break;
		}
		rpm = (rpm * 10U) + (uint32_t)(bluetooth_command[index] - (uint8_t)'0');
		if (rpm > BLUETOOTH_MAX_RPM) {
			numeric = false;
			break;
		}
	}

	if ((numeric == false) || (rpm < BLUETOOTH_MIN_RPM)) {
		xil_printf("[BT] ERROR: USE %d-%d RPM, RUN, LEFT, RIGHT, STOP, FAULT CLEAR, OR AUTO\r\n",
		           (int)BLUETOOTH_MIN_RPM, (int)BLUETOOTH_MAX_RPM);
		return;
	}
	if ((vcu.drive_gate_opened == false) || (vcu.emergency_stop_latched != false)) {
		xil_printf("[BT] ERROR: CONTROL NOT READY\r\n");
		return;
	}
	if (vcu.controlled_stop_pending != false) {
		xil_printf("[BT] ERROR: STOP IN PROGRESS\r\n");
		return;
	}

	if (turn_left != false) {
		requested_rpm.left_rpm = BLUETOOTH_TURN_INNER_RPM;
		requested_rpm.right_rpm = BLUETOOTH_TURN_OUTER_RPM;
	} else if (turn_right != false) {
		requested_rpm.left_rpm = BLUETOOTH_TURN_OUTER_RPM;
		requested_rpm.right_rpm = BLUETOOTH_TURN_INNER_RPM;
	} else {
		requested_rpm.left_rpm = (uint16_t)rpm;
		requested_rpm.right_rpm = (uint16_t)rpm;
	}
	vcu.target_rpm = requested_rpm;
	/* Plain RUN behaves like an ADAS green-light start, so subsequent lane
	 * events remain allowed to update the left/right RPM.  Exact numeric and
	 * turn commands retain manual mode and ignore lane/speed-sign overrides. */
	vcu.bluetooth_manual_control = (adas_run == false);

	if ((vcu.motors_started == false) &&
	    (vcu.motor_start_state == MOTOR_START_IDLE)) {
		if (MOTOR_RequestTargetRPM(&vcu.target_rpm) != false) {
			if (adas_run != false) {
				ResetLaneControlForADASRun();
			}
			ApplyBluetoothRunEnable();
			if (adas_run != false) {
				(void)APP_SetBaseRPM((uint16_t)rpm);
			}
			vcu.motor_start_state = MOTOR_START_WAIT_PARAM;
			if (turn_left != false) {
				xil_printf("[BT] LEFT TURN START REQUESTED L=%d R=%d\r\n", (int)requested_rpm.left_rpm, (int)requested_rpm.right_rpm);
			} else if (turn_right != false) {
				xil_printf("[BT] RIGHT TURN START REQUESTED L=%d R=%d\r\n", (int)requested_rpm.left_rpm, (int)requested_rpm.right_rpm);
			} else {
				xil_printf("[BT] %d RPM START REQUESTED\r\n", (int)rpm);
			}
		} else {
			xil_printf("[BT] ERROR: RPM QUEUE FAIL\r\n");
		}
	} else if (vcu.motors_started != false) {
		if (MOTOR_RequestTargetRPM(&vcu.target_rpm) != false) {
			if (adas_run != false) {
				ResetLaneControlForADASRun();
			}
			ApplyBluetoothRunEnable();
			if (adas_run != false) {
				(void)APP_SetBaseRPM((uint16_t)rpm);
			}
			if (turn_left != false) {
				xil_printf("[BT] LEFT TURN L=%d R=%d\r\n", (int)requested_rpm.left_rpm, (int)requested_rpm.right_rpm);
			} else if (turn_right != false) {
				xil_printf("[BT] RIGHT TURN L=%d R=%d\r\n", (int)requested_rpm.left_rpm, (int)requested_rpm.right_rpm);
			} else {
				xil_printf("[BT] TARGET=%d RPM\r\n", (int)rpm);
			}
		} else {
			xil_printf("[BT] ERROR: RPM QUEUE FAIL\r\n");
		}
	} else {
		xil_printf("[BT] ERROR: START IN PROGRESS\r\n");
	}
}

static void ProcessBluetoothInput(void)
{
	uint8_t data;
	bool command_complete = false;

	BLUETOOTH_ProcessTx();
	BLUETOOTH_ProcessRx();
	while (BLUETOOTH_ReadByte(&data) != false) {
		bluetooth_last_rx_tick = millis();
		if ((data == (uint8_t)'\r') || (data == (uint8_t)'\n')) {
			command_complete = (bluetooth_command_length != 0U);
		} else if ((data == (uint8_t)' ') || (data == (uint8_t)'\t')) {
			/* Ignore surrounding whitespace used by terminal applications. */
		} else if (bluetooth_command_length < BLUETOOTH_COMMAND_BUFFER_SIZE) {
			bluetooth_command[bluetooth_command_length] = data;
			bluetooth_command_length++;
		} else {
			bluetooth_command_length = 0U;
			xil_printf("[BT] ERROR: COMMAND TOO LONG\r\n");
		}
	}

	if ((bluetooth_command_length != 0U) &&
	    ((uint32_t)(millis() - bluetooth_last_rx_tick) >= BLUETOOTH_COMMAND_IDLE_MS)) {
		command_complete = true;
	}

	if (command_complete != false) {
		ProcessBluetoothCommand();
		bluetooth_command_length = 0U;
	}
}

/* =========================================================
 * CancelMotorStartSequence
 *
 * Description:
 * Cancels any pending SET_PARAM-to-RUN sequence and records
 * that the motors are not running.
 *
 * Parameters:
 * None.
 *
 * Return:
 * None.
 * ========================================================= */
static void CancelMotorStartSequence(void)
{
	vcu.motor_start_state = MOTOR_START_IDLE;

	vcu.controlled_stop_pending = false;

	vcu.motors_started = false;
}

/* =========================================================
 * ProcessMotorStartSequence
 *
 * Description:
 * Advances the non-blocking motor start sequence after both
 * Motor ECUs confirm each command through fresh status data.
 * SET_PARAM confirmation permits RUN to be requested, and RUN
 * confirmation permits the VCU to report the running state.
 *
 * Parameters:
 * None.
 *
 * Return:
 * None.
 * ========================================================= */
static void ProcessMotorStartSequence(void)
{
	MotorConfirmationState_t motor1_confirmation;

	MotorConfirmationState_t motor2_confirmation;

	motor1_confirmation = MOTOR_GetConfirmationState(ECU_MOTOR1);

	motor2_confirmation = MOTOR_GetConfirmationState(ECU_MOTOR2);

	switch (vcu.motor_start_state) {

	case MOTOR_START_WAIT_PARAM:

		if ((motor1_confirmation == MOTOR_CONFIRMATION_CONFIRMED) && (motor2_confirmation == MOTOR_CONFIRMATION_CONFIRMED)) {

			MOTOR_ClearConfirmation(ECU_MOTOR1);

			MOTOR_ClearConfirmation(ECU_MOTOR2);

			if (MOTOR_RequestRun() != false) {

				vcu.motor_start_state = MOTOR_START_WAIT_RUN;

				xil_printf("[ADAS] TARGET CONFIRMED - MOTOR RUN REQUESTED\r\n");
			} else {

				APP_SetVehicleEnabled(false);

				ReportFaultOnce(VCU_FAULT_MOTOR_RUN_QUEUE, 0U, 0U);
				LatchEmergencyStop(&vcu.emergency_stop_latched, &vcu.heartbeat);
			}
		}

		break;

	case MOTOR_START_WAIT_RUN:

		if ((motor1_confirmation == MOTOR_CONFIRMATION_CONFIRMED) && (motor2_confirmation == MOTOR_CONFIRMATION_CONFIRMED)) {

			MOTOR_ClearConfirmation(ECU_MOTOR1);

			MOTOR_ClearConfirmation(ECU_MOTOR2);

			vcu.motor_start_state = MOTOR_START_IDLE;

			vcu.motors_started = true;

			vcu.heartbeat.state = ECU_STATE_RUN;

			xil_printf("[ADAS] BOTH MOTORS RUNNING\r\n");
		}

		break;

	case MOTOR_START_IDLE:

	default:

		break;
	}
}

/* =========================================================
 * ProcessControlledStop
 *
 * Description:
 * Completes a normal controlled-stop sequence after both
 * Motor ECUs confirm that they reached a stopped state.
 *
 * Parameters:
 * None.
 *
 * Return:
 * None.
 * ========================================================= */
static void ProcessControlledStop(void)
{
	MotorConfirmationState_t motor1_confirmation;

	MotorConfirmationState_t motor2_confirmation;
	MotorStatus_t motor1_status = {0};
	MotorStatus_t motor2_status = {0};
	static uint32_t stop_retry_tick = 0U;
	bool motor1_stopped;
	bool motor2_stopped;

	if (vcu.controlled_stop_pending == false) {

		return;
	}

	motor1_confirmation = MOTOR_GetConfirmationState(ECU_MOTOR1);

	motor2_confirmation = MOTOR_GetConfirmationState(ECU_MOTOR2);

	motor1_stopped = (GetMotorStatus(ECU_MOTOR1, &motor1_status) != false) &&
		((motor1_status.state == CAN_MOTOR_STATE_ALIGNED) ||
		 (motor1_status.state == CAN_MOTOR_STATE_READY)) &&
		(motor1_status.target_rpm == 0) &&
		(MotorRpmDelta((uint16_t)((motor1_status.rpm < 0) ?
		 -(int32_t)motor1_status.rpm : motor1_status.rpm), 0U) <= 5U);
	motor2_stopped = (GetMotorStatus(ECU_MOTOR2, &motor2_status) != false) &&
		((motor2_status.state == CAN_MOTOR_STATE_ALIGNED) ||
		 (motor2_status.state == CAN_MOTOR_STATE_READY)) &&
		(motor2_status.target_rpm == 0) &&
		(MotorRpmDelta((uint16_t)((motor2_status.rpm < 0) ?
		 -(int32_t)motor2_status.rpm : motor2_status.rpm), 0U) <= 5U);

	if ((motor1_confirmation == MOTOR_CONFIRMATION_CONFIRMED) &&
	    (motor2_confirmation == MOTOR_CONFIRMATION_CONFIRMED) &&
	    (motor1_stopped != false) && (motor2_stopped != false)) {

		MOTOR_ClearConfirmation(ECU_MOTOR1);

		MOTOR_ClearConfirmation(ECU_MOTOR2);

		vcu.controlled_stop_pending = false;

		vcu.motors_started = false;

		vcu.heartbeat.state = ECU_STATE_READY;
		vcu.target_rpm.left_rpm = 0U;
		vcu.target_rpm.right_rpm = 0U;
		stop_retry_tick = 0U;

		xil_printf("[ADAS] BOTH MOTORS STOPPED\r\n");
	} else if ((uint32_t)(millis() - stop_retry_tick) >= 250U) {
		/* A confirmation alone is insufficient: one ECU can acknowledge while
		 * still reporting a nonzero target. Keep issuing the atomic pair STOP
		 * until both fresh status frames prove that both motors are stationary. */
		MOTOR_ClearConfirmation(ECU_MOTOR1);
		MOTOR_ClearConfirmation(ECU_MOTOR2);
		if (MOTOR_RequestStopBoth() != false)
			stop_retry_tick = millis();
	}
}

int APP_VCU_Init(void)
{
	CAN_BTR_t can_btr = {.brp = 9U, .tseg1 = 13U, .tseg2 = 2U, .sjw = 2U};

	SAFETY_DOMAIN_Config_t safety_config = {.wdt_timeout = 10000000U, .ref_window = 10000U, .min_count = 76000U, .max_count = 84000U, .ts_divider = 10000U, .reset_delay = 10U};

	/* A JTAG processor reset does not reset the FPGA safety peripheral. Clear
	 * retained watchdog/reset state before UART prints or lengthy startup work. */
	SAFETY_DOMAIN_PrepareForBoot(SAFETY_DOMAIN0);

	/* VCU runtime state initialization */
	vcu = (VCU_Context_t){0};
	bluetooth_command_length = 0U;
	bluetooth_last_rx_tick = 0U;

	vcu.heartbeat.state = ECU_STATE_READY;
	vcu.heartbeat.alive_counter = 0U;

	/* ZS-040 Bluetooth UART: 115200 baud, 8N1. */
	BLUETOOTH_Initialize();
	BLUETOOTH_SendString("\r\n[BT] VCU START\r\n");

	SAFETY_DOMAIN_Initialization(SAFETY_DOMAIN0, &safety_config);
	SAFETY_DOMAIN_WdtKick(SAFETY_DOMAIN0);
	SAFETY_DOMAIN_EnableMonitor(SAFETY_DOMAIN0);
	SAFETY_DOMAIN_EnableWdt(SAFETY_DOMAIN0);

	xil_printf("\r\n====================================\r\n VCU START\r\n====================================\r\n");

	CAN_Initialization(CAN0, &can_btr);

	GPIO_SetMode(GPIO, GPIO_OUTPUT);
	GPIO_WritePort(GPIO, 0U);

	TIM_Initialize(TIM1, TIMER_1MS_PSC, TIMER_1MS_ARR);
	TIM_Initialize(TIM2, TIMER_10MS_PSC, TIMER_10MS_ARR);

	if (SetupInterruptSystem() != XST_SUCCESS) {
		xil_printf("[ERROR] INTERRUPT INIT FAIL\r\n");
		return -1;
	}

	TIM_StartTimer(TIM1);
	TIM_StartTimer(TIM2);

	return 0;
}

void APP_VCU_Run(void)
{
	uint32_t current_tick;

	uint32_t motor1_status_update_count = 0U;
	uint32_t motor2_status_update_count = 0U;

	bool motor1_update_count_valid = false;
	bool motor2_update_count_valid = false;

	bool motor1_align_status_fresh = false;
	bool motor2_align_status_fresh = false;

	bool motor1_status_valid = false;
	bool motor2_status_valid = false;

	bool all_heartbeats_healthy = false;
	bool both_motors_aligned = false;
	bool startup_alignment_complete = false;

	/* Capture the rising transmit-error counter before automatic CAN recovery
	 * clears the hardware fields. This is diagnostic-only and sends no CAN. */
	CaptureCanTxDiagnostics();

	/*=========================================================
	 * Safety Domain IRQ
	 *=========================================================*/
	if (safety_fault_irq_flag != false) {
		safety_fault_irq_flag = false;

		if (SAFETY_DOMAIN_IsFaultIrq(SAFETY_DOMAIN0) &&
		    (CAN_IsBusOff(CAN0) != false)) {
			/* Auto-release only when this bus-off event created the E-stop. */
			vcu.bus_off_estop_owned = (vcu.emergency_stop_latched == false);
			vcu.bus_off_recovery_active = true;
			vcu.bus_off_start_tick = millis();

			ReportFaultOnce(VCU_FAULT_CAN_BUS_OFF,
			                SAFETY_DOMAIN_GetStatus(SAFETY_DOMAIN0), CAN0->CAN_SR);

			LatchEmergencyStop(&vcu.emergency_stop_latched, &vcu.heartbeat);
		} else {
			/* The FPGA fault output is level-sensitive.  CAN may have already
			 * completed its 128x11-bit recovery before software services the
			 * interrupt; do not create a new E-stop from that stale level. */
			SAFETY_DOMAIN_FaultClear(SAFETY_DOMAIN0);
			xil_printf("[WARN] STALE SAFETY IRQ CLEARED SR=0x%x CAN_SR=0x%x\r\n",
			           (unsigned int)SAFETY_DOMAIN_GetStatus(SAFETY_DOMAIN0),
			           (unsigned int)CAN0->CAN_SR);
			SafetyDomain_EnableInterrupt();
		}
	}

	/*=========================================================
	 * Bus-Off Recovery
	 *=========================================================*/
	if ((vcu.bus_off_recovery_active != false) && (vcu.watchdog_reset_pending == false)) {

		if (CAN_IsBusOff(CAN0) == false) {

			SAFETY_DOMAIN_FaultClear(SAFETY_DOMAIN0);

			if (SAFETY_DOMAIN_IsFaultIrq(SAFETY_DOMAIN0) == false) {

				vcu.bus_off_recovery_active = false;

				SafetyDomain_EnableInterrupt();

				if (vcu.bus_off_estop_owned != false) {
					/* Restart through the normal fault-reset and alignment gate. */
					vcu.bus_off_estop_owned = false;
					vcu.emergency_stop_latched = false;
					vcu.estop_confirmation_started = false;
					vcu.estop_confirmed = false;
					vcu.drive_gate_opened = false;
					vcu.startup_fault_reset_sent = false;
					vcu.startup_align_requested = false;
					vcu.heartbeat_timeout_led_mask = 0U;
					vcu.reported_fault = VCU_FAULT_NONE;
					vcu.heartbeat.state = ECU_STATE_READY;
					CancelMotorStartSequence();
					xil_printf("[RECOVERY] CAN BUS-OFF CLEARED - STARTUP RECHECK\r\n");
				} else {
					xil_printf("[RECOVERY] CAN BUS-OFF CLEARED; PREVIOUS E-STOP REMAINS\r\n");
				}
			}
		} else if ((uint32_t)(millis() - vcu.bus_off_start_tick) >= BUS_OFF_RESET_DELAY_MS) {

			vcu.watchdog_reset_pending = true;

			SAFETY_DOMAIN_SetWdtTimeout(SAFETY_DOMAIN0, 1U);

			/* The original CAN_BUS_OFF line already contains the root cause. */
		}
	}

	/*=========================================================
	 * TIM1 phase 0
	 * CAN RX
	 *=========================================================*/
	/* Drain RX on every superloop pass. The old 10 ms batching allowed UART
	 * diagnostics to fill the software queue before the next scheduled pass. */
	tim_can_rx_flag = false;
	CAN_ProcessRx(millis());

	/*=========================================================
	 * TIM1 phase 1
	 * Fault / Heartbeat / Startup Check
	 *=========================================================*/
	if (tim_fault_check_flag != false) {

		tim_fault_check_flag = false;

		current_tick = millis();

		MOTOR_ProcessConfirmation(current_tick);

		ProcessFaultClear();

		ProcessMotorStartSequence();

		ProcessControlledStop();

		/*-----------------------------------------------------
		 * Motor Command Confirmation
		 *-----------------------------------------------------*/
		{
			MotorConfirmationState_t motor1_confirmation = MOTOR_GetConfirmationState(ECU_MOTOR1);

			MotorConfirmationState_t motor2_confirmation = MOTOR_GetConfirmationState(ECU_MOTOR2);

			if ((motor1_confirmation == MOTOR_CONFIRMATION_TIMEOUT) || (motor2_confirmation == MOTOR_CONFIRMATION_TIMEOUT)) {
				if (vcu.fault_clear_pending != false) {
					vcu.fault_clear_pending = false;
					vcu.estop_confirmed = false;
				}

				if (vcu.emergency_stop_latched == false) {

					/* TEMP TEST: MOTOR_COMMAND_TIMEOUT fault handling disabled.
					 * ReportFaultOnce(VCU_FAULT_MOTOR_COMMAND_TIMEOUT,
					 *                 (uint32_t)motor1_confirmation,
					 *                 (uint32_t)motor2_confirmation);
					 * LatchEmergencyStop(&vcu.emergency_stop_latched, &vcu.heartbeat);
					 */
				}

				MOTOR_ClearConfirmation(ECU_MOTOR1);
				MOTOR_ClearConfirmation(ECU_MOTOR2);
			} else if ((motor1_confirmation == MOTOR_CONFIRMATION_CONFIRMED) && (motor2_confirmation == MOTOR_CONFIRMATION_CONFIRMED)) {

				MOTOR_ClearConfirmation(ECU_MOTOR1);
				MOTOR_ClearConfirmation(ECU_MOTOR2);
			}
		}

		/*-----------------------------------------------------
		 * Heartbeat State
		 *-----------------------------------------------------*/
		vcu.received_heartbeat_mask = CAN_GetHeartbeatReceivedMask();

		vcu.healthy_heartbeat_mask = CAN_GetHeartbeatHealthyMask(current_tick);

		all_heartbeats_healthy = ((vcu.healthy_heartbeat_mask & REQUIRED_HEARTBEAT_MASK) == REQUIRED_HEARTBEAT_MASK);

		if (vcu.startup_align_requested != false) {
		
			motor1_update_count_valid = GetMotorStatusUpdateCount(ECU_MOTOR1,&motor1_status_update_count);
		
			motor2_update_count_valid = GetMotorStatusUpdateCount(ECU_MOTOR2,&motor2_status_update_count);
		
			if (motor1_update_count_valid != false) {
			
				motor1_align_status_fresh = (motor1_status_update_count != vcu.startup_align_motor1_baseline);
			}
		
			if (motor2_update_count_valid != false) {
			
				motor2_align_status_fresh = (motor2_status_update_count != vcu.startup_align_motor2_baseline);
			}
		
			motor1_status_valid = GetMotorStatus(ECU_MOTOR1,&vcu.motor1_status);
		
			motor2_status_valid = GetMotorStatus(ECU_MOTOR2,&vcu.motor2_status);
		
			if ((motor1_status_valid != false) && (motor2_status_valid != false)) {
			
				both_motors_aligned = (vcu.motor1_status.state == CAN_MOTOR_STATE_ALIGNED) && (vcu.motor2_status.state == CAN_MOTOR_STATE_ALIGNED);
			}
		
			startup_alignment_complete = (motor1_align_status_fresh != false) && (motor2_align_status_fresh != false) && (all_heartbeats_healthy != false) && (both_motors_aligned != false);
		}

		/*-----------------------------------------------------
		 * Startup Sequence
		 *-----------------------------------------------------*/
		if (vcu.drive_gate_opened == false) {

			/*
			 * Step 1
			 * Wait until every ECU sends heartbeat once.
			 */
			if ((vcu.startup_fault_reset_sent == false) && ((vcu.received_heartbeat_mask & REQUIRED_HEARTBEAT_MASK) == REQUIRED_HEARTBEAT_MASK)) {

				if (SendGlobalCommand(GB_FAULT_RESET) != false) {

					vcu.startup_fault_reset_sent = true;

					vcu.can_fault_led_mask = 0U;

					(void)CAN_ClearFaultFlag(ECU_MOTOR1);
					(void)CAN_ClearFaultFlag(ECU_MOTOR2);
					(void)CAN_ClearFaultFlag(ECU_ADAS1);
					(void)CAN_ClearFaultFlag(ECU_ADAS2);

					xil_printf("[STARTUP] GLOBAL FAULT RESET QUEUED\r\n");
				} else {

					xil_printf("[ERROR] GLOBAL FAULT RESET QUEUE FAIL\r\n");
				}
			}

			/*
			 * Step 2
			 * Every ECU must be healthy.
			 * Request motor alignment.
			 */
			else if ((vcu.startup_fault_reset_sent != false) && (vcu.startup_align_requested == false) && ((vcu.healthy_heartbeat_mask & REQUIRED_HEARTBEAT_MASK) == REQUIRED_HEARTBEAT_MASK)) {

				if (MOTOR_RequestAlign() != false) {
					
					(void)GetMotorStatusUpdateCount(ECU_MOTOR1,&vcu.startup_align_motor1_baseline);
					(void)GetMotorStatusUpdateCount(ECU_MOTOR2,&vcu.startup_align_motor2_baseline);

					vcu.startup_align_requested = true;

					xil_printf("[STARTUP] MOTOR ALIGN REQUESTED\r\n");
				} else {

					xil_printf("[ERROR] STARTUP MOTOR ALIGN QUEUE FAIL\r\n");
				}
			}

			/*
			 * Step 3
			 * Both motors aligned -> Enable control.
			 */
			else if ((vcu.startup_align_requested != false) && (vcu.emergency_stop_latched == false) && (startup_alignment_complete != false)) {

				CAN_ResetADASControlData();

				vcu.drive_gate_opened = true;

				vcu.control_gate_open_tick = current_tick;

				xil_printf("[READY] ALL ECU HEARTBEATS HEALTHY / MOTORS ALIGNED - CONTROL ENABLED\r\n");
			}
		}

		/*-----------------------------------------------------
		 * Heartbeat Lost after Drive Gate Open
		 *-----------------------------------------------------*/
		else if ((vcu.healthy_heartbeat_mask & REQUIRED_HEARTBEAT_MASK) != REQUIRED_HEARTBEAT_MASK) {

			ReportFaultOnce(VCU_FAULT_HEARTBEAT_INVALID,
			                REQUIRED_HEARTBEAT_MASK & ~vcu.healthy_heartbeat_mask,
			                vcu.healthy_heartbeat_mask);

			LatchEmergencyStop(&vcu.emergency_stop_latched, &vcu.heartbeat);
		}

		/*-----------------------------------------------------
		 * CAN RX / ADAS Result Supervision
		 *-----------------------------------------------------*/
		if ((vcu.drive_gate_opened != false) && (vcu.emergency_stop_latched == false)) {
			bool adas1_received;
			bool adas2_received;
			bool adas1_timed_out;
			bool adas2_timed_out;

			if (CAN_IsRxQueueFaulted() != false) {
				ReportFaultOnce(VCU_FAULT_CAN_RX_QUEUE_OVERFLOW, 0U, 0U);
				LatchEmergencyStop(&vcu.emergency_stop_latched, &vcu.heartbeat);
			}

			adas1_received = CAN_GetADASResultLastTick(ECU_ADAS1, &vcu.adas1_result_last_tick);
			adas2_received = CAN_GetADASResultLastTick(ECU_ADAS2, &vcu.adas2_result_last_tick);

			adas1_timed_out = false;
			adas2_timed_out = false;
#if (ADAS1_ECU_CONNECTED != 0U)
			adas1_timed_out = adas1_received ? ((uint32_t)(current_tick - vcu.adas1_result_last_tick) >= ADAS_RESULT_TIMEOUT_MS) : ((uint32_t)(current_tick - vcu.control_gate_open_tick) >= ADAS_RESULT_TIMEOUT_MS);
#else
			(void)adas1_received;
#endif
#if (ADAS2_ECU_CONNECTED != 0U)
			adas2_timed_out = adas2_received ? ((uint32_t)(current_tick - vcu.adas2_result_last_tick) >= ADAS_RESULT_TIMEOUT_MS) : ((uint32_t)(current_tick - vcu.control_gate_open_tick) >= ADAS_RESULT_TIMEOUT_MS);
#else
			(void)adas2_received;
#endif

			if ((adas1_timed_out != false) || (adas2_timed_out != false)) {
				ReportFaultOnce(VCU_FAULT_ADAS_RESULT_TIMEOUT,
				                (uint32_t)adas1_timed_out, (uint32_t)adas2_timed_out);
				LatchEmergencyStop(&vcu.emergency_stop_latched, &vcu.heartbeat);
			}
		}

		/*-----------------------------------------------------
		 * CAN Fault
		 *-----------------------------------------------------*/
		vcu.can_fault_led_mask |= GetCanFaultLedMask();

		if ((vcu.can_fault_led_mask != 0U) && (vcu.startup_fault_reset_sent != false)) {

			GPIO_WritePort(GPIO, vcu.can_fault_led_mask | vcu.heartbeat_timeout_led_mask);

			ReportFaultOnce(VCU_FAULT_CAN_ECU, vcu.can_fault_led_mask, 0U);

			LatchEmergencyStop(&vcu.emergency_stop_latched, &vcu.heartbeat);
		}
	}

	/* Poll only after the higher-priority safety and fault handling above.
	 * Commands are completed by CR/LF or a short receive-idle interval. */
	ProcessBluetoothInput();

	/*=========================================================
	 * TIM1 phase 2
	 * ADAS / Motor Control
	 *=========================================================*/
	if (tim_control_flag != false) {

		tim_control_flag = false;

		if ((vcu.drive_gate_opened != false) && (vcu.emergency_stop_latched == false)) {

			Event_t event = {0};

			APP_UpdateADAS();

			if ((APP_IsEventQueueFaulted() != false) || (CAN_IsADASEventQueueFaulted() != false)) {
				bool app_queue_fault = APP_IsEventQueueFaulted();
				bool can_queue_fault = CAN_IsADASEventQueueFaulted();

				ReportFaultOnce(VCU_FAULT_ADAS_EVENT_QUEUE_OVERFLOW,
				                (uint32_t)app_queue_fault, (uint32_t)can_queue_fault);

				LatchEmergencyStop(&vcu.emergency_stop_latched, &vcu.heartbeat);
			}

			while ((vcu.emergency_stop_latched == false) && (GetNextEvent(&event) != false)) {
				/* In Bluetooth manual mode, retain only the ADAS stop command.
				 * This keeps red-light stopping active without allowing lane or
				 * speed-sign events to overwrite the requested exact RPM. */
				if ((vcu.bluetooth_manual_control != false) &&
				    (event.type != EVENT_MOTOR_STOP)) {
					continue;
				}
				/* A green frame is only permission to start; it cannot bypass the
				 * alignment, stopped-motor, heartbeat, CAN, or Safety Domain gates. */
				if ((event.type == EVENT_MOTOR_RUN) &&
				    (CanAcceptGreenAutoStart() == false)) {
					continue;
				}

				vcu.target_rpm = APP_ProcessEvent(&event);

				/*---------------------------------------------
				 * MOTOR STOP
				 *---------------------------------------------*/
				if (event.type == EVENT_MOTOR_STOP) {

					if (((vcu.motors_started != false) || (vcu.motor_start_state != MOTOR_START_IDLE)) && (vcu.controlled_stop_pending == false)) {

						if (MOTOR_RequestStopBoth() != false) {

							vcu.motor_start_state = MOTOR_START_IDLE;

							vcu.controlled_stop_pending = true;

							xil_printf("[ADAS] RED - MOTOR STOP REQUESTED\r\n");
						} else {

							ReportFaultOnce(VCU_FAULT_MOTOR_STOP_QUEUE, 0U, 0U);
							LatchEmergencyStop(&vcu.emergency_stop_latched, &vcu.heartbeat);
						}
					}
				}

				/*---------------------------------------------
				 * MOTOR RUN
				 *---------------------------------------------*/
				else if (event.type == EVENT_MOTOR_RUN) {

					if ((vcu.motors_started == false) && (vcu.motor_start_state == MOTOR_START_IDLE)) {

						if (MOTOR_RequestTargetRPM(&vcu.target_rpm) != false) {

							vcu.motor_start_state = MOTOR_START_WAIT_PARAM;

							xil_printf("[ADAS] MOTOR TARGET RPM REQUESTED\r\n");
						} else {

							APP_SetVehicleEnabled(false);

							xil_printf("[ERROR] GREEN RESTART QUEUE FAIL\r\n");
						}
					}
				}

				/*---------------------------------------------
				 * Update RPM
				 *---------------------------------------------*/
				else if (vcu.motors_started != false) {

					if (RequestRateLimitedAutoTarget(&vcu.target_rpm) == false) {

						xil_printf("[ERROR] ADAS MOTOR PARAM QUEUE FAIL\r\n");
					}
				}
			}
		}

		/* Accept the same line commands on the PC UART as on Bluetooth. This
		 * keeps one command implementation for FAULT CLEAR, AUTO and STOP. */
		if (uart_rx_flag != false) {
			uint8_t data = uart_rx_data;
			uart_rx_flag = false;
			bluetooth_last_rx_tick = millis();
			if ((data == (uint8_t)'\r') || (data == (uint8_t)'\n')) {
				if (bluetooth_command_length != 0U) {
					ProcessBluetoothCommand();
					bluetooth_command_length = 0U;
				}
			} else if ((data != (uint8_t)' ') && (data != (uint8_t)'\t')) {
				if (bluetooth_command_length < BLUETOOTH_COMMAND_BUFFER_SIZE) {
					bluetooth_command[bluetooth_command_length++] = data;
				} else {
					bluetooth_command_length = 0U;
					xil_printf("[UART] ERROR: COMMAND TOO LONG\r\n");
				}
			}
		}
	}

	/*=========================================================
	 * TIM1
	 * CAN TX
	 *=========================================================*/
	if (tim_can_tx_flag != false) {

		tim_can_tx_flag = false;

		CAN_ProcessTx();
	}

	/*=========================================================
	 * TIM2 phase 0
	 * Heartbeat Timeout
	 *=========================================================*/
	if (tim_heartbeat_timeout_flag != false) {

		tim_heartbeat_timeout_flag = false;

		vcu.timeout_mask = CAN_UpdateHeartbeatTimeouts(millis());

#if (ADAS1_ECU_CONNECTED == 0U)
		vcu.timeout_mask &= (uint32_t)~CAN_TIMEOUT_ADAS1;
#endif

#if (ADAS2_ECU_CONNECTED == 0U)
		vcu.timeout_mask &= (uint32_t)~CAN_TIMEOUT_ADAS2;
#endif

#if (MOTOR1_ECU_CONNECTED == 0U)
		vcu.timeout_mask &= (uint32_t)~CAN_TIMEOUT_MOTOR1;
#endif

#if (MOTOR2_ECU_CONNECTED == 0U)
		vcu.timeout_mask &= (uint32_t)~CAN_TIMEOUT_MOTOR2;
#endif

		if (vcu.drive_gate_opened != false) {
			vcu.heartbeat_timeout_led_mask |= GetHeartbeatTimeoutLedMask(vcu.timeout_mask);
		} else {
			vcu.heartbeat_timeout_led_mask = GetHeartbeatTimeoutLedMask(vcu.timeout_mask);
		}
		GPIO_WritePort(GPIO, vcu.can_fault_led_mask | vcu.heartbeat_timeout_led_mask);

		if ((vcu.drive_gate_opened != false) && (vcu.timeout_mask != 0U)) {
			ReportFaultOnce(VCU_FAULT_HEARTBEAT_INVALID,
			                vcu.timeout_mask, vcu.healthy_heartbeat_mask);
			LatchEmergencyStop(&vcu.emergency_stop_latched, &vcu.heartbeat);
		}
	}

	/*=========================================================
	 * TIM2 phase 1
	 * VCU Heartbeat TX
	 *=========================================================*/
	if (tim_heartbeat_tx_flag != false) {
		tim_heartbeat_tx_flag = false;

		if (SendHeartbeat(ECU_VCU, &vcu.heartbeat) != false) {
			vcu.heartbeat.alive_counter++;
		} else {
			xil_printf("[ERROR] HEARTBEAT QUEUE FAIL\r\n");
		}
	}

	/*=========================================================
	 * TIM2 phase 2
	 * Watchdog
	 *=========================================================*/
	if (tim_wdt_kick_flag != false) {
		tim_wdt_kick_flag = false;

		if (vcu.watchdog_reset_pending == false) {
			SAFETY_DOMAIN_WdtKick(SAFETY_DOMAIN0);
		}
	}

	/*=========================================================
	 * TIM2 phase 3
	 * Diagnostics
	 *=========================================================*/
	if (tim_diagnostics_flag != false) {
		tim_diagnostics_flag = false;
		vcu.debug_count++;

		/* Lightweight 5 Hz lane trace for the Bluetooth tuning console.
		 * Keep the full diagnostic burst at 1 Hz so observing the lane does
		 * not add meaningful load to the control/CAN paths. */
		if ((vcu.debug_count % 2U) == 0U) {
			ADASResult_t lane_result = {0};
			uint32_t lane_tick = 0U;
			uint32_t lane_now = millis();
			uint16_t lane_raw = 0U;
			uint16_t lane_used = 0U;
			int32_t lane_error_fast = 0;
			bool lane_valid_fast = false;
			bool curve_preview_fast = false;

			if (CAN_PeekADASResult(ECU_ADAS1, &lane_result, &lane_tick) != false) {
				lane_valid_fast = ((lane_now - lane_tick) < ADAS_RESULT_TIMEOUT_MS);
			}
			APP_GetLaneFilterState(&lane_raw, &lane_used, NULL, NULL);
			APP_GetLaneControlState(NULL, &lane_error_fast, NULL, NULL, NULL);
			APP_GetLanePreviewState(NULL, NULL, &curve_preview_fast);
			BLUETOOTH_SetLogMirrorEnabled(false);
			xil_printf("[LANE200] X=%d RAW=%d USED=%d ERR=%d VALID=%d AGE=%dms FAR=%d NEAR=%d FV=%d NV=%d PV=%d\r\n",
			           (int)lane_result.lanex, (int)lane_raw, (int)lane_used,
			           (int)lane_error_fast, (int)lane_valid_fast,
			           (int)(lane_now - lane_tick), (int)lane_result.far_x,
			           (int)lane_result.near_x,
			           (int)lane_result.far_valid,
			           (int)lane_result.near_valid,
			           (int)curve_preview_fast);
			BLUETOOTH_SetLogMirrorEnabled(true);
		}

		if (vcu.debug_count >= 10U) {
			ADASResult_t adas_result = {0};
			ECU_STATE_t adas_state = ECU_STATE_INIT;
			uint32_t adas_tick = 0U;
			uint32_t now = millis();
			bool adas_valid = false;
			uint16_t lane_base_rpm = 0U;
			int32_t lane_error = 0;
			int32_t lane_pd = 0;
			int32_t lane_pd_requested = 0;
			int32_t lane_rate = 0;
			int32_t lane_kp = 0;
			int32_t lane_kd = 0;
			uint16_t raw_lane_x = 0U;
			uint16_t used_lane_x = 0U;
			uint8_t lane_filter = LANE_FILTER_TRACK;
			uint8_t lane_confirm_count = 0U;
			uint16_t far_lane_x = 0U;
			uint16_t near_lane_x = 0U;
			bool curve_preview = false;
			LaneTuning_t lane_tuning_state;
			LaneDiagnostics_t lane_diagnostics;

			vcu.debug_count = 0U;
			BLUETOOTH_SetLogMirrorEnabled(false);
			xil_printf("[TICK] %d ms\r\n", (int)now);

#if (ADAS1_ECU_CONNECTED != 0U)
			if (CAN_PeekADASResult(ECU_ADAS1, &adas_result, &adas_tick) != false) {
				adas_valid = ((now - adas_tick) < ADAS_RESULT_TIMEOUT_MS);
				bool heartbeat_seen = CAN_GetHeartbeatState(ECU_ADAS1, &adas_state);
				xil_printf("[ADAS1] HB=%d STATE=%d VALID=%d AGE=%dms LANE=%d FAR=%d NEAR=%d DUAL=%d OBJ=%d Y0=%d Y1=%d\r\n",
				           (int)heartbeat_seen, (int)adas_state,
				           (int)((now - adas_tick) < ADAS_RESULT_TIMEOUT_MS),
				           (int)(now - adas_tick), (int)adas_result.lanex,
				           (int)adas_result.far_x, (int)adas_result.near_x,
				           (int)adas_result.dual_lane_valid,
				           (int)adas_result.object_type, (int)adas_result.y0, (int)adas_result.y1);
			} else {
				xil_printf("[ADAS1] NO RESULT\r\n");
			}
#endif

#if (ADAS2_ECU_CONNECTED != 0U)
			if (CAN_PeekADASResult(ECU_ADAS2, &adas_result, &adas_tick) != false) {
				bool heartbeat_seen = CAN_GetHeartbeatState(ECU_ADAS2, &adas_state);
				xil_printf("[ADAS2] HB=%d STATE=%d VALID=%d AGE=%dms LANE=%d OBJ=%d Y0=%d Y1=%d\r\n",
				           (int)heartbeat_seen, (int)adas_state,
				           (int)((now - adas_tick) < ADAS_RESULT_TIMEOUT_MS),
				           (int)(now - adas_tick), (int)adas_result.lanex,
				           (int)adas_result.object_type, (int)adas_result.y0, (int)adas_result.y1);
			} else {
				xil_printf("[ADAS2] NO RESULT\r\n");
			}
#endif
			BLUETOOTH_SetLogMirrorEnabled(true);

			/* Compact Bluetooth telemetry consumed by hc06_test.py.  Keep this
			 * line stable: it carries actual/target RPM and the latest ADAS data. */
			(void)GetMotorStatus(ECU_MOTOR1, &vcu.motor1_status);
			(void)GetMotorStatus(ECU_MOTOR2, &vcu.motor2_status);
			APP_GetLanePreviewState(&far_lane_x, &near_lane_x, &curve_preview);
			APP_GetLaneDiagnostics(&lane_diagnostics);
			xil_printf("[TEL] L=%d R=%d TL=%d TR=%d LANE=%d F=%d N=%d FV=%d NV=%d CR=%d CF=%d DIR=%d ST=%d OBJ=%d Y0=%d Y1=%d VALID=%d\r\n",
			           (int)vcu.motor1_status.rpm, (int)vcu.motor2_status.rpm,
			           (int)vcu.motor1_status.target_rpm, (int)vcu.motor2_status.target_rpm,
			           (int)adas_result.lanex,
			           (int)adas_result.far_x, (int)adas_result.near_x,
			           (int)adas_result.far_valid, (int)adas_result.near_valid,
			           (int)lane_diagnostics.curve_raw,
			           (int)lane_diagnostics.curve_filtered,
			           (int)((lane_diagnostics.curve_filtered < 0) ? -1 :
			                 ((lane_diagnostics.curve_filtered > 0) ? 1 : 0)),
			           (int)lane_diagnostics.state,
			           (int)adas_result.object_type,
			           (int)adas_result.y0, (int)adas_result.y1, (int)adas_valid);
			APP_GetLaneControlState(&lane_base_rpm, &lane_error, &lane_pd,
					&lane_kp, &lane_kd);
			APP_GetLaneFilterState(&raw_lane_x, &used_lane_x,
					&lane_filter, &lane_confirm_count);
			APP_GetLaneSlewState(&lane_pd_requested, NULL);
			APP_GetLaneTuning(&lane_tuning_state);
			lane_rate = APP_GetLaneRate();
			BLUETOOTH_SetLogMirrorEnabled(false);
			xil_printf("[TUNE] BASE=%d RAW=%d USED=%d LF=%d LC=%d SM=%d LM=%d LIM=%d ERR=%d RATE=%d PDR=%d PD=%d KP100=%d KD100=%d TL=%d TR=%d FAR=%d NEAR=%d PV=%d\r\n",
			           (int)lane_base_rpm, (int)raw_lane_x, (int)used_lane_x,
			           (int)lane_filter, (int)lane_confirm_count,
			           (int)lane_tuning_state.slew_mode,
			           (int)lane_tuning_state.limit_mode,
			           (int)lane_tuning_state.max_correction_rpm,
			           (int)lane_error, (int)lane_rate,
			           (int)lane_pd_requested, (int)lane_pd,
			           (int)lane_kp, (int)lane_kd,
			           (int)vcu.target_rpm.left_rpm, (int)vcu.target_rpm.right_rpm,
			           (int)far_lane_x, (int)near_lane_x, (int)curve_preview);
			xil_printf("[PATH] ST=%d CONF=%d DIFF=%d CR=%d CF=%d PC=%d FB=%d FF=%d TOT=%d SPD=%d DIST=%d IDX=%d BLEND=%d MTO=%d\r\n",
			           (int)lane_diagnostics.state,
			           (int)lane_diagnostics.confidence,
			           (int)lane_diagnostics.far_near_diff,
			           (int)lane_diagnostics.curve_raw,
			           (int)lane_diagnostics.curve_filtered,
			           (int)lane_diagnostics.path_curve,
			           (int)lane_diagnostics.steer_feedback,
			           (int)lane_diagnostics.steer_feedforward,
			           (int)lane_diagnostics.steer_total,
			           (int)lane_diagnostics.speed_scale_percent,
			           (int)lane_diagnostics.distance_mm,
			           (int)lane_diagnostics.path_index,
			           (int)lane_diagnostics.recover_blend_percent,
			           (int)lane_diagnostics.memory_timed_out);
			xil_printf("[GATE] OPEN=%d RX=0x%x HEALTHY=0x%x REQ=0x%x RESET=%d ALIGNREQ=%d M1STATE=%d M2STATE=%d\r\n",
			           (int)vcu.drive_gate_opened,
			           (unsigned int)vcu.received_heartbeat_mask,
			           (unsigned int)vcu.healthy_heartbeat_mask,
			           (unsigned int)REQUIRED_HEARTBEAT_MASK,
			           (int)vcu.startup_fault_reset_sent,
			           (int)vcu.startup_align_requested,
			           (int)vcu.motor1_status.state,
			           (int)vcu.motor2_status.state);
			BLUETOOTH_SetLogMirrorEnabled(true);
		}

		/* Offset the PC-only motor trace from the larger one-second telemetry
		 * burst and keep it at 1 Hz to bound UART blocking time. */
		if ((vcu.debug_count == 5U) &&
		    (GetMotorStatus(ECU_MOTOR1, &vcu.motor1_status) != false)) {
			/* Keep the 10 Hz motor trace on the fast PC UART only. */
			BLUETOOTH_SetLogMirrorEnabled(false);
			xil_printf("[MOTOR1] RPM=%d TEMP=%d VOLT=%d\r\n", (int)vcu.motor1_status.rpm, (int)vcu.motor1_status.temp, (int)vcu.motor1_status.voltage);
			BLUETOOTH_SetLogMirrorEnabled(true);
		}

		if ((vcu.debug_count == 5U) &&
		    (GetMotorStatus(ECU_MOTOR2, &vcu.motor2_status) != false)) {
			BLUETOOTH_SetLogMirrorEnabled(false);
			xil_printf("[MOTOR2] RPM=%d TEMP=%d VOLT=%d\r\n", (int)vcu.motor2_status.rpm, (int)vcu.motor2_status.temp, (int)vcu.motor2_status.voltage);
			BLUETOOTH_SetLogMirrorEnabled(true);
		}
	}

	/*=========================================================
	 * Emergency Stop
	 *=========================================================*/
	if (vcu.emergency_stop_latched != false) {
		uint32_t now = millis();

		uint32_t motor1_update_count = 0U;
		uint32_t motor2_update_count = 0U;

		bool motor1_status_new;
		bool motor2_status_new;

		bool motor1_fault_confirmed = false;
		bool motor2_fault_confirmed = false;

		/*-----------------------------------------------------
		 * Start E-Stop Confirmation
		 *-----------------------------------------------------*/
		if (vcu.estop_confirmation_started == false) {
			vcu.estop_confirmation_started = true;
			vcu.estop_start_tick = now;
			vcu.estop_last_retry_tick = now - ESTOP_RETRY_INTERVAL_MS;

			(void)GetMotorStatusUpdateCount(ECU_MOTOR1, &vcu.estop_motor1_status_baseline);
			(void)GetMotorStatusUpdateCount(ECU_MOTOR2, &vcu.estop_motor2_status_baseline);
		}

		/*-----------------------------------------------------
		 * Check New Motor Status
		 *-----------------------------------------------------*/
		motor1_status_new = GetMotorStatusUpdateCount(ECU_MOTOR1, &motor1_update_count) && (motor1_update_count != vcu.estop_motor1_status_baseline);
		motor2_status_new = GetMotorStatusUpdateCount(ECU_MOTOR2, &motor2_update_count) && (motor2_update_count != vcu.estop_motor2_status_baseline);

		if ((motor1_status_new != false) && (GetMotorStatus(ECU_MOTOR1, &vcu.motor1_status) != false)) {
			motor1_fault_confirmed = (vcu.motor1_status.state == CAN_MOTOR_STATE_FAULT);
		}

		if ((motor2_status_new != false) && (GetMotorStatus(ECU_MOTOR2, &vcu.motor2_status) != false)) {
			motor2_fault_confirmed = (vcu.motor2_status.state == CAN_MOTOR_STATE_FAULT);
		}

		/*-----------------------------------------------------
		 * E-Stop Confirmed
		 *-----------------------------------------------------*/
		if ((vcu.estop_confirmed == false) && (motor1_fault_confirmed != false) && (motor2_fault_confirmed != false)) {
			vcu.estop_confirmed = true;
			xil_printf("[SAFETY] E-STOP CONFIRMED BY BOTH MOTORS\r\n");
		}

		/*-----------------------------------------------------
		 * Retry Emergency Stop
		 *-----------------------------------------------------*/
		if ((vcu.estop_confirmed == false) && ((uint32_t)(now - vcu.estop_last_retry_tick) >= ESTOP_RETRY_INTERVAL_MS)) {
			vcu.estop_last_retry_tick = now;
			if (SendEmergencyStopCommand() == false) {
				xil_printf("[ERROR] E-STOP RETRY SEND FAIL\r\n");
			}
		}

		/*-----------------------------------------------------
		 * E-Stop Confirmation Timeout
		 *-----------------------------------------------------*/
		if ((vcu.estop_confirmed == false) && (vcu.watchdog_reset_pending == false) && ((uint32_t)(now - vcu.estop_start_tick) >= ESTOP_CONFIRM_TIMEOUT_MS)) {
			vcu.watchdog_reset_pending = true;
			SAFETY_DOMAIN_SetWdtTimeout(SAFETY_DOMAIN0, 1U);
			ReportFaultOnce(VCU_FAULT_ESTOP_CONFIRM_TIMEOUT,
			                (uint32_t)vcu.motor1_status.state,
			                (uint32_t)vcu.motor2_status.state);
		}
	}
}
