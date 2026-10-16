#include "motor.h"

#include <stddef.h>

#include "../can/can.h"

/* The two motors face opposite directions on some vehicle revisions.
 * Keep the mapping here so application/test code is hardware-independent. */
#define MOTOR1_DIRECTION MOTOR_DIR_FORWARD
#define MOTOR2_DIRECTION MOTOR_DIR_FORWARD
/* Each STM32 performs a fresh 300 ms electrical alignment after STOP before
 * reporting RUNNING. Allow for command delivery, status-frame scheduling and
 * normal CAN arbitration so ALIGNING is not mistaken for a failed start. */
#define MOTOR_CONFIRMATION_TIMEOUT_MS (1000U)

typedef struct
{
	MotorConfirmationState_t state;
	MotorCommand_t command;

	int16_t expected_target_rpm;
	uint32_t baseline_update_count;
	uint32_t wait_started_tick;

	bool target_check_required;
	bool timer_started;
} MotorConfirmationContext_t;

static MotorConfirmationContext_t motor1_confirmation = {0};
static MotorConfirmationContext_t motor2_confirmation = {0};

/* Suppress identical continuous ADAS targets. The cache is invalidated by
 * commands that change the motor operating state. */
static MotorTargetrpm_t last_target_rpm = {0U, 0U};
static bool last_target_rpm_valid = false;

/* =========================================================
 * MOTOR_GetConfirmationContext
 *
 * Description:
 * Returns the command-confirmation context associated with
 * the selected Motor ECU.
 *
 * Parameters:
 * target - Motor ECU whose context will be returned.
 *
 * Return:
 * Pointer to the selected context, or NULL when the target
 * ECU is unsupported.
 * ========================================================= */
static MotorConfirmationContext_t *MOTOR_GetConfirmationContext(ECU_t target);

/* =========================================================
 * MOTOR_StartConfirmation
 *
 * Description:
 * Starts confirmation tracking after a motor command has
 * been accepted by the CAN transmit queue. The current Motor
 * Status update counter is saved so only a later status frame
 * can confirm the command.
 *
 * Parameters:
 * target  - Motor ECU that will execute the command.
 * command - Command whose application will be confirmed.
 *
 * Return:
 * None.
 * ========================================================= */
static void MOTOR_StartConfirmation(ECU_t target, const MotorCommand_t *command);

/* =========================================================
 * MOTOR_IsStatusMatching
 *
 * Description:
 * Checks whether a newly received Motor Status matches the
 * command and target RPM stored in a confirmation context.
 *
 * Parameters:
 * context - Active command-confirmation context.
 * status  - Newly received Motor Status to evaluate.
 *
 * Return:
 * true  - Status confirms the requested command.
 * false - Pointer is NULL or the status does not match.
 * ========================================================= */
static bool MOTOR_IsStatusMatching(const MotorConfirmationContext_t *context, const MotorStatus_t *status);

static bool MOTOR_RequestOne(ECU_t target, const MotorCommand_t *command);

/* Checks one motor's latest status and updates its pending confirmation,
 * marking it timed out when the confirmation deadline expires. */
static void MOTOR_ProcessOneConfirmation(ECU_t target, uint32_t current_tick);

static MotorConfirmationContext_t *MOTOR_GetConfirmationContext(ECU_t target)
{
	MotorConfirmationContext_t *context = NULL;

	if (target == ECU_MOTOR1) {
		context = &motor1_confirmation;
	} else if (target == ECU_MOTOR2) {
		context = &motor2_confirmation;
	} else {
		/* Unsupported ECU. */
	}

	return context;
}

static void MOTOR_StartConfirmation(ECU_t target, const MotorCommand_t *command)
{
	MotorConfirmationContext_t *context;
	uint32_t update_count = 0U;
	bool confirmation_already_waiting;

	if (command == NULL) {
		return;
	}

	context = MOTOR_GetConfirmationContext(target);

	if (context == NULL) {
		return;
	}

	confirmation_already_waiting = (context->state == MOTOR_CONFIRMATION_WAITING);

	(void)GetMotorStatusUpdateCount(target, &update_count);

	/* A continuous steering target may arrive before the preceding RUN
	 * command is confirmed. Update the expected target, but preserve the
	 * RUN state check and its original timeout deadline. */
	if ((confirmation_already_waiting != false) && (context->command.command == MOTOR_CMD_RUN) && (command->command == MOTOR_CMD_SET_PARAM)) {
		context->expected_target_rpm = (command->direction == MOTOR_DIR_REVERSE) ? (int16_t)(-(int32_t)command->target_rpm) : (int16_t)command->target_rpm;
		context->target_check_required = true;
		context->baseline_update_count = update_count;
		return;
	}

	switch (command->command) {
	case MOTOR_CMD_SET_PARAM:
		if (command->direction == MOTOR_DIR_REVERSE) {
			context->expected_target_rpm = (int16_t)(-(int32_t)command->target_rpm);
		} else {
			context->expected_target_rpm = (int16_t)command->target_rpm;
		}

		context->target_check_required = true;
		break;

	case MOTOR_CMD_STOP:
		context->expected_target_rpm = 0;
		context->target_check_required = true;
		break;

	case MOTOR_CMD_CLEAR_ERROR:
		context->target_check_required = false;
		break;

	case MOTOR_CMD_ALIGN:
	case MOTOR_CMD_RUN:
		/* Preserve target RPM set by the previous command. */
		break;

	default:
		context->target_check_required = false;
		break;
	}

	context->state = MOTOR_CONFIRMATION_WAITING;
	context->command = *command;
	context->baseline_update_count = update_count;

	/* Continuous ADAS commands must not restart the timeout forever.
	 * Preserve the first outstanding command's deadline until a command
	 * is confirmed, cleared, or times out. */
	if (confirmation_already_waiting == false) {
		context->wait_started_tick = 0U;
		context->timer_started = false;
	}
}

static bool MOTOR_IsStatusMatching(const MotorConfirmationContext_t *context, const MotorStatus_t *status)
{
	if ((context == NULL) || (status == NULL)) {
		return false;
	}

	if ((context->target_check_required != false) && (status->target_rpm != context->expected_target_rpm)) {
		return false;
	}

	switch (context->command.command) {
	case MOTOR_CMD_SET_PARAM:
		return true;

	case MOTOR_CMD_ALIGN:
		return ((status->state == CAN_MOTOR_STATE_ALIGNING) || (status->state == CAN_MOTOR_STATE_ALIGNED));

	case MOTOR_CMD_RUN:
		return (status->state == CAN_MOTOR_STATE_RUNNING);

	case MOTOR_CMD_STOP:
		return ((status->state == CAN_MOTOR_STATE_ALIGNED) || (status->state == CAN_MOTOR_STATE_READY));

	case MOTOR_CMD_CLEAR_ERROR:
		return (status->state != CAN_MOTOR_STATE_FAULT);

	default:
		return false;
	}
}

static void MOTOR_ProcessOneConfirmation(ECU_t target, uint32_t current_tick)
{
	MotorConfirmationContext_t *context;
	MotorStatus_t status = {0};
	uint32_t update_count = 0U;

	context = MOTOR_GetConfirmationContext(target);

	if (context == NULL) {
		return;
	}

	if (context->state != MOTOR_CONFIRMATION_WAITING) {
		return;
	}

	if (context->timer_started == false) {
		context->wait_started_tick = current_tick;
		context->timer_started = true;
	}

	if (GetMotorStatusUpdateCount(target, &update_count) != false) {
		if (update_count != context->baseline_update_count) {
			context->baseline_update_count = update_count;

			if (GetMotorStatus(target, &status) != false) {
				if (MOTOR_IsStatusMatching(context, &status) != false) {
					context->state = MOTOR_CONFIRMATION_CONFIRMED;
					return;
				}
			}
		}
	}

	if ((current_tick - context->wait_started_tick) >= MOTOR_CONFIRMATION_TIMEOUT_MS) {
		context->state = MOTOR_CONFIRMATION_TIMEOUT;
	}
}

/* =========================================================
 * MOTOR_RequestOne
 *
 * Description:
 * Queues one command for the selected Motor ECU and starts
 * status confirmation tracking when the queue operation
 * succeeds.
 *
 * Parameters:
 * target  - Motor ECU that will receive the command.
 * command - Motor command to queue.
 *
 * Return:
 * true  - Command was queued and confirmation was started.
 * false - Command could not be queued.
 * ========================================================= */
static bool MOTOR_RequestOne(ECU_t target, const MotorCommand_t *command)
{
	bool request_ok;

	request_ok = SendMotorCommand(target, command);

	if (request_ok != false) {
		MOTOR_StartConfirmation(target, command);
	}

	return request_ok;
}

/* =========================================================
 * RequestBoth
 *
 * Description:
 * Queues the same command independently for Motor1 and
 * Motor2 and starts confirmation tracking for each successful
 * queue operation.
 *
 * Parameters:
 * command - Motor command to queue for both Motor ECUs.
 *
 * Return:
 * true  - Command was queued for both Motor ECUs.
 * false - Command failed to queue for one or both Motor ECUs.
 * ========================================================= */
static bool RequestBoth(const MotorCommand_t *command)
{
	bool request_ok;

	request_ok = SendMotorCommandPair(command, command);

	if (request_ok != false) {
		MOTOR_StartConfirmation(ECU_MOTOR1, command);

		MOTOR_StartConfirmation(ECU_MOTOR2, command);
	}

	return request_ok;
}

/* =========================================================
 * MOTOR_RequestAlign
 *
 * Description:
 * Requests rotor alignment from both Motor ECUs.
 *
 * Parameters:
 * None.
 *
 * Return:
 * true  - ALIGN was queued for both Motor ECUs.
 * false - ALIGN failed to queue for one or both Motor ECUs.
 * ========================================================= */
bool MOTOR_RequestAlign(void)
{
	MotorCommand_t command = {0};

	command.command = MOTOR_CMD_ALIGN;
	command.direction = MOTOR_DIR_FORWARD;
	last_target_rpm_valid = false;
	return RequestBoth(&command);
}

/* =========================================================
 * MOTOR_RequestTargetRPM
 *
 * Description:
 * Sends the requested left and right target RPM values to
 * Motor1 and Motor2 using their configured directions.
 *
 * Parameters:
 * target_rpm - Left and right target RPM values.
 *
 * Return:
 * true  - Both SET_PARAM commands were queued, or the same target is
 *         already the most recently accepted target.
 * false - Pointer is NULL or either command failed to queue.
 * ========================================================= */
bool MOTOR_RequestTargetRPM(const MotorTargetrpm_t *target_rpm)
{
	MotorCommand_t motor1 = {0};
	MotorCommand_t motor2 = {0};
	bool request_ok;

	if (target_rpm == NULL) {
		return false;
	}

	if ((last_target_rpm_valid != false) && (last_target_rpm.left_rpm == target_rpm->left_rpm) && (last_target_rpm.right_rpm == target_rpm->right_rpm)) {
		return true;
	}

	motor1.command = MOTOR_CMD_SET_PARAM;
	motor1.direction = MOTOR1_DIRECTION;
	motor1.target_rpm = target_rpm->left_rpm;
	motor2.command = MOTOR_CMD_SET_PARAM;
	motor2.direction = MOTOR2_DIRECTION;
	motor2.target_rpm = target_rpm->right_rpm;

	request_ok = SendMotorCommandPair(&motor1, &motor2);

	if (request_ok != false) {
		last_target_rpm = *target_rpm;
		last_target_rpm_valid = true;
		MOTOR_StartConfirmation(ECU_MOTOR1, &motor1);
		MOTOR_StartConfirmation(ECU_MOTOR2, &motor2);
	}

	return request_ok;
}

/* =========================================================
 * MOTOR_RequestRun
 *
 * Description:
 * Requests RUN from both Motor ECUs.
 *
 * Parameters:
 * None.
 *
 * Return:
 * true  - RUN was queued for both Motor ECUs.
 * false - RUN failed to queue for one or both Motor ECUs.
 * ========================================================= */
bool MOTOR_RequestRun(void)
{
	MotorCommand_t command = {0};
	command.command = MOTOR_CMD_RUN;
	return RequestBoth(&command);
}

/* =========================================================
 * MOTOR_RequestStop
 *
 * Description:
 * Requests a controlled stop from the selected Motor ECU.
 *
 * Parameters:
 * target - Motor ECU that will receive the STOP command.
 *
 * Return:
 * true  - STOP was queued successfully.
 * false - Target is unsupported or STOP failed to queue.
 * ========================================================= */
bool MOTOR_RequestStop(ECU_t target)
{
	MotorCommand_t command = {0};
	command.command = MOTOR_CMD_STOP;
	if (MOTOR_RequestOne(target, &command) != false) {
		last_target_rpm_valid = false;
		return true;
	}
	return false;
}

/* =========================================================
 * MOTOR_RequestStopBoth
 *
 * Description:
 * Requests a controlled stop from Motor1 and Motor2 as one
 * atomic TX queue operation.
 *
 * Both STOP frames are queued only when the TX queue has
 * enough space for both commands.
 *
 * Parameters:
 * None.
 *
 * Return:
 * true  - Both STOP commands were queued successfully.
 * false - The TX queue does not have space for both commands.
 * ========================================================= */
bool MOTOR_RequestStopBoth(void)
{
	MotorCommand_t command = {0};

	command.command = MOTOR_CMD_STOP;

	if (RequestBoth(&command) != false) {
		last_target_rpm_valid = false;
		return true;
	}
	return false;
}

/* =========================================================
 * MOTOR_RequestClearError
 *
 * Description:
 * Requests fault-latch clearing from both Motor ECUs.
 *
 * Parameters:
 * None.
 *
 * Return:
 * true  - CLEAR_ERROR was queued for both Motor ECUs.
 * false - CLEAR_ERROR failed to queue for either Motor ECU.
 * ========================================================= */
bool MOTOR_RequestClearError(void)
{
	MotorCommand_t command = {0};
	command.command = MOTOR_CMD_CLEAR_ERROR;
	if (RequestBoth(&command) != false) {
		last_target_rpm_valid = false;
		return true;
	}
	return false;
}

/* =========================================================
 * MOTOR_ProcessConfirmation
 *
 * Description:
 * Processes pending command confirmations for Motor1 and
 * Motor2. This function is intended to run periodically.
 *
 * Parameters:
 * current_tick - Current system time in milliseconds.
 *
 * Return:
 * None.
 * ========================================================= */
void MOTOR_ProcessConfirmation(uint32_t current_tick)
{
	MOTOR_ProcessOneConfirmation(ECU_MOTOR1, current_tick);

	MOTOR_ProcessOneConfirmation(ECU_MOTOR2, current_tick);
}

/* =========================================================
 * MOTOR_GetConfirmationState
 *
 * Description:
 * Returns the current command-confirmation state for the
 * selected Motor ECU.
 *
 * Parameters:
 * target - Motor ECU whose state will be returned.
 *
 * Return:
 * Current confirmation state. MOTOR_CONFIRMATION_IDLE is
 * returned when the target ECU is unsupported.
 * ========================================================= */
MotorConfirmationState_t MOTOR_GetConfirmationState(ECU_t target)
{
	MotorConfirmationContext_t *context;

	context = MOTOR_GetConfirmationContext(target);

	if (context == NULL) {
		return MOTOR_CONFIRMATION_IDLE;
	}

	return context->state;
}

/* =========================================================
 * MOTOR_ClearConfirmation
 *
 * Description:
 * Clears the completed command result and returns it to the
 * idle state. The last requested target RPM is retained so a
 * following ALIGN or RUN command can still verify that target.
 *
 * Parameters:
 * target - Motor ECU whose confirmation context will be reset.
 *
 * Return:
 * None.
 * ========================================================= */
void MOTOR_ClearConfirmation(ECU_t target)
{
	MotorConfirmationContext_t *context;

	context = MOTOR_GetConfirmationContext(target);

	if (context == NULL) {
		return;
	}

	context->state = MOTOR_CONFIRMATION_IDLE;
	context->command.command = MOTOR_CMD_STOP;
	context->command.direction = MOTOR_DIR_FORWARD;
	context->command.target_rpm = 0U;
	context->baseline_update_count = 0U;
	context->wait_started_tick = 0U;
	context->timer_started = false;
}
