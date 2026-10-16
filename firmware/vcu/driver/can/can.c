#include "can.h"
#include <stddef.h>
#include "xil_printf.h"

/* =========================================================
 * Internal CAN Driver State
 *
 * RX queue:
 * Produced by the CAN RX callback and consumed by CAN_ProcessRx.
 *
 * TX queue:
 * Produced by application/driver send functions and consumed
 * by CAN_ProcessTx.
 *
 * Heartbeat timestamps:
 * Store the most recent heartbeat reception time for each ECU.
 *
 * Latest received data:
 * Stores decoded motor status, ADAS result, validity flags,
 * update flags, and fault flags.
 * ========================================================= */

static CAN_Frame_t can_rx_queue[CAN_RX_QUEUE_SIZE];

static CAN_Frame_t can_tx_queue[CAN_TX_QUEUE_SIZE];

typedef struct
{
	ECU_t source;
	ADASResult_t result;
	uint32_t result_tick;
} CAN_ADASEventRecord_t;

static CAN_ADASEventRecord_t adas_event_queue[CAN_ADAS_EVENT_QUEUE_SIZE];
static uint8_t adas_event_head = 0U;
static uint8_t adas_event_tail = 0U;
static bool adas_event_queue_faulted = false;

static uint16_t adas1_latest_lane_x = 0U;
static bool adas1_lane_update = false;

static volatile uint8_t rx_queue_head = 0U;

static volatile uint8_t rx_queue_tail = 0U;

static volatile uint32_t can_rx_overflow_count = 0U;

static uint8_t tx_queue_head = 0U;

static uint8_t tx_queue_tail = 0U;

static uint32_t motor1_last_tick = 0U;

static uint32_t motor2_last_tick = 0U;

static uint32_t adas1_last_tick = 0U;

static uint32_t adas2_last_tick = 0U;

static uint32_t heartbeat_received_mask = 0U;

static ECU_STATE_t motor1_heartbeat_state = ECU_STATE_INIT;

static ECU_STATE_t motor2_heartbeat_state = ECU_STATE_INIT;

static ECU_STATE_t adas1_heartbeat_state = ECU_STATE_INIT;

static ECU_STATE_t adas2_heartbeat_state = ECU_STATE_INIT;

static MotorStatus_t motor1_status = {0};

static MotorStatus_t motor2_status = {0};

static ADASResult_t adas1_result = {0};

static ADASResult_t adas2_result = {0};

static uint32_t adas1_result_last_tick = 0U;

static uint32_t adas2_result_last_tick = 0U;

static bool adas1_result_received = false;

static bool adas2_result_received = false;

static bool motor1_status_valid = false;

static bool motor2_status_valid = false;

static uint32_t motor1_status_update_count = 0U;

static uint32_t motor2_status_update_count = 0U;

static bool adas1_result_valid = false;

static bool adas2_result_valid = false;

static bool adas1_result_update = false;

static bool adas2_result_update = false;

static bool motor1_fault_flag = false;

static bool motor2_fault_flag = false;

static bool adas1_fault_flag = false;

static bool adas2_fault_flag = false;

static uint8_t CAN_GetTxQueueFreeCount(void);

static bool CAN_BuildMotorCommandFrame(ECU_t target, const MotorCommand_t *motor_command, CAN_Frame_t *frame);

static bool CAN_ADASEventQueuePush(ECU_t source, const ADASResult_t *result, uint32_t result_tick)
{
	uint8_t next_head;

	if (result == NULL) {
		return false;
	}

	next_head = (uint8_t)((adas_event_head + 1U) & (uint8_t)CAN_ADAS_EVENT_QUEUE_MASK);

	if (next_head == adas_event_tail) {
		adas_event_queue_faulted = true;
		return false;
	}

	adas_event_queue[adas_event_head].source = source;
	adas_event_queue[adas_event_head].result = *result;
	adas_event_queue[adas_event_head].result_tick = result_tick;
	adas_event_head = next_head;
	return true;
}

/* =========================================================
 * CAN_RXQueuePush
 *
 * Description:
 * Pushes one received CAN frame into the RX circular queue.
 * This function is called from the CAN RX interrupt context.
 *
 * Parameter:
 * frame - Pointer to the CAN frame to be stored.
 *
 * Return:
 * true  - Frame was successfully queued.
 * false - frame is NULL or the RX queue is full.
 * ========================================================= */

static bool CAN_RXQueuePush(const CAN_Frame_t *frame)
{
	uint8_t next_head;
	if (frame == NULL) {
		return false;
	}
	next_head = (uint8_t)((rx_queue_head + 1U) & (uint8_t)CAN_RX_QUEUE_MASK);
	if (next_head == rx_queue_tail) {
		return false;
	}
	can_rx_queue[rx_queue_head] = *frame;
	rx_queue_head = next_head;
	return true;
}

/* =========================================================
 * CAN_RXQueuePop
 *
 * Description:
 * Removes the oldest CAN frame from the RX circular queue.
 * This function is used by the main-context RX processing task.
 *
 * Parameter:
 * frame - Pointer to the destination CAN frame.
 *
 * Return:
 * true  - One frame was successfully removed from the queue.
 * false - frame is NULL or the RX queue is empty.
 * ========================================================= */

static bool CAN_RXQueuePop(CAN_Frame_t *frame)
{
	if (frame == NULL) {
		return false;
	}
	if (rx_queue_head == rx_queue_tail) {
		return false;
	}
	*frame = can_rx_queue[rx_queue_tail];
	rx_queue_tail = (uint8_t)((rx_queue_tail + 1U) & (uint8_t)CAN_RX_QUEUE_MASK);
	return true;
}

/* =========================================================
 * CAN_GetTxQueueFreeCount
 *
 * Description:
 * Calculates the number of CAN frames that can currently be
 * added to the TX circular queue.
 *
 * One queue entry is always left unused so that the full and
 * empty states can be distinguished using only the head and
 * tail indices.
 *
 * Parameters:
 * None.
 *
 * Return:
 * Number of CAN frames that can be added without overflowing
 * the TX queue.
 * ========================================================= */
static uint8_t CAN_GetTxQueueFreeCount(void)
{
	uint8_t used_count;

	used_count = (uint8_t)((tx_queue_head - tx_queue_tail) & (uint8_t)CAN_TX_QUEUE_MASK);

	return (uint8_t)((CAN_TX_QUEUE_SIZE - 1U) - used_count);
}

/* =========================================================
 * CAN_TXQueuePush
 *
 * Description:
 * Pushes one CAN frame into the TX circular queue.
 * The frame remains queued until CAN_ProcessTx successfully
 * transmits it.
 *
 * Parameter:
 * frame - Pointer to the CAN frame to be queued.
 *
 * Return:
 * true  - Frame was successfully queued.
 * false - frame is NULL or the TX queue is full.
 * ========================================================= */

static bool CAN_TXQueuePush(const CAN_Frame_t *frame)
{
	uint8_t next_head;
	if (frame == NULL) {
		return false;
	}
	next_head = (uint8_t)((tx_queue_head + 1U) & (uint8_t)CAN_TX_QUEUE_MASK);
	if (next_head == tx_queue_tail) {
		return false;
	}
	can_tx_queue[tx_queue_head] = *frame;
	tx_queue_head = next_head;
	return true;
}

/* =========================================================
 * CAN_TXQueuePeek
 *
 * Description:
 * Copies the oldest pending TX frame without removing it from
 * the TX queue. Removal is performed only after a successful
 * transmission by CAN_TXQueueCommit.
 *
 * Parameter:
 * frame - Pointer to the destination CAN frame.
 *
 * Return:
 * true  - A pending frame was copied.
 * false - frame is NULL or the TX queue is empty.
 * ========================================================= */

static bool CAN_TXQueuePeek(CAN_Frame_t *frame)
{
	if (frame == NULL) {
		return false;
	}
	if (tx_queue_head == tx_queue_tail) {
		return false;
	}
	*frame = can_tx_queue[tx_queue_tail];
	return true;
}

/* =========================================================
 * CAN_TXQueueCommit
 *
 * Description:
 * Advances the TX queue tail after the current frame has been
 * successfully transmitted.
 *
 * Return:
 * None.
 * ========================================================= */

static void CAN_TXQueueCommit(void)

{
	tx_queue_tail = (uint8_t)((tx_queue_tail + 1U) & (uint8_t)CAN_TX_QUEUE_MASK);
}

/* =========================================================
 * CAN_DiscardPendingTx
 *
 * Description:
 * Discards all pending TX frames by moving the TX queue tail
 * directly to the current head position.
 *
 * Return:
 * None.
 * ========================================================= */

void CAN_DiscardPendingTx(void)

{
	tx_queue_tail = tx_queue_head;
}

/* =========================================================
 * CAN_ReadU16
 *
 * Description:
 * Reads a 16-bit unsigned value from two bytes encoded in
 * little-endian order.
 *
 * Parameter:
 * data - Pointer to the first byte. data[0] is the low byte
 *        and data[1] is the high byte.
 *
 * Return:
 * Decoded 16-bit unsigned value.
 * ========================================================= */

static uint16_t CAN_ReadU16(const uint8_t *data)
{
	uint16_t value;
	value = (uint16_t)data[0];
	value |= (uint16_t)((uint16_t)data[1] << 8U);
	return value;
}

/* =========================================================
 * CAN_RxCallback
 *
 * Description:
 * CAN RX interrupt callback. Checks whether a received frame is
 * pending, reads the frame from the CAN controller, and pushes
 * it into the RX queue. RX queue overflow is counted when the
 * queue cannot accept the received frame.
 *
 * Return:
 * None.
 * ========================================================= */

void CAN_RxCallback(void)
{
	CAN_Frame_t frame = {0};
	CAN_Status_t status;
	if (CAN_IsRxPending(CAN0) != false) {
		status = CAN_ReadFrame(CAN0, &frame);
		if (status == CAN_STATUS_OK) {
			if (CAN_RXQueuePush(&frame) == false) {
				can_rx_overflow_count++;
			}
		}
	}
}

/* =========================================================
 * CAN_EventParser
 *
 * Description:
 * Parses a received CAN frame according to its CAN ID and DLC.
 * The parser updates heartbeat timestamps, motor status data,
 * ADAS results, and ECU fault flags.
 *
 * Parameters:
 * frame        - Pointer to the received CAN frame.
 * current_tick - Current system time used for heartbeat timing.
 *
 * Return:
 * None.
 * ========================================================= */

static void CAN_EventParser(const CAN_Frame_t *frame, uint32_t current_tick)
{
	uint16_t id;
	if (frame == NULL) {
		return;
	}
	id = frame->id;
	/* Heartbeats contribute to the drive gate only with a valid state. */
	if (id == CAN_ID_HEARTBEAT_MOTOR1) {
		if ((frame->dlc == 2U) && (frame->data[0] <= (uint8_t)ECU_STATE_FAULT)) {
			motor1_last_tick = current_tick;
			motor1_heartbeat_state = (ECU_STATE_t)frame->data[0];
			heartbeat_received_mask |= CAN_TIMEOUT_MOTOR1;
		}
	} else if (id == CAN_ID_HEARTBEAT_MOTOR2) {
		if ((frame->dlc == 2U) && (frame->data[0] <= (uint8_t)ECU_STATE_FAULT)) {
			motor2_last_tick = current_tick;
			motor2_heartbeat_state = (ECU_STATE_t)frame->data[0];
			heartbeat_received_mask |= CAN_TIMEOUT_MOTOR2;
		}
	} else if (id == CAN_ID_HEARTBEAT_ADAS1) {
		if ((frame->dlc == 2U) && (frame->data[0] <= (uint8_t)ECU_STATE_FAULT)) {
			adas1_last_tick = current_tick;
			adas1_heartbeat_state = (ECU_STATE_t)frame->data[0];
			heartbeat_received_mask |= CAN_TIMEOUT_ADAS1;
		}
	} else if (id == CAN_ID_HEARTBEAT_ADAS2) {
		if ((frame->dlc == 2U) && (frame->data[0] <= (uint8_t)ECU_STATE_FAULT)) {
			adas2_last_tick = current_tick;
			adas2_heartbeat_state = (ECU_STATE_t)frame->data[0];
			heartbeat_received_mask |= CAN_TIMEOUT_ADAS2;
		}
	}
	/* Motor status payload: temp, rpm, voltage, state, target RPM. */
	else if (id == CAN_ID_MOTOR1_STATUS) {
		if ((frame->dlc == 8U) && (frame->data[5] <= (uint8_t)CAN_MOTOR_STATE_FAULT)) {
			motor1_status.temp = (int8_t)frame->data[0];
			motor1_status.rpm = (int16_t)CAN_ReadU16(&frame->data[1]);
			motor1_status.voltage = CAN_ReadU16(&frame->data[3]);
			motor1_status.state = (CAN_MotorState)frame->data[5];
			motor1_status.target_rpm = (int16_t)CAN_ReadU16(&frame->data[6]);
			motor1_status_update_count++;
			motor1_status_valid = true;
		}
	} else if (id == CAN_ID_MOTOR2_STATUS) {
		if ((frame->dlc == 8U) && (frame->data[5] <= (uint8_t)CAN_MOTOR_STATE_FAULT)) {
			motor2_status.temp = (int8_t)frame->data[0];
			motor2_status.rpm = (int16_t)CAN_ReadU16(&frame->data[1]);
			motor2_status.voltage = CAN_ReadU16(&frame->data[3]);
			motor2_status.state = (CAN_MotorState)frame->data[5];
			motor2_status.target_rpm = (int16_t)CAN_ReadU16(&frame->data[6]);
			motor2_status_update_count++;
			motor2_status_valid = true;
		}
	}
	else if ((id == CAN_ID_MOTOR1_TUNING_ACK) ||
	         (id == CAN_ID_MOTOR2_TUNING_ACK)) {
		if ((frame->dlc == 4U) && (frame->data[0] <= 2U)) {
			xil_printf("[RAMP ACK] M%d BAND=%d TICKS=%d STATUS=%s\r\n",
			           (id == CAN_ID_MOTOR1_TUNING_ACK) ? 1 : 2,
			           (int)frame->data[0],
			           (int)CAN_ReadU16(&frame->data[2]),
			           (frame->data[1] != 0U) ? "OK" : "REJECT");
		}
	}
	else if (id == CAN_ID_ADAS1_YCBCR_ACK) {
		if (frame->dlc == 8U) {
			xil_printf("[YC ACK] %s Y=%d,%d CB=%d,%d CR=%d,%d SEQ=%d\r\n",
			           (frame->data[0] == 0U) ? "OK" : "REJECT",
			           (int)frame->data[1], (int)frame->data[2],
			           (int)frame->data[3], (int)frame->data[4],
			           (int)frame->data[5], (int)frame->data[6],
			           (int)frame->data[7]);
		}
	}
	/* ADAS1 accepts the legacy five-byte result and the dual-ROI eight-byte
	 * extension. Bytes 5..7 pack FAR/NEAR as two 11-bit coordinates. */
	else if (id == CAN_ID_ADAS1_RESULT) {
		if (((frame->dlc == 5U) || (frame->dlc == 8U)) &&
		    (frame->data[2] <= (uint8_t)OBJECT_LANE)) {
			adas1_result.lanex = CAN_ReadU16(&frame->data[0]);
			if (frame->dlc == 8U) {
				adas1_result.far_x = (uint16_t)frame->data[5] |
						(uint16_t)(((uint16_t)frame->data[7] & 0x0007U) << 8U);
				adas1_result.near_x = (uint16_t)frame->data[6] |
						(uint16_t)(((uint16_t)frame->data[7] & 0x0070U) << 4U);
				adas1_result.far_valid = ((frame->data[7] & 0x08U) != 0U);
				adas1_result.near_valid = ((frame->data[7] & 0x80U) != 0U);
				adas1_result.dual_lane_valid = adas1_result.far_valid &&
						adas1_result.near_valid;
			} else {
				adas1_result.far_x = adas1_result.lanex;
				adas1_result.near_x = adas1_result.lanex;
				adas1_result.dual_lane_valid = false;
				adas1_result.far_valid = false;
				adas1_result.near_valid = false;
			}
			adas1_result.object_type = (ObjectType_t)frame->data[2];
			adas1_result.y0 = frame->data[3];
			adas1_result.y1 = frame->data[4];
			adas1_result_valid = true;
			adas1_result_update = true;
			adas1_result_last_tick = current_tick;
			adas1_result_received = true;
			adas1_latest_lane_x = adas1_result.lanex;
			adas1_lane_update = true;
			if (adas1_result.object_type != OBJECT_LANE) {
				(void)CAN_ADASEventQueuePush(ECU_ADAS1, &adas1_result, current_tick);
			}
		}
	} else if (id == CAN_ID_ADAS2_RESULT) {
		if ((frame->dlc == 5U) && (frame->data[2] <= (uint8_t)OBJECT_LANE)) {
			adas2_result.lanex = CAN_ReadU16(&frame->data[0]);
			adas2_result.far_x = adas2_result.lanex;
			adas2_result.near_x = adas2_result.lanex;
			adas2_result.dual_lane_valid = false;
			adas2_result.far_valid = false;
			adas2_result.near_valid = false;
			adas2_result.object_type = (ObjectType_t)frame->data[2];
			adas2_result.y0 = frame->data[3];
			adas2_result.y1 = frame->data[4];
			adas2_result_valid = true;
			adas2_result_update = true;
			adas2_result_last_tick = current_tick;
			adas2_result_received = true;
			/* Retain the legacy ADAS2 mailbox/CAN IDs for diagnostics, but ADAS2
			 * must never enqueue a vehicle-control event in single-ADAS mode. */
		}
	}
	/* Fault frames latch the corresponding ECU fault flag. */
	else if (id == CAN_ID_FAULT_MOTOR1) {
		if (frame->dlc == 0U) {
			motor1_fault_flag = true;
		}
	} else if (id == CAN_ID_FAULT_MOTOR2) {
		if (frame->dlc == 0U) {
			motor2_fault_flag = true;
		}
	} else if (id == CAN_ID_FAULT_ADAS1) {
		if (frame->dlc == 0U) {
			adas1_fault_flag = true;
		}
	} else if (id == CAN_ID_FAULT_ADAS2) {
		if (frame->dlc == 0U) {
			adas2_fault_flag = true;
		}
	} else {
	}
}

/* =========================================================
 * CAN_ClearFaultFlag
 *
 * Description:
 * Clears the stored fault flag for the selected ECU.
 *
 * Parameter:
 * ecu - ECU whose fault flag will be cleared.
 *
 * Return:
 * true  - The ECU type is supported and the flag was cleared.
 * false - The ECU type is not supported.
 * ========================================================= */

bool CAN_ClearFaultFlag(ECU_t ecu)

{
	bool result = true;
	switch (ecu) {
	case ECU_MOTOR1:
		motor1_fault_flag = false;
		break;
	case ECU_MOTOR2:
		motor2_fault_flag = false;
		break;
	case ECU_ADAS1:
		adas1_fault_flag = false;
		break;
	case ECU_ADAS2:
		adas2_fault_flag = false;
		break;
	default:
		result = false;
		break;
	}
	return result;
}

/* =========================================================
 * CAN_ProcessRx
 *
 * Description:
 * Processes all pending received CAN frames in main context.
 *
 * Parameter:
 * current_tick - Current system time passed to the CAN parser.
 *
 * Return:
 * None.
 *
 * Scheduling:
 * Intended to be called from the 10 ms scheduler task.
 * ========================================================= */

void CAN_ProcessRx(uint32_t current_tick)
{
	CAN_Frame_t frame;
	while (CAN_RXQueuePop(&frame) != false) {
		CAN_EventParser(&frame, current_tick);
	}
}

bool CAN_IsRxQueueFaulted(void)
{
	return (can_rx_overflow_count != 0U);
}

void CAN_ClearRxQueueFault(void)
{
	/* ProcessFaultClear runs in main context after CAN_ProcessRx has drained
	 * the queue. Clear the sticky diagnostic so a recovered overflow does not
	 * immediately latch another emergency stop when the drive gate opens. */
	can_rx_overflow_count = 0U;
}

/* =========================================================
 * CAN_UpdateHeartbeatTimeouts
 *
 * Description:
 * Checks the elapsed time since the most recent heartbeat from
 * each monitored ECU. A timeout bit is set when the configured
 * heartbeat timeout is exceeded, and the associated received
 * data is marked invalid.
 *
 * Parameter:
 * current_tick - Current system time in milliseconds.
 *
 * Return:
 * Bit mask containing all currently timed-out ECUs.
 * ========================================================= */

uint32_t CAN_UpdateHeartbeatTimeouts(uint32_t current_tick)

{
	uint32_t ecu_timeout = 0U;
	if ((current_tick - motor1_last_tick) >= HEARTBEAT_TIMEOUT_MS) {
		ecu_timeout |= CAN_TIMEOUT_MOTOR1;
		motor1_status_valid = false;
	}
	if ((current_tick - motor2_last_tick) >= HEARTBEAT_TIMEOUT_MS) {
		ecu_timeout |= CAN_TIMEOUT_MOTOR2;
		motor2_status_valid = false;
	}
	if ((current_tick - adas1_last_tick) >= HEARTBEAT_TIMEOUT_MS) {
		ecu_timeout |= CAN_TIMEOUT_ADAS1;
		adas1_result_valid = false;
	}
	if ((current_tick - adas2_last_tick) >= HEARTBEAT_TIMEOUT_MS) {
		ecu_timeout |= CAN_TIMEOUT_ADAS2;
		adas2_result_valid = false;
	}
	return ecu_timeout;
}

uint32_t CAN_GetHeartbeatReceivedMask(void)
{
	return heartbeat_received_mask;
}

static bool CAN_IsHeartbeatStateHealthy(ECU_STATE_t state)
{
	return ((state == ECU_STATE_READY) || (state == ECU_STATE_RUN));
}

uint32_t CAN_GetHeartbeatHealthyMask(uint32_t current_tick)
{
	uint32_t healthy_mask = 0U;

	if (((heartbeat_received_mask & CAN_TIMEOUT_MOTOR1) != 0U) && ((current_tick - motor1_last_tick) < HEARTBEAT_TIMEOUT_MS) && CAN_IsHeartbeatStateHealthy(motor1_heartbeat_state)) {
		healthy_mask |= CAN_TIMEOUT_MOTOR1;
	}
	if (((heartbeat_received_mask & CAN_TIMEOUT_MOTOR2) != 0U) && ((current_tick - motor2_last_tick) < HEARTBEAT_TIMEOUT_MS) && CAN_IsHeartbeatStateHealthy(motor2_heartbeat_state)) {
		healthy_mask |= CAN_TIMEOUT_MOTOR2;
	}
	if (((heartbeat_received_mask & CAN_TIMEOUT_ADAS1) != 0U) && ((current_tick - adas1_last_tick) < HEARTBEAT_TIMEOUT_MS) && CAN_IsHeartbeatStateHealthy(adas1_heartbeat_state)) {
		healthy_mask |= CAN_TIMEOUT_ADAS1;
	}
	if (((heartbeat_received_mask & CAN_TIMEOUT_ADAS2) != 0U) && ((current_tick - adas2_last_tick) < HEARTBEAT_TIMEOUT_MS) && CAN_IsHeartbeatStateHealthy(adas2_heartbeat_state)) {
		healthy_mask |= CAN_TIMEOUT_ADAS2;
	}

	return healthy_mask;
}

bool CAN_GetHeartbeatState(ECU_t ecu, ECU_STATE_t *state)
{
	if (state == NULL) {
		return false;
	}
	if (ecu == ECU_ADAS1) {
		if ((heartbeat_received_mask & CAN_TIMEOUT_ADAS1) == 0U) {
			return false;
		}
		*state = adas1_heartbeat_state;
		return true;
	}
	if (ecu == ECU_ADAS2) {
		if ((heartbeat_received_mask & CAN_TIMEOUT_ADAS2) == 0U) {
			return false;
		}
		*state = adas2_heartbeat_state;
		return true;
	}
	return false;
}

/* =========================================================
 * CAN_BuildMotorCommandFrame
 *
 * Description:
 * Validates a Motor ECU command and builds the corresponding
 * CAN frame without adding it to the TX queue.
 *
 * Separating frame construction from queue insertion allows
 * multiple frames to be validated before any queue state is
 * changed.
 *
 * Parameters:
 * target        - Motor ECU that will receive the command.
 * motor_command - Motor command to encode.
 * frame         - Destination CAN frame.
 *
 * Return:
 * true  - Command was valid and the frame was built.
 * false - Pointer is NULL, target is unsupported, or command
 *         data is invalid.
 * ========================================================= */
static bool CAN_BuildMotorCommandFrame(ECU_t target, const MotorCommand_t *motor_command, CAN_Frame_t *frame)
{
	uint16_t id;

	if ((motor_command == NULL) || (frame == NULL)) {
		return false;
	}

	if (target == ECU_MOTOR1) {
		id = CAN_ID_MOTOR1_COMMAND;
	} else if (target == ECU_MOTOR2) {
		id = CAN_ID_MOTOR2_COMMAND;
	} else {
		return false;
	}

	switch (motor_command->command) {
	case MOTOR_CMD_STOP:
	case MOTOR_CMD_RUN:
	case MOTOR_CMD_ALIGN:
	case MOTOR_CMD_CLEAR_ERROR:
	case MOTOR_CMD_SET_PARAM:
		break;

	default:
		return false;
	}

	if ((motor_command->direction != MOTOR_DIR_FORWARD) && (motor_command->direction != MOTOR_DIR_REVERSE)) {
		return false;
	}

	frame->id = id;
	frame->rtr = false;
	frame->ide = false;
	frame->dlc = 4U;

	frame->data[0] = (uint8_t)motor_command->command;
	frame->data[1] = (uint8_t)motor_command->direction;
	frame->data[2] = (uint8_t)(motor_command->target_rpm & 0x00FFU);
	frame->data[3] = (uint8_t)((motor_command->target_rpm >> 8U) & 0x00FFU);

	return true;
}

/* =========================================================
 * SendMotorCommand
 *
 * Description:
 * Validates a motor command, selects the target motor CAN ID,
 * packs command, direction, and target RPM into a CAN frame,
 * and pushes the frame into the TX queue.
 *
 * CAN Payload:
 * DATA[0]   - Motor command.
 * DATA[1]   - Motor direction.
 * DATA[2]   - Target RPM low byte.
 * DATA[3]   - Target RPM high byte.
 *
 * Parameters:
 * target       - Target motor ECU.
 * motorcommand - Pointer to the motor command structure.
 *
 * Return:
 * true  - Command frame was successfully queued.
 * false - Input is invalid or the TX queue is full.
 * ========================================================= */

bool SendMotorCommand(ECU_t target, const MotorCommand_t *motor_command)
{
	CAN_Frame_t frame = {0};

	if (CAN_BuildMotorCommandFrame(target, motor_command, &frame) == false) {
		return false;
	}

	return CAN_TXQueuePush(&frame);
}

/* =========================================================
 * SendMotorCommandPair
 *
 * Description:
 * Validates and queues one command for Motor1 and one command
 * for Motor2 as a single operation.
 *
 * Both frames are built before the TX queue is modified. The
 * commands are queued only when at least two queue entries are
 * available, preventing a partial update where only one Motor
 * ECU receives its command.
 *
 * Parameters:
 * motor1_command - Command to send to Motor1.
 * motor2_command - Command to send to Motor2.
 *
 * Return:
 * true  - Both command frames were queued successfully.
 * false - A command is invalid or fewer than two TX queue
 *         entries are available.
 * ========================================================= */
bool SendMotorCommandPair(const MotorCommand_t *motor1_command, const MotorCommand_t *motor2_command)
{
	CAN_Frame_t motor1_frame = {0};
	CAN_Frame_t motor2_frame = {0};

	if (CAN_BuildMotorCommandFrame(ECU_MOTOR1, motor1_command, &motor1_frame) == false) {
		return false;
	}

	if (CAN_BuildMotorCommandFrame(ECU_MOTOR2, motor2_command, &motor2_frame) == false) {
		return false;
	}

	if (CAN_GetTxQueueFreeCount() < 2U) {
		return false;
	}

	if (CAN_TXQueuePush(&motor1_frame) == false) {
		return false;
	}

	if (CAN_TXQueuePush(&motor2_frame) == false) {
		return false;
	}

	return true;
}

bool SendMotorRampTuningPair(uint8_t band, uint16_t interval_ticks)
{
	CAN_Frame_t motor1_frame = {0};
	CAN_Frame_t motor2_frame = {0};
	if ((band > 2U) || (interval_ticks < 5U) || (interval_ticks > 300U) ||
	    (CAN_GetTxQueueFreeCount() < 2U)) return false;
	motor1_frame.id = CAN_ID_MOTOR1_COMMAND;
	motor2_frame.id = CAN_ID_MOTOR2_COMMAND;
	motor1_frame.dlc = motor2_frame.dlc = 4U;
	motor1_frame.data[0] = motor2_frame.data[0] = 0x11U;
	motor1_frame.data[1] = motor2_frame.data[1] = band;
	motor1_frame.data[2] = motor2_frame.data[2] = (uint8_t)(interval_ticks & 0xFFU);
	motor1_frame.data[3] = motor2_frame.data[3] = (uint8_t)(interval_ticks >> 8U);
	if (!CAN_TXQueuePush(&motor1_frame)) return false;
	if (!CAN_TXQueuePush(&motor2_frame)) return false;
	return true;
}

bool SendYcbcrThresholdCommand(const uint8_t values[6], bool query, uint8_t sequence)
{
	CAN_Frame_t frame = {0};
	uint8_t index;
	if (!query && values == NULL) return false;
	frame.id = CAN_ID_ADAS1_YCBCR_COMMAND;
	frame.dlc = 8U;
	frame.data[0] = query ? 2U : 1U;
	for (index = 0U; index < 6U; ++index)
		frame.data[index + 1U] = query ? 0U : values[index];
	frame.data[7] = sequence;
	return CAN_TXQueuePush(&frame);
}

/* =========================================================
 * SendHeartbeat
 *
 * Description:
 * Builds a heartbeat CAN frame for the selected ECU and pushes
 * it into the TX queue.
 *
 * CAN Payload:
 * DATA[0] - ECU state.
 * DATA[1] - Alive counter.
 *
 * Parameters:
 * ecu       - ECU whose heartbeat frame will be generated.
 * heartbeat - Pointer to the heartbeat data.
 *
 * Return:
 * true  - Heartbeat frame was successfully queued.
 * false - Input is invalid, ECU is unsupported, or the queue is full.
 *
 * Scheduling:
 * Intended to be called from the 100 ms scheduler task.
 * ========================================================= */

bool SendHeartbeat(ECU_t ecu, const Heartbeat_t *heartbeat)
{
	CAN_Frame_t frame = {0};
	uint16_t id;
	if (heartbeat == NULL) {
		return false;
	}
	switch (ecu) {
	case ECU_VCU: {
		id = CAN_ID_HEARTBEAT_VCU;
		break;
	}
	case ECU_MOTOR1: {
		id = CAN_ID_HEARTBEAT_MOTOR1;
		break;
	}
	case ECU_MOTOR2: {
		id = CAN_ID_HEARTBEAT_MOTOR2;
		break;
	}
	case ECU_ADAS1: {
		id = CAN_ID_HEARTBEAT_ADAS1;
		break;
	}
	case ECU_ADAS2: {
		id = CAN_ID_HEARTBEAT_ADAS2;
		break;
	}
	default: {
		return false;
	}
	}
	frame.id = id;
	frame.rtr = false;
	frame.ide = false;
	frame.dlc = 2U;
	frame.data[0] = (uint8_t)heartbeat->state;
	frame.data[1] = heartbeat->alive_counter;
	return CAN_TXQueuePush(&frame);
}

/* =========================================================
 * SendGlobalCommand
 *
 * Description:
 * Builds a one-byte global command CAN frame and pushes it into
 * the TX queue.
 *
 * Parameter:
 * command - Global command value to transmit.
 *
 * Return:
 * true  - Global command frame was successfully queued.
 * false - TX queue is full.
 * ========================================================= */

bool SendGlobalCommand(const GB_COMMAND_t command)
{
	CAN_Frame_t frame = {0};
	frame.id = CAN_ID_GLOBAL_COMMAND;
	frame.rtr = false;
	frame.ide = false;
	frame.dlc = 1U;
	frame.data[0] = (uint8_t)command;
	return CAN_TXQueuePush(&frame);
}

/* =========================================================
 * SendEmergencyStopCommand
 *
 * Description:
 * Sends an emergency-stop CAN frame immediately without waiting
 * for the normal TX queue. Transmission is attempted only when
 * the CAN controller is ready and is not in an error or bus-off
 * state.
 *
 * Return:
 * true  - Emergency-stop frame was accepted by the CAN controller.
 * false - CAN controller is not ready or the request failed.
 * ========================================================= */

bool SendEmergencyStopCommand(void)
{
	CAN_Frame_t frame = {0};
	CAN_Status_t status;
	if ((CAN_IsTxReady(CAN0) == false) || (CAN_IsBusOff(CAN0) != false) || (CAN_IsError(CAN0) != false)) {
		return false;
	}
	frame.id = CAN_ID_ESTOP_COMMAND;
	frame.rtr = false;
	frame.ide = false;
	frame.dlc = 0U;
	status = CAN_SendFrame(CAN0, &frame);
	return (status == CAN_STATUS_OK);
}

/* =========================================================
 * SendFault
 *
 * Builds and queues a fault notification for the source ECU.
 *
 * Parameter:
 * source - ECU reporting the fault.
 *
 * Return:
 * true  - Fault frame was queued.
 * false - Source is unsupported or the TX queue is full.
 * ========================================================= */

bool SendFault(ECU_t source)
{
	CAN_Frame_t frame = {0};
	uint16_t id;
	switch (source) {
	case ECU_VCU: {
		id = CAN_ID_FAULT_VCU;
		break;
	}
	case ECU_MOTOR1: {
		id = CAN_ID_FAULT_MOTOR1;
		break;
	}
	case ECU_MOTOR2: {
		id = CAN_ID_FAULT_MOTOR2;
		break;
	}
	case ECU_ADAS1: {
		id = CAN_ID_FAULT_ADAS1;
		break;
	}
	case ECU_ADAS2: {
		id = CAN_ID_FAULT_ADAS2;
		break;
	}
	default: {
		return false;
	}
	}
	frame.id = id;
	frame.rtr = false;
	frame.ide = false;
	frame.dlc = 0U;
	if (CAN_TXQueuePush(&frame) == false) {
		return false;
	}
	return true;
}

/* =========================================================
 * GetMotorStatus
 *
 * Description:
 * Returns the latest valid status data for the selected motor
 * ECU.
 *
 * Parameters:
 * ecu    - Motor ECU to read.
 * status - Pointer to the destination MotorStatus_t structure.
 *
 * Return:
 * true  - Valid motor status was copied.
 * false - Pointer is NULL, ECU is unsupported, or data is invalid.
 * ========================================================= */

bool GetMotorStatus(ECU_t ecu, MotorStatus_t *status)
{
	if (status == NULL) {
		return false;
	}
	if (ecu == ECU_MOTOR1) {
		if (motor1_status_valid == false) {
			return false;
		}
		*status = motor1_status;
		return true;
	}
	if (ecu == ECU_MOTOR2) {
		if (motor2_status_valid == false) {
			return false;
		}
		*status = motor2_status;
		return true;
	}
	return false;
}

/* =========================================================
 * GetMotorStatusUpdateCount
 *
 * Description:
 * Returns the number of valid status frames received from the
 * selected Motor ECU.
 *
 * The counter is incremented whenever a valid 8-byte Motor
 * Status frame is parsed. The caller can compare this value
 * before and after sending a command to determine whether a
 * new status frame has arrived.
 *
 * Parameters:
 * ecu          - Motor ECU whose status counter will be read.
 * update_count - Pointer to the destination counter.
 *
 * Return:
 * true  - Status counter was copied successfully.
 * false - Pointer is NULL, ECU is unsupported, or no valid
 *         status frame has been received.
 * ========================================================= */

bool GetMotorStatusUpdateCount(ECU_t ecu, uint32_t *update_count)
{
	if (update_count == NULL) {
		return false;
	}

	if (ecu == ECU_MOTOR1) {
		if (motor1_status_valid == false) {
			return false;
		}

		*update_count = motor1_status_update_count;
		return true;
	}

	if (ecu == ECU_MOTOR2) {
		if (motor2_status_valid == false) {
			return false;
		}

		*update_count = motor2_status_update_count;
		return true;
	}

	return false;
}

/* =========================================================
 * GetADASResult
 *
 * Description:
 * Returns the latest newly-updated ADAS result for the selected
 * ADAS ECU. After a result is read, its update flag is cleared
 * so the same detection result is not processed repeatedly.
 *
 * Parameters:
 * ecu    - ADAS ECU to read.
 * result - Pointer to the destination ADASResult_t structure.
 *
 * Return:
 * true  - A new valid ADAS result was copied.
 * false - Pointer is NULL, ECU is unsupported, or no new valid
 *         result is available.
 * ========================================================= */

bool GetADASResult(ECU_t ecu, ADASResult_t *result)
{
	if (result == NULL) {
		return false;
	}
	if (ecu == ECU_ADAS1) {
		if ((adas1_result_valid == false) || (adas1_result_update == false)) {
			return false;
		}
		*result = adas1_result;
		adas1_result_update = false;
		return true;
	}
	if (ecu == ECU_ADAS2) {
		if ((adas2_result_valid == false) || (adas2_result_update == false)) {
			return false;
		}
		*result = adas2_result;
		adas2_result_update = false;
		return true;
	}
	return false;
}

bool CAN_PeekADASResult(ECU_t ecu, ADASResult_t *result, uint32_t *last_tick)
{
	if ((result == NULL) || (last_tick == NULL)) {
		return false;
	}
	if (ecu == ECU_ADAS1) {
		if (adas1_result_received == false) {
			return false;
		}
		*result = adas1_result;
		*last_tick = adas1_result_last_tick;
		return true;
	}
	if (ecu == ECU_ADAS2) {
		if (adas2_result_received == false) {
			return false;
		}
		*result = adas2_result;
		*last_tick = adas2_result_last_tick;
		return true;
	}
	return false;
}

bool CAN_GetADASResultLastTick(ECU_t ecu, uint32_t *last_tick)
{
	if (last_tick == NULL) {
		return false;
	}

	if (ecu == ECU_ADAS1) {
		if (adas1_result_received == false) {
			return false;
		}
		*last_tick = adas1_result_last_tick;
		return true;
	}

	if (ecu == ECU_ADAS2) {
		if (adas2_result_received == false) {
			return false;
		}
		*last_tick = adas2_result_last_tick;
		return true;
	}

	return false;
}

bool CAN_GetLatestADAS1Lane(uint16_t *lane_x, uint16_t *far_x,
		uint16_t *near_x, bool *far_valid, bool *near_valid)
{
	if ((lane_x == NULL) || (far_x == NULL) || (near_x == NULL) ||
	    (far_valid == NULL) || (near_valid == NULL) ||
	    (adas1_result_valid == false) ||
	    (adas1_lane_update == false)) {
		return false;
	}

	*lane_x = adas1_latest_lane_x;
	*far_x = adas1_result.far_x;
	*near_x = adas1_result.near_x;
	*far_valid = adas1_result.far_valid;
	*near_valid = adas1_result.near_valid;
	adas1_lane_update = false;
	return true;
}

bool CAN_GetNextADASEvent(ECU_t *source, ADASResult_t *result, uint32_t *result_tick)
{
	if ((source == NULL) || (result == NULL) || (result_tick == NULL)) {
		return false;
	}

	if (adas_event_head == adas_event_tail) {
		return false;
	}

	*source = adas_event_queue[adas_event_tail].source;
	*result = adas_event_queue[adas_event_tail].result;
	*result_tick = adas_event_queue[adas_event_tail].result_tick;
	adas_event_tail = (uint8_t)((adas_event_tail + 1U) & (uint8_t)CAN_ADAS_EVENT_QUEUE_MASK);
	return true;
}

bool CAN_IsADASEventQueueFaulted(void)
{
	return adas_event_queue_faulted;
}

void CAN_ResetADASControlData(void)
{
	adas_event_tail = adas_event_head;
	adas_event_queue_faulted = false;
	adas1_lane_update = false;
}

/* =========================================================
 * CAN_ProcessTx
 *
 * Description:
 * Attempts to transmit one pending CAN frame from the TX queue.
 * The frame is first peeked without removal. The queue entry is
 * committed only when CAN_SendFrame reports successful
 * transmission.
 *
 * Return:
 * None.
 *
 * Scheduling:
 * Intended to be called from the 1 ms scheduler task.
 * A maximum of one queued frame is transmitted per call.
 * ========================================================= */

void CAN_ProcessTx(void)

{
	CAN_Frame_t frame = {0};
	if ((CAN_IsTxReady(CAN0) == false) || (CAN_IsError(CAN0) != false) || (CAN_IsBusOff(CAN0) != false)) {
		return;
	}
	if (CAN_TXQueuePeek(&frame) == false) {
		return;
	}
	if (CAN_SendFrame(CAN0, &frame) == CAN_STATUS_OK) {
		CAN_TXQueueCommit();
	}
}

/* =========================================================
 * CAN_IsFaultDetected
 *
 * Description
 * Reads the stored fault-detection state for the selected ECU.
 *
 * Parameter:
 * ecu - ECU whose fault state will be checked.
 *
 * Return:
 * true  - A fault has been detected for the selected ECU.
 * false - No fault is stored or the ECU type is unsupported.
 * ========================================================= */

bool CAN_IsFaultDetected(ECU_t ecu)

{
	bool detected = false;
	switch (ecu) {
	case ECU_MOTOR1:
		detected = motor1_fault_flag;
		break;
	case ECU_MOTOR2:
		detected = motor2_fault_flag;
		break;
	case ECU_ADAS1:
		detected = adas1_fault_flag;
		break;
	case ECU_ADAS2:
		detected = adas2_fault_flag;
		break;
	default:
		break;
	}
	return detected;
}
