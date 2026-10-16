#ifndef SRC_DRIVER_DRIVER_CAN_H_
#define SRC_DRIVER_DRIVER_CAN_H_

#include <stdint.h>
#include <stdbool.h>

#include "../../hal/can/can.h"
#include "../vcu/vcu.h"

/* =========================================================
 * Queue Size
 * ========================================================= */

#define CAN_RX_QUEUE_SIZE (64U)
#define CAN_RX_QUEUE_MASK (CAN_RX_QUEUE_SIZE - 1U)

#define CAN_TX_QUEUE_SIZE (16U)
#define CAN_TX_QUEUE_MASK (CAN_TX_QUEUE_SIZE - 1U)

#define CAN_ADAS_EVENT_QUEUE_SIZE (16U)
#define CAN_ADAS_EVENT_QUEUE_MASK (CAN_ADAS_EVENT_QUEUE_SIZE - 1U)

/* =========================================================
 * Timeout
 * ========================================================= */

/* Allow up to 1 s for transient CAN arbitration/TX mailbox delays. */
#define HEARTBEAT_TIMEOUT_MS (1000U)

/* =========================================================
 * CAN ID
 * ========================================================= */

/* Emergency Stop */
#define CAN_ID_ESTOP_COMMAND (0x001U)

/* Fault */
#define CAN_ID_FAULT_MOTOR1 (0x020U)
#define CAN_ID_FAULT_MOTOR2 (0x021U)
#define CAN_ID_FAULT_ADAS1 (0x022U)
#define CAN_ID_FAULT_ADAS2 (0x023U)
#define CAN_ID_FAULT_VCU (0x024U)

/* Global Command */
#define CAN_ID_GLOBAL_COMMAND (0x040U)

/* VCU */
#define CAN_ID_HEARTBEAT_VCU (0x100U)

#define CAN_ID_MOTOR1_COMMAND (0x110U)
#define CAN_ID_MOTOR2_COMMAND (0x111U)

/* Motor ECU1 */
#define CAN_ID_HEARTBEAT_MOTOR1 (0x200U)
#define CAN_ID_MOTOR1_STATUS (0x201U)
#define CAN_ID_MOTOR1_TUNING_ACK (0x205U)

/* Motor ECU2 */
#define CAN_ID_HEARTBEAT_MOTOR2 (0x210U)
#define CAN_ID_MOTOR2_STATUS (0x211U)
#define CAN_ID_MOTOR2_TUNING_ACK (0x215U)

/* ADAS ECU1 */
#define CAN_ID_HEARTBEAT_ADAS1 (0x300U)
#define CAN_ID_ADAS1_RESULT (0x301U)
#define CAN_ID_ADAS1_YCBCR_COMMAND (0x302U)
#define CAN_ID_ADAS1_YCBCR_ACK (0x303U)

/* ADAS ECU2 */
#define CAN_ID_HEARTBEAT_ADAS2 (0x310U)
#define CAN_ID_ADAS2_RESULT (0x311U)

/* =========================================================
 * Timeout Mask
 * ========================================================= */

#define CAN_TIMEOUT_MOTOR1 (1UL << 0U)
#define CAN_TIMEOUT_MOTOR2 (1UL << 1U)
#define CAN_TIMEOUT_ADAS1 (1UL << 2U)
#define CAN_TIMEOUT_ADAS2 (1UL << 3U)

#define CAN_HEARTBEAT_ALL_ECUS (CAN_TIMEOUT_MOTOR1 | CAN_TIMEOUT_MOTOR2 | CAN_TIMEOUT_ADAS1 | CAN_TIMEOUT_ADAS2)

/* =========================================================
 * Global Command
 *
 * ���� VCU ���� ������ GREEN Event ����.
 * Global Command�� CAN Protocol ������θ� ����.
 * ========================================================= */

typedef enum
{
	GB_SYSTEM_RESET = 0x00U,
	GB_DRIVE_START = 0x01U,
	GB_DRIVE_STOP = 0x02U,
	GB_ENABLE = 0x03U,
	GB_DISABLE = 0x04U,
	GB_FAULT_RESET = 0x05U,
	GB_SAFE_STATE = 0x06U

} GB_COMMAND_t;

/* =========================================================
 * Heartbeat
 * ========================================================= */

typedef enum
{
	ECU_STATE_INIT = 0x00U,
	ECU_STATE_READY = 0x01U,
	ECU_STATE_RUN = 0x02U,
	ECU_STATE_FAULT = 0x03U

} ECU_STATE_t;

typedef struct
{
	ECU_STATE_t state;

	uint8_t alive_counter;

} Heartbeat_t;

/* =========================================================
 * API
 * ========================================================= */

/* =========================================================
 * RX
 * ========================================================= */

void CAN_RxCallback(void);

void CAN_ProcessRx(uint32_t current_tick);

bool CAN_IsRxQueueFaulted(void);

void CAN_ClearRxQueueFault(void);

/* =========================================================
 * TX
 * ========================================================= */

void CAN_ProcessTx(void);

void CAN_DiscardPendingTx(void);

/* =========================================================
 * Heartbeat
 * ========================================================= */

uint32_t CAN_UpdateHeartbeatTimeouts(uint32_t current_tick);

uint32_t CAN_GetHeartbeatReceivedMask(void);

uint32_t CAN_GetHeartbeatHealthyMask(uint32_t current_tick);

bool CAN_GetHeartbeatState(ECU_t ecu, ECU_STATE_t *state);

bool SendHeartbeat(ECU_t ecu, const Heartbeat_t *heartbeat);

/* =========================================================
 * Motor Command
 * ========================================================= */

bool SendMotorCommand(ECU_t target, const MotorCommand_t *motorcommand);

bool SendMotorCommandPair(const MotorCommand_t *motor1_command, const MotorCommand_t *motor2_command);
bool SendMotorRampTuningPair(uint8_t band, uint16_t interval_ticks);
bool SendYcbcrThresholdCommand(const uint8_t values[6], bool query, uint8_t sequence);

/* =========================================================
 * Global Command
 * ========================================================= */

bool SendGlobalCommand(const GB_COMMAND_t command);

/* =========================================================
 * Emergency Stop
 * ========================================================= */

bool SendEmergencyStopCommand(void);

/* =========================================================
 * Fault
 * ========================================================= */

bool SendFault(ECU_t source);

bool CAN_IsFaultDetected(ECU_t ecu);

bool CAN_ClearFaultFlag(ECU_t ecu);

/* =========================================================
 * RX Getter
 * ========================================================= */

bool GetMotorStatus(ECU_t ecu, MotorStatus_t *status);

bool GetMotorStatusUpdateCount(ECU_t ecu, uint32_t *update_count);

bool GetADASResult(ECU_t ecu, ADASResult_t *result);

bool CAN_PeekADASResult(ECU_t ecu, ADASResult_t *result, uint32_t *last_tick);

bool CAN_GetADASResultLastTick(ECU_t ecu, uint32_t *last_tick);

bool CAN_GetLatestADAS1Lane(uint16_t *lane_x, uint16_t *far_x,
		uint16_t *near_x, bool *far_valid, bool *near_valid);

bool CAN_GetNextADASEvent(ECU_t *source, ADASResult_t *result, uint32_t *result_tick);

bool CAN_IsADASEventQueueFaulted(void);

void CAN_ResetADASControlData(void);

#endif /* SRC_DRIVER_DRIVER_CAN_H_ */
