#ifndef CAN_H_
#define CAN_H_

#include <can.h>
#include <stdbool.h>
#include <stdint.h>

#include "_NODE_CONFIG.h"

#define CAN_HEARTBEAT_PERIOD_MS           100U
#define CAN_HEARTBEAT_TIMEOUT_MS          300U

#define CAN_ID_ESTOP_COMMAND              0x001U
#define CAN_ID_ESTOP_REQ_MOTOR1           0x010U
#define CAN_ID_ESTOP_REQ_MOTOR2           0x011U

#define CAN_ID_FAULT_MOTOR1               0x020U
#define CAN_ID_FAULT_MOTOR2               0x021U
#define CAN_ID_FAULT_VCU                  0x024U

#define CAN_ID_GLOBAL_COMMAND             0x040U

#define CAN_ID_HEARTBEAT_VCU              0x100U
#define CAN_ID_MOTOR1_COMMAND             0x110U
#define CAN_ID_MOTOR2_COMMAND             0x111U

#define CAN_ID_HEARTBEAT_MOTOR1           0x200U
#define CAN_ID_MOTOR1_STATUS              0x201U
#define CAN_ID_MOTOR1_TELEMETRY_A         0x202U
#define CAN_ID_MOTOR1_TELEMETRY_B         0x203U
#define CAN_ID_MOTOR1_TELEMETRY_C         0x204U
#define CAN_ID_MOTOR1_TUNING_ACK           0x205U

#define CAN_ID_HEARTBEAT_MOTOR2           0x210U
#define CAN_ID_MOTOR2_STATUS              0x211U
#define CAN_ID_MOTOR2_TELEMETRY_A         0x212U
#define CAN_ID_MOTOR2_TELEMETRY_B         0x213U
#define CAN_ID_MOTOR2_TELEMETRY_C         0x214U
#define CAN_ID_MOTOR2_TUNING_ACK           0x215U

#if (NODE_ID == 1U)
#define CAN_ID_ESTOP_REQ                  CAN_ID_ESTOP_REQ_MOTOR1
#define CAN_ID_FAULT                      CAN_ID_FAULT_MOTOR1
#define CAN_ID_MOTOR_COMMAND              CAN_ID_MOTOR1_COMMAND
#define CAN_ID_HEARTBEAT                  CAN_ID_HEARTBEAT_MOTOR1
#define CAN_ID_MOTOR_STATUS               CAN_ID_MOTOR1_STATUS
#define CAN_ID_MOTOR_TELEMETRY_A          CAN_ID_MOTOR1_TELEMETRY_A
#define CAN_ID_MOTOR_TELEMETRY_B          CAN_ID_MOTOR1_TELEMETRY_B
#define CAN_ID_MOTOR_TELEMETRY_C          CAN_ID_MOTOR1_TELEMETRY_C
#define CAN_ID_MOTOR_TUNING_ACK            CAN_ID_MOTOR1_TUNING_ACK
#elif (NODE_ID == 2U)
#define CAN_ID_ESTOP_REQ                  CAN_ID_ESTOP_REQ_MOTOR2
#define CAN_ID_FAULT                      CAN_ID_FAULT_MOTOR2
#define CAN_ID_MOTOR_COMMAND              CAN_ID_MOTOR2_COMMAND
#define CAN_ID_HEARTBEAT                  CAN_ID_HEARTBEAT_MOTOR2
#define CAN_ID_MOTOR_STATUS               CAN_ID_MOTOR2_STATUS
#define CAN_ID_MOTOR_TELEMETRY_A          CAN_ID_MOTOR2_TELEMETRY_A
#define CAN_ID_MOTOR_TELEMETRY_B          CAN_ID_MOTOR2_TELEMETRY_B
#define CAN_ID_MOTOR_TELEMETRY_C          CAN_ID_MOTOR2_TELEMETRY_C
#define CAN_ID_MOTOR_TUNING_ACK            CAN_ID_MOTOR2_TUNING_ACK
#else
#error "NODE_ID must be 1U or 2U"
#endif

/* Motor command, ID 0x110/0x111, DLC 4.
 * Data[0] command: 0 STOP, 1 RUN, 2 ALIGN, 3 CLEAR_ERROR, 0x10 SET_PARAM
 * Data[1] bit0 direction: 0 forward, 1 reverse. Bits 7:1 must be zero.
 * Data[2:3] target RPM magnitude, uint16 little-endian.
 */
#define CAN_COMMAND_DIR_POS               0U
#define CAN_COMMAND_DIR_MASK              (1U << CAN_COMMAND_DIR_POS)
#define CAN_COMMAND_RESERVED_MASK         0xFEU

#define CAN_MOTOR_DIR_FORWARD             0x00U
#define CAN_MOTOR_DIR_REVERSE             0x01U

typedef enum
{
    CAN_ECU_STATE_INIT  = 0x00U,
    CAN_ECU_STATE_READY = 0x01U,
    CAN_ECU_STATE_RUN   = 0x02U,
    CAN_ECU_STATE_FAULT = 0x03U
} CAN_ECUState;

typedef enum
{
    CAN_GLOBAL_SYSTEM_RESET = 0x00U,
    CAN_GLOBAL_DRIVE_START  = 0x01U,
    CAN_GLOBAL_DRIVE_STOP   = 0x02U,
    CAN_GLOBAL_ENABLE       = 0x03U,
    CAN_GLOBAL_DISABLE      = 0x04U,
    CAN_GLOBAL_FAULT_RESET  = 0x05U,
    CAN_GLOBAL_SAFE_STATE   = 0x06U
} CAN_GlobalCommand;

typedef enum
{
    CAN_EVENT_NONE = 0,
    CAN_EVENT_EMERGENCY_STOP,
    CAN_EVENT_VCU_FAULT,
    CAN_EVENT_GLOBAL_SYSTEM_RESET,
    CAN_EVENT_GLOBAL_START,
    CAN_EVENT_GLOBAL_STOP,
    CAN_EVENT_GLOBAL_ENABLE,
    CAN_EVENT_GLOBAL_DISABLE,
    CAN_EVENT_GLOBAL_FAULT_RESET,
    CAN_EVENT_GLOBAL_SAFE_STATE,
    CAN_EVENT_MOTOR_STOP,
    CAN_EVENT_MOTOR_RUN,
    CAN_EVENT_MOTOR_ALIGN,
    CAN_EVENT_CLEAR_ERROR,
    CAN_EVENT_SET_PARAM,
    CAN_EVENT_SET_RAMP_INTERVAL
} CAN_EventType;

typedef struct
{
    CAN_EventType type;
    uint8_t direction;
    uint16_t target_rpm;
} CAN_Event;

typedef enum
{
    CAN_MOTOR_STATE_READY = 0x00U,
    CAN_MOTOR_STATE_ALIGNING = 0x01U,
    CAN_MOTOR_STATE_ALIGNED = 0x02U,
    CAN_MOTOR_STATE_RUNNING = 0x03U,
    CAN_MOTOR_STATE_STOPPING = 0x04U,
    CAN_MOTOR_STATE_FAULT = 0x05U
} CAN_MotorState;

typedef struct
{
    int8_t temperature_c;
    int16_t rpm;
    uint16_t power_10mv;
    CAN_MotorState state;
    int16_t target_rpm;
} CAN_MotorStatus;

void CAN_Config(void);
bool CAN_GetNextEvent(CAN_Event *event);

void CAN_SendMotorStatus(const CAN_MotorStatus *status);
void CAN_SendHeartbeat(CAN_ECUState state);
void CAN_SendFault(void);
void CAN_SendEmergencyStopRequest(void);
void CAN_SendRampTuningAck(uint8_t band, uint16_t ticks, bool accepted);
void CAN_ProcessPendingTx(void);

bool CAN_IsVcuHeartbeatTimedOut(uint32_t current_tick);
bool CAN_HasVcuHeartbeat(void);
uint8_t CAN_GetVcuAliveCounter(void);
CAN_ECUState CAN_GetVcuState(void);

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);

#endif /* CAN_H_ */
