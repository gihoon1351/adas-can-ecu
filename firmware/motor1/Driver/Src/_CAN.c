#include "../Inc/_CAN.h"

#define CAN_EVENT_QUEUE_LENGTH 8U

static CAN_TxHeaderTypeDef tx_header;
static CAN_RxHeaderTypeDef rx_header;
static CAN_FilterTypeDef can_filter;
static uint32_t tx_mailbox;
static uint8_t rx_data[8];

static CAN_Event event_queue[CAN_EVENT_QUEUE_LENGTH];
static volatile uint8_t event_head = 0U;
static volatile uint8_t event_tail = 0U;

static volatile bool emergency_stop_pending = false;
static volatile bool vcu_fault_pending = false;
static volatile bool fault_tx_pending = false;
static volatile bool estop_req_tx_pending = false;

static volatile bool vcu_heartbeat_seen = false;
static volatile uint32_t vcu_heartbeat_last_tick = 0U;
static volatile uint8_t vcu_alive_counter = 0U;
static volatile CAN_ECUState vcu_state = CAN_ECU_STATE_INIT;

static uint8_t motor_alive_counter = 0U;

static bool CAN_Send(uint32_t id, const uint8_t *data, uint8_t length);
static void CAN_QueueEvent(const CAN_Event *event);
static uint16_t CAN_ReadU16(const uint8_t *data);

void CAN_Config(void)
{
    can_filter.FilterBank = 0;
    can_filter.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    can_filter.FilterActivation = ENABLE;
    can_filter.FilterIdHigh = 0x0000;
    can_filter.FilterIdLow = 0x0000;
    can_filter.FilterMaskIdHigh = 0x0000;
    can_filter.FilterMaskIdLow = 0x0000;

    if (HAL_CAN_ConfigFilter(&hcan, &can_filter) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_CAN_Start(&hcan) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        Error_Handler();
    }

    vcu_heartbeat_last_tick = HAL_GetTick();
}

bool CAN_GetNextEvent(CAN_Event *event)
{
    uint32_t interrupt_state;

    if (event == NULL)
    {
        return false;
    }

    interrupt_state = __get_PRIMASK();
    __disable_irq();

    if (emergency_stop_pending)
    {
        emergency_stop_pending = false;
        event->type = CAN_EVENT_EMERGENCY_STOP;
        event->direction = 0U;
        event->target_rpm = 0U;
        if (interrupt_state == 0U) __enable_irq();
        return true;
    }

    if (vcu_fault_pending)
    {
        vcu_fault_pending = false;
        event->type = CAN_EVENT_VCU_FAULT;
        event->direction = 0U;
        event->target_rpm = 0U;
        if (interrupt_state == 0U) __enable_irq();
        return true;
    }

    if (event_tail == event_head)
    {
        if (interrupt_state == 0U) __enable_irq();
        return false;
    }

    *event = event_queue[event_tail];
    event_tail = (uint8_t)((event_tail + 1U) % CAN_EVENT_QUEUE_LENGTH);

    if (interrupt_state == 0U) __enable_irq();
    return true;
}

void CAN_SendMotorStatus(const CAN_MotorStatus *status)
{
    uint8_t data[8];
    uint16_t rpm_raw;
    uint16_t target_rpm_raw;

    if (status == NULL)
    {
        return;
    }

    rpm_raw = (uint16_t)status->rpm;
    target_rpm_raw = (uint16_t)status->target_rpm;

    data[0] = (uint8_t)status->temperature_c;
    data[1] = (uint8_t)(rpm_raw & 0xFFU);
    data[2] = (uint8_t)(rpm_raw >> 8U);
    data[3] = (uint8_t)(status->power_10mv & 0xFFU);
    data[4] = (uint8_t)(status->power_10mv >> 8U);
    data[5] = (uint8_t)status->state;
    data[6] = (uint8_t)(target_rpm_raw & 0x00FFU);
    data[7] = (uint8_t)(target_rpm_raw >> 8U);
    
    (void)CAN_Send(CAN_ID_MOTOR_STATUS, data, 8U);
}

void CAN_SendHeartbeat(CAN_ECUState state)
{
    uint8_t data[2];

    data[0] = (uint8_t)state;
    data[1] = motor_alive_counter++;

    (void)CAN_Send(CAN_ID_HEARTBEAT, data, 2U);
}

void CAN_SendFault(void)
{
    fault_tx_pending = !CAN_Send(CAN_ID_FAULT, NULL, 0U);
}

void CAN_SendEmergencyStopRequest(void)
{
    estop_req_tx_pending = !CAN_Send(CAN_ID_ESTOP_REQ, NULL, 0U);
}

void CAN_SendRampTuningAck(uint8_t band, uint16_t ticks, bool accepted)
{
#if MOTOR_REMOTE_TUNING_ENABLE
    uint8_t data[4];
    data[0] = band;
    data[1] = accepted ? 1U : 0U;
    data[2] = (uint8_t)(ticks & 0xFFU);
    data[3] = (uint8_t)(ticks >> 8U);
    (void)CAN_Send(CAN_ID_MOTOR_TUNING_ACK, data, 4U);
#else
    (void)band; (void)ticks; (void)accepted;
#endif
}

void CAN_ProcessPendingTx(void)
{
    if (estop_req_tx_pending)
    {
        estop_req_tx_pending = !CAN_Send(CAN_ID_ESTOP_REQ, NULL, 0U);
    }

    if (fault_tx_pending)
    {
        fault_tx_pending = !CAN_Send(CAN_ID_FAULT, NULL, 0U);
    }
}

bool CAN_IsVcuHeartbeatTimedOut(uint32_t current_tick)
{
    return ((current_tick - vcu_heartbeat_last_tick) >= CAN_HEARTBEAT_TIMEOUT_MS);
}

bool CAN_HasVcuHeartbeat(void)
{
    return vcu_heartbeat_seen;
}

uint8_t CAN_GetVcuAliveCounter(void)
{
    return vcu_alive_counter;
}

CAN_ECUState CAN_GetVcuState(void)
{
    return vcu_state;
}

static bool CAN_Send(uint32_t id, const uint8_t *data, uint8_t length)
{
    static const uint8_t empty_data[8] = {0};
    const uint8_t *tx_data = data;

    if (length > 8U)
    {
        return false;
    }

    if (tx_data == NULL)
    {
        tx_data = empty_data;
    }

    tx_header.StdId = id;
    tx_header.ExtId = 0U;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.IDE = CAN_ID_STD;
    tx_header.DLC = length;
    tx_header.TransmitGlobalTime = DISABLE;

    return (HAL_CAN_AddTxMessage(&hcan, &tx_header, (uint8_t *)tx_data,
                                 &tx_mailbox) == HAL_OK);
}

static void CAN_QueueEvent(const CAN_Event *event)
{
    uint8_t next_head;

    if (event == NULL)
    {
        return;
    }

    next_head = (uint8_t)((event_head + 1U) % CAN_EVENT_QUEUE_LENGTH);
    if (next_head == event_tail)
    {
        return;
    }

    event_queue[event_head] = *event;
    __DMB();
    event_head = next_head;
}

static uint16_t CAN_ReadU16(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_Event event = {0};

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK)
    {
        return;
    }

    if ((rx_header.IDE != CAN_ID_STD) || (rx_header.RTR != CAN_RTR_DATA))
    {
        return;
    }

    if ((rx_header.StdId == CAN_ID_ESTOP_COMMAND) && (rx_header.DLC == 0U))
    {
        emergency_stop_pending = true;
        return;
    }

    if ((rx_header.StdId == CAN_ID_HEARTBEAT_VCU) && (rx_header.DLC == 2U))
    {
        if (rx_data[0] <= (uint8_t)CAN_ECU_STATE_FAULT)
        {
            vcu_state = (CAN_ECUState)rx_data[0];
            vcu_alive_counter = rx_data[1];
            vcu_heartbeat_last_tick = HAL_GetTick();
            vcu_heartbeat_seen = true;
        }
        return;
    }

    if ((rx_header.StdId == CAN_ID_FAULT_VCU) && (rx_header.DLC == 0U))
    {
        vcu_fault_pending = true;
        return;
    }

    if ((rx_header.StdId == CAN_ID_GLOBAL_COMMAND) && (rx_header.DLC == 1U))
    {
        switch (rx_data[0])
        {
        case CAN_GLOBAL_SYSTEM_RESET: event.type = CAN_EVENT_GLOBAL_SYSTEM_RESET; break;
        case CAN_GLOBAL_DRIVE_START:  event.type = CAN_EVENT_GLOBAL_START; break;
        case CAN_GLOBAL_DRIVE_STOP:   event.type = CAN_EVENT_GLOBAL_STOP; break;
        case CAN_GLOBAL_ENABLE:       event.type = CAN_EVENT_GLOBAL_ENABLE; break;
        case CAN_GLOBAL_DISABLE:      event.type = CAN_EVENT_GLOBAL_DISABLE; break;
        case CAN_GLOBAL_FAULT_RESET:  event.type = CAN_EVENT_GLOBAL_FAULT_RESET; break;
        case CAN_GLOBAL_SAFE_STATE:   event.type = CAN_EVENT_GLOBAL_SAFE_STATE; break;
        default: return;
        }

        CAN_QueueEvent(&event);
        return;
    }

    if ((rx_header.StdId == CAN_ID_MOTOR_COMMAND) && (rx_header.DLC == 4U))
    {
#if MOTOR_REMOTE_TUNING_ENABLE
        if (rx_data[0] == 0x11U)
        {
            event.type = CAN_EVENT_SET_RAMP_INTERVAL;
            event.direction = rx_data[1];
            event.target_rpm = CAN_ReadU16(&rx_data[2]);
            CAN_QueueEvent(&event);
            return;
        }
#endif
        if ((rx_data[1] & CAN_COMMAND_RESERVED_MASK) != 0U)
        {
            return;
        }

        event.direction = (uint8_t)((rx_data[1] & CAN_COMMAND_DIR_MASK) >>
                                    CAN_COMMAND_DIR_POS);
        event.target_rpm = CAN_ReadU16(&rx_data[2]);

        switch (rx_data[0])
        {
        case 0x00U: event.type = CAN_EVENT_MOTOR_STOP; break;
        case 0x01U: event.type = CAN_EVENT_MOTOR_RUN; break;
        case 0x02U: event.type = CAN_EVENT_MOTOR_ALIGN; break;
        case 0x03U: event.type = CAN_EVENT_CLEAR_ERROR; break;
        case 0x10U: event.type = CAN_EVENT_SET_PARAM; break;
        default: return;
        }

        CAN_QueueEvent(&event);
    }
}
