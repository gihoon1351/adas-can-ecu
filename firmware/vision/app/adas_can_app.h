#ifndef ADAS_CAN_APP_H
#define ADAS_CAN_APP_H

#include <stdbool.h>
#include <stdint.h>

#include "../hal/can_hw.h"

#define ADAS_CAN_ID_HEARTBEAT (0x300U)
#define ADAS_CAN_ID_RESULT    (0x301U)
#define ADAS_RESULT_DLC       (8U)
#define ADAS_HEARTBEAT_DLC    (2U)
#define ADAS_RESULT_X_LOW     (0U)
#define ADAS_RESULT_X_HIGH    (1U)
#define ADAS_RESULT_CLASS     (2U)
#define ADAS_RESULT_Y0        (3U)
#define ADAS_RESULT_Y1        (4U)
#define ADAS_RESULT_FAR_LOW   (5U)
#define ADAS_RESULT_NEAR_LOW  (6U)
#define ADAS_RESULT_X_HIGH4   (7U)
#define ADAS_RESULT_FAR_VALID (0x08U)
#define ADAS_RESULT_NEAR_VALID (0x80U)
#define ADAS_X_MAX            (1279U)
#define ADAS_BOX_Y_MAX        (224U)

typedef enum
{
    ADAS_SPEED_BUMP = 0U,
    ADAS_TRAFFIC_GREEN = 1U,
    ADAS_TRAFFIC_RED = 2U,
    ADAS_CHILD_SIGN_SLOWDOWN = 3U,
    ADAS_SIGN_70 = 4U,
    ADAS_LANE_ONLY = 5U
} AdasVcuClass;

typedef enum
{
    ADAS_ECU_INIT = 0U,
    ADAS_ECU_READY = 1U,
    ADAS_ECU_RUN = 2U,
    ADAS_ECU_FAULT = 3U
} AdasEcuState;

typedef struct
{
    uint32_t result_period_ms;
    uint32_t heartbeat_period_ms;
    uint32_t x_timeout_ms;
    uint32_t detection_timeout_ms;
} AdasCanAppConfig;

typedef struct
{
    AdasCanAppConfig cfg;
    CanSend send;
    void *send_context;
    uint32_t x_at_ms;
    uint32_t detection_at_ms;
    uint32_t result_sent_ms;
    uint32_t heartbeat_sent_ms;
    uint16_t x;
    uint16_t far_x;
    uint16_t near_x;
    uint8_t vcu_class;
    uint8_t y0;
    uint8_t y1;
    uint8_t heartbeat_counter;
    uint8_t ecu_state;
    bool x_valid;
    bool far_valid;
    bool near_valid;
    bool detection_valid;
    /* A fresh lane IRQ sample, not a CNN hit, creates a transmit request. */
    bool result_pending;
    uint32_t result_sent_count;
    uint32_t result_replaced_count;
} AdasCanApp;

bool AdasCanApp_Init(AdasCanApp *app,
                     const AdasCanAppConfig *cfg,
                     CanSend send,
                     void *send_context,
                     uint32_t now_ms);
bool AdasCanApp_SetState(AdasCanApp *app, AdasEcuState state);
bool AdasCanApp_UpdateX(AdasCanApp *app, uint16_t x, uint32_t now_ms);
bool AdasCanApp_UpdateLane(AdasCanApp *app,
                           uint16_t x,
                           uint16_t far_x,
                           uint16_t near_x,
                           bool far_valid,
                           bool near_valid,
                           uint32_t now_ms);
void AdasCanApp_ClearX(AdasCanApp *app);
bool AdasCanApp_UpdateVcuDetection(AdasCanApp *app,
                                   uint8_t vcu_class,
                                   uint8_t y0,
                                   uint8_t y1,
                                   uint32_t now_ms);
bool AdasCanApp_UpdateModelDetection(AdasCanApp *app,
                                     uint8_t model_class,
                                     uint8_t y0,
                                     uint8_t y1,
                                     uint32_t now_ms);
void AdasCanApp_ClearDetection(AdasCanApp *app);
bool AdasCanApp_MakeResult(uint16_t x,
                           uint16_t far_x,
                           uint16_t near_x,
                           bool far_valid,
                           bool near_valid,
                           uint8_t vcu_class,
                           uint8_t y0,
                           uint8_t y1,
                           CanFrame *frame);
void AdasCanApp_Poll(AdasCanApp *app, uint32_t now_ms);

#endif
