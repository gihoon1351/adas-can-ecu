#include "adas_can_app.h"

#include <stddef.h>
#include <string.h>

#define ADAS_MODEL_CLASS_COUNT (5U)

static bool AdasCanApp_IsResultValid(uint16_t x,
                                     uint8_t vcu_class,
                                     uint8_t y0,
                                     uint8_t y1)
{
    bool valid = true;

    if ((x > ADAS_X_MAX) || (vcu_class > ADAS_LANE_ONLY) ||
        (y0 > y1) || (y1 > ADAS_BOX_Y_MAX))
    {
        valid = false;
    }
    else if ((vcu_class == ADAS_LANE_ONLY) &&
             ((y0 != 0U) || (y1 != 0U)))
    {
        valid = false;
    }
    else if ((vcu_class != ADAS_LANE_ONLY) && (y0 == y1))
    {
        valid = false;
    }
    else
    {
        /* Inputs are valid. */
    }

    return valid;
}

bool AdasCanApp_MakeResult(uint16_t x,
                           uint16_t far_x,
                           uint16_t near_x,
                           bool far_valid,
                           bool near_valid,
                           uint8_t vcu_class,
                           uint8_t y0,
                           uint8_t y1,
                           CanFrame *frame)
{
    bool success = false;

    if ((frame != NULL) && AdasCanApp_IsResultValid(x, vcu_class, y0, y1) &&
        (far_x <= ADAS_X_MAX) && (near_x <= ADAS_X_MAX))
    {
        (void)memset(frame, 0, sizeof(*frame));
        frame->id = ADAS_CAN_ID_RESULT;
        frame->dlc = ADAS_RESULT_DLC;
        frame->data[ADAS_RESULT_X_LOW] = (uint8_t)(x & 0x00FFU);
        frame->data[ADAS_RESULT_X_HIGH] =
            (uint8_t)((x >> 8U) & 0x00FFU);
        frame->data[ADAS_RESULT_CLASS] = vcu_class;
        frame->data[ADAS_RESULT_Y0] = y0;
        frame->data[ADAS_RESULT_Y1] = y1;
        /* FAR and NEAR are 11-bit image coordinates. Preserve the original
         * five-byte payload and pack both upper nibbles into byte 7. */
        frame->data[ADAS_RESULT_FAR_LOW] = (uint8_t)(far_x & 0x00FFU);
        frame->data[ADAS_RESULT_NEAR_LOW] = (uint8_t)(near_x & 0x00FFU);
        frame->data[ADAS_RESULT_X_HIGH4] =
            (uint8_t)(((far_x >> 8U) & 0x0FU) |
                      ((near_x >> 4U) & 0xF0U));
        if (far_valid) frame->data[ADAS_RESULT_X_HIGH4] |= ADAS_RESULT_FAR_VALID;
        if (near_valid) frame->data[ADAS_RESULT_X_HIGH4] |= ADAS_RESULT_NEAR_VALID;
        success = true;
    }

    return success;
}

bool AdasCanApp_Init(AdasCanApp *app,
                     const AdasCanAppConfig *cfg,
                     CanSend send,
                     void *send_context,
                     uint32_t now_ms)
{
    bool success = false;

    if ((app != NULL) && (cfg != NULL) && (send != NULL) &&
        (cfg->result_period_ms > 0U) &&
        (cfg->heartbeat_period_ms > 0U) &&
        (cfg->x_timeout_ms > 0U) &&
        (cfg->detection_timeout_ms > 0U))
    {
        (void)memset(app, 0, sizeof(*app));
        app->cfg = *cfg;
        app->send = send;
        app->send_context = send_context;
        app->ecu_state = (uint8_t)ADAS_ECU_READY;
        app->result_sent_ms = now_ms - cfg->result_period_ms;
        app->heartbeat_sent_ms = now_ms - cfg->heartbeat_period_ms;
        success = true;
    }

    return success;
}

bool AdasCanApp_SetState(AdasCanApp *app, AdasEcuState state)
{
    bool success = false;

    if ((app != NULL) && (state <= ADAS_ECU_FAULT))
    {
        app->ecu_state = (uint8_t)state;
        success = true;
    }

    return success;
}

bool AdasCanApp_UpdateX(AdasCanApp *app, uint16_t x, uint32_t now_ms)
{
    return AdasCanApp_UpdateLane(app, x, x, x, true, true, now_ms);
}

bool AdasCanApp_UpdateLane(AdasCanApp *app,
                           uint16_t x,
                           uint16_t far_x,
                           uint16_t near_x,
                           bool far_valid,
                           bool near_valid,
                           uint32_t now_ms)
{
    bool success = false;

    if (app != NULL)
    {
        if ((x <= ADAS_X_MAX) && (far_x <= ADAS_X_MAX) &&
            (near_x <= ADAS_X_MAX))
        {
            if (app->result_pending) ++app->result_replaced_count;
            app->x = x;
            app->far_x = far_x;
            app->near_x = near_x;
            app->far_valid = far_valid;
            app->near_valid = near_valid;
            app->x_at_ms = now_ms;
            app->x_valid = true;
            app->result_pending = true;
            success = true;
        }
        else
        {
            app->x_valid = false;
            app->result_pending = false;
        }
    }

    return success;
}

void AdasCanApp_ClearX(AdasCanApp *app)
{
    if (app != NULL)
    {
        app->x_valid = false;
        app->result_pending = false;
    }
}

void AdasCanApp_ClearDetection(AdasCanApp *app)
{
    if (app != NULL)
    {
        app->detection_valid = false;
    }
}

bool AdasCanApp_UpdateVcuDetection(AdasCanApp *app,
                                   uint8_t vcu_class,
                                   uint8_t y0,
                                   uint8_t y1,
                                   uint32_t now_ms)
{
    bool success = false;

    if (app != NULL)
    {
        if ((vcu_class <= ADAS_SIGN_70) &&
            (y0 < y1) && (y1 <= ADAS_BOX_Y_MAX))
        {
            app->vcu_class = vcu_class;
            app->y0 = y0;
            app->y1 = y1;
            app->detection_at_ms = now_ms;
            app->detection_valid = true;
            success = true;
        }
        else
        {
            AdasCanApp_ClearDetection(app);
        }
    }

    return success;
}

bool AdasCanApp_UpdateModelDetection(AdasCanApp *app,
                                     uint8_t model_class,
                                     uint8_t y0,
                                     uint8_t y1,
                                     uint32_t now_ms)
{
    static const uint8_t model_to_vcu[ADAS_MODEL_CLASS_COUNT] =
    {
        (uint8_t)ADAS_CHILD_SIGN_SLOWDOWN,
        (uint8_t)ADAS_SIGN_70,
        (uint8_t)ADAS_SPEED_BUMP,
        (uint8_t)ADAS_TRAFFIC_RED,
        (uint8_t)ADAS_TRAFFIC_GREEN
    };
    bool success = false;

    if (app != NULL)
    {
        if (model_class < ADAS_MODEL_CLASS_COUNT)
        {
            success = AdasCanApp_UpdateVcuDetection(
                app, model_to_vcu[model_class], y0, y1, now_ms);
        }
        else
        {
            AdasCanApp_ClearDetection(app);
        }
    }

    return success;
}

static void AdasCanApp_GetDetection(const AdasCanApp *app,
                                    uint32_t now_ms,
                                    uint8_t *vcu_class,
                                    uint8_t *y0,
                                    uint8_t *y1)
{
    *vcu_class = (uint8_t)ADAS_LANE_ONLY;
    *y0 = 0U;
    *y1 = 0U;

    if (app->detection_valid &&
        ((uint32_t)(now_ms - app->detection_at_ms) <=
         app->cfg.detection_timeout_ms))
    {
        *vcu_class = app->vcu_class;
        *y0 = app->y0;
        *y1 = app->y1;
    }
}

static bool AdasCanApp_SendResult(AdasCanApp *app,
                                  uint32_t now_ms,
                                  uint8_t vcu_class,
                                  uint8_t y0,
                                  uint8_t y1)
{
    CanFrame frame;
    bool sent = false;

    if (AdasCanApp_MakeResult(app->x, app->far_x, app->near_x,
                              app->far_valid, app->near_valid,
                              vcu_class, y0, y1, &frame))
    {
        sent = app->send(app->send_context, &frame);
        if (sent)
        {
            app->result_sent_ms = now_ms;
            app->result_pending = false;
            ++app->result_sent_count;
        }
    }

    return sent;
}

static bool AdasCanApp_SendHeartbeat(AdasCanApp *app, uint32_t now_ms)
{
    CanFrame frame;
    bool sent;

    (void)memset(&frame, 0, sizeof(frame));
    frame.id = ADAS_CAN_ID_HEARTBEAT;
    frame.dlc = ADAS_HEARTBEAT_DLC;
    frame.data[0] = app->ecu_state;
    frame.data[1] = app->heartbeat_counter;

    sent = app->send(app->send_context, &frame);
    if (sent)
    {
        app->heartbeat_sent_ms = now_ms;
        ++app->heartbeat_counter;
    }

    return sent;
}

void AdasCanApp_Poll(AdasCanApp *app, uint32_t now_ms)
{
    uint32_t heartbeat_elapsed;
    uint32_t result_elapsed;
    uint8_t vcu_class;
    uint8_t y0;
    uint8_t y1;
    bool heartbeat_due;
    bool result_due;
    bool sent = false;

    if ((app != NULL) && (app->send != NULL))
    {
        heartbeat_elapsed = (uint32_t)(now_ms - app->heartbeat_sent_ms);
        result_elapsed = (uint32_t)(now_ms - app->result_sent_ms);
        heartbeat_due =
            heartbeat_elapsed >= app->cfg.heartbeat_period_ms;
        result_due =
            (result_elapsed >= app->cfg.result_period_ms) &&
            app->result_pending &&
            app->x_valid &&
            ((uint32_t)(now_ms - app->x_at_ms) <= app->cfg.x_timeout_ms);

        AdasCanApp_GetDetection(app, now_ms, &vcu_class, &y0, &y1);

        if (result_due &&
            ((!heartbeat_due) || (result_elapsed >= heartbeat_elapsed)))
        {
            sent = AdasCanApp_SendResult(app, now_ms, vcu_class, y0, y1);
        }

        if ((!sent) && heartbeat_due)
        {
            sent = AdasCanApp_SendHeartbeat(app, now_ms);
        }

        if ((!sent) && result_due)
        {
            (void)AdasCanApp_SendResult(app, now_ms, vcu_class, y0, y1);
        }
    }
}
