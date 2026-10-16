#ifndef SRC_DRIVER_MOTOR_DRIVER_MOTOR_H_
#define SRC_DRIVER_MOTOR_DRIVER_MOTOR_H_

#include <stdint.h>
#include <stdbool.h>
#include "../can/can.h"

typedef enum
{
	MOTOR_CONFIRMATION_IDLE = 0U,
	MOTOR_CONFIRMATION_WAITING,
	MOTOR_CONFIRMATION_CONFIRMED,
	MOTOR_CONFIRMATION_TIMEOUT
} MotorConfirmationState_t;

bool MOTOR_RequestAlign(void);
bool MOTOR_RequestTargetRPM(const MotorTargetrpm_t *target_rpm);
bool MOTOR_RequestRun(void);
bool MOTOR_RequestStop(ECU_t target);
bool MOTOR_RequestClearError(void);
bool MOTOR_RequestStopBoth(void);

void MOTOR_ProcessConfirmation(uint32_t current_tick);
MotorConfirmationState_t MOTOR_GetConfirmationState(ECU_t target);
void MOTOR_ClearConfirmation(ECU_t target);

#endif /* SRC_DRIVER_MOTOR_DRIVER_MOTOR_H_ */