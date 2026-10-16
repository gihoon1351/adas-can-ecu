#ifndef SRC_APP_TEST_H_
#define SRC_APP_TEST_H_

#include <stdbool.h>
#include <stdint.h>

#include "../driver/vcu/vcu.h"
#include "../driver/can/can.h"

void TEST_PrintUartHelp(void);
bool TEST_StartAdasScenario(MotorTargetrpm_t *target_rpm, bool *motors_started, Heartbeat_t *heartbeat, bool emergency_stop_latched, bool manual_estop_latched);
void TEST_ProcessAdasScenario(MotorTargetrpm_t *target_rpm, bool *motors_started, Heartbeat_t *heartbeat, bool emergency_stop_latched, bool manual_estop_latched);
void TEST_ProcessAdasControl(MotorTargetrpm_t *target_rpm, bool *motors_started, Heartbeat_t *heartbeat, bool emergency_stop_latched);
void TEST_ProcessUartCommand(uint8_t command, MotorTargetrpm_t *target_rpm, bool *motors_started, bool *manual_estop_latched, Heartbeat_t *heartbeat, bool emergency_stop_latched);
void TEST_ProcessPendingUartRun(bool *motors_started, bool *manual_estop_latched, Heartbeat_t *heartbeat, bool emergency_stop_latched);
void TEST_CancelSequences(void);

#endif /* SRC_APP_TEST_H_ */
