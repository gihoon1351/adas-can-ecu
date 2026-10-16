#ifndef INC__NODE_CONFIG_H_
#define INC__NODE_CONFIG_H_

/* Motor ECU selection: 1U = Motor1, 2U = Motor2. */
#define NODE_ID    1U

/* Keep bench tests possible without a live VCU heartbeat. */
#define MOTOR_TUNING_MODE    0U
#define MOTOR_REMOTE_TUNING_ENABLE 1U

#if ((NODE_ID != 1U) && (NODE_ID != 2U))
#error "NODE_ID must be 1U or 2U"
#endif

#if ((MOTOR_TUNING_MODE != 0U) && (MOTOR_TUNING_MODE != 1U))
#error "MOTOR_TUNING_MODE must be 0U or 1U"
#endif

#endif /* INC__NODE_CONFIG_H_ */
