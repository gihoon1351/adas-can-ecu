#ifndef SRC_COMMON_INTERRUPT_INTERRUPT_H_
#define SRC_COMMON_INTERRUPT_INTERRUPT_H_

#include <stdbool.h>
#include <stdint.h>

#include "xparameters.h"
#include "xintc.h"
#include "xil_exception.h"
#include "xuartlite.h"

#include "../../hal/can/can.h"

#define INTC_DEV_ID (XPAR_INTC_0_DEVICE_ID)
#define UARTLITE_DEV_ID (XPAR_UARTLITE_0_DEVICE_ID)

#define CAN_VEC_ID (XPAR_AXI_INTC_0_CAN_CONTROLLER_RX_INTR_INTR)
#define TIM1_VEC_ID (XPAR_AXI_INTC_0_TIME1_INTR_INTR)
#define TIM2_VEC_ID (XPAR_AXI_INTC_0_TIME2_INTR_INTR)
#define UART_VEC_ID (XPAR_AXI_INTC_0_AXI_UARTLITE_INTERRUPT_INTR)
#define SAFETY_VEC_ID (XPAR_AXI_INTC_0_SAFETY_DOMAIN_FAULT_IRQ_INTR)

extern volatile bool uart_rx_flag;
extern volatile uint8_t uart_rx_data;

/* TIM1: 10 ms tasks staggered at 1 ms offsets; CAN TX runs every 1 ms. */
extern volatile bool tim_can_rx_flag;
extern volatile bool tim_fault_check_flag;
extern volatile bool tim_control_flag;
extern volatile bool tim_can_tx_flag;

/* TIM2: 100 ms tasks staggered at 10 ms offsets. */
extern volatile bool tim_heartbeat_timeout_flag;
extern volatile bool tim_heartbeat_tx_flag;
extern volatile bool tim_wdt_kick_flag;
extern volatile bool tim_diagnostics_flag;

extern volatile bool safety_fault_irq_flag;

void SafetyDomain_EnableInterrupt(void);

int SetupInterruptSystem(void);

#endif /* SRC_COMMON_INTERRUPT_INTERRUPT_H_ */
