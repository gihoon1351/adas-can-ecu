#include "interrupt.h"
#include "../delay/delay.h"
#include "../../driver/can/can.h"
#include "../../hal/safety_domain/safety_domain.h"

#include <stddef.h>

static XIntc IntrController;
static XUartLite UartLite;

static uint8_t uart_rx_buffer = 0U;

volatile bool uart_rx_flag = false;
volatile uint8_t uart_rx_data = 0U;
volatile bool safety_fault_irq_flag = false;

/* =========================================================
 * Interrupt Callbacks
 * ========================================================= */

/* Save the received byte and re-arm the one-byte UART receive. */
static void UARTLite_RecvHandler(void *CallbackRef, unsigned int EventData)
{
	(void)CallbackRef;

	if (EventData > 0U) {
		uart_rx_data = uart_rx_buffer;
		uart_rx_flag = true;

		/* Prepare to receive the next byte. */
		(void)XUartLite_Recv(&UartLite, &uart_rx_buffer, 1U);
	}
}

/* Forward the receive interrupt to the CAN driver. */
static void CAN_RxISR(void *CallbackRef)
{
	(void)CallbackRef;

	CAN_RxCallback();
}

/* Mask the level-sensitive vector and defer fault handling to main context. */
static void SafetyDomain_ISR(void *CallbackRef)
{
	(void)CallbackRef;

	safety_fault_irq_flag = true;
	XIntc_Disable(&IntrController, SAFETY_VEC_ID);
}

/* Re-enable the safety vector after main-context fault handling. */
void SafetyDomain_EnableInterrupt(void)
{
	XIntc_Enable(&IntrController, SAFETY_VEC_ID);
}

/* =========================================================
 * Timer Callbacks
 * ========================================================= */

volatile bool tim_can_rx_flag = false;
volatile bool tim_fault_check_flag = false;
volatile bool tim_control_flag = false;
volatile bool tim_can_tx_flag = false;

volatile bool tim_heartbeat_timeout_flag = false;
volatile bool tim_heartbeat_tx_flag = false;
volatile bool tim_wdt_kick_flag = false;
volatile bool tim_diagnostics_flag = false;

/* TIM1 runs every 1 ms. RX, fault, and control tasks have a 10 ms period.
 * CAN TX is released on every tick; CAN_ProcessTx() still sends only when
 * the controller reports that it is ready. */
static void SchedulerTimer_ISR(void *CallbackRef)
{
	static uint8_t phase_10ms = 0U;

	(void)CallbackRef;

	incTick();
	tim_can_tx_flag = true;

	switch (phase_10ms) {
	case 0U:
		tim_can_rx_flag = true;
		break;
	case 1U:
		tim_fault_check_flag = true;
		break;
	case 2U:
		tim_control_flag = true;
		break;
	default:
		break;
	}

	phase_10ms++;
	if (phase_10ms >= 10U) {
		phase_10ms = 0U;
	}
}

/* TIM2 runs every 10 ms. Each task has a 100 ms period and is released
 * at a separate 10 ms phase. */
static void Timer2_ISR(void *CallbackRef)
{
	static uint8_t phase_100ms = 0U;

	(void)CallbackRef;

	switch (phase_100ms) {
	case 0U:
		tim_heartbeat_timeout_flag = true;
		break;
	case 1U:
		tim_heartbeat_tx_flag = true;
		break;
	case 2U:
		tim_wdt_kick_flag = true;
		break;
	case 3U:
		tim_diagnostics_flag = true;
		break;
	default:
		break;
	}

	phase_100ms++;
	if (phase_100ms >= 10U) {
		phase_100ms = 0U;
	}
}

/* =========================================================
 * Interrupt System Setup
 * ========================================================= */

int SetupInterruptSystem(void)
{
	int status;

	/* Initialize UARTLite. */
	status = XUartLite_Initialize(&UartLite, UARTLITE_DEV_ID);

	if (status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	XUartLite_SetRecvHandler(&UartLite, UARTLite_RecvHandler, &UartLite);

	/* Initialize the interrupt controller. */
	status = XIntc_Initialize(&IntrController, INTC_DEV_ID);

	if (status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	/* Connect the CAN receive handler. */
	status = XIntc_Connect(&IntrController, CAN_VEC_ID, (XInterruptHandler)CAN_RxISR, NULL);

	if (status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	/* Connect the UARTLite interrupt handler. */
	status = XIntc_Connect(&IntrController, UART_VEC_ID, (XInterruptHandler)XUartLite_InterruptHandler, &UartLite);

	if (status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	/* Connect the system tick timer handler. */
	status = XIntc_Connect(&IntrController, TIM1_VEC_ID, (XInterruptHandler)SchedulerTimer_ISR, NULL);

	if (status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	/* Connect the periodic application timer handler. */
	status = XIntc_Connect(&IntrController, TIM2_VEC_ID, (XInterruptHandler)Timer2_ISR, NULL);

	if (status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	/* Connect the safety-domain fault handler. */
	status = XIntc_Connect(&IntrController, SAFETY_VEC_ID, (XInterruptHandler)SafetyDomain_ISR, NULL);

	if (status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	status = XIntc_Start(&IntrController, XIN_REAL_MODE);

	if (status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	/* Clear any pending CAN receive interrupt before enabling vectors. */
	CAN_ClearRxInterrupt(CAN0);

	/* Enable peripheral interrupt vectors. */
	XIntc_Enable(&IntrController, CAN_VEC_ID);
	XIntc_Enable(&IntrController, UART_VEC_ID);
	XIntc_Enable(&IntrController, TIM1_VEC_ID);
	XIntc_Enable(&IntrController, TIM2_VEC_ID);
	XIntc_Enable(&IntrController, SAFETY_VEC_ID);

	/* Initialize CPU exception handling and register the dispatcher. */
	Xil_ExceptionInit();

	Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT, (Xil_ExceptionHandler)XIntc_InterruptHandler, &IntrController);

	/* Arm one-byte UART reception and enable UART interrupts. */
	(void)XUartLite_Recv(&UartLite, &uart_rx_buffer, 1U);

	XUartLite_EnableInterrupt(&UartLite);

	/* Enable CPU-level interrupts. */
	Xil_ExceptionEnable();

	return XST_SUCCESS;
}
