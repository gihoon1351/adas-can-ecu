#ifndef SRC_APP_UART_FILTER_CONTROL_H_
#define SRC_APP_UART_FILTER_CONTROL_H_

#include <stdbool.h>
#include <stdint.h>

/* Initial tuning values, not calibrated camera measurements. Inclusive. */
#define FILTER_Y_LOW_DEFAULT   (120U)
#define FILTER_Y_HIGH_DEFAULT  (255U)
#define FILTER_CB_LOW_DEFAULT  (0U)
#define FILTER_CB_HIGH_DEFAULT (140U)
#define FILTER_CR_LOW_DEFAULT  (105U)
#define FILTER_CR_HIGH_DEFAULT (200U)

/* Call before pcam_init starts video. Yellow and Sobel start enabled.
 * Returns XST_SUCCESS or XST_FAILURE. */
int UartFilterControl_Init(void);
/* Bounded RX polling; xil_printf can still block on the UART TX FIFO. */
void UartFilterControl_Poll(void);
void UartFilterControl_PrintHelp(void);
void UartFilterControl_PrintStatus(void);
bool UartFilterControl_SetThresholds(uint8_t y_low, uint8_t y_high,
                                     uint8_t cb_low, uint8_t cb_high,
                                     uint8_t cr_low, uint8_t cr_high);
bool UartFilterControl_GetThresholds(uint8_t values[6]);
#endif
