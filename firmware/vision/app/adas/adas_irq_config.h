#ifndef ADAS_IRQ_CONFIG_H
#define ADAS_IRQ_CONFIG_H

#include "xparameters.h"

/* Use the generated BSP IDs. Manual overrides belong HERE (or -D options).
 * Never copy an interrupt number from another bitstream's xlconcat order.
 * APP_X_AXIS_IRQ_ID is x_axis_finder DONE, not the sobel_filter IRQ.
 * APP_CNN_IRQ_ID is CNN accelerator DONE, not AXI DMA's MM2S/S2MM IRQ.
 */
#ifndef APP_GIC_DEVICE_ID
#if defined(XPAR_SCUGIC_SINGLE_DEVICE_ID)
#define APP_GIC_DEVICE_ID XPAR_SCUGIC_SINGLE_DEVICE_ID
#elif defined(XPAR_SCUGIC_0_DEVICE_ID)
#define APP_GIC_DEVICE_ID XPAR_SCUGIC_0_DEVICE_ID
#else
#error "GIC device ID missing: regenerate the Zynq standalone BSP"
#endif
#endif

#ifndef APP_X_AXIS_IRQ_ID
#if defined(XPAR_FABRIC_AXI_X_AXIS_FINDER_V1_0_0_IRQ_INTR)
#define APP_X_AXIS_IRQ_ID XPAR_FABRIC_AXI_X_AXIS_FINDER_V1_0_0_IRQ_INTR
#elif defined(XPAR_FABRIC_AXI_X_AXIS_FINDER_V1_0_IRQ_INTR)
#define APP_X_AXIS_IRQ_ID XPAR_FABRIC_AXI_X_AXIS_FINDER_V1_0_IRQ_INTR
#elif defined(XPAR_FABRIC_AXI_X_AXIS_FINDER_8_0_IRQ_INTR)
#define APP_X_AXIS_IRQ_ID XPAR_FABRIC_AXI_X_AXIS_FINDER_8_0_IRQ_INTR
#elif defined(XPAR_FABRIC_AXI_TEMPLATE_8_V1_0_0_IRQ_INTR)
#define APP_X_AXIS_IRQ_ID XPAR_FABRIC_AXI_TEMPLATE_8_V1_0_0_IRQ_INTR
#elif defined(XPAR_FABRIC_AXI_X_AXIS_FINDER_V1_0_0_INTERRUPT_INTR)
#define APP_X_AXIS_IRQ_ID XPAR_FABRIC_AXI_X_AXIS_FINDER_V1_0_0_INTERRUPT_INTR
#else
#error "Set APP_X_AXIS_IRQ_ID to this XSA's x_axis_finder PL->GIC interrupt ID"
#endif
#endif

#ifndef APP_CNN_IRQ_ID
#if defined(XPAR_FABRIC_CNN_V1_0_0_INTR_INTR)
#define APP_CNN_IRQ_ID XPAR_FABRIC_CNN_V1_0_0_INTR_INTR
#elif defined(XPAR_FABRIC_CNN_V1_0_0_INTERRUPT_INTR)
#define APP_CNN_IRQ_ID XPAR_FABRIC_CNN_V1_0_0_INTERRUPT_INTR
#else
#error "Set APP_CNN_IRQ_ID to this XSA's CNN accelerator PL->GIC interrupt ID"
#endif
#endif

/* 1 = active-high level (sticky DONE), 3 = rising edge (one-cycle IRQ).
 * Override independently if the existing hardware emits a pulse. */
#ifndef APP_X_AXIS_IRQ_TRIGGER
#define APP_X_AXIS_IRQ_TRIGGER (3U)
#endif
#ifndef APP_CNN_IRQ_TRIGGER
#define APP_CNN_IRQ_TRIGGER (3U)
#endif
#define APP_X_AXIS_IRQ_PRIORITY (0x90U)
#define APP_CNN_IRQ_PRIORITY    (0xA0U)

#endif
