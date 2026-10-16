#ifndef CNN_BOX_CONFIG_H
#define CNN_BOX_CONFIG_H

#include "xparameters.h"

/*
 * Use the address generated from the currently selected XSA/BSP.
 * Do not hard-code 0x43CB0000 here: that address is used by the YCbCr/yellow
 * filter in the merged design. Assign cnn_result_box_v1_0_0 a free AXI-Lite
 * segment in Vivado, then regenerate the platform/BSP.
 */
#if defined(XPAR_CNN_RESULT_BOX_V1_0_0_BASEADDR)
#define APP_CNN_BOX_BASEADDR XPAR_CNN_RESULT_BOX_V1_0_0_BASEADDR
#elif defined(XPAR_CNN_RESULT_BOX_V1_0_0_S00_AXI_BASEADDR)
#define APP_CNN_BOX_BASEADDR XPAR_CNN_RESULT_BOX_V1_0_0_S00_AXI_BASEADDR
#else
#error "CNN result-box IP missing: add it to the video path and regenerate the BSP"
#endif

/* 24-bit RGB red outline, four pixels wide in the 1280x720 output. */
#define APP_CNN_BOX_COLOR     (0x00FF0000UL)
#define APP_CNN_BOX_THICKNESS (4U)

#endif /* CNN_BOX_CONFIG_H */
