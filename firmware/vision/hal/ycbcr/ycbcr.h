#ifndef SRC_HAL_YCBCR_H_
#define SRC_HAL_YCBCR_H_
#include "xil_types.h"
/* ycbcr.xsa: AXI GPIO0 channel 1 bit 0 -> rgb2ycbcr_0.en. */
int ycbcr_init(void);
int ycbcr_set(u8 enabled);
u8 ycbcr_get(void);
#endif
