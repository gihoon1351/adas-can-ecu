/*
 * sobel.h
 *
 *  Created on: 2026. 9. 21.
 *      Author: kccistc
 */

#ifndef SRC_SOBEL_SOBEL_H_
#define SRC_SOBEL_SOBEL_H_

#include <stdint.h>

#include "xil_types.h"

#define SOBEL_THR_DEFAULT       (255U)
#define SOBEL_THR_STEP          (10U)
#define SOBEL_THR_MIN           (0U)
#define SOBEL_THR_MAX           (255U)

int sobel_init(void);
void sobel_set(u8 enabled);
u8 sobel_get(void);
void sobel_set_threshold(u8 th);
u8 sobel_get_threshold(void);
void sobel_adjust_threshold(int32_t delta);
u8 sobel_is_ready(void);

#endif /* SRC_SOBEL_SOBEL_H_ */
