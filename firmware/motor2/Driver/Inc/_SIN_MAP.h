#ifndef SIN_MAP_H_
#define SIN_MAP_H_

#include <stdint.h>

#define SIN_LUT_SIZE   1024
#define SIN_LUT_MASK   0x03FF // 2**10
#define SIN_LUT_SCALE  4096

extern const int16_t sin_pi_m2[SIN_LUT_SIZE + 1];

static inline int16_t SinLUT_Get(uint16_t angle14)
{
    uint16_t idx = (angle14 >> 4) & SIN_LUT_MASK; // 0~16383 -> 0~1023
    return sin_pi_m2[idx];
}

static inline int16_t CosLUT_Get(uint16_t angle14)
{
    uint16_t idx = ((angle14 >> 4) + 256) & SIN_LUT_MASK; // +90도
    return sin_pi_m2[idx];
}

#endif /* SIN_MAP_H_ */
