/*
 * sobel.c
 *
 * Created on: 2026. 9. 21.
 *     Author: kccistc
 */

#include "sobel.h"
#include "xparameters.h"
#include "xil_io.h"
#include "xil_printf.h"
#include "xstatus.h"

#define SOBEL_BASE              XPAR_SOBEL_FILTER_V1_0_0_BASEADDR
#define SOBEL_CTRL_OFFSET       (0x00U)
#define SOBEL_THR_OFFSET        (0x04U)
#define SOBEL_ENABLE_MASK       (0x00000001U)
#define SOBEL_DISABLE_THRESHOLD (0x0000FFFFU)

#define WR(off, value) \
    Xil_Out32((UINTPTR)(SOBEL_BASE + (off)), (u32)(value))

#define RD(off) \
    Xil_In32((UINTPTR)(SOBEL_BASE + (off)))

static u8 ready = 0U;
static u8 enabled_shadow = 1U;
static u8 threshold_shadow = (u8)SOBEL_THR_DEFAULT;


int sobel_init(void)
{
    u32 control;
    u8 readback;

    if (ready != 0U) {
        return XST_SUCCESS;
    }

    /* REG1 contains the 8-bit software threshold. */
    WR(SOBEL_THR_OFFSET, SOBEL_THR_DEFAULT);

    /* REG0 bit 0 selects Sobel processing (1) or bypass (0). */
    control = RD(SOBEL_CTRL_OFFSET);
    control |= SOBEL_ENABLE_MASK;
    WR(SOBEL_CTRL_OFFSET, control);

    readback = (u8)(RD(SOBEL_THR_OFFSET) & 0xFFU);

    if (readback != (u8)SOBEL_THR_DEFAULT) {
        xil_printf(
            "sobel_filter : register test failed "
            "(write %d, read %d)\r\n",
            (int)SOBEL_THR_DEFAULT,
            (int)readback
        );

        return XST_FAILURE;
    }

    ready = 1U;
    enabled_shadow = 1U;
    threshold_shadow = (u8)SOBEL_THR_DEFAULT;

    xil_printf(
        "sobel_filter : ok at 0x%08X (threshold %d)\r\n",
        (unsigned int)SOBEL_BASE,
        (int)readback
    );

    return XST_SUCCESS;
}

void sobel_set(u8 enabled)
{
    u32 control;

    if (ready == 0U) {
        return;
    }

    control = RD(SOBEL_CTRL_OFFSET);
    control &= ~SOBEL_ENABLE_MASK;

    if (enabled != 0U) {
        control |= SOBEL_ENABLE_MASK;
        WR(SOBEL_THR_OFFSET, (u32)threshold_shadow);
        enabled_shadow = 1U;
    } else {
        /*
         * The high threshold suppresses every edge if an older RTL build has
         * REG0 implemented as storage but has not connected cfg_en yet.
         * With proper RTL bypass support, REG0 still performs the real bypass.
         */
        WR(SOBEL_THR_OFFSET, SOBEL_DISABLE_THRESHOLD);
        enabled_shadow = 0U;
    }

    WR(SOBEL_CTRL_OFFSET, control);
}

u8 sobel_get(void)
{
    return (ready != 0U) ? enabled_shadow : 0U;
}

void sobel_set_threshold(u8 th)
{
    if (ready == 0U) {
        return;
    }

    threshold_shadow = th;
    if (enabled_shadow != 0U) {
        WR(SOBEL_THR_OFFSET, (u32)threshold_shadow);
    }
}


u8 sobel_get_threshold(void)
{
    return (ready != 0U) ? threshold_shadow : 0U;
}


void sobel_adjust_threshold(int32_t delta)
{
    int32_t value;

    if (ready == 0U) {
        return;
    }

    value = (int32_t)sobel_get_threshold() + delta;

    if (value < (int32_t)SOBEL_THR_MIN) {
        value = (int32_t)SOBEL_THR_MIN;
    } else if (value > (int32_t)SOBEL_THR_MAX) {
        value = (int32_t)SOBEL_THR_MAX;
    }

    sobel_set_threshold((u8)value);
}

u8 sobel_is_ready(void)
{
    return ready;
}
