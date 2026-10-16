#include "uart_filter_control.h"
#include <stddef.h>
#include "xil_printf.h"
#include "xparameters.h"
#include "xstatus.h"
#include "xuartps_hw.h"
#include "../driver/yellow/yellow_driver.h"
#include "../hal/sobel/sobel.h"
#include "../hal/x_axis/x_axis.h"
#include "../hal/ycbcr/ycbcr.h"
#include "../hal/uart_log/uart_log_hal.h"
#define xil_printf UartLogHal_Printf

#if defined(STDIN_BASEADDRESS)
#define UART_FILTER_BASEADDR ((UINTPTR)STDIN_BASEADDRESS)
#elif defined(XPAR_XUARTPS_0_BASEADDR)
#define UART_FILTER_BASEADDR ((UINTPTR)XPAR_XUARTPS_0_BASEADDR)
#elif defined(XPAR_PS7_UART_1_BASEADDR)
#define UART_FILTER_BASEADDR ((UINTPTR)XPAR_PS7_UART_1_BASEADDR)
#else
#error "BSP UART base address is missing"
#endif

#if defined(XPAR_YELLOW_FILTER_V1_0_0_BASEADDR)
#define YELLOW_BASEADDR ((UINTPTR)XPAR_YELLOW_FILTER_V1_0_0_BASEADDR)
#elif defined(XPAR_YELLOW_FILTER_V1_0_0_S00_AXI_BASEADDR)
#define YELLOW_BASEADDR ((UINTPTR)XPAR_YELLOW_FILTER_V1_0_0_S00_AXI_BASEADDR)
#else
#error "Update the hardware platform/BSP from ycbcr.xsa: yellow filter is missing"
#endif

#define UART_FILTER_MAX_CHARS_PER_POLL (16U)
static YellowDriver s_yellow;
static bool s_ready;
static unsigned int s_selected; /* Y low/high, Cb low/high, Cr low/high */
static bool s_editing;
static bool s_bad_number;
static unsigned int s_digits;
static unsigned int s_value;
static const char *const s_names[6] =
    {"Y low", "Y high", "Cb low", "Cb high", "Cr low", "Cr high"};

static void SetSelected(int value, bool relative)
{
    YellowDriverConfig config;
    YellowRange range;
    YellowDriverStatus status = YellowDriver_ReadConfig(&s_yellow, &config);
    const bool upper = (s_selected & 1U) != 0U;
    const YellowHalChannel channel = (YellowHalChannel)(s_selected / 2U);
    if (status != YELLOW_DRIVER_STATUS_OK) {
        xil_printf("Yellow read failed: %d\r\n", (int)status);
        return;
    }
    range = (channel == YELLOW_HAL_CHANNEL_Y) ? config.thresholds.y :
            ((channel == YELLOW_HAL_CHANNEL_CB) ? config.thresholds.cb : config.thresholds.cr);
    if (relative) {
        value += (int)(upper ? range.high : range.low);
        if (value < 0) value = 0;
        if (value > 255) value = 255;
        if (upper && value < (int)range.low) value = (int)range.low;
        if (!upper && value > (int)range.high) value = (int)range.high;
    }
    if (upper) range.high = (uint16_t)value;
    else range.low = (uint16_t)value;
    status = YellowDriver_SetRange(&s_yellow, channel, range);
    if (status != YELLOW_DRIVER_STATUS_OK)
        xil_printf("Threshold rejected/failed: %d (require 0 <= low <= high <= 255)\r\n", (int)status);
    UartFilterControl_PrintStatus();
}

/* Numeric entry is v<decimal><Enter>; no hardware write before Enter.
 * A malformed/oversized line is consumed and rejected as a whole. */
static void Process(u8 command)
{
    if (s_editing) {
        if (command == 27U) { s_editing = false; return; }
        if ((command == '\r') || (command == '\n')) {
            s_editing = false;
            if (s_bad_number || s_digits == 0U)
                xil_printf("Invalid value. Use v0..v255 then Enter.\r\n");
            else SetSelected((int)s_value, false);
        } else if ((command == 8U) || (command == 127U)) {
            if (!s_bad_number && s_digits > 0U) { s_value /= 10U; --s_digits; }
        } else if (command >= '0' && command <= '9' && !s_bad_number) {
            if (s_digits == 3U) s_bad_number = true;
            else {
                s_value = s_value * 10U + (unsigned int)(command - '0');
                ++s_digits;
                if (s_value > 255U) s_bad_number = true;
            }
        } else s_bad_number = true;
        return;
    }
    if (x_axis_roi_uart_handle(command)) {
        return;
    }
    if (command >= '1' && command <= '6') {
        s_selected = (unsigned int)(command - '1');
        UartFilterControl_PrintStatus();
        return;
    }
    switch (command) {
    case 's': case 'S':
        sobel_set((sobel_get() == 0U) ? 1U : 0U);
        UartFilterControl_PrintStatus(); break;
    case 'q': case 'Q':
        sobel_adjust_threshold(-(int32_t)SOBEL_THR_STEP);
        UartFilterControl_PrintStatus(); break;
    case 'w': case 'W':
        sobel_adjust_threshold((int32_t)SOBEL_THR_STEP);
        UartFilterControl_PrintStatus(); break;
    case 'b': case 'B': {
        YellowDriverConfig config;
        YellowDriverStatus status = YellowDriver_ReadConfig(&s_yellow, &config);
        if (status == YELLOW_DRIVER_STATUS_OK)
            status = YellowDriver_SetEnabled(&s_yellow, !config.enabled);
        if (status != YELLOW_DRIVER_STATUS_OK)
            xil_printf("Yellow toggle failed: %d\r\n", (int)status);
        UartFilterControl_PrintStatus(); break;
    }
    case 'a': case '-': SetSelected(-1, true); break;
    case 'd': case '+': SetSelected(1, true); break;
    case 'A': SetSelected(-10, true); break;
    case 'D': SetSelected(10, true); break;
    case 'v': case 'V':
        s_editing = true; s_bad_number = false; s_value = 0U; s_digits = 0U;
        xil_printf("Set %s: decimal 0..255 + Enter (Esc cancels)\r\n", s_names[s_selected]); break;
    case 'p': case 'P': UartFilterControl_PrintStatus(); break;
    case 'h': case 'H': case '?': UartFilterControl_PrintHelp(); break;
    case '\r': case '\n': case ' ': case 27: break;
    default: xil_printf("Unknown filter command. Press h for help.\r\n"); break;
    }
}

int UartFilterControl_Init(void)
{
    YellowDriverConfig config;
    YellowDriverStatus status;
    if (s_ready) return XST_SUCCESS;
    config.base_address = YELLOW_BASEADDR;
    config.thresholds.y.low = FILTER_Y_LOW_DEFAULT;
    config.thresholds.y.high = FILTER_Y_HIGH_DEFAULT;
    config.thresholds.cb.low = FILTER_CB_LOW_DEFAULT;
    config.thresholds.cb.high = FILTER_CB_HIGH_DEFAULT;
    config.thresholds.cr.low = FILTER_CR_LOW_DEFAULT;
    config.thresholds.cr.high = FILTER_CR_HIGH_DEFAULT;
    config.enabled = true;
    if (ycbcr_init() != XST_SUCCESS || ycbcr_set(1U) != XST_SUCCESS) {
        xil_printf("YCBCR_INIT_ERROR\r\n"); return XST_FAILURE;
    }
    status = YellowDriver_Init(&s_yellow, &config);
    if (status != YELLOW_DRIVER_STATUS_OK) {
        xil_printf("YELLOW_INIT_ERROR: %d\r\n", (int)status); return XST_FAILURE;
    }
    if (sobel_init() != XST_SUCCESS) {
        xil_printf("SOBEL_INIT_ERROR\r\n"); return XST_FAILURE;
    }
    s_ready = true;
    s_selected = 0U;
    s_editing = false;
    UartFilterControl_PrintHelp();
    UartFilterControl_PrintStatus();
    return XST_SUCCESS;
}

void UartFilterControl_Poll(void)
{
    u32 processed = 0U;
    if (!s_ready) return;
    while (processed < UART_FILTER_MAX_CHARS_PER_POLL &&
           XUartPs_IsReceiveData(UART_FILTER_BASEADDR) != 0U) {
        Process((u8)XUartPs_ReadReg(UART_FILTER_BASEADDR, XUARTPS_FIFO_OFFSET));
        ++processed;
    }
}

void UartFilterControl_PrintHelp(void)
{
    xil_printf("\r\n[UART filter control]\r\n");
    xil_printf(" s: Sobel ON/OFF, q/w: Sobel threshold -/+10\r\n");
    xil_printf(" b: Yellow ON/OFF (YCbCr conversion stays ON)\r\n");
    xil_printf(" 1/2: select Y low/high, 3/4: Cb low/high, 5/6: Cr low/high\r\n");
    xil_printf(" a/d or -/+: selected bound -/+1; A/D: -/+10\r\n");
    xil_printf(" v<number><Enter>: exact value, e.g. 4v100<Enter>; Esc: cancel\r\n");
    xil_printf(" p: programmed settings; h/?: help. Changes apply at input SOF.\r\n");
    xil_printf(" ROI: r select, ,/. -/+, </> large -/+, z reset, o status, O help\r\n");
}

void UartFilterControl_PrintStatus(void)
{
    YellowDriverConfig config;
    if (!s_ready) { xil_printf("Filters not initialized\r\n"); return; }
    if (YellowDriver_ReadConfig(&s_yellow, &config) != YELLOW_DRIVER_STATUS_OK) {
        xil_printf("Yellow status read failed\r\n"); return;
    }
    xil_printf("FILTER programmed: YCbCr=%u Yellow=%u Y=[%u,%u] Cb=[%u,%u] Cr=[%u,%u] "
               "Sobel=%u threshold=%u select=%s\r\n",
               (unsigned int)ycbcr_get(), (unsigned int)config.enabled,
               (unsigned int)config.thresholds.y.low, (unsigned int)config.thresholds.y.high,
               (unsigned int)config.thresholds.cb.low, (unsigned int)config.thresholds.cb.high,
               (unsigned int)config.thresholds.cr.low, (unsigned int)config.thresholds.cr.high,
               (unsigned int)sobel_get(), (unsigned int)sobel_get_threshold(), s_names[s_selected]);
}

bool UartFilterControl_SetThresholds(uint8_t y_low, uint8_t y_high,
                                     uint8_t cb_low, uint8_t cb_high,
                                     uint8_t cr_low, uint8_t cr_high)
{
    YellowThresholds thresholds;
    if (!s_ready || y_low > y_high || cb_low > cb_high || cr_low > cr_high)
        return false;
    thresholds.y.low = y_low;
    thresholds.y.high = y_high;
    thresholds.cb.low = cb_low;
    thresholds.cb.high = cb_high;
    thresholds.cr.low = cr_low;
    thresholds.cr.high = cr_high;
    return YellowDriver_SetThresholds(&s_yellow, &thresholds) ==
           YELLOW_DRIVER_STATUS_OK;
}

bool UartFilterControl_GetThresholds(uint8_t values[6])
{
    YellowDriverConfig config;
    if (!s_ready || values == NULL ||
        YellowDriver_ReadConfig(&s_yellow, &config) != YELLOW_DRIVER_STATUS_OK)
        return false;
    values[0] = (uint8_t)config.thresholds.y.low;
    values[1] = (uint8_t)config.thresholds.y.high;
    values[2] = (uint8_t)config.thresholds.cb.low;
    values[3] = (uint8_t)config.thresholds.cb.high;
    values[4] = (uint8_t)config.thresholds.cr.low;
    values[5] = (uint8_t)config.thresholds.cr.high;
    return true;
}
