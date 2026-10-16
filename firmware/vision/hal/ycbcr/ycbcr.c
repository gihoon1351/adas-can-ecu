#include "ycbcr.h"
#include "xgpio.h"
#include "xparameters.h"
#include "xstatus.h"

#if defined(XPAR_AXI_GPIO_0_DEVICE_ID)
#define YCBCR_GPIO_DEVICE_ID XPAR_AXI_GPIO_0_DEVICE_ID
#elif defined(XPAR_XGPIO_0_DEVICE_ID)
#define YCBCR_GPIO_DEVICE_ID XPAR_XGPIO_0_DEVICE_ID
#else
#error "Update the BSP from ycbcr.xsa: AXI GPIO0 is missing"
#endif

static XGpio s_gpio;
static u8 s_ready;

int ycbcr_init(void)
{
    if (s_ready != 0U) return XST_SUCCESS;
    if (XGpio_Initialize(&s_gpio, YCBCR_GPIO_DEVICE_ID) != XST_SUCCESS)
        return XST_FAILURE;
    XGpio_SetDataDirection(&s_gpio, 1U, 0U);
    XGpio_DiscreteWrite(&s_gpio, 1U, 0U);
    s_ready = 1U;
    return XST_SUCCESS;
}

int ycbcr_set(u8 enabled)
{
    u32 control;
    if (s_ready == 0U) return XST_FAILURE;
    control = XGpio_DiscreteRead(&s_gpio, 1U);
    control = (control & ~1U) | ((enabled != 0U) ? 1U : 0U);
    XGpio_DiscreteWrite(&s_gpio, 1U, control);
    return ((XGpio_DiscreteRead(&s_gpio, 1U) & 1U) == (control & 1U)) ?
           XST_SUCCESS : XST_FAILURE;
}

u8 ycbcr_get(void)
{
    return (s_ready != 0U) ? (u8)(XGpio_DiscreteRead(&s_gpio, 1U) & 1U) : 0U;
}
