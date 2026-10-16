#include "pcam.h"

#include "xaxivdma.h"
#include "sleep.h"
#include "xil_printf.h"
#include "xparameters.h"
#include "xstatus.h"

#include "../../hal/camera/cam_ae/cam_ae.h"
#include "../../hal/camera/cam_gpio/cam_gpio.h"
#include "../../hal/camera/display_ctrl_hdmi/display_ctrl.h"
#include "../../hal/camera/display_ctrl_hdmi/lcd_modes.h"
#include "../../hal/camera/gamma/gamma.h"
#include "../../hal/camera/iic_sccb_cfg/iic_sccb_cfg.h"
#include "../../hal/camera/mipi_rx/mipi_rx.h"
#include "../../hal/camera/ov5640/OV5640.h"
#include "../../hal/camera/vdma_api/vdma_api.h"

#define PCAM_VDMA_DEVICE_ID   (XPAR_AXIVDMA_0_DEVICE_ID)
#define PCAM_FRAME_BUFFER_ADDR \
    (XPAR_PS7_DDR_0_S_AXI_BASEADDR + 0x0A000000UL)
#define PCAM_VTC_DEVICE_ID    (XPAR_VTC_0_DEVICE_ID)
#define PCAM_DYNCLK_DEVICE_ID (XPAR_DYNCLK_0_DEVICE_ID)
#define PCAM_MANUAL_EXPOSURE_LINES 60U /* 嶺뚯쉧猷귛뜝占� �뜝�럥占쎌룊�삕�땻節륁삕�뜝占� �뜝�럥堉꾢뜝�럡�� �뛾�룇鍮볡뵳怨ㅼ삕占쎈꺄 �뜝�럥六삣뜝�럩�굚�뜝�럩�젍 */
#define PCAM_MANUAL_GAIN_X16      128U /* 8�뛾�룊�삕 �뇦猿딆뒩占쎈데. �뤆�룆占썬굩�삕 �뛾�룄�ｏ옙紐� * 16 */

static XAxiVdma s_vdma;
static DisplayCtrl s_display;
static VideoMode s_video_mode;

int32_t pcam_init(void)
{
    uint16_t width = 0U;
    uint16_t height = 0U;
    int32_t status = XST_FAILURE;

    xil_printf("\r\n\r\n");
    xil_printf("=================================================\r\n");
    xil_printf(" Zybo Z7-20 + Pcam 5C + HDMI 720p60\r\n");
    xil_printf("=================================================\r\n");

    if (cam_gpio_init() != CAM_GPIO_OK)
    {
        xil_printf("EMIO GPIO init failed\r\n");
    }
    else
    {
        mipi_rx_reset();
        mipi_rx_print_version();

        if (OV5640_Init() != IIC_SCCB_OK)
        {
            xil_printf("SCCB bus init failed\r\n");
        }
        else
        {
            OV5640_PowerCycle();
            if (OV5640_InitSensor() != 0)
            {
                xil_printf("OV5640 detection failed\r\n");
            }
            else
            {
                xil_printf("OV5640 detected successfully\r\n");

                if (iic_sccb_error_count() != 0U)
                {
                    xil_printf("WARNING: %d SCCB initialization errors\r\n",
                               (int)iic_sccb_error_count());
                    iic_sccb_clear_errors();
                }

                s_video_mode = VMODE_1280x720;
                run_vdma_frame_buffer(&s_vdma,
                                      PCAM_VDMA_DEVICE_ID,
                                      s_video_mode.width,
                                      s_video_mode.height,
                                      PCAM_FRAME_BUFFER_ADDR,
                                      0,
                                      0,
                                      BOTH);

                mipi_rx_enable();
                gamma_init();
                OV5640_SetMode720p();
                OV5640_SetAWB(AWB_ADVANCED);
                cam_ae_init();


                if (DisplayInitialize(&s_display,
                                     PCAM_VTC_DEVICE_ID,
                                     PCAM_DYNCLK_DEVICE_ID) != XST_SUCCESS)
                {
                    xil_printf("HDMI output init failed\r\n");
                }
                else
                {
                    DisplaySetMode(&s_display, &s_video_mode);
                    if (DisplayStart(&s_display) != XST_SUCCESS)
                    {
                        xil_printf("Pixel clock generation failed\r\n");
                    }
                    else
                    {
                        OV5640_GetImageInfo(&width, &height);
                        xil_printf("camera: %dx%d RAW10, 2 lane MIPI\r\n",
                                   (int)width,
                                   (int)height);
                        xil_printf("frame buffer: 0x%08X\r\n",
                                   (unsigned int)PCAM_FRAME_BUFFER_ADDR);
                        /* Read back the shutter after the sensor has run. */
                        sleep(1U);
                        OV5640_ModifySCCB(0x3503U, 0x03U, 0x03U);
                        //cam_ae_dump();
                        status = XST_SUCCESS;
                    }
                }
            }
        }
    }

    return status;
}
