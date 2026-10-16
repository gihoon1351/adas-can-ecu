/*
 * adas.h
 *
 *  Created on: 2026. 9. 29.
 *      Author: kccistc
 */

#ifndef SRC_APP_ADAS_ADAS_H_
#define SRC_APP_ADAS_ADAS_H_

#include "sleep.h"
#include "xil_cache.h"
#include "xil_printf.h"
#include "xparameters.h"
#include "xstatus.h"
#include "xtime_l.h"

#include "../adas_can_app.h"
#include "../benchmark/cnn_benchmark.h"
#include "../cnn_postprocess_fixed.h"
#include "../cnn_result_app.h"
#include "../benchmark/cnn_sw_reference.h"
#include "../../driver/cnn/cnn_driver.h"
#include "../../driver/cnn/cnn_weight_data.h"
#include "../../driver/cnn/cnn_param_data.h"
#include "../../driver/pcam/pcam.h"
#include "../../hal/sobel/sobel.h"
#include "../../hal/x_axis/x_axis.h"
#include "../../driver/x_axis/x_axis_driver.h"

#define		SOBEL_AVG			10

#if defined(XPAR_AXIDMA_0_DEVICE_ID)
#define APP_CNN_DMA_DEVICE_ID (XPAR_AXIDMA_0_DEVICE_ID)
#elif defined(XPAR_AXI_DMA_0_DEVICE_ID)
#define APP_CNN_DMA_DEVICE_ID (XPAR_AXI_DMA_0_DEVICE_ID)
#else
#error "CNN AXI DMA device ID is missing from xparameters.h"
#endif

/* cnn_dma.xsa: axi_dma_1 MM2S at 0x40410000 -> CNN S_AXIS_PARAM. */
#if defined(XPAR_AXI_DMA_1_DEVICE_ID)
#define APP_CNN_PARAM_DMA_DEVICE_ID (XPAR_AXI_DMA_1_DEVICE_ID)
#elif defined(XPAR_AXIDMA_1_DEVICE_ID)
#define APP_CNN_PARAM_DMA_DEVICE_ID (XPAR_AXIDMA_1_DEVICE_ID)
#else
#error "Parameter DMA1 missing: update hardware platform from cnn_dma.xsa and regenerate BSP"
#endif
#define APP_CNN_PARAM_TIMEOUT_MS      (1000UL)

#define APP_CNN_IDLE_TIMEOUT_MS       (100UL)
#define APP_CNN_INFERENCE_TIMEOUT_MS  (5000UL)
#define APP_CNN_PERIOD_MS             (50UL) /* target: 20 starts/second */
#define APP_IO_POLL_PERIOD_US         (100UL) /* not a Sobel sampling timer */
#define APP_CAN_LOG_PERIOD_MS         (500UL)
#define APP_RATE_LOG_PERIOD_MS        (1000UL)
#ifndef APP_CNN_VERBOSE_LOG_ENABLE
#define APP_CNN_VERBOSE_LOG_ENABLE    (0U)
#endif
#ifndef APP_CNN_BENCHMARK_ENABLE
#define APP_CNN_BENCHMARK_ENABLE          (0U)
#endif
#ifndef APP_CNN_HW_BENCHMARK_WARMUP_RUNS
#define APP_CNN_HW_BENCHMARK_WARMUP_RUNS  (2U)
#endif
#ifndef APP_CNN_HW_BENCHMARK_RUNS
#define APP_CNN_HW_BENCHMARK_RUNS         (20U)
#endif
#ifndef APP_CNN_SW_BENCHMARK_WARMUP_RUNS
#define APP_CNN_SW_BENCHMARK_WARMUP_RUNS  (0U)
#endif
#ifndef APP_CNN_SW_BENCHMARK_RUNS
#define APP_CNN_SW_BENCHMARK_RUNS         (1U)
#endif
#define APP_IO_POLLS_PER_CNN_PERIOD \
    ((APP_CNN_PERIOD_MS * 1000UL) / APP_IO_POLL_PERIOD_US)

/* Required DMA alignment. Deviation: GCC alignment attribute is used because
 * the Xilinx standalone BSP has no portable C11 aligned allocation service. */

/* Reserve a whole number of cache lines so DMA invalidation cannot touch
 * a neighboring mutable object. OA transfer remains exactly 392 words. */
#define APP_CNN_OUTPUT_CAPACITY_WORDS ((CNN_PP_OA_WORDS + 15U) & ~15U)
extern uint32_t s_cnn_output[APP_CNN_OUTPUT_CAPACITY_WORDS];
extern CnnDriver s_cnn_driver;
extern CanHw s_can;
extern AdasCanApp s_adas;
extern bool s_can_initialized;

uint32_t App_NowMs(void);
void App_PrintCnnStatus(CnnHalSnapshot snapshot);
void App_PollIo(uint32_t now_ms);
void App_ServiceCnn(uint32_t now_ms);
bool App_Init(void);

#endif /* SRC_APP_ADAS_ADAS_H_ */
