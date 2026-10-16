#ifndef CNN_DRIVER_H
#define CNN_DRIVER_H

#include <stdbool.h>
#include <stdint.h>

#include "../../hal/cnn/cnn_hal.h"
#include "../../hal/dma/dma_hal.h"
#include "xil_types.h"
#include "xtime_l.h"

typedef enum
{
    CNN_DRIVER_STATUS_OK = 0,
    CNN_DRIVER_STATUS_INVALID_ARGUMENT,
    CNN_DRIVER_STATUS_CNN_HAL_INIT_FAILED,
    CNN_DRIVER_STATUS_DMA_INIT_FAILED,
    CNN_DRIVER_STATUS_IDLE_TIMEOUT,
    CNN_DRIVER_STATUS_RX_START_FAILED,
    CNN_DRIVER_STATUS_TX_START_FAILED,
    CNN_DRIVER_STATUS_INFERENCE_TIMEOUT,
    CNN_DRIVER_STATUS_NOT_INITIALIZED,
    CNN_DRIVER_STATUS_PARAM_DMA_INIT_FAILED,
    CNN_DRIVER_STATUS_PARAM_DMA_CONFIG_INVALID,
    CNN_DRIVER_STATUS_PARAM_TX_START_FAILED,
    CNN_DRIVER_STATUS_PARAM_DMA_ERROR,
    CNN_DRIVER_STATUS_PARAM_PROTOCOL_ERROR,
    CNN_DRIVER_STATUS_PARAM_TIMEOUT,
    CNN_DRIVER_STATUS_PARAM_NOT_READY,
    CNN_DRIVER_STATUS_PARAM_DMA_RESET_FAILED,
    CNN_DRIVER_STATUS_IN_PROGRESS,
    CNN_DRIVER_STATUS_BUSY,
    CNN_DRIVER_STATUS_INFERENCE_DMA_ERROR
} CnnDriverStatus;

typedef struct
{
    UINTPTR cnn_base_address;
    uint16_t dma_device_id;
    uint16_t param_dma_device_id;
    const uint32_t *param_words;
    uint32_t param_byte_count;
    uint32_t param_timeout_ms;
    const uint32_t *weight_words;
    uint32_t weight_byte_count;
    uint32_t output_word_count;
    uint32_t idle_timeout_ms;
    uint32_t inference_timeout_ms;
} CnnDriverConfig;

typedef struct
{
    uint32_t elapsed_us;
    uint32_t output_checksum;
    CnnHalSnapshot hardware_status;
    uint32_t irq_elapsed_us;
} CnnDriverResult;

typedef struct
{
    CnnHal cnn;
    DmaHal dma;
    DmaHal param_dma;
    CnnDriverConfig config;
    uint32_t *output_words;
    uint32_t output_capacity_words;
    DmaHalStatus last_dma_status;
    uint32_t last_param_dma_hw_status;
    uint32_t last_param_cnn_status;
    bool initialized;
    volatile bool active;
    volatile bool done_irq;
    volatile uint32_t done_irq_count;
    XTime start_ticks;
    XTime done_irq_ticks;
    CnnHalSnapshot done_irq_snapshot;
} CnnDriver;

CnnDriverStatus CnnDriver_Init(CnnDriver *driver,
                               const CnnDriverConfig *config,
                               uint32_t *output_words,
                               uint32_t output_capacity_words);
CnnDriverStatus CnnDriver_Run(CnnDriver *driver, CnnDriverResult *result);
/* Runtime API: no completion/idle busy-wait; requires a connected GIC IRQ. */
CnnDriverStatus CnnDriver_StartAsync(CnnDriver *driver);
CnnDriverStatus CnnDriver_TryFinish(CnnDriver *driver, CnnDriverResult *result);
void CnnDriver_IrqHandler(void *context);
bool CnnDriver_IsActive(const CnnDriver *driver);
CnnHalSnapshot CnnDriver_GetHardwareStatus(const CnnDriver *driver);
DmaHalStatus CnnDriver_GetLastDmaStatus(const CnnDriver *driver);
uint32_t CnnDriver_GetParameterDmaStatus(const CnnDriver *driver);
uint32_t CnnDriver_GetParameterCnnStatus(const CnnDriver *driver);
const uint32_t *CnnDriver_GetOutput(const CnnDriver *driver);

#endif
