#include "cnn_driver.h"

#include <stddef.h>

#include "xil_cache.h"
#include "xtime_l.h"
#include "../../hal/irq/irq_hal.h"

#define CNN_DRIVER_PARAM_BYTE_COUNT (16384UL)
#define CNN_DRIVER_PARAM_ALIGNMENT_MASK ((UINTPTR)63U)

#define CNN_DRIVER_BYTES_PER_WORD (4UL)
#define CNN_DRIVER_FNV_OFFSET      (2166136261UL)
#define CNN_DRIVER_FNV_PRIME       (16777619UL)

static bool CnnDriver_TimeoutExpired(XTime start, uint32_t timeout_ms)
{
    XTime now;
    uint64_t elapsed_ticks;
    uint64_t limit_ticks;

    XTime_GetTime(&now);
    elapsed_ticks = (uint64_t)(now - start);
    limit_ticks = ((uint64_t)COUNTS_PER_SECOND * (uint64_t)timeout_ms) /
                  1000ULL;

    return elapsed_ticks > limit_ticks;
}

static uint32_t CnnDriver_ElapsedUs(XTime start, XTime end)
{
    const uint64_t ticks = (uint64_t)(end - start);
    const uint64_t microseconds =
        (ticks * 1000000ULL) / (uint64_t)COUNTS_PER_SECOND;

    return (uint32_t)microseconds;
}

static uint32_t CnnDriver_Checksum(const uint32_t *data,
                                   uint32_t word_count)
{
    uint32_t hash = CNN_DRIVER_FNV_OFFSET;
    uint32_t index;

    if (data != NULL)
    {
        for (index = 0U; index < word_count; ++index)
        {
            hash ^= data[index];
            hash *= CNN_DRIVER_FNV_PRIME;
        }
    }

    return hash;
}

static void CnnDriver_ClearOutput(CnnDriver *driver)
{
    uint32_t index;

    for (index = 0U; index < driver->config.output_word_count; ++index)
    {
        driver->output_words[index] = 0U;
    }
}

static CnnDriverStatus CnnDriver_WaitForIdle(CnnDriver *driver)
{
    CnnDriverStatus status = CNN_DRIVER_STATUS_OK;
    XTime start;

    XTime_GetTime(&start);
    while (!CnnHal_IsIdle(&driver->cnn))
    {
        if (CnnDriver_TimeoutExpired(start, driver->config.idle_timeout_ms))
        {
            status = CNN_DRIVER_STATUS_IDLE_TIMEOUT;
            break;
        }
    }

    return status;
}

/* Initialization only: DMA1 -> S_AXIS_PARAM, one complete bias+scale packet. */
static CnnDriverStatus CnnDriver_LoadParameters(CnnDriver *driver)
{
    CnnDriverStatus status;
    DmaHalStatus reset_status;
    CnnHalSnapshot snapshot;
    uint32_t dma_status;
    XTime start;

    /* Prepare clears START; parameters may only be written while CNN is idle. */
    CnnHal_Prepare(&driver->cnn);
    status = CnnDriver_WaitForIdle(driver);
    if (status != CNN_DRIVER_STATUS_OK)
    {
        return status;
    }

    driver->last_dma_status = DmaHal_Init(
        &driver->param_dma, driver->config.param_dma_device_id);
    if (driver->last_dma_status != DMA_HAL_STATUS_OK)
    {
        return CNN_DRIVER_STATUS_PARAM_DMA_INIT_FAILED;
    }
    if (!DmaHal_TxLengthSupported(&driver->param_dma,
                                  driver->config.param_byte_count))
    {
        return CNN_DRIVER_STATUS_PARAM_DMA_CONFIG_INVALID;
    }

    /* DMA reset is already complete; discard any previous partial loader state. */
    CnnHal_AbortParameterLoad(&driver->cnn);
    DmaHal_ClearTxIrq(&driver->param_dma);
    Xil_DCacheFlushRange((UINTPTR)driver->config.param_words,
                         (u32)driver->config.param_byte_count);
    driver->last_dma_status = DmaHal_StartTx(
        &driver->param_dma, (UINTPTR)driver->config.param_words,
        driver->config.param_byte_count);
    if (driver->last_dma_status != DMA_HAL_STATUS_OK)
    {
        status = CNN_DRIVER_STATUS_PARAM_TX_START_FAILED;
    }
    else
    {
        XTime_GetTime(&start);
        for (;;)
        {
            dma_status = DmaHal_GetTxStatus(&driver->param_dma);
            snapshot = CnnHal_ReadSnapshot(&driver->cnn);
            driver->last_param_dma_hw_status = dma_status;
            driver->last_param_cnn_status = snapshot.raw;
            if ((dma_status & (XAXIDMA_ERR_ALL_MASK |
                               XAXIDMA_IRQ_ERROR_MASK)) != 0U)
            {
                driver->last_dma_status = DMA_HAL_STATUS_HARDWARE_ERROR;
                status = CNN_DRIVER_STATUS_PARAM_DMA_ERROR;
                break;
            }
            if (snapshot.params_error)
            {
                status = CNN_DRIVER_STATUS_PARAM_PROTOCOL_ERROR;
                break;
            }
            /* Check both the DMA completion and the receiving RTL's result.
             * IOC alone or IDLE alone cannot establish that parameters are valid. */
            if (((dma_status & XAXIDMA_IRQ_IOC_MASK) != 0U) &&
                !DmaHal_IsTxBusy(&driver->param_dma) &&
                snapshot.params_loaded && !snapshot.params_loading &&
                snapshot.idle && !snapshot.busy)
            {
                DmaHal_ClearTxIrq(&driver->param_dma);
                return CNN_DRIVER_STATUS_OK;
            }
            if (CnnDriver_TimeoutExpired(start, driver->config.param_timeout_ms))
            {
                driver->last_dma_status = DMA_HAL_STATUS_TIMEOUT;
                status = CNN_DRIVER_STATUS_PARAM_TIMEOUT;
                break;
            }
        }
    }

    /* Preserve diagnostics before resetting the failing transfer. Stop the
     * source first: aborting only the receiver can misframe residual data. */
    driver->last_param_dma_hw_status = DmaHal_GetTxStatus(&driver->param_dma);
    driver->last_param_cnn_status = CnnHal_ReadStatus(&driver->cnn);
    reset_status = DmaHal_Reset(&driver->param_dma);
    if (reset_status != DMA_HAL_STATUS_OK)
    {
        driver->last_dma_status = reset_status;
        return CNN_DRIVER_STATUS_PARAM_DMA_RESET_FAILED;
    }
    if (CnnHal_IsIdle(&driver->cnn))
    {
        CnnHal_AbortParameterLoad(&driver->cnn);
    }
    return status;
}

CnnDriverStatus CnnDriver_Init(CnnDriver *driver,
                               const CnnDriverConfig *config,
                               uint32_t *output_words,
                               uint32_t output_capacity_words)
{
    CnnDriverStatus status = CNN_DRIVER_STATUS_INVALID_ARGUMENT;

    if (driver != NULL)
    {
        driver->initialized = false;
        driver->active = false;
        driver->done_irq = false;
        driver->done_irq_count = 0U;
    }
    if ((driver != NULL) && (config != NULL) && (output_words != NULL) &&
        (config->param_words != NULL) &&
        (config->param_dma_device_id != config->dma_device_id) &&
        (config->param_byte_count == CNN_DRIVER_PARAM_BYTE_COUNT) &&
        (config->param_timeout_ms > 0U) &&
        (((UINTPTR)config->param_words & CNN_DRIVER_PARAM_ALIGNMENT_MASK) == 0U) &&
        (config->weight_words != NULL) &&
        (config->weight_byte_count > 0U) &&
        ((config->weight_byte_count % CNN_DRIVER_BYTES_PER_WORD) == 0U) &&
        (config->output_word_count > 0U) &&
        (config->output_word_count <=
         (UINT32_MAX / CNN_DRIVER_BYTES_PER_WORD)) &&
        (config->output_word_count <= output_capacity_words) &&
        (config->idle_timeout_ms > 0U) &&
        (config->inference_timeout_ms > 0U))
    {
        driver->initialized = false;
        driver->config = *config;
        driver->output_words = output_words;
        driver->output_capacity_words = output_capacity_words;
        driver->last_dma_status = DMA_HAL_STATUS_NOT_INITIALIZED;
        driver->last_param_dma_hw_status = 0U;
        driver->last_param_cnn_status = 0U;

        if (CnnHal_Init(&driver->cnn, config->cnn_base_address) !=
            CNN_HAL_STATUS_OK)
        {
            status = CNN_DRIVER_STATUS_CNN_HAL_INIT_FAILED;
        }
        else
        {
            driver->last_dma_status =
                DmaHal_Init(&driver->dma, config->dma_device_id);

            if (driver->last_dma_status != DMA_HAL_STATUS_OK)
            {
                status = CNN_DRIVER_STATUS_DMA_INIT_FAILED;
            }
            else
            {
                Xil_DCacheFlushRange((UINTPTR)config->weight_words,
                                     (u32)config->weight_byte_count);
                status = CnnDriver_LoadParameters(driver);
                if (status == CNN_DRIVER_STATUS_OK)
                {
                    driver->initialized = true;
                }
            }
        }
    }

    return status;
}

/* Legacy synchronous helper retained for startup benchmarks ONLY. Runtime
 * uses StartAsync/TryFinish; IRQ must not be enabled during this benchmark. */
CnnDriverStatus CnnDriver_Run(CnnDriver *driver, CnnDriverResult *result)
{
    CnnDriverStatus status = CNN_DRIVER_STATUS_INVALID_ARGUMENT;
    XTime start;
    XTime end;
    uint32_t output_byte_count;
    bool transfer_busy;

    if ((driver != NULL) && (result != NULL))
    {
        result->elapsed_us = 0U;
        result->irq_elapsed_us = 0U;
        result->output_checksum = 0U;
        result->hardware_status.raw = 0U;
        result->hardware_status.layer = 0U;
        result->hardware_status.output_group = 0U;
        result->hardware_status.input_group = 0U;
        result->hardware_status.idle = false;
        result->hardware_status.busy = false;
        result->hardware_status.done = false;
        result->hardware_status.params_loaded = false;
        result->hardware_status.params_loading = false;
        result->hardware_status.params_error = false;

        if (!driver->initialized)
        {
            status = CNN_DRIVER_STATUS_NOT_INITIALIZED;
        }
        else if (!CnnHal_AreParametersReady(&driver->cnn))
        {
            /* Do not silently reload every frame. A PL reset requires re-init. */
            result->hardware_status = CnnHal_ReadSnapshot(&driver->cnn);
            status = CNN_DRIVER_STATUS_PARAM_NOT_READY;
        }
        else
        {
            status = CnnDriver_WaitForIdle(driver);
            if (status == CNN_DRIVER_STATUS_OK)
            {
                output_byte_count = driver->config.output_word_count *
                                    CNN_DRIVER_BYTES_PER_WORD;
                CnnDriver_ClearOutput(driver);
                Xil_DCacheFlushRange((UINTPTR)driver->output_words,
                                     (u32)output_byte_count);
                Xil_DCacheInvalidateRange((UINTPTR)driver->output_words,
                                          (u32)output_byte_count);
                CnnHal_Prepare(&driver->cnn);

                driver->last_dma_status = DmaHal_StartRx(
                    &driver->dma,
                    (UINTPTR)driver->output_words,
                    output_byte_count);

                if (driver->last_dma_status != DMA_HAL_STATUS_OK)
                {
                    status = CNN_DRIVER_STATUS_RX_START_FAILED;
                }
                else
                {
                    driver->last_dma_status = DmaHal_StartTx(
                        &driver->dma,
                        (UINTPTR)driver->config.weight_words,
                        driver->config.weight_byte_count);

                    if (driver->last_dma_status != DMA_HAL_STATUS_OK)
                    {
                        status = CNN_DRIVER_STATUS_TX_START_FAILED;
                    }
                    else
                    {
                        XTime_GetTime(&start);
                        CnnHal_Start(&driver->cnn);

                        while (!CnnHal_IsDone(&driver->cnn))
                        {
                            if (CnnDriver_TimeoutExpired(
                                    start,
                                    driver->config.inference_timeout_ms))
                            {
                                status = CNN_DRIVER_STATUS_INFERENCE_TIMEOUT;
                                break;
                            }
                        }

                        if (status == CNN_DRIVER_STATUS_OK)
                        {
                            do
                            {
                                transfer_busy =
                                    DmaHal_IsTxBusy(&driver->dma) ||
                                    DmaHal_IsRxBusy(&driver->dma);

                                if (CnnDriver_TimeoutExpired(
                                        start,
                                        driver->config.inference_timeout_ms))
                                {
                                    status =
                                        CNN_DRIVER_STATUS_INFERENCE_TIMEOUT;
                                    break;
                                }
                            } while (transfer_busy);
                        }

                        XTime_GetTime(&end);
                        Xil_DCacheInvalidateRange(
                            (UINTPTR)driver->output_words,
                            (u32)output_byte_count);
                        result->elapsed_us =
                            CnnDriver_ElapsedUs(start, end);
                        result->output_checksum = CnnDriver_Checksum(
                            driver->output_words,
                            driver->config.output_word_count);
                        result->hardware_status =
                            CnnHal_ReadSnapshot(&driver->cnn);
                        CnnHal_ClearDone(&driver->cnn);
                    }
                }
            }
        }
    }

    return status;
}

bool CnnDriver_IsActive(const CnnDriver *driver)
{
    return driver != NULL && driver->initialized && driver->active;
}

CnnDriverStatus CnnDriver_StartAsync(CnnDriver *driver)
{
    uint32_t bytes;
    uint32_t saved;
    if (driver == NULL) return CNN_DRIVER_STATUS_INVALID_ARGUMENT;
    if (!driver->initialized) return CNN_DRIVER_STATUS_NOT_INITIALIZED;
    if (driver->active || !CnnHal_IsIdle(&driver->cnn))
        return CNN_DRIVER_STATUS_BUSY;
    if (!CnnHal_AreParametersReady(&driver->cnn))
        return CNN_DRIVER_STATUS_PARAM_NOT_READY;

    bytes = driver->config.output_word_count * CNN_DRIVER_BYTES_PER_WORD;
    CnnDriver_ClearOutput(driver);
    Xil_DCacheFlushRange((UINTPTR)driver->output_words, (u32)bytes);
    Xil_DCacheInvalidateRange((UINTPTR)driver->output_words, (u32)bytes);
    CnnHal_Prepare(&driver->cnn);
    DmaHal_ClearRxIrq(&driver->dma);
    DmaHal_ClearTxIrq(&driver->dma);
    driver->last_dma_status = DmaHal_StartRx(
        &driver->dma, (UINTPTR)driver->output_words, bytes);
    if (driver->last_dma_status != DMA_HAL_STATUS_OK)
        return CNN_DRIVER_STATUS_RX_START_FAILED;
    driver->last_dma_status = DmaHal_StartTx(
        &driver->dma, (UINTPTR)driver->config.weight_words,
        driver->config.weight_byte_count);
    if (driver->last_dma_status != DMA_HAL_STATUS_OK)
        return CNN_DRIVER_STATUS_TX_START_FAILED;

    saved = IrqHal_SaveDisable();
    driver->done_irq = false;
    driver->active = true;
    XTime_GetTime(&driver->start_ticks);
    CnnHal_Start(&driver->cnn);
    IrqHal_Restore(saved);
    return CNN_DRIVER_STATUS_OK;
}

void CnnDriver_IrqHandler(void *context)
{
    CnnDriver *driver = (CnnDriver *)context;
    CnnHalSnapshot snapshot;
    if (driver == NULL || !driver->initialized) return;
    snapshot = CnnHal_ReadSnapshot(&driver->cnn);
    if (!snapshot.done) return;
    if (driver->active && !driver->done_irq)
    {
        XTime_GetTime(&driver->done_irq_ticks);
        driver->done_irq_snapshot = snapshot;
        ++driver->done_irq_count;
        driver->done_irq = true;
    }
    /* Keep IRQ_ENABLE high: cnn_pixel_fifo also uses this done signal. */
    CnnHal_ClearDone(&driver->cnn);
}

CnnDriverStatus CnnDriver_TryFinish(CnnDriver *driver, CnnDriverResult *result)
{
    uint32_t dma_status;
    uint32_t saved;
    XTime end;
    XTime irq_ticks;
    CnnHalSnapshot irq_snapshot;
    if (driver == NULL || result == NULL)
        return CNN_DRIVER_STATUS_INVALID_ARGUMENT;
    if (!driver->initialized) return CNN_DRIVER_STATUS_NOT_INITIALIZED;
    if (!driver->active) return CNN_DRIVER_STATUS_BUSY;

    dma_status = DmaHal_GetTxStatus(&driver->dma) |
                 DmaHal_GetRxStatus(&driver->dma);
    if ((dma_status & (XAXIDMA_ERR_ALL_MASK | XAXIDMA_IRQ_ERROR_MASK)) != 0U)
    {
        driver->last_dma_status = DMA_HAL_STATUS_HARDWARE_ERROR;
        driver->active = false;
        return CNN_DRIVER_STATUS_INFERENCE_DMA_ERROR;
    }
    if (CnnDriver_TimeoutExpired(driver->start_ticks,
                                 driver->config.inference_timeout_ms))
    {
        driver->active = false;
        return CNN_DRIVER_STATUS_INFERENCE_TIMEOUT;
    }
    /* A done status without a CPU IRQ is NOT accepted as completion.
     * DMA is checked nonblocking because CNN DONE can precede S2MM drain. */
    if (!driver->done_irq || DmaHal_IsTxBusy(&driver->dma) ||
        DmaHal_IsRxBusy(&driver->dma)) return CNN_DRIVER_STATUS_IN_PROGRESS;

    saved = IrqHal_SaveDisable();
    irq_ticks = driver->done_irq_ticks;
    irq_snapshot = driver->done_irq_snapshot;
    driver->active = false;
    driver->done_irq = false;
    IrqHal_Restore(saved);
    XTime_GetTime(&end);
    Xil_DCacheInvalidateRange((UINTPTR)driver->output_words,
        (u32)(driver->config.output_word_count * CNN_DRIVER_BYTES_PER_WORD));
    result->elapsed_us = CnnDriver_ElapsedUs(driver->start_ticks, end);
    result->irq_elapsed_us = CnnDriver_ElapsedUs(driver->start_ticks, irq_ticks);
    result->output_checksum = CnnDriver_Checksum(driver->output_words,
                                                 driver->config.output_word_count);
    result->hardware_status = irq_snapshot;
    return CNN_DRIVER_STATUS_OK;
}

CnnHalSnapshot CnnDriver_GetHardwareStatus(const CnnDriver *driver)
{
    CnnHalSnapshot status = {0};

    if (driver != NULL)
    {
        status = CnnHal_ReadSnapshot(&driver->cnn);
    }

    return status;
}

DmaHalStatus CnnDriver_GetLastDmaStatus(const CnnDriver *driver)
{
    DmaHalStatus status = DMA_HAL_STATUS_NOT_INITIALIZED;

    if (driver != NULL)
    {
        status = driver->last_dma_status;
    }

    return status;
}

const uint32_t *CnnDriver_GetOutput(const CnnDriver *driver)
{
    const uint32_t *output = NULL;

    if ((driver != NULL) && driver->initialized)
    {
        output = driver->output_words;
    }

    return output;
}

uint32_t CnnDriver_GetParameterDmaStatus(const CnnDriver *driver)
{
    return (driver != NULL) ? driver->last_param_dma_hw_status : 0U;
}

uint32_t CnnDriver_GetParameterCnnStatus(const CnnDriver *driver)
{
    return (driver != NULL) ? driver->last_param_cnn_status : 0U;
}
