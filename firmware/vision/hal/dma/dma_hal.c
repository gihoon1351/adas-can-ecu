#include "dma_hal.h"

#include <stddef.h>

#include "xstatus.h"

#define DMA_HAL_RESET_POLL_LIMIT (1000000UL)

static bool DmaHal_IsValid(const DmaHal *dma)
{
    return (dma != NULL) && dma->initialized;
}

DmaHalStatus DmaHal_Init(DmaHal *dma, uint16_t device_id)
{
    DmaHalStatus status = DMA_HAL_STATUS_INVALID_ARGUMENT;
    XAxiDma_Config *configuration;
    uint32_t poll_count;
    int vendor_status;

    if (dma != NULL)
    {
        dma->initialized = false;
        configuration = XAxiDma_LookupConfig((u16)device_id);

        if (configuration == NULL)
        {
            status = DMA_HAL_STATUS_CONFIG_NOT_FOUND;
        }
        else
        {
            vendor_status = XAxiDma_CfgInitialize(&dma->instance,
                                                   configuration);
            if (vendor_status != XST_SUCCESS)
            {
                status = DMA_HAL_STATUS_INITIALIZATION_FAILED;
            }
            else if (XAxiDma_HasSg(&dma->instance) != 0)
            {
                status = DMA_HAL_STATUS_SCATTER_GATHER_UNSUPPORTED;
            }
            else
            {
                XAxiDma_IntrDisable(&dma->instance,
                                    XAXIDMA_IRQ_ALL_MASK,
                                    XAXIDMA_DMA_TO_DEVICE);
                XAxiDma_IntrDisable(&dma->instance,
                                    XAXIDMA_IRQ_ALL_MASK,
                                    XAXIDMA_DEVICE_TO_DMA);
                XAxiDma_Reset(&dma->instance);

                poll_count = 0U;
                while ((XAxiDma_ResetIsDone(&dma->instance) == 0) &&
                       (poll_count < DMA_HAL_RESET_POLL_LIMIT))
                {
                    ++poll_count;
                }

                if (poll_count >= DMA_HAL_RESET_POLL_LIMIT)
                {
                    status = DMA_HAL_STATUS_RESET_TIMEOUT;
                }
                else
                {
                    dma->initialized = true;
                    status = DMA_HAL_STATUS_OK;
                }
            }
        }
    }

    return status;
}

static DmaHalStatus DmaHal_StartTransfer(DmaHal *dma,
                                         UINTPTR address,
                                         uint32_t byte_count,
                                         int direction)
{
    DmaHalStatus status = DMA_HAL_STATUS_INVALID_ARGUMENT;
    int vendor_status;

    if (!DmaHal_IsValid(dma))
    {
        status = DMA_HAL_STATUS_NOT_INITIALIZED;
    }
    else if ((address == (UINTPTR)0U) || (byte_count == 0U))
    {
        status = DMA_HAL_STATUS_INVALID_ARGUMENT;
    }
    else
    {
        vendor_status = XAxiDma_SimpleTransfer(&dma->instance,
                                               address,
                                               (u32)byte_count,
                                               direction);
        status = (vendor_status == XST_SUCCESS)
               ? DMA_HAL_STATUS_OK
               : DMA_HAL_STATUS_TRANSFER_FAILED;
    }

    return status;
}

DmaHalStatus DmaHal_StartTx(DmaHal *dma, UINTPTR address,
                            uint32_t byte_count)
{
    return DmaHal_StartTransfer(dma, address, byte_count,
                                XAXIDMA_DMA_TO_DEVICE);
}

DmaHalStatus DmaHal_StartRx(DmaHal *dma, UINTPTR address,
                            uint32_t byte_count)
{
    return DmaHal_StartTransfer(dma, address, byte_count,
                                XAXIDMA_DEVICE_TO_DMA);
}

bool DmaHal_IsTxBusy(DmaHal *dma)
{
    bool busy = false;

    if (DmaHal_IsValid(dma))
    {
        busy = XAxiDma_Busy(&dma->instance,
                            XAXIDMA_DMA_TO_DEVICE) != 0;
    }

    return busy;
}

bool DmaHal_IsRxBusy(DmaHal *dma)
{
    bool busy = false;

    if (DmaHal_IsValid(dma))
    {
        busy = XAxiDma_Busy(&dma->instance,
                            XAXIDMA_DEVICE_TO_DMA) != 0;
    }

    return busy;
}

DmaHalStatus DmaHal_Reset(DmaHal *dma)
{
    uint32_t poll_count = 0U;
    if (!DmaHal_IsValid(dma))
    {
        return DMA_HAL_STATUS_NOT_INITIALIZED;
    }
    XAxiDma_Reset(&dma->instance);
    while ((XAxiDma_ResetIsDone(&dma->instance) == 0) &&
           (poll_count < DMA_HAL_RESET_POLL_LIMIT))
    {
        ++poll_count;
    }
    if (poll_count >= DMA_HAL_RESET_POLL_LIMIT)
    {
        dma->initialized = false;
        return DMA_HAL_STATUS_RESET_TIMEOUT;
    }
    if (dma->instance.HasMm2S != 0)
    {
        XAxiDma_IntrDisable(&dma->instance, XAXIDMA_IRQ_ALL_MASK,
                            XAXIDMA_DMA_TO_DEVICE);
    }
    if (dma->instance.HasS2Mm != 0)
    {
        XAxiDma_IntrDisable(&dma->instance, XAXIDMA_IRQ_ALL_MASK,
                            XAXIDMA_DEVICE_TO_DMA);
    }
    return DMA_HAL_STATUS_OK;
}

uint32_t DmaHal_GetTxStatus(const DmaHal *dma)
{
    uint32_t status = 0U;
    if (DmaHal_IsValid(dma) && (dma->instance.HasMm2S != 0))
    {
        status = (uint32_t)XAxiDma_ReadReg(
            dma->instance.RegBase + XAXIDMA_TX_OFFSET, XAXIDMA_SR_OFFSET);
    }
    return status;
}

bool DmaHal_TxLengthSupported(const DmaHal *dma, uint32_t byte_count)
{
    return DmaHal_IsValid(dma) && (dma->instance.HasMm2S != 0) &&
           (byte_count > 0U) &&
           (byte_count <= dma->instance.TxBdRing.MaxTransferLen);
}

uint32_t DmaHal_GetRxStatus(const DmaHal *dma)
{
    uint32_t status = 0U;
    if (DmaHal_IsValid(dma) && (dma->instance.HasS2Mm != 0))
    {
        status = (uint32_t)XAxiDma_ReadReg(
            dma->instance.RegBase + XAXIDMA_RX_OFFSET, XAXIDMA_SR_OFFSET);
    }
    return status;
}

void DmaHal_ClearRxIrq(DmaHal *dma)
{
    if (DmaHal_IsValid(dma) && (dma->instance.HasS2Mm != 0))
    {
        XAxiDma_IntrAckIrq(&dma->instance, XAXIDMA_IRQ_ALL_MASK,
                           XAXIDMA_DEVICE_TO_DMA);
    }
}

void DmaHal_ClearTxIrq(DmaHal *dma)
{
    if (DmaHal_IsValid(dma) && (dma->instance.HasMm2S != 0))
    {
        XAxiDma_IntrAckIrq(&dma->instance, XAXIDMA_IRQ_ALL_MASK,
                           XAXIDMA_DMA_TO_DEVICE);
    }
}
