#ifndef DMA_HAL_H
#define DMA_HAL_H

#include <stdbool.h>
#include <stdint.h>

#include "xaxidma.h"
#include "xil_types.h"

typedef enum
{
    DMA_HAL_STATUS_OK = 0,
    DMA_HAL_STATUS_INVALID_ARGUMENT,
    DMA_HAL_STATUS_CONFIG_NOT_FOUND,
    DMA_HAL_STATUS_INITIALIZATION_FAILED,
    DMA_HAL_STATUS_SCATTER_GATHER_UNSUPPORTED,
    DMA_HAL_STATUS_RESET_TIMEOUT,
    DMA_HAL_STATUS_TRANSFER_FAILED,
    DMA_HAL_STATUS_NOT_INITIALIZED,
    DMA_HAL_STATUS_HARDWARE_ERROR,
    DMA_HAL_STATUS_TIMEOUT
} DmaHalStatus;

typedef struct
{
    XAxiDma instance;
    bool initialized;
} DmaHal;

DmaHalStatus DmaHal_Init(DmaHal *dma, uint16_t device_id);
DmaHalStatus DmaHal_Reset(DmaHal *dma);
uint32_t DmaHal_GetTxStatus(const DmaHal *dma);
uint32_t DmaHal_GetRxStatus(const DmaHal *dma);
void DmaHal_ClearRxIrq(DmaHal *dma);
bool DmaHal_TxLengthSupported(const DmaHal *dma, uint32_t byte_count);
void DmaHal_ClearTxIrq(DmaHal *dma);
DmaHalStatus DmaHal_StartTx(DmaHal *dma, UINTPTR address,
                            uint32_t byte_count);
DmaHalStatus DmaHal_StartRx(DmaHal *dma, UINTPTR address,
                            uint32_t byte_count);
bool DmaHal_IsTxBusy(DmaHal *dma);
bool DmaHal_IsRxBusy(DmaHal *dma);

#endif
