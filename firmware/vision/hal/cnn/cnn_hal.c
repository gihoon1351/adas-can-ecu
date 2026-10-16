#include "cnn_hal.h"

#include <stddef.h>

#include "xil_io.h"

#define CNN_HAL_CR_OFFSET              (0x00U)
#define CNN_HAL_SR_OFFSET              (0x04U)

#define CNN_HAL_CR_PARAM_ABORT_MASK    (1UL << 3U)
#define CNN_HAL_SR_PARAM_LOADED_MASK   (1UL << 17U)
#define CNN_HAL_SR_PARAM_LOADING_MASK  (1UL << 18U)
#define CNN_HAL_SR_PARAM_ERROR_MASK    (1UL << 19U)

#define CNN_HAL_CR_START_MASK          (1UL << 0U)
#define CNN_HAL_CR_IRQ_ENABLE_MASK     (1UL << 1U)
#define CNN_HAL_CR_DONE_CLEAR_MASK     (1UL << 2U)

#define CNN_HAL_SR_IDLE_MASK           (1UL << 0U)
#define CNN_HAL_SR_BUSY_MASK           (1UL << 1U)
#define CNN_HAL_SR_DONE_MASK           (1UL << 2U)
#define CNN_HAL_SR_LAYER_SHIFT         (3U)
#define CNN_HAL_SR_OUTPUT_GROUP_SHIFT  (6U)
#define CNN_HAL_SR_INPUT_GROUP_SHIFT   (11U)
#define CNN_HAL_SR_LAYER_MASK          (0x07UL)
#define CNN_HAL_SR_OUTPUT_GROUP_MASK   (0x1FUL)
#define CNN_HAL_SR_INPUT_GROUP_MASK    (0x3FUL)
#define CNN_HAL_ADDRESS_ALIGNMENT_MASK ((UINTPTR)0x03U)

static bool CnnHal_IsValid(const CnnHal *instance)
{
    return (instance != NULL) && (instance->base_address != (UINTPTR)0U);
}

static void CnnHal_WriteControl(const CnnHal *instance, uint32_t value)
{
    if (CnnHal_IsValid(instance))
    {
        Xil_Out32(instance->base_address + (UINTPTR)CNN_HAL_CR_OFFSET,
                  (u32)value);
    }
}

CnnHalStatus CnnHal_Init(CnnHal *instance, UINTPTR base_address)
{
    CnnHalStatus status = CNN_HAL_STATUS_INVALID_ARGUMENT;

    if ((instance != NULL) &&
        (base_address != (UINTPTR)0U) &&
        ((base_address & CNN_HAL_ADDRESS_ALIGNMENT_MASK) == (UINTPTR)0U))
    {
        instance->base_address = base_address;
        status = CNN_HAL_STATUS_OK;
    }

    return status;
}

void CnnHal_Prepare(CnnHal *instance)
{
    CnnHal_WriteControl(instance,
                        CNN_HAL_CR_IRQ_ENABLE_MASK |
                        CNN_HAL_CR_DONE_CLEAR_MASK);
    CnnHal_WriteControl(instance, CNN_HAL_CR_IRQ_ENABLE_MASK);
}

void CnnHal_Start(CnnHal *instance)
{
    CnnHal_WriteControl(instance,
                        CNN_HAL_CR_IRQ_ENABLE_MASK |
                        CNN_HAL_CR_START_MASK);
    CnnHal_WriteControl(instance, CNN_HAL_CR_IRQ_ENABLE_MASK);
}

void CnnHal_ClearDone(CnnHal *instance)
{
    CnnHal_WriteControl(instance,
                        CNN_HAL_CR_IRQ_ENABLE_MASK |
                        CNN_HAL_CR_DONE_CLEAR_MASK);
    CnnHal_WriteControl(instance, CNN_HAL_CR_IRQ_ENABLE_MASK);
}

void CnnHal_SetInterruptEnabled(CnnHal *instance, bool enabled)
{
    const uint32_t control = enabled ? CNN_HAL_CR_IRQ_ENABLE_MASK : 0U;
    CnnHal_WriteControl(instance, control);
}

uint32_t CnnHal_ReadStatus(const CnnHal *instance)
{
    uint32_t status = 0U;

    if (CnnHal_IsValid(instance))
    {
        status = (uint32_t)Xil_In32(
            instance->base_address + (UINTPTR)CNN_HAL_SR_OFFSET);
    }

    return status;
}

CnnHalSnapshot CnnHal_ReadSnapshot(const CnnHal *instance)
{
    CnnHalSnapshot snapshot = {0};

    snapshot.raw = CnnHal_ReadStatus(instance);
    snapshot.idle = (snapshot.raw & CNN_HAL_SR_IDLE_MASK) != 0U;
    snapshot.busy = (snapshot.raw & CNN_HAL_SR_BUSY_MASK) != 0U;
    snapshot.done = (snapshot.raw & CNN_HAL_SR_DONE_MASK) != 0U;
    snapshot.params_loaded = (snapshot.raw & CNN_HAL_SR_PARAM_LOADED_MASK) != 0U;
    snapshot.params_loading = (snapshot.raw & CNN_HAL_SR_PARAM_LOADING_MASK) != 0U;
    snapshot.params_error = (snapshot.raw & CNN_HAL_SR_PARAM_ERROR_MASK) != 0U;
    snapshot.layer = (uint8_t)((snapshot.raw >> CNN_HAL_SR_LAYER_SHIFT) &
                               CNN_HAL_SR_LAYER_MASK);
    snapshot.output_group =
        (uint8_t)((snapshot.raw >> CNN_HAL_SR_OUTPUT_GROUP_SHIFT) &
                  CNN_HAL_SR_OUTPUT_GROUP_MASK);
    snapshot.input_group =
        (uint8_t)((snapshot.raw >> CNN_HAL_SR_INPUT_GROUP_SHIFT) &
                  CNN_HAL_SR_INPUT_GROUP_MASK);

    return snapshot;
}

bool CnnHal_IsIdle(const CnnHal *instance)
{
    return (CnnHal_ReadStatus(instance) & CNN_HAL_SR_IDLE_MASK) != 0U;
}

bool CnnHal_IsBusy(const CnnHal *instance)
{
    return (CnnHal_ReadStatus(instance) & CNN_HAL_SR_BUSY_MASK) != 0U;
}

bool CnnHal_IsDone(const CnnHal *instance)
{
    return (CnnHal_ReadStatus(instance) & CNN_HAL_SR_DONE_MASK) != 0U;
}

/* Caller must stop the parameter DMA and wait for CNN IDLE before aborting. */
void CnnHal_AbortParameterLoad(CnnHal *instance)
{
    uint32_t control;
    if (CnnHal_IsValid(instance))
    {
        control = (uint32_t)Xil_In32(instance->base_address + CNN_HAL_CR_OFFSET)
                  & CNN_HAL_CR_IRQ_ENABLE_MASK;
        CnnHal_WriteControl(instance, control | CNN_HAL_CR_PARAM_ABORT_MASK);
        CnnHal_WriteControl(instance, control);
    }
}

bool CnnHal_AreParametersReady(const CnnHal *instance)
{
    const uint32_t mask = CNN_HAL_SR_PARAM_LOADED_MASK |
                          CNN_HAL_SR_PARAM_LOADING_MASK |
                          CNN_HAL_SR_PARAM_ERROR_MASK;
    return (CnnHal_ReadStatus(instance) & mask) == CNN_HAL_SR_PARAM_LOADED_MASK;
}
