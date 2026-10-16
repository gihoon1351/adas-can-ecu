#ifndef CNN_HAL_H
#define CNN_HAL_H

#include <stdbool.h>
#include <stdint.h>

#include "xil_types.h"

typedef enum
{
    CNN_HAL_STATUS_OK = 0,
    CNN_HAL_STATUS_INVALID_ARGUMENT
} CnnHalStatus;

typedef struct
{
    UINTPTR base_address;
} CnnHal;

typedef struct
{
    uint32_t raw;
    uint8_t layer;
    uint8_t output_group;
    uint8_t input_group;
    bool idle;
    bool busy;
    bool done;
    bool params_loaded;
    bool params_loading;
    bool params_error;
} CnnHalSnapshot;

CnnHalStatus CnnHal_Init(CnnHal *instance, UINTPTR base_address);
void CnnHal_Prepare(CnnHal *instance);
void CnnHal_AbortParameterLoad(CnnHal *instance);
bool CnnHal_AreParametersReady(const CnnHal *instance);
void CnnHal_Start(CnnHal *instance);
void CnnHal_ClearDone(CnnHal *instance);
void CnnHal_SetInterruptEnabled(CnnHal *instance, bool enabled);
uint32_t CnnHal_ReadStatus(const CnnHal *instance);
CnnHalSnapshot CnnHal_ReadSnapshot(const CnnHal *instance);
bool CnnHal_IsIdle(const CnnHal *instance);
bool CnnHal_IsBusy(const CnnHal *instance);
bool CnnHal_IsDone(const CnnHal *instance);

#endif
