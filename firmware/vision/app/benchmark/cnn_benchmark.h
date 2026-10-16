#ifndef CNN_BENCHMARK_H
#define CNN_BENCHMARK_H

#include <stdbool.h>
#include <stdint.h>

#include "../../driver/cnn/cnn_driver.h"

typedef bool (*CnnBenchmarkSoftwareRun)(void *context,
                                        uint32_t *output_words,
                                        uint32_t output_capacity_words);

typedef enum
{
    CNN_BENCHMARK_STATUS_OK = 0,
    CNN_BENCHMARK_STATUS_INVALID_ARGUMENT,
    CNN_BENCHMARK_STATUS_EXECUTION_FAILED
} CnnBenchmarkStatus;

typedef struct
{
    uint32_t requested_runs;
    uint32_t completed_runs;
    uint32_t failed_runs;
    uint32_t minimum_us;
    uint32_t maximum_us;
    uint32_t average_us;
    uint32_t internal_average_us;
    uint32_t last_checksum;
} CnnBenchmarkStats;

typedef struct
{
    uint32_t compared_words;
    uint32_t different_words;
    uint32_t different_bytes;
    uint32_t maximum_absolute_error;
    uint64_t total_absolute_error;
    uint32_t mean_absolute_error_milli;
} CnnBenchmarkDifference;

CnnBenchmarkStatus CnnBenchmark_RunHardware(
    CnnDriver *driver,
    uint32_t warmup_runs,
    uint32_t measured_runs,
    CnnBenchmarkStats *stats);

CnnBenchmarkStatus CnnBenchmark_RunSoftware(
    CnnBenchmarkSoftwareRun software_run,
    void *software_context,
    uint32_t *output_words,
    uint32_t output_capacity_words,
    uint32_t warmup_runs,
    uint32_t measured_runs,
    CnnBenchmarkStats *stats);

CnnBenchmarkStatus CnnBenchmark_CompareInt8Outputs(
    const uint32_t *hardware_output,
    const uint32_t *software_output,
    uint32_t word_count,
    CnnBenchmarkDifference *difference);

void CnnBenchmark_PrintStats(const char *name,
                             const CnnBenchmarkStats *stats);
void CnnBenchmark_PrintSpeedup(const CnnBenchmarkStats *software,
                               const CnnBenchmarkStats *hardware);
void CnnBenchmark_PrintDifference(
    const CnnBenchmarkDifference *difference);

#endif
