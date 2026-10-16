#include "cnn_benchmark.h"

#include <limits.h>
#include <stddef.h>

#include "xil_printf.h"
#include "xtime_l.h"

#define CNN_BENCHMARK_FNV_OFFSET (2166136261UL)
#define CNN_BENCHMARK_FNV_PRIME  (16777619UL)
#define CNN_BENCHMARK_BYTES_PER_WORD (4UL)

static uint32_t CnnBenchmark_ElapsedUs(XTime start, XTime end)
{
    uint64_t ticks;
    uint64_t microseconds;

    ticks = (uint64_t)(end - start);
    microseconds = (ticks * 1000000ULL) /
                   (uint64_t)COUNTS_PER_SECOND;

    return (uint32_t)microseconds;
}

static uint32_t CnnBenchmark_Checksum(const uint32_t *data,
                                      uint32_t word_count)
{
    uint32_t hash = CNN_BENCHMARK_FNV_OFFSET;
    uint32_t index;

    if (data != NULL)
    {
        for (index = 0U; index < word_count; ++index)
        {
            hash ^= data[index];
            hash *= CNN_BENCHMARK_FNV_PRIME;
        }
    }

    return hash;
}

static void CnnBenchmark_ResetStats(CnnBenchmarkStats *stats,
                                    uint32_t measured_runs)
{
    stats->requested_runs = measured_runs;
    stats->completed_runs = 0U;
    stats->failed_runs = 0U;
    stats->minimum_us = UINT32_MAX;
    stats->maximum_us = 0U;
    stats->average_us = 0U;
    stats->internal_average_us = 0U;
    stats->last_checksum = 0U;
}

static void CnnBenchmark_AddSample(CnnBenchmarkStats *stats,
                                   uint32_t elapsed_us)
{
    if (elapsed_us < stats->minimum_us)
    {
        stats->minimum_us = elapsed_us;
    }
    if (elapsed_us > stats->maximum_us)
    {
        stats->maximum_us = elapsed_us;
    }
    ++stats->completed_runs;
}

CnnBenchmarkStatus CnnBenchmark_RunHardware(
    CnnDriver *driver,
    uint32_t warmup_runs,
    uint32_t measured_runs,
    CnnBenchmarkStats *stats)
{
    CnnDriverResult result;
    CnnDriverStatus driver_status;
    CnnBenchmarkStatus status = CNN_BENCHMARK_STATUS_INVALID_ARGUMENT;
    XTime start;
    XTime end;
    uint64_t total_us = 0ULL;
    uint64_t internal_total_us = 0ULL;
    uint32_t index;
    uint32_t elapsed_us;

    if ((driver != NULL) && (stats != NULL) && (measured_runs > 0U))
    {
        CnnBenchmark_ResetStats(stats, measured_runs);
        status = CNN_BENCHMARK_STATUS_OK;

        for (index = 0U; index < warmup_runs; ++index)
        {
            driver_status = CnnDriver_Run(driver, &result);
            if (driver_status != CNN_DRIVER_STATUS_OK)
            {
                status = CNN_BENCHMARK_STATUS_EXECUTION_FAILED;
                break;
            }
        }

        if (status == CNN_BENCHMARK_STATUS_OK)
        {
            for (index = 0U; index < measured_runs; ++index)
            {
                XTime_GetTime(&start);
                driver_status = CnnDriver_Run(driver, &result);
                XTime_GetTime(&end);

                if (driver_status == CNN_DRIVER_STATUS_OK)
                {
                    elapsed_us = CnnBenchmark_ElapsedUs(start, end);
                    CnnBenchmark_AddSample(stats, elapsed_us);
                    total_us += (uint64_t)elapsed_us;
                    internal_total_us += (uint64_t)result.elapsed_us;
                    stats->last_checksum = result.output_checksum;
                }
                else
                {
                    ++stats->failed_runs;
                    status = CNN_BENCHMARK_STATUS_EXECUTION_FAILED;
                }
            }
        }

        if (stats->completed_runs > 0U)
        {
            stats->average_us =
                (uint32_t)(total_us / (uint64_t)stats->completed_runs);
            stats->internal_average_us = (uint32_t)(
                internal_total_us / (uint64_t)stats->completed_runs);
        }
        else
        {
            stats->minimum_us = 0U;
        }
    }

    return status;
}

CnnBenchmarkStatus CnnBenchmark_RunSoftware(
    CnnBenchmarkSoftwareRun software_run,
    void *software_context,
    uint32_t *output_words,
    uint32_t output_capacity_words,
    uint32_t warmup_runs,
    uint32_t measured_runs,
    CnnBenchmarkStats *stats)
{
    CnnBenchmarkStatus status = CNN_BENCHMARK_STATUS_INVALID_ARGUMENT;
    XTime start;
    XTime end;
    uint64_t total_us = 0ULL;
    uint32_t index;
    uint32_t elapsed_us;
    bool run_ok;

    if ((software_run != NULL) && (output_words != NULL) &&
        (output_capacity_words > 0U) && (stats != NULL) &&
        (measured_runs > 0U))
    {
        CnnBenchmark_ResetStats(stats, measured_runs);
        status = CNN_BENCHMARK_STATUS_OK;

        for (index = 0U; index < warmup_runs; ++index)
        {
            run_ok = software_run(software_context,
                                  output_words,
                                  output_capacity_words);
            if (!run_ok)
            {
                status = CNN_BENCHMARK_STATUS_EXECUTION_FAILED;
                break;
            }
        }

        if (status == CNN_BENCHMARK_STATUS_OK)
        {
            for (index = 0U; index < measured_runs; ++index)
            {
                XTime_GetTime(&start);
                run_ok = software_run(software_context,
                                      output_words,
                                      output_capacity_words);
                XTime_GetTime(&end);

                if (run_ok)
                {
                    elapsed_us = CnnBenchmark_ElapsedUs(start, end);
                    CnnBenchmark_AddSample(stats, elapsed_us);
                    total_us += (uint64_t)elapsed_us;
                    stats->last_checksum = CnnBenchmark_Checksum(
                        output_words, output_capacity_words);
                }
                else
                {
                    ++stats->failed_runs;
                    status = CNN_BENCHMARK_STATUS_EXECUTION_FAILED;
                }
            }
        }

        if (stats->completed_runs > 0U)
        {
            stats->average_us =
                (uint32_t)(total_us / (uint64_t)stats->completed_runs);
        }
        else
        {
            stats->minimum_us = 0U;
        }
    }

    return status;
}

CnnBenchmarkStatus CnnBenchmark_CompareInt8Outputs(
    const uint32_t *hardware_output,
    const uint32_t *software_output,
    uint32_t word_count,
    CnnBenchmarkDifference *difference)
{
    CnnBenchmarkStatus status = CNN_BENCHMARK_STATUS_INVALID_ARGUMENT;
    uint32_t word_index;
    uint32_t byte_index;
    uint32_t shift;
    uint32_t absolute_error;
    uint32_t compared_bytes;
    int32_t hardware_value;
    int32_t software_value;

    if ((hardware_output != NULL) && (software_output != NULL) &&
        (word_count > 0U) && (difference != NULL) &&
        (word_count <= (UINT32_MAX / CNN_BENCHMARK_BYTES_PER_WORD)))
    {
        difference->compared_words = word_count;
        difference->different_words = 0U;
        difference->different_bytes = 0U;
        difference->maximum_absolute_error = 0U;
        difference->total_absolute_error = 0ULL;
        difference->mean_absolute_error_milli = 0U;

        for (word_index = 0U; word_index < word_count; ++word_index)
        {
            if (hardware_output[word_index] != software_output[word_index])
            {
                ++difference->different_words;
            }

            for (byte_index = 0U;
                 byte_index < CNN_BENCHMARK_BYTES_PER_WORD;
                 ++byte_index)
            {
                shift = byte_index * 8U;
                hardware_value = (int32_t)(int8_t)(
                    (hardware_output[word_index] >> shift) & 0xFFU);
                software_value = (int32_t)(int8_t)(
                    (software_output[word_index] >> shift) & 0xFFU);

                if (hardware_value >= software_value)
                {
                    absolute_error =
                        (uint32_t)(hardware_value - software_value);
                }
                else
                {
                    absolute_error =
                        (uint32_t)(software_value - hardware_value);
                }

                if (absolute_error != 0U)
                {
                    ++difference->different_bytes;
                }
                if (absolute_error > difference->maximum_absolute_error)
                {
                    difference->maximum_absolute_error = absolute_error;
                }
                difference->total_absolute_error +=
                    (uint64_t)absolute_error;
            }
        }

        compared_bytes = word_count * CNN_BENCHMARK_BYTES_PER_WORD;
        difference->mean_absolute_error_milli = (uint32_t)(
            (difference->total_absolute_error * 1000ULL) /
            (uint64_t)compared_bytes);
        status = CNN_BENCHMARK_STATUS_OK;
    }

    return status;
}

void CnnBenchmark_PrintStats(const char *name,
                             const CnnBenchmarkStats *stats)
{
    uint32_t fps_milli = 0U;

    if ((name != NULL) && (stats != NULL))
    {
        if (stats->average_us > 0U)
        {
            fps_milli = (uint32_t)(1000000000ULL /
                                   (uint64_t)stats->average_us);
        }
        xil_printf("\r\n\r\n\r\n\r\n");
        xil_printf("___________________________\r\n");
        xil_printf("\r\n\r\n\r\n\r\n");


        xil_printf("[BENCH][%s] runs=%u/%u fail=%u "
                   "avg=%u us min=%u us max=%u us "
                   "fps=%u.%03u checksum=%08x\r\n",
                   name,
                   (unsigned int)stats->completed_runs,
                   (unsigned int)stats->requested_runs,
                   (unsigned int)stats->failed_runs,
                   (unsigned int)stats->average_us,
                   (unsigned int)stats->minimum_us,
                   (unsigned int)stats->maximum_us,
                   (unsigned int)(fps_milli / 1000U),
                   (unsigned int)(fps_milli % 1000U),
                   (unsigned int)stats->last_checksum);



        if (stats->internal_average_us > 0U)
        {
            xil_printf("[BENCH][%s] accelerator+DMA average=%u us\r\n",
                       name,
                       (unsigned int)stats->internal_average_us);
        }
        xil_printf("\r\n\r\n\r\n\r\n");

        xil_printf("___________________________\r\n");
        xil_printf("\r\n\r\n\r\n\r\n");
    }
}

void CnnBenchmark_PrintSpeedup(const CnnBenchmarkStats *software,
                               const CnnBenchmarkStats *hardware)
{
    uint64_t speedup_milli;

    if ((software != NULL) && (hardware != NULL) &&
        (software->average_us > 0U) && (hardware->average_us > 0U))
    {
        speedup_milli =
            ((uint64_t)software->average_us * 1000ULL) /
            (uint64_t)hardware->average_us;
        xil_printf("[BENCH] speedup=%u.%03ux (SW/HW)\r\n",
                   (unsigned int)(speedup_milli / 1000ULL),
                   (unsigned int)(speedup_milli % 1000ULL));
    }
}

void CnnBenchmark_PrintDifference(
    const CnnBenchmarkDifference *difference)
{
    if (difference != NULL)
    {
        xil_printf("[BENCH][OA] words=%u different_words=%u "
                   "different_bytes=%u max_abs=%u mean_abs=%u.%03u\r\n",
                   (unsigned int)difference->compared_words,
                   (unsigned int)difference->different_words,
                   (unsigned int)difference->different_bytes,
                   (unsigned int)difference->maximum_absolute_error,
                   (unsigned int)(difference->mean_absolute_error_milli /
                                  1000U),
                   (unsigned int)(difference->mean_absolute_error_milli %
                                  1000U));
    }
}
