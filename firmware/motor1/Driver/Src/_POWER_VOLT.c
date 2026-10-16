#include "_POWER_VOLT.h"
#include "adc.h"

#define POWER_VOLT_ADC_VREF          3.3f
#define POWER_VOLT_ADC_MAX_COUNT     4095.0f
#define POWER_VOLT_R_TOP             10000.0f
#define POWER_VOLT_R_BOTTOM          1000.0f
#define POWER_VOLT_DIVIDER_GAIN      \
    ((POWER_VOLT_R_TOP + POWER_VOLT_R_BOTTOM) / POWER_VOLT_R_BOTTOM)
#define POWER_VOLT_SAMPLE_COUNT      9U
#define POWER_VOLT_FILTER_WEIGHT     3U

static uint16_t cached_power_10mv = 0;
static uint8_t power_voltage_valid = 0U;

uint8_t PowerVolt_Update(void)
{
    uint16_t adc_samples[POWER_VOLT_SAMPLE_COUNT];
    uint16_t adc_value;
    uint16_t sample;
    uint32_t index;
    uint32_t insert;
    float adc_voltage;
    float power_voltage;
    float voltage_10mv;

    /* PWM switching can corrupt an individual conversion.  Take an odd-sized
     * burst and use its median so a switching spike cannot become the reported
     * supply voltage. */
    for (index = 0U; index < POWER_VOLT_SAMPLE_COUNT; index++)
    {
        if (HAL_ADC_Start(&hadc2) != HAL_OK)
        {
            return 0;
        }
        if (HAL_ADC_PollForConversion(&hadc2, 10) != HAL_OK)
        {
            HAL_ADC_Stop(&hadc2);
            return 0;
        }
        adc_samples[index] = (uint16_t)HAL_ADC_GetValue(&hadc2);
        HAL_ADC_Stop(&hadc2);
    }

    for (index = 1U; index < POWER_VOLT_SAMPLE_COUNT; index++)
    {
        sample = adc_samples[index];
        insert = index;
        while ((insert > 0U) && (adc_samples[insert - 1U] > sample))
        {
            adc_samples[insert] = adc_samples[insert - 1U];
            insert--;
        }
        adc_samples[insert] = sample;
    }
    adc_value = adc_samples[POWER_VOLT_SAMPLE_COUNT / 2U];

    adc_voltage = ((float)adc_value * POWER_VOLT_ADC_VREF)
                / POWER_VOLT_ADC_MAX_COUNT;
    power_voltage = adc_voltage * POWER_VOLT_DIVIDER_GAIN;
    voltage_10mv = power_voltage * 100.0f;

    if (voltage_10mv > 65535.0f)
    {
        voltage_10mv = 65535.0f;
    }

    if (power_voltage_valid == 0U)
    {
        cached_power_10mv = (uint16_t)voltage_10mv;
        power_voltage_valid = 1U;
    }
    else
    {
        cached_power_10mv = (uint16_t)((
            ((uint32_t)cached_power_10mv * POWER_VOLT_FILTER_WEIGHT) +
            (uint32_t)voltage_10mv + (POWER_VOLT_FILTER_WEIGHT / 2U)) /
            (POWER_VOLT_FILTER_WEIGHT + 1U));
    }
    return 1;
}


uint16_t PowerVolt_Get10mV(void)
{
    return cached_power_10mv;
}
