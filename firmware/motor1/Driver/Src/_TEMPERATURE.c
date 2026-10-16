#include "_TEMPERATURE.h"
#include "adc.h"

#define TEMPERATURE_AVG_SLOPE       4.3E-03f
#define TEMPERATURE_V25             1.43f
#define TEMPERATURE_ADC_TO_VOLT     (3.3f / 4096.0f)

static int8_t cached_temperature_c = 0;

uint8_t Temperature_Update(void)
{
    uint16_t adc_value;
    float sense_voltage;
    float temperature;

    if (HAL_ADC_Start(&hadc1) != HAL_OK)
    {
        return 0;
    }

    if (HAL_ADC_PollForConversion(&hadc1, 10) != HAL_OK)
    {
        HAL_ADC_Stop(&hadc1);
        return 0;
    }

    adc_value = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);

    sense_voltage = (float)adc_value * TEMPERATURE_ADC_TO_VOLT;
    temperature = (TEMPERATURE_V25 - sense_voltage)
                / TEMPERATURE_AVG_SLOPE + 25.0f;

    if (temperature > 127.0f)
    {
        temperature = 127.0f;
    }
    else if (temperature < -128.0f)
    {
        temperature = -128.0f;
    }

    cached_temperature_c = (int8_t)temperature;
    return 1;
}


int8_t Temperature_GetCelsius(void)
{
    return cached_temperature_c;
}
