#ifndef __HAL_ADC_H__
#define __HAL_ADC_H__

#include <stdint.h>
#include <stdbool.h>

#define ADC_MAX_VALUE   4096

typedef enum {
    HAL_ADC_CHANNEL_THERMISTOR_0,
    HAL_ADC_CHANNEL_THERMISTOR_1,
    HAL_ADC_CHANNEL_12V,
    HAL_ADC_CHANNEL_5V,
    HAL_ADC_CHANNEL_3V3,
    HAL_ADC_CHANNEL_COUNT
} HalAdcChannel;

typedef void (*HalAdcConversionDoneCallback)(void);

void halAdcInit(void);
void halAdcStartConversion(HalAdcConversionDoneCallback callback);
uint32_t halAdcGetConversionResult(HalAdcChannel channel);

#endif
