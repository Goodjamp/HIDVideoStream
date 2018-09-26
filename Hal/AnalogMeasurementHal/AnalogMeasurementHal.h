#ifndef __ANALOG_MEASUREMENT_H__
#define __ANALOG_MEASUREMENT_H__

#include <stdint.h>
#include <stdbool.h>

#define ADC_MAX_VALUE   4095

typedef enum AnalogMeasurementType {
    ANALOG_THERMISTOR_0,
    ANALOG_THERMISTOR_1,
    ANALOG_THERMISTOR_2,
    ANALOG_THERMISTOR_3,
    ANALOG_THERMISTOR_4,
    ANALOG_VOLTAGE_12V,
    ANALOG_VOLTAGE_5V,
    ANALOG_VOLTAGE_3V3,
    ANALOG_MEASUREMENT_COUNT,
} AnalogMeasurementType;

typedef void (*AnalogMeasurementReadyCb)(void);

void analogMeasurementInit(AnalogMeasurementReadyCb analogMeasurementReadyCb);
void analogMeasurementStart(void);
uint32_t analogMeasurementGetValue(AnalogMeasurementType analogMeasurement);

#endif
