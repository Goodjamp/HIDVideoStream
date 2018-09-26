#ifndef __ANALOG_MEASUREMENT_SYTEM_H__
#define __ANALOG_MEASUREMENT_SYTEM_H__

#include <stdint.h>

void temperatureSetExternal(uint32_t fanNumber, uint32_t temperature);
uint32_t temperatureGetExternal(uint32_t fanNumber);
uint32_t temperatureGetInternal(uint32_t temperatureIndex);
uint32_t powerGetVoltage(uint32_t voltageIndex);
void analogMeasurementSystemInit(void);
void analogMeasurementSystemStartConversion(void);
int32_t analogMeasurementTemperatureFromAdcValue(uint32_t resT);

#endif
