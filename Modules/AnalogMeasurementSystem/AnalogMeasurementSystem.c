#include "AnalogMeasurementSystem.h"
#include "HalAdc.h"
#include <math.h>

#define MAX_TEMP_COUNT                      6
#define ANALOG_WINDOW_SIZE                  16
#define ANALOG_WINDOW_SIZEMASK              0x0000000F
#define ANALOG_WINDOW_SHIFT_MULT            4

#define VOLTAGE_DIVIDER_3V3_A               11
#define VOLTAGE_DIVIDER_3V3_B               10

#define VOLTAGE_DIVIDER_5V_A                11
#define VOLTAGE_DIVIDER_5V_B                10

#define VOLTAGE_DIVIDER_12V_A               11
#define VOLTAGE_DIVIDER_12V_B               10

static uint32_t externalTemperature[MAX_TEMP_COUNT];
static uint32_t internalMeasurements[HAL_ADC_CHANNEL_COUNT];

static int32_t analogDataWindow[HAL_ADC_CHANNEL_COUNT][ANALOG_WINDOW_SIZE];
static uint32_t analogDataWindowHead;

static void onAnalogMeasurementReady(void)
{
    uint32_t i = 0;

    for (i = 0; i < HAL_ADC_CHANNEL_COUNT; i++) {
        int32_t mu = 0;
        uint32_t j = 0;

        for (j = 0; j < ANALOG_WINDOW_SIZE; j++) {
            mu += analogDataWindow[i][j];
        }
        mu >>= ANALOG_WINDOW_SHIFT_MULT;
        internalMeasurements[i] = mu;
        internalMeasurements[i] = internalMeasurements[i] < 0 ? 0 : internalMeasurements[i];
        analogDataWindow[i][analogDataWindowHead] = halAdcGetConversionResult((HalAdcChannel) i);
    }
    analogDataWindowHead = (analogDataWindowHead + 1) & ANALOG_WINDOW_SIZEMASK;
}

void temperatureSetExternal(uint32_t fanNumber, uint32_t temperature)
{
    externalTemperature[fanNumber] = temperature;
}

uint32_t temperatureGetExternal(uint32_t fanNumber)
{
    return externalTemperature[fanNumber];
}

uint32_t temperatureGetInternal(uint32_t temperatureIndex)
{
    return internalMeasurements[temperatureIndex];
}

uint32_t powerGetVoltage(uint32_t voltageIndex)
{
    uint32_t voltage = internalMeasurements[(uint32_t) HAL_ADC_CHANNEL_THERMISTOR_1 + voltageIndex + 1];
    if (voltageIndex == 0) {
        voltage = voltage * 3300 * 20 / 4096 / 5;
    }

    if (voltageIndex == 1) {
        voltage = voltage * 3300 * 25 / 4096 / 15;
    }

    if (voltageIndex == 2) {
        voltage = voltage * 3300 * 165 / 4096 / 150;
    }

    return voltage;
}

void analogMeasurementSystemInit(void)
{
    halAdcInit();
}

void analogMeasurementSystemStartConversion(void)
{
    halAdcStartConversion(onAnalogMeasurementReady);
}

int32_t analogMeasurementTemperatureFromAdcValue(uint32_t resT)
{
    #define Thermistor_K2C               (273.15f)
    #define Thermistor_SCALE             (100.0f)

    #define Thermistor_THA               (0.0006728238f)
    #define Thermistor_THB               (0.0002910997f)
    #define Thermistor_THC               (8.412704E-11f)
    #define RESISTOR_DIVIDER_VALUE       10000


    uint32_t thermistorValue;
    if (resT)
        thermistorValue = (RESISTOR_DIVIDER_VALUE * (ADC_MAX_VALUE - resT)) / resT;
    else {
        thermistorValue = RESISTOR_DIVIDER_VALUE;
    }

    int32_t tempTR;
    float stEqn;
    float logrT;

    logrT = logf(thermistorValue);
    stEqn = Thermistor_THA + Thermistor_THB * logrT + Thermistor_THC * logrT * logrT * logrT;
    tempTR = ((1.0f / stEqn) - Thermistor_K2C) * Thermistor_SCALE + 0.5f;

    return tempTR;
}
