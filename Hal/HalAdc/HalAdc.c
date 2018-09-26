#include "HalAdc.h"

#include "HalGpio.h"

#include "fsl_adc.h"
#include "fsl_clock.h"
#include "fsl_power.h"


#define ADC_CHANNEL_0   0
#define ADC_CHANNEL_1   1
#define ADC_CHANNEL_2   2
#define ADC_CHANNEL_3   3
#define ADC_CHANNEL_4   4
#define ADC_CHANNEL_5   5
#define ADC_CHANNEL_6   6
#define ADC_CHANNEL_7   7
#define ADC_CHANNEL_8   8
#define ADC_CHANNEL_9   9
#define ADC_CHANNEL_10  10

#define DEMO_ADC_IRQ_ID ADC0_SEQA_IRQn


static HalAdcConversionDoneCallback halAdcConversionDoneCallback = NULL;
static uint32_t adcChannelMap[HAL_ADC_CHANNEL_COUNT] = {
    [HAL_ADC_CHANNEL_THERMISTOR_0] = ADC_CHANNEL_7,
    [HAL_ADC_CHANNEL_THERMISTOR_1] = ADC_CHANNEL_8,
    [HAL_ADC_CHANNEL_12V] = ADC_CHANNEL_10,
    [HAL_ADC_CHANNEL_5V] = ADC_CHANNEL_6,
    [HAL_ADC_CHANNEL_3V3] = ADC_CHANNEL_5,
};

static void hallAdcClockPowerConfiguration(void)
{
    // SYSCON power.
    POWER_DisablePD(kPDRUNCFG_PD_VDDA);    // Power on VDDA.
    POWER_DisablePD(kPDRUNCFG_PD_ADC0);    // Power on the ADC converter.
    POWER_DisablePD(kPDRUNCFG_PD_VD2_ANA); // Power on the analog power supply.
    POWER_DisablePD(kPDRUNCFG_PD_VREFP);   // Power on the reference voltage source.
    POWER_DisablePD(kPDRUNCFG_PD_TS);      // Power on the temperature sensor.
    CLOCK_EnableClock(kCLOCK_Adc0); // SYSCON->AHBCLKCTRL[0] |= SYSCON_AHBCLKCTRL_ADC0_MASK;
}

//Configure the ADC as normal converter in polling mode.
static void halAdcConfiguration(void)
{
    adc_config_t adcConfigStruct;
    adc_conv_seq_config_t adcConvSeqConfigStruct;

    // Configure the converter.
    adcConfigStruct.clockMode = kADC_ClockSynchronousMode; // Using sync clock source.
    adcConfigStruct.clockDividerNumber = 1;                // The divider for sync clock is 2.
    adcConfigStruct.resolution = kADC_Resolution12bit;
    adcConfigStruct.enableBypassCalibration = false;
    adcConfigStruct.sampleTimeNumber = 0U;
    ADC_Init(ADC0, &adcConfigStruct);

    // Enable channel DEMO_ADC_SAMPLE_CHANNEL_NUMBER's conversion in Sequence A.
    // Includes channel DEMO_ADC_SAMPLE_CHANNEL_NUMBER.
    uint32_t mask = 0;

    for (size_t i = 0; i < HAL_ADC_CHANNEL_COUNT; i++) {
        mask |= 1 << adcChannelMap[i];
    }

    adcConvSeqConfigStruct.channelMask = mask;
    adcConvSeqConfigStruct.triggerMask = 0U;
    adcConvSeqConfigStruct.triggerPolarity = kADC_TriggerPolarityPositiveEdge;
    adcConvSeqConfigStruct.enableSingleStep = false;
    adcConvSeqConfigStruct.enableSyncBypass = false;
    adcConvSeqConfigStruct.interruptMode = kADC_InterruptForEachSequence;
    ADC_SetConvSeqAConfig(ADC0, &adcConvSeqConfigStruct);
    ADC_EnableConvSeqA(ADC0, true);
}

void ADC0_SEQA_IRQHandler(void)
{
    if (kADC_ConvSeqAInterruptFlag == (kADC_ConvSeqAInterruptFlag & ADC_GetStatusFlags(ADC0)))
    {
        ADC_ClearStatusFlags(ADC0, kADC_ConvSeqAInterruptFlag);

        if (halAdcConversionDoneCallback) {
            halAdcConversionDoneCallback();
        }
    }
    // Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F Store immediate overlapping
    // exception return operation might vector to incorrect interrupt
    #if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
}

void halAdcInit(void)
{
    halGpioPinInit(halGpio.adcTermistor0);
    halGpioPinInit(halGpio.adcTermistor1);
    halGpioPinInit(halGpio.adc12v);
    halGpioPinInit(halGpio.adc5v);
    halGpioPinInit(halGpio.adc3v3);
    // Enable the power and clock for ADC.
    hallAdcClockPowerConfiguration();
    // Calibration.
    ADC_DoSelfCalibration(ADC0);
    // Configure the ADC as basic polling mode.
    halAdcConfiguration();
    // Enable the interrupt the for sequence A done.
    ADC_EnableInterrupts(ADC0, kADC_ConvSeqAInterruptEnable);
    NVIC_EnableIRQ(DEMO_ADC_IRQ_ID);
}

void halAdcStartConversion(HalAdcConversionDoneCallback callback)
{
    halAdcConversionDoneCallback = callback;
    ADC_DoSoftwareTriggerConvSeqA(ADC0);
}

uint32_t halAdcGetConversionResult(HalAdcChannel channel)
{
    if (channel >= HAL_ADC_CHANNEL_COUNT) {
        return 0;
    }

    adc_result_info_t gAdcResultInfoStruct;
    ADC_GetChannelConversionResult(ADC0, adcChannelMap[channel], &gAdcResultInfoStruct);
    return gAdcResultInfoStruct.result;
}

