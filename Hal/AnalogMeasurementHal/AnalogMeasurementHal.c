#include <stddef.h>

#include "fsl_adc.h"
#include "fsl_clock.h"
#include "fsl_power.h"
#include "fsl_dma.h"
#include "fsl_inputmux.h"

#include "AnalogMeasurementHal.h"

#define ADC_SAMPLE_RATE                         10000
#define ADC_CHANNEL_0                           0
#define ADC_CHANNEL_1                           2
#define ADC_CHANNEL_2                           4
#define ADC_CHANNEL_3                           5
#define ADC_CHANNEL_4                           9
#define ADC_CHANNEL_5                           10
#define ADC_CHANNEL_6                           11
#define ADC_CHANNEL_7                           5
#define ADC_CHANNEL_8                           9
#define ADC_CHANNEL_9                           7
#define ADC_CHANNEL_10                          3
#define ADC_CHANNEL_11                          0

#define ADC_DMA_CHANNEL                         1U

#define DEMO_ADC_SAMPLE_CHANNEL_NUMBER 0U
#define DEMO_ADC_IRQ_ID                         ADC0_SEQA_IRQn
#define DEMO_ADC_IRQ_HANDLER_FUNC               ADC0_SEQA_IRQHandler
#define DEMO_DMA_ADC_CHANNEL                    0U
#define DEMO_ADC_BASE                           ADC0
#define DEMO_ADC_DATA_REG_ADDR                  (uint32_t) (&(ADC0->DAT[DEMO_ADC_SAMPLE_CHANNEL_NUMBER]))

#define ADC_CHANNEL                             0
#define DEMO_ADC_IRQ_ID                         ADC0_SEQA_IRQn

// Analog input channel 0
#define ANALOG_INPUT_PIN_0                      9
#define ANALOG_INPUT_PIO_0                      1
#define ANALOG_INPUT_FUN_0                      IOCON_FUNC3
// Analog input channel 1
#define ANALOG_INPUT_PIN_1                      23
#define ANALOG_INPUT_PIO_1                      0
#define ANALOG_INPUT_FUN_1                      IOCON_FUNC1
// Analog input channel 2
#define ANALOG_INPUT_PIN_2                      16
#define ANALOG_INPUT_PIO_2                      0
#define ANALOG_INPUT_FUN_2                      IOCON_FUNC1
// Analog input channel 3
#define ANALOG_INPUT_PIN_3                      15
#define ANALOG_INPUT_PIO_3                      0
#define ANALOG_INPUT_FUN_3                      IOCON_FUNC2
// Analog input channel 4
#define ANALOG_INPUT_PIN_4                      22
#define ANALOG_INPUT_PIO_4                      1
#define ANALOG_INPUT_FUN_4                      IOCON_FUNC3
// Analog input channel 5
#define ANALOG_INPUT_PIN_5                      3
#define ANALOG_INPUT_PIO_5                      1
#define ANALOG_INPUT_FUN_5                      IOCON_FUNC4
// Analog input channel 6
#define ANALOG_INPUT_PIN_6                      14
#define ANALOG_INPUT_PIO_6                      0
#define ANALOG_INPUT_FUN_6                      IOCON_FUNC2
// Analog input channel 7
#define ANALOG_INPUT_PIN_7                      13
#define ANALOG_INPUT_PIO_7                      0
#define ANALOG_INPUT_FUN_7                      IOCON_FUNC2
// Analog input channel 8
#define ANALOG_INPUT_PIN_8                      12
#define ANALOG_INPUT_PIO_8                      0
#define ANALOG_INPUT_FUN_8                      IOCON_FUNC2
// Analog input channel 9
#define ANALOG_INPUT_PIN_9                      11
#define ANALOG_INPUT_PIO_9                      0
#define ANALOG_INPUT_FUN_9                      IOCON_FUNC2
// Analog input channel 10
#define ANALOG_INPUT_PIN_10                     29
#define ANALOG_INPUT_PIO_10                     1
#define ANALOG_INPUT_FUN_10                     IOCON_FUNC4
// Analog input channel 11
#define ANALOG_INPUT_PIN_11                     22
#define ANALOG_INPUT_PIO_11                     0
#define ANALOG_INPUT_FUN_11                     IOCON_FUNC1


static const PINMUX_GRP_T analogPinMuxingTable[] = {
    {ANALOG_INPUT_PIO_0, ANALOG_INPUT_PIN_0, (ANALOG_INPUT_FUN_0 | IOCON_ADMODE_EN | IOCON_MODE_INACT)},
    {ANALOG_INPUT_PIO_1, ANALOG_INPUT_PIN_1, (ANALOG_INPUT_FUN_1 | IOCON_ADMODE_EN | IOCON_MODE_INACT)},
    {ANALOG_INPUT_PIO_2, ANALOG_INPUT_PIN_2, (ANALOG_INPUT_FUN_2 | IOCON_ADMODE_EN | IOCON_MODE_INACT)},
	{ANALOG_INPUT_PIO_3, ANALOG_INPUT_PIN_3, (ANALOG_INPUT_FUN_3 | IOCON_ADMODE_EN | IOCON_MODE_INACT)},

    {ANALOG_INPUT_PIO_4, ANALOG_INPUT_PIN_4, (ANALOG_INPUT_FUN_4 | IOCON_ADMODE_EN | IOCON_MODE_INACT)},
	{ANALOG_INPUT_PIO_5, ANALOG_INPUT_PIN_5, (ANALOG_INPUT_FUN_5 | IOCON_ADMODE_EN | IOCON_MODE_INACT)},
	{ANALOG_INPUT_PIO_6, ANALOG_INPUT_PIN_6, (ANALOG_INPUT_FUN_6 | IOCON_ADMODE_EN | IOCON_MODE_INACT)},
	{ANALOG_INPUT_PIO_7, ANALOG_INPUT_PIN_7, (ANALOG_INPUT_FUN_7 | IOCON_ADMODE_EN | IOCON_MODE_INACT)},

    {ANALOG_INPUT_PIO_8, ANALOG_INPUT_PIN_8, (ANALOG_INPUT_FUN_8 | IOCON_ADMODE_EN | IOCON_MODE_INACT)},
	{ANALOG_INPUT_PIO_9, ANALOG_INPUT_PIN_9, (ANALOG_INPUT_FUN_9 | IOCON_ADMODE_EN | IOCON_MODE_INACT)},
	{ANALOG_INPUT_PIO_10, ANALOG_INPUT_PIN_10, (ANALOG_INPUT_FUN_10 | IOCON_ADMODE_EN | IOCON_MODE_INACT)},
	{ANALOG_INPUT_PIO_11, ANALOG_INPUT_PIN_11, (ANALOG_INPUT_FUN_11 | IOCON_ADMODE_EN | IOCON_MODE_INACT)},
};

static const uint32_t analogChannelMap[] = {ADC_CHANNEL_0, ADC_CHANNEL_1, ADC_CHANNEL_2,
                                            ADC_CHANNEL_3, ADC_CHANNEL_4, ADC_CHANNEL_5,
                                            ADC_CHANNEL_6, ADC_CHANNEL_7, ADC_CHANNEL_8,
                                            ADC_CHANNEL_9, ADC_CHANNEL_10, ADC_CHANNEL_11};

static uint32_t analogMeasurementBuffer[ANALOG_MEASUREMENT_COUNT];
static AnalogMeasurementReadyCb analogMeasurementReadyCallback = NULL;

static void ADC_Configuration(void);
static void DMA_Configfuation(void);
static void NVIC_Configuration(void);

dma_handle_t gDmaHandleStruct;  /* Handler structure for using DMA. */
uint32_t gDemoAdcConvResult[1]; /* Keep the ADC conversion resulut moved from ADC data register by DMA. */
volatile bool bDmaTransferDone; /* Flag of DMA transfer done trigger by ADC conversion. */


void analogMeasurementInit(AnalogMeasurementReadyCb analogMeasurementReadyCb)
{
    analogMeasurementReadyCallback = analogMeasurementReadyCb;

    /* SYSCON power. */
    POWER_DisablePD(kPDRUNCFG_PD_VDDA);    /* Power on VDDA. */
    POWER_DisablePD(kPDRUNCFG_PD_ADC0);    /* Power on the ADC converter. */
    POWER_DisablePD(kPDRUNCFG_PD_VD2_ANA); /* Power on the analog power supply. */
    POWER_DisablePD(kPDRUNCFG_PD_VREFP);   /* Power on the reference voltage source. */
    POWER_DisablePD(kPDRUNCFG_PD_TS);      /* Power on the temperature sensor. */

    /* Enable the clock. */
    CLOCK_AttachClk(kFRO12M_to_MAIN_CLK);

    /* CLOCK_AttachClk(kMAIN_CLK_to_ADC_CLK); */
    /* Sync clock source is not used. Using sync clock source and would be divided by 2.
     * The divider would be set when configuring the converter.
     */

    CLOCK_EnableClock(kCLOCK_Adc0); /* SYSCON->AHBCLKCTRL[0] |= SYSCON_AHBCLKCTRL_ADC0_MASK; */

    INPUTMUX_AttachSignal(INPUTMUX, DEMO_DMA_ADC_CHANNEL, kINPUTMUX_Adc0SeqaIrqToDma);

    dma_transfer_config_t dmaTransferConfigStruct;
    dma_channel_trigger_t dmaChannelTriggerStruct;

    /* Configure DMAMUX. */
    INPUTMUX_Init(INPUTMUX);


    /* Configure DMA. */
    DMA_Init(DMA0);
    DMA_EnableChannel(DMA0, DEMO_DMA_ADC_CHANNEL);
    DMA_CreateHandle(&gDmaHandleStruct, DMA0, DEMO_DMA_ADC_CHANNEL);
    DMA_SetCallback(&gDmaHandleStruct, DEMO_DMA_Callback, NULL);
    /*
     * Configure the DMA trigger:
     * The DATAVALID of ADC will trigger the interrupt. This signal is also for thie DMA triger, which is changed 0 ->
     * 1.
     */
    dmaChannelTriggerStruct.burst = kDMA_EdgeBurstTransfer1;
    dmaChannelTriggerStruct.type = kDMA_RisingEdgeTrigger;
    dmaChannelTriggerStruct.wrap = kDMA_NoWrap;
    DMA_ConfigureChannelTrigger(DMA0, DEMO_DMA_ADC_CHANNEL, &dmaChannelTriggerStruct);
    /* Prepare and submit the transfer. */
    DMA_PrepareTransfer(&dmaTransferConfigStruct,       /* To keep the configuration. */
                        (void *)DEMO_ADC_DATA_REG_ADDR, /* DMA transfer source address. */
                        (void *)gDemoAdcConvResult,     /* DMA transfer destination address. */
                        sizeof(uint32_t),               /* DMA transfer destination address width(bytes). */
                        sizeof(uint32_t),               /* DMA transfer bytes to be transferred. */
                        kDMA_MemoryToMemory,            /* DMA transfer type. */
                        NULL                            /* nextDesc Chain custom descriptor to transfer. */
                        );
    DMA_SubmitTransfer(&gDmaHandleStruct, &dmaTransferConfigStruct);

    adc_config_t adcConfigStruct;
    adc_conv_seq_config_t adcConvSeqConfigStruct;

    adcConfigStruct.clockMode = kADC_ClockSynchronousMode; /* Using sync clock source. */
    adcConfigStruct.clockDividerNumber = 1;                /* The divider for sync clock is 2. */
    adcConfigStruct.resolution = kADC_Resolution12bit;
    adcConfigStruct.enableBypassCalibration = false;
    adcConfigStruct.sampleTimeNumber = 0U;
    ADC_Init(DEMO_ADC_BASE, &adcConfigStruct);

    ADC_DoSelfCalibration(DEMO_ADC_BASE);
    adcConvSeqConfigStruct.channelMask = (1U << DEMO_ADC_SAMPLE_CHANNEL_NUMBER); /* Includes channel DEMO_ADC_SAMPLE_CHANNEL_NUMBER. */
    adcConvSeqConfigStruct.triggerMask = 0U;
    adcConvSeqConfigStruct.triggerPolarity = kADC_TriggerPolarityPositiveEdge;
    adcConvSeqConfigStruct.enableSingleStep = false;
    adcConvSeqConfigStruct.enableSyncBypass = false;
    adcConvSeqConfigStruct.interruptMode = kADC_InterruptForEachSequence; /* Enable the interrupt/DMA trigger. */
    ADC_SetConvSeqAConfig(DEMO_ADC_BASE, &adcConvSeqConfigStruct);
    ADC_EnableConvSeqA(DEMO_ADC_BASE, true); /* Enable the conversion sequence A. */

    ADC_EnableInterrupts(DEMO_ADC_BASE, kADC_ConvSeqAInterruptEnable);

    NVIC_EnableIRQ(DMA0_IRQn);

}

void DEMO_DMA_Callback(dma_handle_t *handle, void *param, bool transferDone, uint32_t tcds)
{
    bDmaTransferDone = true;
}

void ADCA_IRQHandler(void)
{
    Chip_ADC_ClearFlags(LPC_ADC, Chip_ADC_GetFlags(LPC_ADC));

    uint32_t i;
    for (i = 0; i < ANALOG_MEASUREMENT_COUNT; i++)
        analogMeasurementBuffer[i] = ADC_DR_RESULT(Chip_ADC_GetDataReg(LPC_ADC, analogChannelMap[i]));

    if (analogMeasurementReadyCallback)
        analogMeasurementReadyCallback();
}

void analogMeasurementStart(void)
{
    DMA_StartTransfer(&gDmaHandleStruct);         /* Enable the DMA every time for each transfer. */
    ADC_DoSoftwareTriggerConvSeqA(DEMO_ADC_BASE); /* Trigger the ADC and start the conversion. */
}

uint32_t analogMeasurementGetValue(AnalogMeasurementType analogMeasurement)
{
    if (analogMeasurement < ANALOG_MEASUREMENT_COUNT) {
        adc_result_info_t gAdcResultInfoStruct;
        ADC_GetChannelConversionResult(ADC0, ADC_CHANNEL , &gAdcResultInfoStruct);
        return gAdcResultInfoStruct.result;

        return analogMeasurementBuffer[analogMeasurement];
    } else {
        return 0xFFFFFFFF;
    }
}

