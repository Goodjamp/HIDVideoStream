#include <stddef.h>
#include "fsl_iocon.h"
#include "fsl_dma.h"
#include "fsl_ctimer.h"
#include "fsl_inputmux.h"

#include "LedController.h"

// Periodic sampling timer
#define WS2812B_FREQ        800000
#define WS2812B_SHORT_LEVEL (17 * 2)
#define WS2812B_LONG_LEVEL  (38 * 2)

#define HD59731B_FREQ           406000
#define HD59731B_SHORT_LEVEL    (17 * 4)
#define HD59731B_LONG_LEVEL     (38 * 4)

#define LED_CONTROLLER_TIMER_UNIT               CTIMER3

#define LED_CONTROLLER_TIMER_MATCH_NUMBER       0
#define LED_CONTROLLER_TIMER_MATCH_LED_0        1
#define LED_CONTROLLER_TIMER_MATCH_LED_1        2

#define LED_CONTROLLER_DMA_CHANNEL_LED_0        0

// Led output 0
#define LED_CONTROLLER_PWM_PIN_0                30
#define LED_CONTROLLER_PWM_PIO_0                4
#define LED_CONTROLLER_PWM_FUN_0                IOCON_FUNC3

// Led output 1
#define LED_CONTROLLER_PWM_PIN_1                14
#define LED_CONTROLLER_PWM_PIO_1                3
#define LED_CONTROLLER_PWM_FUN_1                IOCON_FUNC3

// LED definitions
#define BITS_PER_LED                            24
#define MAX_LED_COUNT                           72
#define MAX_BITS_COUNT                          (MAX_LED_COUNT * BITS_PER_LED)


#define DMA_ADDR                                (void *)

static const iocon_group_t ledControllerPinMuxingTable[] = {
    {LED_CONTROLLER_PWM_PIO_0, LED_CONTROLLER_PWM_PIN_0,  (LED_CONTROLLER_PWM_FUN_0 | IOCON_DIGITAL_EN | IOCON_INPFILT_OFF)}, // CTIMER2_MAT3
	{LED_CONTROLLER_PWM_PIO_1, LED_CONTROLLER_PWM_PIN_1,  (LED_CONTROLLER_PWM_FUN_1 | IOCON_DIGITAL_EN | IOCON_INPFILT_OFF)},
};

static LedControllerOnDataSentCb onDataSentCallback = NULL;
static dma_handle_t hdma;
static dma_descriptor_t dmaDesc __attribute__((aligned(sizeof(dma_descriptor_t))));


static uint32_t ledDmaBuffer[MAX_BITS_COUNT + 1] __attribute__((aligned(sizeof(uint32_t))));

volatile bool ledControllerBusyFlag = false;

void dmaCallback(struct _dma_handle *handle, void *userData, bool transferDone, uint32_t intmode)
{
    LED_CONTROLLER_TIMER_UNIT->TCR &= ~(1);
    if (onDataSentCallback) {
        onDataSentCallback();
    }
    ledControllerBusyFlag = false;
}

void ledControllerSetLedData(uint32_t portNumber, LedType ledType, const uint32_t *rgbArray, uint32_t rgbArraylength)
{
    uint32_t ledBitFrequency = 0;
    uint32_t ledBitLongLevel = 0;
    uint32_t ledBitShortLevel = 0;

    switch(ledType) {
        case LED_TYPE_WS2812B:
            ledBitFrequency = WS2812B_FREQ;
            ledBitLongLevel = WS2812B_LONG_LEVEL;
            ledBitShortLevel = WS2812B_SHORT_LEVEL;
            break;

        case LED_TYPE_HD59731B:
            ledBitFrequency = HD59731B_FREQ;
            ledBitLongLevel = HD59731B_LONG_LEVEL;
            ledBitShortLevel = HD59731B_SHORT_LEVEL;
            break;

        default:
            return;
    }

    if (rgbArraylength > MAX_LED_COUNT) {
        return;
    }

    while (ledControllerBusyFlag == true)
        {};

    ledControllerBusyFlag = true;

    const uint32_t timerClockSpeed = 96000000UL;

    for(size_t ledCounter = 0; ledCounter < rgbArraylength; ledCounter++) {
        for(size_t bitNumber = 0; bitNumber < BITS_PER_LED; bitNumber++) {
            ledDmaBuffer[ledCounter * BITS_PER_LED + bitNumber] = (rgbArray[ledCounter] & (1 << (BITS_PER_LED - bitNumber - 1))) ? ledBitLongLevel : ledBitShortLevel;
        }
    }
    ledDmaBuffer[rgbArraylength * BITS_PER_LED] = 0;

    const ctimer_match_config_t periodMatchConfig = {
        .enableCounterReset = true,
        .enableCounterStop  = false,
        .matchValue         = timerClockSpeed / ledBitFrequency - 1,
        .outControl         = kCTIMER_Output_NoAction,
        .outPinInitState    = true,
        .enableInterrupt    = false,
    };

    const ctimer_match_config_t dutyCycleMatchConfig = {
        .enableCounterReset = false,
        .enableCounterStop  = false,
        .matchValue         = 0,
        .outControl         = kCTIMER_Output_NoAction,
        .outPinInitState    = true,
        .enableInterrupt    = false,
    };

    if (portNumber == 0) {
        LED_CONTROLLER_TIMER_UNIT->PWMC = 1 << 0;
        CTIMER_SetupMatch(LED_CONTROLLER_TIMER_UNIT, 0, &dutyCycleMatchConfig);
        CTIMER_SetupMatch(LED_CONTROLLER_TIMER_UNIT, 1, &periodMatchConfig);

        INPUTMUX->DMA_ITRIG_INMUX[LED_CONTROLLER_DMA_CHANNEL_LED_0] = 15; // match 1

    } else {
        LED_CONTROLLER_TIMER_UNIT->PWMC = 1 << LED_CONTROLLER_TIMER_MATCH_LED_0;
        CTIMER_SetupMatch(LED_CONTROLLER_TIMER_UNIT, 1, &dutyCycleMatchConfig);
        CTIMER_SetupMatch(LED_CONTROLLER_TIMER_UNIT, 0, &periodMatchConfig);

        INPUTMUX->DMA_ITRIG_INMUX[LED_CONTROLLER_DMA_CHANNEL_LED_0] = 14; // match 0
    }

    dma_transfer_config_t dmaTransferCfg;
    DMA_PrepareTransfer(&dmaTransferCfg, (uint8_t *)&ledDmaBuffer[0], (void *)&LED_CONTROLLER_TIMER_UNIT->MR[portNumber], 4, (rgbArraylength * BITS_PER_LED / 2) * 4, kDMA_MemoryToPeripheral, &dmaDesc);
    dmaTransferCfg.isPeriph = false;
    dmaTransferCfg.xfercfg.intA = false;
    dmaTransferCfg.xfercfg.intB = false;
    DMA_SubmitTransfer(&hdma, &dmaTransferCfg);

    DMA_PrepareTransfer(&dmaTransferCfg, (uint8_t *)&ledDmaBuffer[rgbArraylength * BITS_PER_LED / 2], (void *) &LED_CONTROLLER_TIMER_UNIT->MR[portNumber], 4, (rgbArraylength * BITS_PER_LED / 2) * 4, kDMA_MemoryToPeripheral, NULL);
    dmaTransferCfg.isPeriph = false;
    dmaTransferCfg.xfercfg.intA = true;
    dmaTransferCfg.xfercfg.intB = false;
    DMA_CreateDescriptor(&dmaDesc, &dmaTransferCfg.xfercfg, dmaTransferCfg.srcAddr, dmaTransferCfg.dstAddr, dmaTransferCfg.nextDesc);

    DMA_StartTransfer(&hdma);

    LED_CONTROLLER_TIMER_UNIT->TC = timerClockSpeed / ledBitFrequency - 1;
    LED_CONTROLLER_TIMER_UNIT->MR[portNumber] = ledDmaBuffer[0];
    LED_CONTROLLER_TIMER_UNIT->TCR |= 1;
}

void ledControllerInit(LedControllerOnDataSentCb onDataSentCb)
{
    onDataSentCallback = onDataSentCb;
    const uint32_t timerClockSpeed = 96000000UL;

    IOCON_SetPinMuxing(IOCON, ledControllerPinMuxingTable, sizeof(ledControllerPinMuxingTable) / sizeof(iocon_group_t));

    ctimer_config_t config;
    CTIMER_GetDefaultConfig(&config);
    CTIMER_Init(LED_CONTROLLER_TIMER_UNIT, &config);
    ctimer_match_config_t matchConfig = {
        .enableCounterReset = true,
        .enableCounterStop  = false,
        .matchValue         = timerClockSpeed / WS2812B_FREQ - 1,
        .outControl         = kCTIMER_Output_NoAction,
        .outPinInitState    = false,
        .enableInterrupt    = false,
    };

    ctimer_match_config_t matchConfig1 = {
        .enableCounterReset = false,
        .enableCounterStop  = false,
        .matchValue         = 0,
        .outControl         = kCTIMER_Output_NoAction,
        .outPinInitState    = true,
        .enableInterrupt    = false,
    };

    CTIMER_SetupMatch(LED_CONTROLLER_TIMER_UNIT, LED_CONTROLLER_TIMER_MATCH_NUMBER, &matchConfig);
    CTIMER_SetupMatch(LED_CONTROLLER_TIMER_UNIT, LED_CONTROLLER_TIMER_MATCH_LED_0, &matchConfig1);


    LED_CONTROLLER_TIMER_UNIT->PWMC = (1 << LED_CONTROLLER_TIMER_MATCH_LED_0) | (1 << 0);
    //INPUTMUX_AttachSignal(INPUTMUX, 0, 0 + (0x0E << 20));
    INPUTMUX->DMA_ITRIG_INMUX[LED_CONTROLLER_DMA_CHANNEL_LED_0] = 14; // match 0
    DMA_Init(DMA0);
    DMA_EnableChannel(DMA0, LED_CONTROLLER_DMA_CHANNEL_LED_0);

    DMA_CreateHandle(&hdma, DMA0, 0);
    DMA_SetCallback(&hdma, dmaCallback, NULL);

    DMA_SetChannelPriority(DMA0, LED_CONTROLLER_DMA_CHANNEL_LED_0, kDMA_ChannelPriority0);
    DMA_EnableChannelInterrupts(DMA0, LED_CONTROLLER_DMA_CHANNEL_LED_0);
    dma_channel_trigger_t dmaChannelTrigger;
    dmaChannelTrigger.burst = kDMA_EdgeBurstTransfer1;
    dmaChannelTrigger.type = kDMA_RisingEdgeTrigger;
    dmaChannelTrigger.wrap = kDMA_NoWrap;
    DMA_ConfigureChannelTrigger(DMA0, LED_CONTROLLER_DMA_CHANNEL_LED_0, &dmaChannelTrigger);

    NVIC_EnableIRQ(DMA0_IRQn);
}
