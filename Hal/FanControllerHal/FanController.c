#include <stddef.h>

#include "FanController.h"

#include "fsl_ctimer.h"
#include "fsl_iocon.h"
#include "fsl_inputmux.h"
#include "fsl_gpio.h"
#include "fsl_sctimer.h"

#define FAN_PWM_CONTROL_TIMER_FREQUECY          25000
#define TACHOMETER_TIMER_COUNTER_MAX            (0xFFFFFFFFu)


typedef struct TachometerInputState {
    uint32_t firstEdgeTimestamp;
    uint32_t secondEdgeTimestamp;
    bool isSecondEdge;
    bool isMeasurementDone;
} TachometerInputState;

static FanControllerSpeedMeasuredCb fanSpeedMeasuredCb = NULL;
static uint32_t tachometerTicksPerHalfRpm;
static uint32_t fanControllerMaxPwmValue;

static TachometerInputState tachometerStateArray[FAN_COUNT];

typedef struct FanChannel {
    CTIMER_Type *timerBase;
    ctimer_match_t matchRegister;
    iocon_group_t fanVoltagePwm;
    iocon_group_t fanSpeedPwm;
} FanChannel;

FanChannel fanChannel[] = {
    {
        .timerBase      = CTIMER0,
        .matchRegister  = kCTIMER_Match_0,
        .fanVoltagePwm  = { .port = 2, .pin = 8, .modefunc = (IOCON_FUNC4 | IOCON_DIGITAL_EN | IOCON_INPFILT_OFF) }
    },
    {
        .timerBase      = CTIMER0,
        .matchRegister  = kCTIMER_Match_1,
        .fanVoltagePwm  = { .port = 2, .pin = 9, .modefunc = (IOCON_FUNC4 | IOCON_DIGITAL_EN | IOCON_INPFILT_OFF) }
    },
    {
        .timerBase      = CTIMER0,
        .matchRegister  = kCTIMER_Match_2,
        .fanVoltagePwm  = { .port = 2, .pin = 14, .modefunc = (IOCON_FUNC4 | IOCON_DIGITAL_EN | IOCON_INPFILT_OFF) }
    },
    {
        .timerBase      = CTIMER1,
        .matchRegister  = kCTIMER_Match_0,
        .fanVoltagePwm  = { .port = 4, .pin = 23, .modefunc = (IOCON_FUNC5 | IOCON_DIGITAL_EN | IOCON_INPFILT_OFF) }
    },
    {
        .timerBase      = CTIMER1,
        .matchRegister  = kCTIMER_Match_1,
        .fanVoltagePwm  = { .port = 4, .pin = 24, .modefunc = (IOCON_FUNC5 | IOCON_DIGITAL_EN | IOCON_INPFILT_OFF) }
    },
    {
        .timerBase      = CTIMER1,
        .matchRegister  = kCTIMER_Match_2,
        .fanVoltagePwm  = { .port = 4, .pin = 25, .modefunc = (IOCON_FUNC5 | IOCON_DIGITAL_EN | IOCON_INPFILT_OFF) }
    },
    {
        .timerBase      = CTIMER3,
        .matchRegister  = kCTIMER_Match_0,
        .fanVoltagePwm  = { .port = 1, .pin = 5, .modefunc = (IOCON_FUNC3 | IOCON_DIGITAL_EN | IOCON_INPFILT_OFF) }
    },
};

void fanControllerPwmInit(FanChannel channel[], uint32_t count)
{
    CLOCK_EnableClock(kCLOCK_Iocon);

    for (uint32_t i = 0; i < count; i++) {
        ctimer_config_t config;

        CTIMER_GetDefaultConfig(&config);
        CTIMER_Init(channel[i].timerBase, &config);
        // setup PWM frequency to 25kHz
        ctimer_match_config_t matchConfig = {
            .enableCounterReset = true,
            .enableCounterStop  = false,
            .matchValue         = 96000000UL / 25000, // 180000000/25000
            .outControl         = kCTIMER_Output_NoAction,
            .outPinInitState    = false,
            .enableInterrupt    = false,
        };

        CTIMER_SetupMatch(channel[i].timerBase, kCTIMER_Match_3, &matchConfig);
    }

    for (uint32_t i = 0; i < count; i++) {
        IOCON_PinMuxSet(IOCON, channel[i].fanVoltagePwm.port, channel[i].fanVoltagePwm.pin, channel[i].fanVoltagePwm.modefunc);
        ctimer_match_config_t matchConfig = {
            .enableCounterReset = false,
            .enableCounterStop  = false,
            .matchValue         = 1920,
            .outControl         = kCTIMER_Output_NoAction,
            .outPinInitState    = true,
            .enableInterrupt    = false,
        };

        CTIMER_SetupMatch(channel[i].timerBase, channel[i].matchRegister, &matchConfig);
        // enable PWM on selected match channel
        channel[i].timerBase->PWMC |= (1 << channel[i].matchRegister);
    }

    for (uint32_t i = 0; i < count; i++) {
        CTIMER_StartTimer(channel[i].timerBase);
    }
}

void sctimerCb(void)
{
}
void SCT0_IRQHandler(void)
{
    unsigned int register channelCounter = 0;
    for (channelCounter = 0; channelCounter < FAN_COUNT; channelCounter++) {
        if (SCT0->EVFLAG & (1 << channelCounter)) {
            uint32_t sctCaptureValue = SCT0->SCTCAP[channelCounter];
            SCT0->EVFLAG = (1 << channelCounter);
            TachometerInputState *tachometerState = &tachometerStateArray[channelCounter];
            if (tachometerState->isSecondEdge) {
                SCT0->EVEN &= ~(1 << channelCounter);
                tachometerState->secondEdgeTimestamp = sctCaptureValue;
                tachometerState->isSecondEdge = false;
                // if (T1 > T2) DT = MAX_COUNTER + T2 - T1
                // else DT = T2 - T1
                register uint32_t rpm = 0;
                if (tachometerState->secondEdgeTimestamp < tachometerState->firstEdgeTimestamp) {
                    rpm = TACHOMETER_TIMER_COUNTER_MAX - tachometerState->firstEdgeTimestamp + tachometerState->secondEdgeTimestamp + 1;
                } else {
                    rpm = tachometerState->secondEdgeTimestamp - tachometerState->firstEdgeTimestamp;
                }

                if (fanSpeedMeasuredCb) {
                    // Avoid division by zero
                    rpm = tachometerTicksPerHalfRpm / (rpm ? rpm : 1);
                    fanSpeedMeasuredCb(channelCounter, rpm);
                }
                tachometerState->isMeasurementDone = true;
            } else {
                tachometerState->firstEdgeTimestamp = sctCaptureValue;
                tachometerState->isSecondEdge = true;
            }
        }
    }
}

void fanControllerTachometerInit()
{
    CLOCK_EnableClock(kCLOCK_Gpio0);
    CLOCK_EnableClock(kCLOCK_Iocon);

    INPUTMUX->SCT0_INMUX[0] = 0;
    INPUTMUX->SCT0_INMUX[1] = 1;
    INPUTMUX->SCT0_INMUX[2] = 2;
    INPUTMUX->SCT0_INMUX[3] = 3;
    INPUTMUX->SCT0_INMUX[4] = 4;
    INPUTMUX->SCT0_INMUX[5] = 5;
    INPUTMUX->SCT0_INMUX[6] = 6;

    gpio_pin_config_t config = { kGPIO_DigitalInput, 0 };

    sctimer_config_t sctimerInfo;
    SCTIMER_GetDefaultConfig(&sctimerInfo);
     sctimerInfo.enableCounterUnify = true;
    SCTIMER_Init(SCT0, &sctimerInfo);
    #define SCT_CONFIG_IN_SYNC_OFS  9
    SCT0->CONFIG |= 0b1111111 << SCT_CONFIG_IN_SYNC_OFS;
    SCTIMER_SetCallback(SCT0, sctimerCb, 0);
    SCTIMER_SetCallback(SCT0, sctimerCb, 1);
    SCTIMER_SetCallback(SCT0, sctimerCb, 2);
    SCTIMER_SetCallback(SCT0, sctimerCb, 3);
    SCTIMER_SetCallback(SCT0, sctimerCb, 4);
    SCTIMER_SetCallback(SCT0, sctimerCb, 5);
    SCTIMER_SetCallback(SCT0, sctimerCb, 6);

    IOCON_PinMuxSet(IOCON, 4, 7, (IOCON_FUNC5 | IOCON_DIGITAL_EN | IOCON_MODE_PULLUP));
    IOCON_PinMuxSet(IOCON, 4, 8, (IOCON_FUNC5 | IOCON_DIGITAL_EN | IOCON_MODE_PULLUP));
    IOCON_PinMuxSet(IOCON, 4, 9, (IOCON_FUNC5 | IOCON_DIGITAL_EN | IOCON_MODE_PULLUP));
    IOCON_PinMuxSet(IOCON, 4, 10, (IOCON_FUNC5 | IOCON_DIGITAL_EN | IOCON_MODE_PULLUP));
    IOCON_PinMuxSet(IOCON, 4, 11, (IOCON_FUNC5 | IOCON_DIGITAL_EN | IOCON_MODE_PULLUP));
    IOCON_PinMuxSet(IOCON, 4, 12, (IOCON_FUNC5 | IOCON_DIGITAL_EN | IOCON_MODE_PULLUP));
    IOCON_PinMuxSet(IOCON, 4, 13, (IOCON_FUNC5 | IOCON_DIGITAL_EN | IOCON_MODE_PULLUP));

    GPIO_PinInit(GPIO, 4, 7, &config);
    GPIO_PinInit(GPIO, 4, 8, &config);
    GPIO_PinInit(GPIO, 4, 9, &config);
    GPIO_PinInit(GPIO, 4, 10, &config);
    GPIO_PinInit(GPIO, 4, 11, &config);
    GPIO_PinInit(GPIO, 4, 12, &config);
    GPIO_PinInit(GPIO, 4, 13, &config);

    uint32_t captureRegister = 0;
    uint32_t eventNumber = 0;

    SCTIMER_SetupCaptureAction(SCT0, kSCTIMER_Counter_L, &captureRegister, 0); // event 0
    SCTIMER_CreateAndScheduleEvent(SCT0, kSCTIMER_InputFallEvent, 0, 0, kSCTIMER_Counter_L, &eventNumber);

    SCTIMER_SetupCaptureAction(SCT0, kSCTIMER_Counter_L, &captureRegister, 1); // event 0
    SCTIMER_CreateAndScheduleEvent(SCT0, kSCTIMER_InputFallEvent, 0, 1, kSCTIMER_Counter_L, &eventNumber);

    SCTIMER_SetupCaptureAction(SCT0, kSCTIMER_Counter_L, &captureRegister, 2); // event 0
    SCTIMER_CreateAndScheduleEvent(SCT0, kSCTIMER_InputFallEvent, 0, 2, kSCTIMER_Counter_L, &eventNumber);

    SCTIMER_SetupCaptureAction(SCT0, kSCTIMER_Counter_L, &captureRegister, 3); // event 0
    SCTIMER_CreateAndScheduleEvent(SCT0, kSCTIMER_InputFallEvent, 0, 3, kSCTIMER_Counter_L, &eventNumber);

    SCTIMER_SetupCaptureAction(SCT0, kSCTIMER_Counter_L, &captureRegister, 4); // event 0
    SCTIMER_CreateAndScheduleEvent(SCT0, kSCTIMER_InputFallEvent, 0, 4, kSCTIMER_Counter_L, &eventNumber);

    SCTIMER_SetupCaptureAction(SCT0, kSCTIMER_Counter_L, &captureRegister, 5); // event 0
    SCTIMER_CreateAndScheduleEvent(SCT0, kSCTIMER_InputFallEvent, 0, 5, kSCTIMER_Counter_L, &eventNumber);

    SCTIMER_SetupCaptureAction(SCT0, kSCTIMER_Counter_L, &captureRegister, 6); // event 0
    SCTIMER_CreateAndScheduleEvent(SCT0, kSCTIMER_InputFallEvent, 0, 6, kSCTIMER_Counter_L, &eventNumber);

    SCTIMER_StartTimer(SCT0, kSCTIMER_Counter_L);

    NVIC_EnableIRQ(SCT0_IRQn);
}

static void tachometerInputTimersInit()
{
    fanControllerTachometerInit();
    uint32_t blockCounter = 0;
    for (blockCounter = 0; blockCounter < FAN_COUNT; blockCounter++) {
        tachometerStateArray[blockCounter].isSecondEdge = false;
        tachometerStateArray[blockCounter].isMeasurementDone = true;
        tachometerStateArray[blockCounter].firstEdgeTimestamp = tachometerStateArray[blockCounter].secondEdgeTimestamp = 0;
    }

    #define SECOND_PER_MINUTE   60u
    tachometerTicksPerHalfRpm = 96000000 * (SECOND_PER_MINUTE >> 1);
}

static void pwmContorlTimersInit(uint32_t pwmFrequency)
{
    const uint32_t timerClockSpeed = 96000000;

    fanControllerMaxPwmValue = timerClockSpeed / pwmFrequency;

    fanControllerPwmInit(fanChannel, sizeof(fanChannel) / sizeof(fanChannel[0]));
}

void fanControllerInit(FanControllerSpeedMeasuredCb speedMeasuredCb)
{
    fanSpeedMeasuredCb = speedMeasuredCb;
    #if defined(USE_PERIODIC_TIMER) && (USE_PERIODIC_TIMER == 1)
    periodicSamplingTimerInit(PERIODIC_SAMPLING_TIMER_FREQUECY);
    #endif
    pwmContorlTimersInit(FAN_PWM_CONTROL_TIMER_FREQUECY);
    tachometerInputTimersInit();
}

void fanControllerSetPwm(uint32_t fanNumber, uint32_t fanPwm)
{
    if (fanNumber >= sizeof(fanChannel) / sizeof(fanChannel[0])) {
        return;
    }

    fanChannel[fanNumber].timerBase->MR[fanChannel[fanNumber].matchRegister] = fanControllerMaxPwmValue - fanPwm;
}

void fanControllerStartMeasurement(void)
{
    size_t channelCounter = 0;
    for (channelCounter = 0; channelCounter < FAN_COUNT; channelCounter++) {
        TachometerInputState *tachometerState = &tachometerStateArray[channelCounter];
        if (!(SCT0->EVEN & (1 << channelCounter)) && tachometerState->isMeasurementDone) {
            tachometerState->isMeasurementDone = false;
            tachometerState->isSecondEdge = false;
            SCT0->EVFLAG = (1 << channelCounter);
            SCT0->EVEN |= (1 << channelCounter);
        }
    }
}

uint32_t fanControllerGetMaxPwmValue(void)
{
    return fanControllerMaxPwmValue;
}

uint32_t fanControllerGetPwm(uint32_t fanNumber)
{
    if (fanNumber >= sizeof(fanChannel) / sizeof(fanChannel[0])) {
        return 0;
    }

    return fanChannel[fanNumber].matchRegister;
}

uint32_t fanControllerGetTachCount(void)
{
    return FAN_COUNT;
}

uint32_t fanControllerGetPwmCount(void)
{
    return FAN_COUNT;
}

void fanControllerReInitPwm(FanControllerChannelConfig fanControllerChannelConfig[], uint32_t channelCount)
{
    if (!fanControllerChannelConfig)
        return;

    uint32_t channelCounter = 0;
    for (channelCounter = 0; channelCounter < channelCount; channelCounter++) {
        switch (fanControllerChannelConfig[channelCounter].channelState) {
        case FAN_CONTROLLER_CHANNEL_3P :
            break;

        case FAN_CONTROLLER_CHANNEL_4P :
            break;

        case FAN_CONTROLLER_CHANNEL_NONE :
        default :
            break;

        }
    }
}
