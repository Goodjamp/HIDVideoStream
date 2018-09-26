#include "fsl_sctimer.h"
#include "HalPwm.h"


#define PWM_MAX_VALUE   0xFF
#define PWM_FREQ        10000U
#define PWM_DEFAULT     50


typedef struct {
    uint32_t event;
    sctimer_out_t output;
} HalPwmChannelData;


static HalPwmChannelData halPwmChannel[] = {
    { .event = 0, .output = kSCTIMER_Out_0 },
    { .event = 0, .output = kSCTIMER_Out_1 },
    { .event = 0, .output = kSCTIMER_Out_2 },
    { .event = 0, .output = kSCTIMER_Out_3 },
    { .event = 0, .output = kSCTIMER_Out_4 },
    { .event = 0, .output = kSCTIMER_Out_5 },
    { .event = 0, .output = kSCTIMER_Out_6 },
    { .event = 0, .output = kSCTIMER_Out_7 },
    { .event = 0, .output = kSCTIMER_Out_7 + 1 },
};


bool halPwmInit(void)
{
    sctimer_config_t sctimerInfo;
    SCTIMER_GetDefaultConfig(&sctimerInfo);
    // Initialize SCTimer module
    SCTIMER_Init(SCT0, &sctimerInfo);
    // Set unify bit to operate in 32-bit counter mode
    SCT0->CONFIG |= SCT_CONFIG_UNIFY_MASK;
    uint32_t sctClock = CLOCK_GetFreq(kCLOCK_BusClk) / (((SCT0->CTRL & SCT_CTRL_PRE_L_MASK) >> SCT_CTRL_PRE_L_SHIFT) + 1);
    uint32_t period = (sctClock / PWM_FREQ) - 1;
    // Calculate pulse width match value
    uint32_t pulsePeriod = (period * PWM_DEFAULT) / PWM_MAX_VALUE;
    // For 100% dutycyle, make pulse period greater than period so the event will never occur
    if (PWM_DEFAULT == PWM_MAX_VALUE) {
        pulsePeriod = period + 2;
    }
    // Schedule an event when we reach the PWM period
    uint32_t periodEvent = 0;
    SCTIMER_CreateAndScheduleEvent(SCT0, kSCTIMER_MatchEventOnly, period, 0, kSCTIMER_Counter_L, &periodEvent);
    // Reset the counter when we reach the PWM period
    SCTIMER_SetupCounterLimitAction(SCT0, kSCTIMER_Counter_L, periodEvent);

    for (int i = 0; i < sizeof(halPwmChannel) / sizeof(halPwmChannel[0]); i++) {
        // Schedule an event when we reach the pulse width
        SCTIMER_CreateAndScheduleEvent(SCT0, kSCTIMER_MatchEventOnly, pulsePeriod, 0, kSCTIMER_Counter_L, &halPwmChannel[i].event);
        // Set the initial output level to low which is the inactive state
        SCT0->OUTPUT &= ~(1U << halPwmChannel[i].output);
        // Set the output when we reach the PWM period
        SCTIMER_SetupOutputSetAction(SCT0, halPwmChannel[i].output, periodEvent);
        // Clear the output when we reach the PWM pulse value
        SCTIMER_SetupOutputClearAction(SCT0, halPwmChannel[i].output, halPwmChannel[i].event);
    }

    SCTIMER_StartTimer(SCT0, kSCTIMER_Counter_L);
    //halPwmSetDutyCycle(0, 25);
    return true;
}

bool halPwmSetDutyCycle(HalPwmChannel channel, uint8_t dutyCycle)
{
    // Retrieve the match register number for the PWM period
    uint32_t periodMatchReg = SCT0->EVENT[0].CTRL & SCT_EVENT_CTRL_MATCHSEL_MASK;
    // Retrieve the match register number for the PWM pulse period
    uint32_t pulseMatchReg = SCT0->EVENT[halPwmChannel[channel].event].CTRL & SCT_EVENT_CTRL_MATCHSEL_MASK;
    uint32_t period = SCT0->SCTMATCH[periodMatchReg];
    // Calculate pulse width match value
    uint32_t pulsePeriod = (period * dutyCycle) / PWM_MAX_VALUE;

    if (dutyCycle == PWM_MAX_VALUE) {
        pulsePeriod = period + 2;
    }
    // Stop the counter before updating match register
    SCTIMER_StopTimer(SCT0, kSCTIMER_Counter_L);
    // Update dutycycle
    SCT0->SCTMATCH[pulseMatchReg] = SCT_SCTMATCH_MATCHn_L(pulsePeriod);
    SCT0->SCTMATCHREL[pulseMatchReg] = SCT_SCTMATCHREL_RELOADn_L(pulsePeriod);
    // Restart the counter
    SCTIMER_StartTimer(SCT0, kSCTIMER_Counter_L);

    return true;
}

/*
typedef struct
{
    CTIMER_Type *timerBase;
    ctimer_match_t matchRegister;
    iocon_group_t fanVoltagePwm;
    iocon_group_t fanSpeedPwm;
} FanChannel;

FanChannel fanChannel[] = {
    {
        .timerBase      = CTIMER0,
        .matchRegister  = kCTIMER_Match_0,
        .fanVoltagePwm  = { .port = 0, .pin = 0, .modefunc = (IOCON_FUNC3 | IOCON_DIGITAL_EN | IOCON_INPFILT_OFF) }
    },
    {
        .timerBase      = CTIMER0,
        .matchRegister  = kCTIMER_Match_1,
        .fanVoltagePwm  = { .port = 0, .pin = 31, .modefunc = (IOCON_FUNC3 | IOCON_DIGITAL_EN | IOCON_INPFILT_OFF) }
    },
    {
        .timerBase      = CTIMER1,
        .matchRegister  = kCTIMER_Match_0,
        .fanVoltagePwm  = { .port = 2, .pin = 1, .modefunc = (IOCON_FUNC4 | IOCON_DIGITAL_EN | IOCON_INPFILT_OFF) }
    },
    {
        .timerBase      = CTIMER1,
        .matchRegister  = kCTIMER_Match_1,
        .fanVoltagePwm  = { .port = 2, .pin = 2, .modefunc = (IOCON_FUNC4 | IOCON_DIGITAL_EN | IOCON_INPFILT_OFF) }
    },
    {
        .timerBase      = CTIMER1,
        .matchRegister  = kCTIMER_Match_2,
        .fanVoltagePwm  = { .port = 3, .pin = 2, .modefunc = (IOCON_FUNC4 | IOCON_DIGITAL_EN | IOCON_INPFILT_OFF) }
    },
    {
        .timerBase      = CTIMER3,
        .matchRegister  = kCTIMER_Match_0,
        .fanVoltagePwm  = { .port = 2, .pin = 18, .modefunc = (IOCON_FUNC4 | IOCON_DIGITAL_EN | IOCON_INPFILT_OFF) }
    },
    {
        .timerBase      = CTIMER3,
        .matchRegister  = kCTIMER_Match_1,
        .fanVoltagePwm  = { .port = 3, .pin = 14, .modefunc = (IOCON_FUNC3 | IOCON_DIGITAL_EN | IOCON_INPFILT_OFF) }
    },
    {
        .timerBase      = CTIMER3,
        .matchRegister  = kCTIMER_Match_2,
        .fanVoltagePwm  = { .port = 2, .pin = 20, .modefunc = (IOCON_FUNC4 | IOCON_DIGITAL_EN | IOCON_INPFILT_OFF) }
    },
    {
        .timerBase      = CTIMER2,
        .matchRegister  = kCTIMER_Match_3,
        .fanVoltagePwm  = { .port = 1, .pin = 22, .modefunc = (IOCON_FUNC3 | IOCON_DIGITAL_EN | IOCON_INPFILT_OFF) }
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
            .matchValue         = 7200UL, // 180000000/25000
            .outControl         = kCTIMER_Output_NoAction,
            .outPinInitState    = false,
            .enableInterrupt    = false,
        };

        CTIMER_SetupMatch(channel[i].timerBase, kCTIMER_Match_0, &matchConfig);
    }

    for (uint32_t i = 0; i < count; i++) {
        IOCON_PinMuxSet(IOCON, channel[i].fanVoltagePwm.port, channel[i].fanVoltagePwm.pin, channel[i].fanVoltagePwm.modefunc);
        ctimer_match_config_t matchConfig = {
            .enableCounterReset = false,
            .enableCounterStop  = false,
            .matchValue         = 3600UL,
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
    uint32_t static prevTime = 0;
    volatile uint32_t capture = SCT0->SCTMATCH[0] - prevTime;
    prevTime = SCT0->SCTMATCH[0];
    (void) capture;
}

void fanControllerTachometerInit()
{
    CLOCK_EnableClock(kCLOCK_Gpio0);
    CLOCK_EnableClock(kCLOCK_Iocon);


    INPUTMUX_AttachSignal(INPUTMUX, SCT0_PMUX_ID, kINPUTMUX_SctGpi0ToSct0);

    IOCON_PinMuxSet(IOCON, 0, 24, (IOCON_FUNC4 | IOCON_DIGITAL_EN | IOCON_MODE_PULLUP));
    gpio_pin_config_t config = { kGPIO_DigitalInput, 0 };

    GPIO_PinInit(GPIO, 0, 24, &config);

    sctimer_config_t sctimerInfo;
    SCTIMER_GetDefaultConfig(&sctimerInfo);
    SCTIMER_Init(SCT0, &sctimerInfo);

    uint32_t captureRegister = 0;
    SCTIMER_SetupCaptureAction(SCT0, kSCTIMER_Counter_L, &captureRegister, 0); // event 0

    uint32_t eventNumber = 0;
    SCTIMER_CreateAndScheduleEvent(SCT0, kSCTIMER_InputFallEvent, 0, 0, kSCTIMER_Counter_L, &eventNumber);
    SCTIMER_EnableInterrupts(SCT0, kSCTIMER_Event0InterruptEnable);
    SCTIMER_SetCallback(SCT0, sctimerCb, 0);
    SCTIMER_StartTimer(SCT0, kSCTIMER_Counter_L);
    NVIC_EnableIRQ(SCT0_IRQn);
}


*/

/*
    //fanControllerPwmInit(fanChannel, sizeof(fanChannel) / sizeof(fanChannel[0]));
    //fanControllerTachometerInit();
*/
