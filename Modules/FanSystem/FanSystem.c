#include "FanSystem.h"
#include "PidController.h"

#include <stddef.h>

#define FAN_PID_KP                          (int32_t) 1
#define FAN_PID_KI                          (int32_t) 20
#define FAN_PID_KD                          (int32_t) 0
#define FAN_PID_P_SATURATION                (int32_t) 1920L * 2L
#define FAN_PID_N_SATURATION                (int32_t) -1L
#define FAN_PID_DAMPING                     (3)

#define FAN_MEASUREMENT_VALID_TIME_MS       250
#define FAN_MEASUREMENT_WINDOW_SIZE         16
#define FAN_MEASUREMENT_WINDOW_SIZEMASK     0x0000000F
#define FAN_MEASUREMENT_WINDOW_SHIFT_MULT   4

#define TIMESTAMP_MAX                       0xFFFFFFFF

typedef struct FanSpeedMeasurement {
    uint32_t lastRpm;
    uint32_t rpm;
    uint32_t timeStamp;
    bool wasRead;
    bool isValid;

    int32_t dataWindow[FAN_MEASUREMENT_WINDOW_SIZE];
    uint32_t dataWindowHead;
} FanSpeedMeasurement;

#define MAX_FAN_COUNT   FAN_COUNT

typedef struct FanParametersRegistry {
    FanSystemMode fanSystemMode[MAX_FAN_COUNT];
    FanSpeedTempCurve fanSpeedTempCurve[MAX_FAN_COUNT];
    uint32_t temperatureIndex[MAX_FAN_COUNT];
    uint32_t fanRpmTargetValue[MAX_FAN_COUNT];
    uint32_t fanPwmTargetValue[MAX_FAN_COUNT];
} FanParametersRegistry;

FanControllerChannelConfig fanChannelConfigRegistry[MAX_FAN_COUNT];

static FanSystemGetTemperature fanGetTemperatureCb = NULL;
static FanSystemMeasurementEvent fanSystemMeasurementEventCb = NULL;
static FanSystemGetTimestamp fanSystemGetTimestampCb = NULL;

static FanSpeedMeasurement fanSpeedMeasurements[MAX_FAN_COUNT];
static uint32_t fanRpmActualValue[MAX_FAN_COUNT];

static FanParametersRegistry fanSystemParameters;
static PidController pidControllers[MAX_FAN_COUNT];
static uint32_t maxPwmValue;

static bool isFanDetectionTaskRunning = false;

uint32_t sqrtI(uint32_t value)
{
    uint32_t reminder = 0, root = 0, bitPairCntr = 16;

    while (bitPairCntr-- > 0) {
        root <<= 1;
        reminder = (reminder << 2) + (value >> 30);
        value <<= 2;

        if (root < reminder)
        {
            reminder -= ++root;
            root++;
        }
    }

    return (root >> 1);
}

void fanSystemOnMeasurementDone(uint32_t fanNumber, uint32_t rpm)
{
    if (fanSystemGetTimestampCb)
        fanSpeedMeasurements[fanNumber].timeStamp = fanSystemGetTimestampCb();
    fanSpeedMeasurements[fanNumber].lastRpm = rpm;
    fanSpeedMeasurements[fanNumber].wasRead = false;

    if (fanSystemMeasurementEventCb)
        fanSystemMeasurementEventCb();
}

void fanSystemInit(FanSystemGetTemperature fanGetTemperatureCallback, FanSystemGetTimestamp fanSystemGetTimestampCallback, FanSystemMeasurementEvent fanSystemMeasurementEventCallback)
{
    size_t fanCounter;

    fanSystemGetTimestampCb = fanSystemGetTimestampCallback;
    fanGetTemperatureCb = fanGetTemperatureCallback;
    fanSystemMeasurementEventCb = fanSystemMeasurementEventCallback;
    fanControllerInit(fanSystemOnMeasurementDone);
    maxPwmValue = fanControllerGetMaxPwmValue();
    for (fanCounter = 0; fanCounter < MAX_FAN_COUNT; fanCounter++) {
        pidControllerInit(&pidControllers[fanCounter], FAN_PID_KP, FAN_PID_KI, FAN_PID_KD, FAN_PID_P_SATURATION, FAN_PID_N_SATURATION, FAN_PID_DAMPING);
    }
}

static void fanSystemSetRpm(uint32_t fanNumber, int32_t error)
{
    int32_t pidOutput = pidControllerUpdate(&pidControllers[fanNumber], error);

    if (pidOutput < 0) {
        pidOutput = 0;
    }

    if (pidOutput > maxPwmValue) {
        pidOutput = maxPwmValue;
    }

    fanControllerSetPwm(fanNumber, pidOutput);
}

static void fanSystemUpdateCurrentRpm(uint32_t fanNumber)
{
    uint32_t lastMeasurementTime = 0;

    uint32_t i, rpm = fanSpeedMeasurements[fanNumber].lastRpm;
    uint32_t head = fanSpeedMeasurements[fanNumber].dataWindowHead;

    int32_t mu = 0;
    for (i = 0; i < FAN_MEASUREMENT_WINDOW_SIZE; i++) {
        mu += fanSpeedMeasurements[fanNumber].dataWindow[i];
    }
    mu >>= FAN_MEASUREMENT_WINDOW_SHIFT_MULT;

    int32_t sd = 0;
    for (i = 0; i < FAN_MEASUREMENT_WINDOW_SIZE; i++) {
        sd += (mu - fanSpeedMeasurements[fanNumber].dataWindow[i]) * (mu - fanSpeedMeasurements[fanNumber].dataWindow[i]);
    }
    sd = sqrtI(sd >> FAN_MEASUREMENT_WINDOW_SHIFT_MULT);

    fanSpeedMeasurements[fanNumber].dataWindow[head] = rpm;
    fanSpeedMeasurements[fanNumber].dataWindowHead = (head + 1) & FAN_MEASUREMENT_WINDOW_SIZEMASK;
    fanSpeedMeasurements[fanNumber].rpm = rpm;

    if ((rpm < (mu + ((sd << 1) + sd))) && (rpm > (mu - ((sd << 1) + sd)))) {
        fanSpeedMeasurements[fanNumber].isValid = true;
    } else {
        fanSpeedMeasurements[fanNumber].isValid = false;
        //fanSpeedMeasurements[fanNumber].dataWindow[head] = mu;
    }

    if (fanSystemGetTimestampCb) {
        lastMeasurementTime = fanSystemGetTimestampCb();
    }

    if (lastMeasurementTime < fanSpeedMeasurements[fanNumber].timeStamp) {
        lastMeasurementTime = TIMESTAMP_MAX - fanSpeedMeasurements[fanNumber].timeStamp + lastMeasurementTime + 1;
    } else {
        lastMeasurementTime = lastMeasurementTime - fanSpeedMeasurements[fanNumber].timeStamp;
    }

    if (lastMeasurementTime > FAN_MEASUREMENT_VALID_TIME_MS) {
        fanSpeedMeasurements[fanNumber].rpm = 0;
    }

    fanRpmActualValue[fanNumber] = fanSpeedMeasurements[fanNumber].rpm;

    fanSpeedMeasurements[fanNumber].wasRead = true;
}

static int32_t fanSystemGetRpmErrorByTemperature(uint32_t fanNumber)
{
    uint32_t temperature;
    size_t pointCounter;

    if (fanGetTemperatureCb) {
        temperature = fanGetTemperatureCb(fanSystemParameters.temperatureIndex[fanNumber], fanNumber);
    }

    if (!fanSystemParameters.fanSpeedTempCurve[fanNumber].curvePointCount) {
        return 0;
    }

    if (temperature <= fanSystemParameters.fanSpeedTempCurve[fanNumber].curvePoints[0].temperature) {
        return ((int32_t) fanSystemParameters.fanSpeedTempCurve[fanNumber].curvePoints[0].rpm) - ((int32_t) fanRpmActualValue[fanNumber]);
    }

    if (temperature >= fanSystemParameters.fanSpeedTempCurve[fanNumber].curvePoints[fanSystemParameters.fanSpeedTempCurve[fanNumber].curvePointCount - 1].temperature) {
        return ((int32_t) fanSystemParameters.fanSpeedTempCurve[fanNumber].curvePoints[fanSystemParameters.fanSpeedTempCurve[fanNumber].curvePointCount - 1].rpm) - ((int32_t) fanRpmActualValue[fanNumber]);
    }

    for (pointCounter = 0; pointCounter < fanSystemParameters.fanSpeedTempCurve[fanNumber].curvePointCount - 1; pointCounter++) {
        if ((temperature >= fanSystemParameters.fanSpeedTempCurve[fanNumber].curvePoints[pointCounter].temperature) &&
                (temperature <= fanSystemParameters.fanSpeedTempCurve[fanNumber].curvePoints[pointCounter + 1].temperature)) {
            break;
        }
    }

    FanSpeedTempCurvePoint *leftPoint = &fanSystemParameters.fanSpeedTempCurve[fanNumber].curvePoints[pointCounter];
    FanSpeedTempCurvePoint *rightPoint = &fanSystemParameters.fanSpeedTempCurve[fanNumber].curvePoints[pointCounter + 1];
    int32_t rpmValue = leftPoint->rpm * (rightPoint->temperature - temperature) / (rightPoint->temperature - leftPoint->temperature) +
                       rightPoint->rpm * (temperature - leftPoint->temperature) / (rightPoint->temperature - leftPoint->temperature);
    return rpmValue -= ((int32_t) fanRpmActualValue[fanNumber]);
}

typedef enum FanDetectionState {
    FAN_DETECTION_STATE_FULL_POWER,
    FAN_DETECTION_STATE_WAIT_FULL_POWER,
    FAN_DETECTION_STATE_GET_FULL_POWER_RPM,
    FAN_DETECTION_STATE_50_PERCENT_POWER,
    FAN_DETECTION_STATE_WAIT_50_PERCENT_POWER,
    FAN_DETECTION_STATE_GET_50_PERCENT_POWER_RPM,
    FAN_DETECTION_STATE_READY
} FanDetectionState;

static FanDetectionState fanDetectionState;

static void fanSystemFanDetectionTask(void)
{
    #define FAN_DETECTION_DELAY_COUNT                   200
    #define FAN_DETECTION_SPEED_POSITIVE_THRESHOLD      200
    #define FAN_DETECTION_SPEED_NEGATIVE_THRESHOLD      -100
    static uint32_t delayCounter = 0;
    static int32_t fanSpeedMeausrements[MAX_FAN_COUNT];
    static int32_t fanSpeedDelta[MAX_FAN_COUNT];
    uint32_t fanCounter = 0;
    switch (fanDetectionState) {
        case FAN_DETECTION_STATE_FULL_POWER :
            for (fanCounter = 0; fanCounter < MAX_FAN_COUNT; fanCounter++) {
                fanControllerSetPwm(fanCounter, maxPwmValue);
            }
            delayCounter = 0;
            fanDetectionState = FAN_DETECTION_STATE_WAIT_FULL_POWER;

            break;

        case FAN_DETECTION_STATE_WAIT_FULL_POWER :
        case FAN_DETECTION_STATE_WAIT_50_PERCENT_POWER :
            if (delayCounter++ >= FAN_DETECTION_DELAY_COUNT) {
                delayCounter = 0;
                if (fanDetectionState == FAN_DETECTION_STATE_WAIT_FULL_POWER) {
                    fanDetectionState = FAN_DETECTION_STATE_GET_FULL_POWER_RPM;
                } else {
                    fanDetectionState = FAN_DETECTION_STATE_GET_50_PERCENT_POWER_RPM;
                }
            }

            break;

        case FAN_DETECTION_STATE_GET_FULL_POWER_RPM :
            for (fanCounter = 0; fanCounter < MAX_FAN_COUNT; fanCounter++) {
                fanSpeedMeausrements[fanCounter] = fanRpmActualValue[fanCounter];
            }
            fanDetectionState = FAN_DETECTION_STATE_50_PERCENT_POWER;
            break;

        case FAN_DETECTION_STATE_50_PERCENT_POWER :
            for (fanCounter = 0; fanCounter < MAX_FAN_COUNT; fanCounter++) {
                fanControllerSetPwm(fanCounter, maxPwmValue >> 1);
            }
            delayCounter = 0;
            fanDetectionState = FAN_DETECTION_STATE_WAIT_50_PERCENT_POWER;

            break;

        case FAN_DETECTION_STATE_GET_50_PERCENT_POWER_RPM :
            for (fanCounter = 0; fanCounter < MAX_FAN_COUNT; fanCounter++) {
                fanSpeedDelta[fanCounter] = fanSpeedMeausrements[fanCounter] - fanRpmActualValue[fanCounter];
            }
            fanDetectionState = FAN_DETECTION_STATE_READY;
            break;

        case FAN_DETECTION_STATE_READY :
            for (fanCounter = 0; fanCounter < MAX_FAN_COUNT; fanCounter++) {
                if (fanSpeedDelta[fanCounter] < FAN_DETECTION_SPEED_NEGATIVE_THRESHOLD) {
                    fanChannelConfigRegistry[fanCounter].channelNumber = fanCounter;
                    fanChannelConfigRegistry[fanCounter].channelState = FAN_CONTROLLER_CHANNEL_DETECTION_ERROR;
                } else if (fanSpeedDelta[fanCounter] > FAN_DETECTION_SPEED_POSITIVE_THRESHOLD) {
                    fanChannelConfigRegistry[fanCounter].channelState = FAN_CONTROLLER_CHANNEL_4P;
                } else if (fanSpeedMeausrements[fanCounter] != 0) {
                    fanChannelConfigRegistry[fanCounter].channelState = FAN_CONTROLLER_CHANNEL_3P;
                } else {
                    fanChannelConfigRegistry[fanCounter].channelState = FAN_CONTROLLER_CHANNEL_NONE;
                }
            }

            fanControllerReInitPwm(fanChannelConfigRegistry, MAX_FAN_COUNT);
            isFanDetectionTaskRunning = false;
            break;

        default :
            fanDetectionState = FAN_DETECTION_STATE_FULL_POWER;
            break;
    }
}

void fanSystemTaskRun(void)
{
    size_t fanCounter;

    if (isFanDetectionTaskRunning) {
        for (fanCounter = 0; fanCounter < MAX_FAN_COUNT; fanCounter++)
            fanSystemUpdateCurrentRpm(fanCounter);

        fanSystemFanDetectionTask();
    } else {
        for (fanCounter = 0; fanCounter < MAX_FAN_COUNT; fanCounter++) {
            fanSystemUpdateCurrentRpm(fanCounter);

            switch (fanSystemParameters.fanSystemMode[fanCounter]) {
            case FAN_SYSTEM_MODE_PWM :
                fanControllerSetPwm(fanCounter, fanSystemParameters.fanPwmTargetValue[fanCounter]);
                break;

            case FAN_SYSTEM_MODE_RPM :
                if (fanSpeedMeasurements[fanCounter].isValid)
                    fanSystemSetRpm(fanCounter, ((int32_t) fanSystemParameters.fanRpmTargetValue[fanCounter]) - ((int32_t) fanRpmActualValue[fanCounter]));
                break;

            case FAN_SYSTEM_MODE_CURVE :
                if (fanSpeedMeasurements[fanCounter].isValid)
                    fanSystemSetRpm(fanCounter, fanSystemGetRpmErrorByTemperature(fanCounter));
                break;

            case FAN_SYSTEM_MODE_NONE :
            default :
                break;

            }
        }
    }
    fanControllerStartMeasurement();
}

void fanSystemSetMode(uint32_t fanNumber, FanSystemMode mode)
{
    if (fanNumber < MAX_FAN_COUNT) {
        fanSystemParameters.fanSystemMode[fanNumber] = mode;
    }
}

FanSystemMode fanSystemGetMode(uint32_t fanNumber)
{
    if (fanNumber < MAX_FAN_COUNT) {
        return fanSystemParameters.fanSystemMode[fanNumber];
    } else {
        return FAN_SYSTEM_MODE_NONE;
    }
}

void fanSystemSetTargetRpm(uint32_t fanNumber, uint32_t fanRpm)
{
    if (fanNumber < MAX_FAN_COUNT) {
        fanSystemParameters.fanRpmTargetValue[fanNumber] = fanRpm;
    }
}

uint32_t fanSystemGetTargetRpm(uint32_t fanNumber)
{
    if (fanNumber < MAX_FAN_COUNT) {
        return fanSystemParameters.fanRpmTargetValue[fanNumber];
    } else {
        return 0;
    }
}

uint32_t fanSystemGetCurrentRpm(uint32_t fanNumber)
{
    if (fanNumber < MAX_FAN_COUNT) {
        return fanRpmActualValue[fanNumber];
    } else {
        return 0;
    }
}

void fanSystemSetPwm(uint32_t fanNumber, uint32_t fanPwm)
{
    if (fanNumber < MAX_FAN_COUNT) {
        fanSystemParameters.fanPwmTargetValue[fanNumber] = fanPwm;
    }
}

uint32_t fanSystemGetPwm(uint32_t fanNumber)
{
    if (fanNumber < MAX_FAN_COUNT) {
        return fanSystemParameters.fanPwmTargetValue[fanNumber];
    } else {
        return 0;
    }
}

void fanSystemSetCurve(uint32_t fanNumber, const FanSpeedTempCurve *fanSpeedTempCurve)
{
    if ((fanNumber < MAX_FAN_COUNT) && fanSpeedTempCurve) {
        fanSystemParameters.fanSpeedTempCurve[fanNumber] = *fanSpeedTempCurve;
    }
}

void fanSystemSetCurveTemperatureIndex(uint32_t fanNumber, uint32_t temperatureIndex)
{
    if ((fanNumber < MAX_FAN_COUNT)) {
        fanSystemParameters.temperatureIndex[fanNumber] = temperatureIndex;
    }
}

void fanSystemGetCurve(uint32_t fanNumber, FanSpeedTempCurve *fanSpeedTempCurve)
{
    if ((fanNumber < MAX_FAN_COUNT) && fanSpeedTempCurve) {
        *fanSpeedTempCurve = fanSystemParameters.fanSpeedTempCurve[fanNumber];
    }
}

uint32_t fanSystemGetMeanRpm(uint32_t fanNumber)
{
    if (!fanSystemGetCurrentRpm(fanNumber))
        return 0;

    uint32_t i = 0;
    uint32_t rpmValue = 0;
    for (i = 0; i < FAN_MEASUREMENT_WINDOW_SIZE; i++) {
        rpmValue += fanSpeedMeasurements[fanNumber].dataWindow[i];
    }

    return rpmValue >> FAN_MEASUREMENT_WINDOW_SHIFT_MULT;
}

FanControllerChannelConfig * fanSystemGetChannelConfig(uint32_t *channelCount)
{
    if (fanDetectionState == FAN_DETECTION_STATE_READY) {
        if (channelCount) {
            *channelCount = MAX_FAN_COUNT;
        }
        return fanChannelConfigRegistry;
    }

    if (channelCount) {
        *channelCount = 0;
    }

    return NULL;
}

void fanSystemStartFanDetection(void)
{
    isFanDetectionTaskRunning = true;
}
