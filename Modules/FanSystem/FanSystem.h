#ifndef __FAN_SYSTEM_H__
#define __FAN_SYSTEM_H__

#include <stdint.h>
#include <stdbool.h>

#include "FanController.h"

typedef struct FanSpeedTempCurvePoint {
    uint32_t rpm;
    uint32_t temperature;
} FanSpeedTempCurvePoint;

typedef struct FanSpeedTempCurve {
    #define MAX_CURVE_POINT_COUNT   6
    FanSpeedTempCurvePoint curvePoints[MAX_CURVE_POINT_COUNT];
    uint32_t curvePointCount;
} FanSpeedTempCurve;

typedef enum FanSystemMode {
    FAN_SYSTEM_MODE_NONE,
    FAN_SYSTEM_MODE_PWM,
    FAN_SYSTEM_MODE_RPM,
    FAN_SYSTEM_MODE_CURVE,
} FanSystemMode;

// Interface callback function types
typedef uint32_t (*FanSystemGetTemperature)(uint32_t temperatureIndex, uint32_t fanIndex);
typedef void (*FanSystemMeasurementEvent)(void);
typedef uint32_t (*FanSystemGetTimestamp)(void);

// Interface functions
void fanSystemInit(FanSystemGetTemperature fanGetTemperatureCallback, FanSystemGetTimestamp fanSystemGetTimestampCallback, FanSystemMeasurementEvent fanSystemMeasurementEventCallback);
void fanSystemTaskRun(void);

void fanSystemSetMode(uint32_t fanNumber, FanSystemMode mode);
FanSystemMode fanSystemGetMode(uint32_t fanNumber);

void fanSystemSetTargetRpm(uint32_t fanNumber, uint32_t fanRpm);
uint32_t fanSystemGetTargetRpm(uint32_t fanNumber);
uint32_t fanSystemGetCurrentRpm(uint32_t fanNumber);
uint32_t fanSystemGetMeanRpm(uint32_t fanNumber);

void fanSystemSetPwm(uint32_t fanNumber, uint32_t fanRpm);
uint32_t fanSystemGetPwm(uint32_t fanNumber);

void fanSystemSetCurve(uint32_t fanNumber, const FanSpeedTempCurve *fanSpeedTempCurve);
void fanSystemSetCurveTemperatureIndex(uint32_t fanNumber, uint32_t temperatureIndex);
void fanSystemGetCurve(uint32_t fanNumber, FanSpeedTempCurve *fanSpeedTempCurve);
FanControllerChannelConfig * fanSystemGetChannelConfig(uint32_t *channelCount);
void fanSystemStartFanDetection(void);

#endif
