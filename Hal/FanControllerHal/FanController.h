#ifndef __FAN_CONTROLLER_H__
#define __FAN_CONTROLLER_H__

#include <stdint.h>
#include <stdbool.h>

#define FAN_COUNT   7

typedef enum FanControllerChannelState {
    FAN_CONTROLLER_CHANNEL_NONE,
    FAN_CONTROLLER_CHANNEL_3P,
    FAN_CONTROLLER_CHANNEL_4P,
    FAN_CONTROLLER_CHANNEL_DETECTION_ERROR,
} FanControllerChannelState;

typedef struct FanControllerChannelConfig {
    uint32_t channelNumber;
    FanControllerChannelState channelState;
} FanControllerChannelConfig;

typedef void (*FanControllerSpeedMeasuredCb)(uint32_t fanNumber, uint32_t rpm);

void fanControllerInit(FanControllerSpeedMeasuredCb speedMeasuredCb);

void fanControllerStartMeasurement(void);

void fanControllerSetPwm(uint32_t fanNumber, uint32_t fanPwm);
uint32_t fanControllerGetPwm(uint32_t fanNumber);

uint32_t fanControllerGetTachCount(void);
uint32_t fanControllerGetPwmCount(void);
uint32_t fanControllerGetMaxPwmValue(void);

void fanControllerReInitPwm(FanControllerChannelConfig fanControllerChannelConfig[], uint32_t channelCount);

#endif
