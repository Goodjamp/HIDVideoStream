#ifndef __HAL_PWM_H__
#define __HAL_PWM_H__

#include <stdint.h>
#include <stdbool.h>


typedef enum {
    HAL_PWM_CHANNEL_NUM_LOCK_R,
    HAL_PWM_CHANNEL_NUM_LOCK_G,
    HAL_PWM_CHANNEL_NUM_LOCK_B,

    HAL_PWM_CHANNEL_CAPS_LOCK_R,
    HAL_PWM_CHANNEL_CAPS_LOCK_G,
    HAL_PWM_CHANNEL_CAPS_LOCK_B,

    HAL_PWM_CHANNEL_SCROL_LOCK_R,
    HAL_PWM_CHANNEL_SCROL_LOCK_G,
    HAL_PWM_CHANNEL_SCROL_LOCK_B,

    HAL_PWM_CHANNEL_COUNT
} HalPwmChannel;


bool halPwmInit(void);

// duty cycle can be between 0 and 0xFF
bool halPwmSetDutyCycle(HalPwmChannel channel, uint8_t dutyCycle);

#endif
