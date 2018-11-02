#ifndef __HAL_GPIO_H__
#define __HAL_GPIO_H__

#include <stdint.h>
#include <stdbool.h>

typedef struct HalGpioPinStruct *HalGpioPin;

typedef struct HalGpio {
    HalGpioPin usbVbus;

    HalGpioPin spiFlashMiso;
    HalGpioPin spiFlashMosi;
    HalGpioPin spiFlashSck;
    HalGpioPin spiFlashCs;
    HalGpioPin spiFlashWp;
    HalGpioPin spiFlashHold;

    HalGpioPin oledSdin;
    HalGpioPin oledSck;
    HalGpioPin oledCs;
    HalGpioPin oledDc;
    HalGpioPin oledRst;
    HalGpioPin oledSda;
    HalGpioPin oledScl;

    HalGpioPin ledSel0;
    HalGpioPin ledSel1;
    HalGpioPin ledSel2;

    HalGpioPin ledPwm0;
    HalGpioPin ledPwm1;

    HalGpioPin pwmChannel0;
    HalGpioPin pwmChannel1;
    HalGpioPin pwmChannel2;
    HalGpioPin pwmChannel3;
    HalGpioPin pwmChannel4;
    HalGpioPin pwmChannel5;
    HalGpioPin pwmChannel6;

    HalGpioPin tachChannel0;
    HalGpioPin tachChannel1;
    HalGpioPin tachChannel2;
    HalGpioPin tachChannel3;
    HalGpioPin tachChannel4;
    HalGpioPin tachChannel5;
    HalGpioPin tachChannel6;

    HalGpioPin adcTermistor0;
    HalGpioPin adcTermistor1;

    HalGpioPin adc12v;
    HalGpioPin adc5v;
    HalGpioPin adc3v3;

} HalGpio;

extern const HalGpio halGpio;

void halGpioInit(void);
void halGpioPinInit(HalGpioPin pin);
void halGpioPinDeinit(HalGpioPin pin);
void halGpioSetPin(HalGpioPin pin, bool value);
bool halGpioGetPin(HalGpioPin pin);

#endif
