#include "HalGpio.h"

#include "fsl_gpio.h"
#include "fsl_iocon.h"

#define GPIO_CONFIG_OUT_LOGIC_0 { .pinDirection = kGPIO_DigitalOutput, .outputLogic = 0 }
#define GPIO_CONFIG_OUT_LOGIC_1 { .pinDirection = kGPIO_DigitalOutput, .outputLogic = 1 }
#define GPIO_CONFIG_INPUT       { .pinDirection = kGPIO_DigitalInput, .outputLogic = 0 }

#define DIGITAL_WITH_PULLUP (IOCON_MODE_PULLUP | IOCON_DIGITAL_EN | IOCON_INPFILT_OFF)

typedef struct HalGpioPinStruct {
    iocon_group_t iocon;
    gpio_pin_config_t config;
} HalGpioPinStruct;

const HalGpio halGpio = {
    .usbVbus = &(HalGpioPinStruct) {
        .iocon = {
            .port = 1,
            .pin = 16,
            .modefunc = IOCON_FUNC0 | DIGITAL_WITH_PULLUP
        },
        .config = GPIO_CONFIG_OUT_LOGIC_0
    },

   .spiFlashMiso = &(HalGpioPinStruct) {
        .iocon = {
            .port = 5,
            .pin = 1,
            .modefunc = IOCON_FUNC0 | DIGITAL_WITH_PULLUP
        },
        .config	= GPIO_CONFIG_INPUT
    },
    .spiFlashMosi = &(HalGpioPinStruct) {
        .iocon = {
            .port = 5,
            .pin = 0,
            .modefunc = IOCON_FUNC0 | DIGITAL_WITH_PULLUP
        },
        .config	= GPIO_CONFIG_INPUT
    },
   .spiFlashSck = &(HalGpioPinStruct) {
        .iocon = {
            .port = 4,
            .pin = 31,
            .modefunc = IOCON_FUNC1 | DIGITAL_WITH_PULLUP
        },
        .config	= GPIO_CONFIG_INPUT
    },
    .spiFlashCs = &(HalGpioPinStruct) {
        .iocon = {
            .port = 5,
            .pin = 2,
            .modefunc = IOCON_FUNC1 | DIGITAL_WITH_PULLUP
        },
        .config = GPIO_CONFIG_OUT_LOGIC_0
    },
    .spiFlashWp = &(HalGpioPinStruct) {
        .iocon = {
            .port = 5,
            .pin = 3,
            .modefunc = IOCON_FUNC2 | DIGITAL_WITH_PULLUP
        },
        .config = GPIO_CONFIG_OUT_LOGIC_1
    },
    .spiFlashHold = &(HalGpioPinStruct) {
        .iocon = {
            .port = 5,
            .pin = 4,
            .modefunc = IOCON_FUNC1 | DIGITAL_WITH_PULLUP
        },
        .config = GPIO_CONFIG_OUT_LOGIC_1
    },

    .oledSdin = &(HalGpioPinStruct) {
        .iocon = {
            .port = 2,
            .pin = 18,
            .modefunc = IOCON_FUNC2 | DIGITAL_WITH_PULLUP
        },
        .config	= GPIO_CONFIG_OUT_LOGIC_0
    },
    .oledSck = &(HalGpioPinStruct) {
        .iocon = {
            .port = 2,
            .pin = 26,
            .modefunc = IOCON_FUNC2 | DIGITAL_WITH_PULLUP
        },
        .config	= GPIO_CONFIG_OUT_LOGIC_0
    },
    .oledCs = &(HalGpioPinStruct) {
        .iocon = {
            .port = 2,
            .pin = 21,
            .modefunc = IOCON_FUNC2 | DIGITAL_WITH_PULLUP
        },
        .config	= GPIO_CONFIG_OUT_LOGIC_0
    },
    .oledDc = &(HalGpioPinStruct) {
        .iocon = {
            .port = 2,
            .pin = 20,
            .modefunc = IOCON_FUNC4 | DIGITAL_WITH_PULLUP
        },
        .config	= GPIO_CONFIG_OUT_LOGIC_0
    },
    .oledRst = &(HalGpioPinStruct) {
        .iocon = {
            .port = 2,
            .pin = 19,
            .modefunc = IOCON_FUNC4 | DIGITAL_WITH_PULLUP
        },
        .config	= GPIO_CONFIG_OUT_LOGIC_0
    },
    .oledSda = &(HalGpioPinStruct) {
        .iocon = {
            .port = 2,
            .pin = 12,
            .modefunc = IOCON_FUNC4 | DIGITAL_WITH_PULLUP
        },
        .config	= GPIO_CONFIG_OUT_LOGIC_0
    },
    .oledScl = &(HalGpioPinStruct) {
        .iocon = {
            .port = 2,
            .pin = 13,
            .modefunc = IOCON_FUNC4 | DIGITAL_WITH_PULLUP
        },
        .config	= GPIO_CONFIG_OUT_LOGIC_0
    },

    .ledSel0 = &(HalGpioPinStruct) {
        .iocon = {
            .port = 1,
            .pin = 4,
            .modefunc = IOCON_FUNC4 | DIGITAL_WITH_PULLUP
        },
        .config	= GPIO_CONFIG_OUT_LOGIC_0
    },
    .ledSel1 = &(HalGpioPinStruct) {
        .iocon = {
            .port = 1,
            .pin = 6,
            .modefunc = IOCON_FUNC4 | DIGITAL_WITH_PULLUP
        },
        .config	= GPIO_CONFIG_OUT_LOGIC_0
    },
    .ledSel2 = &(HalGpioPinStruct) {
        .iocon = {
            .port = 1,
            .pin = 7,
            .modefunc = IOCON_FUNC4 | DIGITAL_WITH_PULLUP
        },
        .config	= GPIO_CONFIG_OUT_LOGIC_0
    },

    .ledPwm0 = &(HalGpioPinStruct) {
        .iocon = {
            .port = 4,
            .pin = 30,
            .modefunc = IOCON_FUNC4 | DIGITAL_WITH_PULLUP
        },
        .config	= GPIO_CONFIG_OUT_LOGIC_0
    },
    .ledPwm1 = &(HalGpioPinStruct) {
        .iocon = {
            .port = 3,
            .pin = 14,
            .modefunc = IOCON_FUNC4 | DIGITAL_WITH_PULLUP
        },
        .config	= GPIO_CONFIG_OUT_LOGIC_0
    },

    .pwmChannel0 = &(HalGpioPinStruct) {
        .iocon = {
            .port = 2,
            .pin = 8,
            .modefunc = IOCON_FUNC2 | DIGITAL_WITH_PULLUP
        },
        .config	= GPIO_CONFIG_OUT_LOGIC_0
    },
    .pwmChannel1 = &(HalGpioPinStruct) {
        .iocon = {
            .port = 2,
            .pin = 9,
            .modefunc = IOCON_FUNC2 | DIGITAL_WITH_PULLUP
        },
        .config	= GPIO_CONFIG_OUT_LOGIC_0
    },
    .pwmChannel2 = &(HalGpioPinStruct) {
        .iocon = {
            .port = 2,
            .pin = 14,
            .modefunc = IOCON_FUNC2 | DIGITAL_WITH_PULLUP
        },
        .config	= GPIO_CONFIG_OUT_LOGIC_0
    },
    .pwmChannel3 = &(HalGpioPinStruct) {
        .iocon = {
            .port = 4,
            .pin = 23,
            .modefunc = IOCON_FUNC4 | DIGITAL_WITH_PULLUP
        },
        .config	= GPIO_CONFIG_OUT_LOGIC_0
    },
    .pwmChannel4 = &(HalGpioPinStruct) {
        .iocon = {
            .port = 4,
            .pin = 24,
            .modefunc = IOCON_FUNC4 | DIGITAL_WITH_PULLUP
        },
        .config	= GPIO_CONFIG_OUT_LOGIC_0
    },
    .pwmChannel5 = &(HalGpioPinStruct) {
        .iocon = {
            .port = 4,
            .pin = 25,
            .modefunc = IOCON_FUNC4 | DIGITAL_WITH_PULLUP
        },
        .config	= GPIO_CONFIG_OUT_LOGIC_0
    },
    .pwmChannel6 = &(HalGpioPinStruct) {
        .iocon = {
            .port = 1,
            .pin = 5,
            .modefunc = IOCON_FUNC4 | DIGITAL_WITH_PULLUP
        },
        .config	= GPIO_CONFIG_OUT_LOGIC_0
    },

    .tachChannel0 = &(HalGpioPinStruct) {
        .iocon = {
            .port = 4,
            .pin = 7,
            .modefunc = IOCON_FUNC2 | DIGITAL_WITH_PULLUP
        },
        .config	= GPIO_CONFIG_OUT_LOGIC_0
    },
    .tachChannel1 = &(HalGpioPinStruct) {
        .iocon = {
            .port = 4,
            .pin = 8,
            .modefunc = IOCON_FUNC2 | DIGITAL_WITH_PULLUP
        },
        .config	= GPIO_CONFIG_OUT_LOGIC_0
    },
    .tachChannel2 = &(HalGpioPinStruct) {
        .iocon = {
            .port = 4,
            .pin = 9,
            .modefunc = IOCON_FUNC2 | DIGITAL_WITH_PULLUP
        },
        .config	= GPIO_CONFIG_OUT_LOGIC_0
    },
    .tachChannel3 = &(HalGpioPinStruct) {
        .iocon = {
            .port = 4,
            .pin = 10,
            .modefunc = IOCON_FUNC4 | DIGITAL_WITH_PULLUP
        },
        .config	= GPIO_CONFIG_OUT_LOGIC_0
    },
    .tachChannel4 = &(HalGpioPinStruct) {
        .iocon = {
            .port = 4,
            .pin = 11,
            .modefunc = IOCON_FUNC4 | DIGITAL_WITH_PULLUP
        },
        .config	= GPIO_CONFIG_OUT_LOGIC_0
    },
    .tachChannel5 = &(HalGpioPinStruct) {
        .iocon = {
            .port = 4,
            .pin = 12,
            .modefunc = IOCON_FUNC4 | DIGITAL_WITH_PULLUP
        },
        .config	= GPIO_CONFIG_OUT_LOGIC_0
    },
    .tachChannel6 = &(HalGpioPinStruct) {
        .iocon = {
            .port = 4,
            .pin = 13,
            .modefunc = IOCON_FUNC4 | DIGITAL_WITH_PULLUP
        },
        .config	= GPIO_CONFIG_OUT_LOGIC_0
    },

    .adcTermistor0 = &(HalGpioPinStruct) {
        .iocon = {
            .port = 2,
            .pin = 0,
            .modefunc = IOCON_FUNC0 | IOCON_ANALOG_EN
        },
        .config	= GPIO_CONFIG_INPUT
    },
    .adcTermistor1 = &(HalGpioPinStruct) {
        .iocon = {
            .port = 2,
            .pin = 1,
            .modefunc = IOCON_FUNC0 | IOCON_ANALOG_EN
        },
        .config	= GPIO_CONFIG_INPUT
    },
    .adc12v = &(HalGpioPinStruct) {
        .iocon = {
            .port = 3,
            .pin = 22,
            .modefunc = IOCON_FUNC0 | IOCON_ANALOG_EN
        },
        .config	= GPIO_CONFIG_INPUT
    },
    .adc5v = &(HalGpioPinStruct) {
        .iocon = {
            .port = 1,
            .pin = 0,
            .modefunc = IOCON_FUNC0 | IOCON_ANALOG_EN
        },
        .config	= GPIO_CONFIG_INPUT
    },
    .adc3v3 = &(HalGpioPinStruct) {
        .iocon = {
            .port = 0,
            .pin = 31,
            .modefunc = IOCON_FUNC0 | IOCON_ANALOG_EN
        },
        .config	= GPIO_CONFIG_INPUT
    },

};

void halGpioPinInit(HalGpioPin pin)
{
    IOCON_PinMuxSet(IOCON, pin->iocon.port, pin->iocon.pin, pin->iocon.modefunc);
    GPIO_PinInit(GPIO, pin->iocon.port, pin->iocon.pin, &pin->config);
}

void halGpioPinDeinit(HalGpioPin pin)
{
    HalGpioPinStruct halGpioPinStruct = {
        .iocon = {
            .port = pin->iocon.port,
            .pin = pin->iocon.pin,
            .modefunc = IOCON_FUNC0 | IOCON_ANALOG_EN
        },
        .config	= GPIO_CONFIG_INPUT
    };
    IOCON_PinMuxSet(IOCON, pin->iocon.port, pin->iocon.pin, halGpioPinStruct.iocon.modefunc);
    GPIO_PinInit(GPIO, pin->iocon.port, pin->iocon.pin, &halGpioPinStruct.config);
}

void halGpioSetPin(HalGpioPin pin, bool value)
{
    GPIO_WritePinOutput(GPIO, pin->iocon.port, pin->iocon.pin, value);
}

bool halGpioGetPin(HalGpioPin pin)
{
    return GPIO_ReadPinInput(GPIO, pin->iocon.port, pin->iocon.pin);
}
