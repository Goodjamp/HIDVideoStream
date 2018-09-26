#ifndef __LED_CONTROLLER_H__
#define __LED_CONTROLLER_H__

#include <stdint.h>
#include <stdbool.h>

#define LED_DRIVER_CHANNEL_COUNT    144

typedef enum {
    LED_TYPE_WS2812B,
    LED_TYPE_HD59731B
} LedType;

typedef void (*LedControllerOnDataSentCb)();

void ledControllerInit(LedControllerOnDataSentCb onDataSentCb);
void ledControllerSetLedData(uint32_t portNumber, LedType ledType, const uint32_t *rgbArray, uint32_t rgbArraylength);

#endif
