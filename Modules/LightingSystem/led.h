#ifndef LED_H
#define LED_H

#include <stdint.h>
#include <stdbool.h>
#include "LedController.h"

#define LED_GROUP_MAX_COUNT 8

typedef struct {
    uint8_t b;
    uint8_t r;
    uint8_t g;
    uint8_t padding;
} Color;

typedef enum {
    EFFECT_RAINBOW_WAVE = 0,
    EFFECT_COLOR_SHIFT = 1,
    EFFECT_COLOR_PULSE = 2,
    EFFECT_COLOR_WAVE = 3,
    EFFECT_STATIC = 4,
    EFFECT_TEMPERATURE = 5,
    EFFECT_VISOR = 6,
    EFFECT_MARQUEE = 7,
    EFFECT_STROBING = 8,
    EFFECT_SEQUENTIAL = 9,
    EFFECT_RAINBOW = 10
} Effect;

typedef enum {
    EFFECT_SPEED_FAST = 0,
    EFFECT_SPEED_MIDDLE = 1,
    EFFECT_SPEED_SLOW = 2
} EffectSpeed;

typedef enum {
    LED_STRIP_1,
    LED_STRIP_2,
    LED_STRIP_COUNT
} LedStripIndex;

typedef enum {
    LED_STRIP_MODE_OFF = 0,
    LED_STRIP_MODE_PRESET = 1,
    LED_STRIP_MODE_DIRECT = 2
} LedStripMode;

typedef struct {
    EffectSpeed speed;
    bool forwardDirection;
} RainbowWave;

typedef struct {
    EffectSpeed speed;
    bool randomColor;
    Color color[2];
} ColorShift;

typedef struct {
    EffectSpeed speed;
} Rainbow;

typedef struct {
    Color color;
} StaticEffect;

typedef struct {
    bool randomColor;
    EffectSpeed speed;
    Color color[2];
} Strobing;

typedef struct {
    EffectSpeed speed;
    Color color;
} Marquee;

typedef struct {
    EffectSpeed speed;
    bool randomColor;
    bool forwardDirection;
    Color color;
} Sequential;

typedef struct {
    bool randomColor;
    EffectSpeed speed;
    Color color[2];
} Visor;

typedef struct {
    EffectSpeed speed;
    bool randomColor;
    Color color[2];
} ColorPulse;

typedef struct {
    EffectSpeed speed;
    bool randomColor;
    bool forwardDirection;
    Color color[2];
} ColorWave;

typedef struct {
    Color color[3];
    uint16_t temperature[3];
    uint8_t temperatureIndex;
} TemperatureEffect;

typedef struct {
    Effect effectType;
    uint8_t startIndex;
    uint8_t stopIndex;
    union {
        RainbowWave rainbowWave;
        ColorShift colorShift;
        ColorPulse colorPulse;
        ColorWave colorWave;
        StaticEffect staticEffect;
        TemperatureEffect temperatureEffect;
        Sequential sequential;
        Marquee marquee;
        Rainbow rainbow;
        Strobing strobing;
        Visor visor;
    } effect;
} LedGroup;

typedef enum {
    RES_OK                   = 0x00,
    RES_INVALID_COMMAND      = 0x01,
    RES_INVALID_PORT_INDEX   = 0x10,
    RES_SENSOR_ABSENT        = 0x11,
    RES_INVALID_ARGUMENT     = 0x12,
} Error;


void ledInit(void);
void ledEnablePlayEffect(void);
void ledPlayEffects(void);
void ledSetAllEffectsInitialState(void);
uint8_t *ledGetConfigurationLocation(uint16_t *length);
void ledAllStripClear(void);


void ledGenerateRandomColor(Color *color);

Error ledSetLedType(LedStripIndex stripIndex, LedType ledType);
Error ledSetLedCount(LedStripIndex stripIndex, uint8_t count);
Error ledSetLedBrightness(LedStripIndex stripIndex, uint8_t brightness);
Error ledStripSetGroupExternalTemperature(LedStripIndex stripIndex, uint8_t groupNumber, uint16_t temperature);
Error ledStripClear(LedStripIndex stripIndex);
Error ledSetStripMode(LedStripIndex stripIndex, LedStripMode mode);
Error ledStripAppendGroup(LedStripIndex stripIndex, LedGroup *ledGroup);

#endif
