#include "led.h"
#include <string.h>
#include "LedController.h"

//#define JOTUN

#ifdef JOTUN
    #define LED_STRIP_1_MAX_LED_COUNT 94
#else
    #define LED_STRIP_1_MAX_LED_COUNT 72
#endif

#define LED_STRIP_2_MAX_LED_COUNT 72

#define LED_BRIGHTNESS_MAX 100

#define BARBUDA_DEFAULT_TEMPERATURE (uint16_t) (4000)

static uint32_t timerGetTime()
{
    return 1000;
}


typedef struct {
    LedStripMode mode;
    uint8_t brightness;
    uint8_t ledsCount;
    LedType ledType;
    uint8_t groupCount;
    LedGroup group[LED_GROUP_MAX_COUNT];
} LedStrip;

typedef struct {
    struct {
        uint32_t wheelPosition;
    } rainbowWaveSettings;
    struct {
        uint8_t position;
    } colorShiftSettings;
    struct {
        uint32_t wheelPosition;
    } rainbowSettings;
    struct {
        uint16_t time;
        uint8_t colorIndex;
    } strobing;
    struct {
        uint16_t time;
    } marquee;
    struct {
        Color backgroundColor;
        uint16_t time;
    } sequential;
    struct {
        uint16_t time1;
        uint16_t time2;
    } visor;
    struct {
        uint16_t time;
    } colorPulse;
    struct {
        uint16_t time;
        uint8_t colorIndex;
    } colorWave;
    struct {
        uint16_t previousTemperature;
        uint16_t temperature;
        uint32_t timeStamp;
    } temperatureColor;
} LedGroupState;

static LedGroupState ledGroupState[LED_STRIP_COUNT][LED_GROUP_MAX_COUNT];
static LedStrip ledStrip[LED_STRIP_COUNT];
static Color stripArray[LED_STRIP_COUNT][LED_STRIP_1_MAX_LED_COUNT];
static bool playEffect = false;


static Color wheel(uint8_t wheelPos)
{
    Color color = {0, 0, 0};
    wheelPos = UINT8_MAX - wheelPos;

    if (wheelPos < 85)
    {
        color.r = UINT8_MAX - wheelPos * 3;
        color.b = wheelPos * 3;
    }
    else if (wheelPos < 170)
    {
        wheelPos -= 85;
        color.g = wheelPos * 3;
        color.b = UINT8_MAX - wheelPos * 3;
    }
    else
    {
        wheelPos -= 170;
        color.r = wheelPos * 3;
        color.g = UINT8_MAX - wheelPos * 3;
    }
    return color;
}

static Color lightIntensity(Color color, uint16_t intensity)
{
    color.r = ((uint16_t) color.r * intensity) / 100;
    color.g = ((uint16_t) color.g * intensity) / 100;
    color.b = ((uint16_t) color.b * intensity) / 100 ;

    return color;
}

static Color blendColor(Color color1, Color color2, uint8_t coef)
{
	Color color;
    color.r = (color1.r * (uint16_t)coef + color2.r * (0xff - (uint16_t)coef)) >> 8;
    color.g = (color1.g * (uint16_t)coef + color2.g * (0xff - (uint16_t)coef)) >> 8;
    color.b = (color1.b * (uint16_t)coef + color2.b * (0xff - (uint16_t)coef)) >> 8;
	return color;
}

static uint32_t randI(void)
{
    static uint32_t y = 0xA55AA55A;
    y ^= (y >> 13);
    y ^= (y >> 17);
    y ^= (y << 5);
    return  y;
}

void ledGenerateRandomColor(Color *color)
{
	static uint32_t hPrevious = 0;
	uint32_t h = ((hPrevious + randI() + 30) % (360 - 60)) % 360;
	uint32_t s = UINT8_MAX;
	uint32_t v = UINT8_MAX;
	uint8_t r, f, q, t;

    hPrevious = h;
    r = h / (UINT8_MAX / 6);
    f = (h - (r * (UINT8_MAX / 6))) * 6;
    q = (v * (UINT8_MAX - ((s * f) >> 8))) >> 8;
    t = (v * (UINT8_MAX - ((s * (UINT8_MAX - f)) >> 8))) >> 8;

    switch (r) {
        case 0:  color->r = v; color->g = t; color->b = 0; break;
        case 1:  color->r = q; color->g = v; color->b = 0; break;
        case 2:  color->r = 0; color->g = v; color->b = t; break;
        case 3:  color->r = 0; color->g = q; color->b = v; break;
        case 4:  color->r = t; color->g = 0; color->b = v; break;
        default: color->r = v; color->g = 0; color->b = q; break;
    }
}

static void rainbowWave(LedStripIndex stripIndex, uint8_t groupIndex)
{
    LedGroup *group = &ledStrip[stripIndex].group[groupIndex];
    RainbowWave *effect = &group->effect.rainbowWave;
    uint32_t *wheelPosition = &ledGroupState[stripIndex][groupIndex].rainbowWaveSettings.wheelPosition;
    uint8_t index;

    for (index = group->startIndex; index < group->stopIndex; ++index) {
        uint8_t newIndex = (!effect->forwardDirection) ? index : ((group->stopIndex - 1) - index);
        uint8_t wheelPos = newIndex * 256 / (group->stopIndex - group->startIndex );
        Color color;

        color = wheel(wheelPos + *wheelPosition);
        color = lightIntensity(color, ledStrip[stripIndex].brightness);
        stripArray[stripIndex][index] = color;
    }

    switch (effect->speed) {
        case EFFECT_SPEED_FAST:    *wheelPosition = *wheelPosition + 4;  break;
        case EFFECT_SPEED_MIDDLE:  *wheelPosition = *wheelPosition + 2;   break;
        case EFFECT_SPEED_SLOW:    *wheelPosition = *wheelPosition + 1;   break;
        default:                                                          break;
    }
}

void colorShift(LedStripIndex stripIndex, uint8_t groupIndex)
{
    LedGroup *group = &ledStrip[stripIndex].group[groupIndex];
    ColorShift *effect = &group->effect.colorShift;
    uint8_t effectStep = 0;
    Color color;
    uint8_t index;

    color = blendColor(effect->color[0], effect->color[1], ledGroupState[stripIndex][groupIndex].colorShiftSettings.position);
    color = lightIntensity(color, ledStrip[stripIndex].brightness);

    for (index = group->startIndex; index < group->stopIndex; ++index) {
        stripArray[stripIndex][index] = color;
    }

    switch (effect->speed) {
        case EFFECT_SPEED_FAST:    effectStep = 8;  break;
        case EFFECT_SPEED_MIDDLE:  effectStep = 4;  break;
        case EFFECT_SPEED_SLOW:    effectStep = 1;  break;
        default:                                    break;
    }

    if (ledGroupState[stripIndex][groupIndex].colorShiftSettings.position > 0xff - effectStep) {
        if (effect->randomColor) {
            effect->color[1] = effect->color[0];
            ledGenerateRandomColor(&effect->color[0]);
        } else {
            Color colorTemp;
            colorTemp = effect->color[0];
            effect->color[0] = effect->color[1];
            effect->color[1] = colorTemp;
        }
    }

    ledGroupState[stripIndex][groupIndex].colorShiftSettings.position += effectStep;
}

static void rainbow(LedStripIndex stripIndex, uint8_t groupIndex)
{
    LedGroup *group = &ledStrip[stripIndex].group[groupIndex];
    Rainbow *effect = &group->effect.rainbow;
    uint32_t *wheelPosition = &ledGroupState[stripIndex][groupIndex].rainbowSettings.wheelPosition;
    uint8_t index;
    Color color;

    color = wheel(*wheelPosition);
    color = lightIntensity(color, ledStrip[stripIndex].brightness);

    for (index = group->startIndex; index < group->stopIndex; ++index) {
        stripArray[stripIndex][index] = color;
    }

    switch (effect->speed) {
        case EFFECT_SPEED_FAST:    *wheelPosition = *wheelPosition + 4;  break;
        case EFFECT_SPEED_MIDDLE:  *wheelPosition = *wheelPosition + 2;   break;
        case EFFECT_SPEED_SLOW:    *wheelPosition = *wheelPosition + 1;   break;
        default:                                                          break;
    }
}

static void colorStatic(LedStripIndex stripIndex, uint8_t groupIndex)
{
    LedGroup *group = &ledStrip[stripIndex].group[groupIndex];
    StaticEffect *effect = &group->effect.staticEffect;
    uint8_t index;
    Color color;

    color = lightIntensity(effect->color, ledStrip[stripIndex].brightness);

    for (index = group->startIndex; index < group->stopIndex; ++index) {
        stripArray[stripIndex][index] = color;
    }
}

static void strobing(LedStripIndex stripIndex, uint8_t groupIndex)
{
    LedGroup *group = &ledStrip[stripIndex].group[groupIndex];
    Strobing *effect = &group->effect.strobing;
    uint8_t effectPeriod = 0;
    Color color = {0, 0, 0};
    uint8_t index;
    uint16_t *time = &ledGroupState[stripIndex][groupIndex].strobing.time;
    uint8_t *colorIndex = &ledGroupState[stripIndex][groupIndex].strobing.colorIndex;

    switch (effect->speed) {
        case EFFECT_SPEED_FAST:    effectPeriod = 12;  break;
        case EFFECT_SPEED_MIDDLE:  effectPeriod = 24;  break;
        case EFFECT_SPEED_SLOW:    effectPeriod = 39;  break;
        default:                                break;
    }

    if (*time < effectPeriod) {
        color = lightIntensity(effect->color[*colorIndex], ledStrip[stripIndex].brightness);
    } else if (*time >= (effectPeriod << 1)) {
        *time = 0;
        *colorIndex = (*colorIndex) ? 0 : 1;

        if (effect->randomColor) {
            ledGenerateRandomColor(&effect->color[0]);
            ledGenerateRandomColor(&effect->color[1]);
        }
    }
    *time = *time +1;

    for (index = group->startIndex; index < group->stopIndex; ++index) {
        stripArray[stripIndex][index] = color;
    }
}

static void marquee(LedStripIndex stripIndex, uint8_t groupIndex)
{
    LedGroup *group = &ledStrip[stripIndex].group[groupIndex];
    Marquee *effect = &group->effect.marquee;
    uint8_t index;
    Color color;
    Color colorEmpty = {0, 0, 0};
    uint8_t speed = 3;
    uint16_t *time = &ledGroupState[stripIndex][groupIndex].marquee.time;

    switch (effect->speed) {
        case EFFECT_SPEED_FAST:    speed = 3;  break;
        case EFFECT_SPEED_MIDDLE:  speed = 6;  break;
        case EFFECT_SPEED_SLOW:    speed = 12;  break;
        default:    break;
    }

    color = lightIntensity(effect->color, ledStrip[stripIndex].brightness);

    for (index = group->startIndex; index < group->stopIndex; ++index) {
        stripArray[stripIndex][index] = colorEmpty;
    }

    if (*time / speed == 0) {
        for (index = group->startIndex; index < group->stopIndex; index += 3) {
            stripArray[stripIndex][index] = color;
        }

        for (index = group->startIndex+1; index < group->stopIndex; index += 3) {
            stripArray[stripIndex][index] = color;
        }
    } else if (*time / speed == 1) {
        for (index = group->startIndex; index < group->stopIndex; index += 3) {
            stripArray[stripIndex][index] = color;
        }

        for (index = group->startIndex+2; index < group->stopIndex; index += 3) {
            stripArray[stripIndex][index] = color;
        }
    } else if (*time / speed == 2) {
        for (index = group->startIndex+1; index < group->stopIndex; index += 3) {
            stripArray[stripIndex][index] = color;
        }

        for (index = group->startIndex+2; index < group->stopIndex; index += 3) {
            stripArray[stripIndex][index] = color;
        }
    }

    *time = *time + 1;

    if (*time / speed == 3) {
        *time = 0;
    }
}

static void sequential(LedStripIndex stripIndex, uint8_t groupIndex)
{
    LedGroup *group = &ledStrip[stripIndex].group[groupIndex];
    Sequential *effect = &group->effect.sequential;
    Color backgroundColor;
    Color color;
    uint8_t i;
    uint16_t *time = &ledGroupState[stripIndex][groupIndex].sequential.time;

    color = lightIntensity(effect->color, ledStrip[stripIndex].brightness);
    backgroundColor = lightIntensity(ledGroupState[stripIndex][groupIndex].sequential.backgroundColor, ledStrip[stripIndex].brightness);

    for (i = group->startIndex; i < group->stopIndex; i++) {
        uint16_t index = (uint16_t)(i - group->startIndex) << 8;
        uint8_t ledIndex = (effect->forwardDirection) ? i : (group->stopIndex - i - 1 + group->startIndex);

        if (index >= *time) {
            stripArray[stripIndex][ledIndex] = backgroundColor;
        } else if (index + 0xff <= *time) {
            stripArray[stripIndex][ledIndex] = color;
        } else {
            stripArray[stripIndex][ledIndex] = blendColor(color, backgroundColor, *time - index);
        }
    }

    if (ledStrip[stripIndex].ledType == LED_TYPE_HD59731B) {
        // SP fans
        switch (effect->speed) {
            case EFFECT_SPEED_FAST:    *time = *time + 43;  break;
            case EFFECT_SPEED_MIDDLE:  *time = *time + 21;  break;
            case EFFECT_SPEED_SLOW:    *time = *time + 13;  break;
            default:                                        break;
        }
    } else {
        switch (effect->speed) {
            case EFFECT_SPEED_FAST:    *time = *time + 512; break;
            case EFFECT_SPEED_MIDDLE:  *time = *time + 256; break;
            case EFFECT_SPEED_SLOW:    *time = *time + 128; break;
            default:                                        break;
        }
    }

    if (*time > ((uint16_t)(group->stopIndex - group->startIndex) << 8)) {
        *time = 0;

        if (effect->randomColor) {
            // set previous color as background color
            ledGroupState[stripIndex][groupIndex].sequential.backgroundColor = effect->color;
            // generate new color
            ledGenerateRandomColor(&effect->color);
        }
    }
}

static Color visorGetColor(Color color1, Color color2, uint16_t pos)
{
    if (pos <= 255) {
		Color colorBlack = {0, 0, 0};
		return blendColor(color1, colorBlack, pos);
	}

    if (pos >= 768) {
		Color colorBlack = {0, 0, 0};
		return blendColor(colorBlack, color2, pos - 768);
	}

	return blendColor(color2, color1, (pos - 255) >> 1);
}

static void visor(LedStripIndex stripIndex, uint8_t groupIndex)
{
    LedGroup *group = &ledStrip[stripIndex].group[groupIndex];
    Visor *effect = &group->effect.visor;
    Color firstColor;
    Color lastColor;
    uint8_t index;
    uint8_t speed;
    uint16_t *time1 = &ledGroupState[stripIndex][groupIndex].visor.time1;
    uint16_t *time2 = &ledGroupState[stripIndex][groupIndex].visor.time2;

    switch (effect->speed) {
        case EFFECT_SPEED_FAST:    speed = 192;  break;
        case EFFECT_SPEED_MIDDLE:  speed = 96;   break;
        case EFFECT_SPEED_SLOW:    speed = 64;   break;
        default:                                 break;
    }

    if (*time1 == 0 && *time2 == 0 && effect->randomColor) {
        ledGenerateRandomColor(&effect->color[0]);
        ledGenerateRandomColor(&effect->color[1]);
    }

    firstColor = lightIntensity(effect->color[1], ledStrip[stripIndex].brightness);
    lastColor = lightIntensity(effect->color[0], ledStrip[stripIndex].brightness);

	for (index = 0; index < group->stopIndex - group->startIndex; index++) {
		uint16_t t = ((uint16_t)index << 8) - *time1 + 1024;

	    if (t < 1024) {
			Color colorToSet;
            colorToSet = *time2 ? visorGetColor(lastColor, firstColor, t) : visorGetColor(firstColor, lastColor, t);
			stripArray[stripIndex][group->startIndex + index] = colorToSet;
		} else {
            Color colorEmpty = {0, 0, 0};
			stripArray[stripIndex][group->startIndex + index] = colorEmpty;
		}
	}

	if (*time2 == 0) {
		*time1 = *time1 + speed;

		if (*time1 >= ((uint16_t)(group->stopIndex - group->startIndex) << 8) + 768) {
			*time2 = 1;
		}
	} else {
		*time1 = *time1 - speed;
		if (*time1 == 0) {
			*time2 = 0;
		}
	}
}

static void colorPulse(LedStripIndex stripIndex, uint8_t groupIndex)
{
    LedGroup *group = &ledStrip[stripIndex].group[groupIndex];
    ColorPulse *effect = &group->effect.colorPulse;
    uint16_t *time = &ledGroupState[stripIndex][groupIndex].colorPulse.time;
    Color color = {0, 0, 0};
    Color colorBlack = {0, 0, 0};
    uint8_t speed;
    uint8_t index;

    switch (effect->speed) {
        case EFFECT_SPEED_FAST:    speed = 8;  break;
        case EFFECT_SPEED_MIDDLE:  speed = 4;  break;
        case EFFECT_SPEED_SLOW:    speed = 1;  break;
        default:                               break;
    }

    *time = *time + speed;

    if (*time <= 0xff) {
        color = blendColor(effect->color[0], colorBlack, *time);
    } else if (*time <= 2*(uint16_t)0xff) {
        color = blendColor(effect->color[0], colorBlack, 2*(uint16_t)0xff - *time);
    } else if (*time <= 3*(uint16_t)0xff) {
        // no color
    } else if (*time <= 4*(uint16_t)0xff) {
        color = blendColor(effect->color[1], colorBlack, *time - 3*(uint16_t)0xff );
    } else if (*time <= 5*(uint16_t)0xff) {
        color = blendColor(effect->color[1], colorBlack, 5*(uint16_t)0xff - *time);
    } else if ((*time <= 6*(uint16_t)0xff)) {
        // no color
    } else {
        *time = 0;

        if (effect->randomColor) {
            ledGenerateRandomColor(&effect->color[0]);
            ledGenerateRandomColor(&effect->color[1]);
        }
    }

    color = lightIntensity(color, ledStrip[stripIndex].brightness);

    for (index = group->startIndex; index < group->stopIndex; ++index) {
        stripArray[stripIndex][index] = color;
    }
}

static Color colorWaveGetColor(Color color, uint16_t pos)
{
    if (pos <= 255) {
		Color colorBlack = {0, 0, 0};
		return blendColor(color, colorBlack, pos);
	}

    if (pos >= 2048- 256) {
		Color colorBlack = {0, 0, 0};
		return blendColor(colorBlack, color, pos - 2048+ 256);
	}

	return color;
}

static void colorWave(LedStripIndex stripIndex, uint8_t groupIndex)
{
    LedGroup *group = &ledStrip[stripIndex].group[groupIndex];
    ColorWave *effect = &group->effect.colorWave;
    Color firstColor;
    uint8_t index;
    uint8_t speed;
    uint16_t *time = &ledGroupState[stripIndex][groupIndex].colorWave.time;
    uint8_t *colorIndex = &ledGroupState[stripIndex][groupIndex].colorWave.colorIndex;

    switch (effect->speed) {
        case EFFECT_SPEED_FAST:    speed = 192;  break;
        case EFFECT_SPEED_MIDDLE:  speed = 96;   break;
        case EFFECT_SPEED_SLOW:    speed = 48;   break;
        default:                                 break;
    }

    if (*time == 0 && effect->randomColor) {
        ledGenerateRandomColor(&effect->color[0]);
        ledGenerateRandomColor(&effect->color[1]);
    }

    firstColor = lightIntensity(effect->color[*colorIndex], ledStrip[stripIndex].brightness);

	for (index = 0; index < group->stopIndex - group->startIndex; index++) {
		uint16_t t = ((uint16_t)index << 8) - *time + 2048;

	    if (t < 2048 ) {
			Color colorToSet;
            colorToSet = colorWaveGetColor(firstColor, t);
            if (effect->forwardDirection) {
                stripArray[stripIndex][group->startIndex + index] = colorToSet;
            } else {
                stripArray[stripIndex][group->stopIndex - 1 - index] = colorToSet;
            }
		} else {
            Color colorEmpty = {0, 0, 0};
            if (effect->forwardDirection) {
                stripArray[stripIndex][group->startIndex + index] = colorEmpty;
            } else {
                stripArray[stripIndex][group->stopIndex - 1 - index] = colorEmpty;
            }
		}
	}

    *time = *time + speed;

	if (*time >= ((uint16_t)(group->stopIndex - group->startIndex) << 8) + 2048 - 256) {
        *time = 0;
        *colorIndex = *colorIndex ? 0 : 1;
    }
}


static void temperatureEffect(LedStripIndex stripIndex, uint8_t groupIndex)
{
    LedGroup *group = &ledStrip[stripIndex].group[groupIndex];
    TemperatureEffect *effect = &group->effect.temperatureEffect;
    Color color = {0, 0, 0};
    uint16_t temp;
    uint8_t  ledPosition;

#define EXTERNAL_TEMPERATURE_INDEX  0xFF
    if (effect->temperatureIndex == EXTERNAL_TEMPERATURE_INDEX) {
#undef EXTERNAL_TEMPERATURE_INDEX
        // if link didn't send external temperature more than 5 seconds -> set default temperature 40 degree
        if (timerGetTime() - ledGroupState[stripIndex][groupIndex].temperatureColor.timeStamp > 5000) {
            ledGroupState[stripIndex][groupIndex].temperatureColor.previousTemperature = BARBUDA_DEFAULT_TEMPERATURE;
            ledGroupState[stripIndex][groupIndex].temperatureColor.temperature = BARBUDA_DEFAULT_TEMPERATURE;
            temp = BARBUDA_DEFAULT_TEMPERATURE;
        } else {
            uint16_t *previousTemperature =  &ledGroupState[stripIndex][groupIndex].temperatureColor.previousTemperature;
            uint16_t temperature = ledGroupState[stripIndex][groupIndex].temperatureColor.temperature;

            if (*previousTemperature == 0) {
                *previousTemperature = temperature;
            }

            temp = *previousTemperature*0.95 + temperature*0.05;
            *previousTemperature = temp;
        }
    } else {
//        temp = analogMeasurementTemperatureFromAdcValue(temperatureGetInternal(effect->temperatureIndex));
    }

    if (0 <= temp && temp < effect->temperature[0]) {
        color = effect->color[0];
    } else if (effect->temperature[0] <= temp && temp < effect->temperature[1]) {
        color.r = (effect->color[0].r * ((uint32_t)effect->temperature[1] - temp) + effect->color[1].r * ((uint32_t)temp - effect->temperature[0])) / (effect->temperature[1] - effect->temperature[0]);
        color.g = (effect->color[0].g * ((uint32_t)effect->temperature[1] - temp) + effect->color[1].g * ((uint32_t)temp - effect->temperature[0])) / (effect->temperature[1] - effect->temperature[0]);
        color.b = (effect->color[0].b * ((uint32_t)effect->temperature[1] - temp) + effect->color[1].b * ((uint32_t)temp - effect->temperature[0])) / (effect->temperature[1] - effect->temperature[0]);
    } else if (effect->temperature[1] <= temp && temp < effect->temperature[2]) {
        color.r = (effect->color[1].r * ((uint32_t)effect->temperature[2] - temp) + effect->color[2].r * ((uint32_t)temp - effect->temperature[1])) / (effect->temperature[2] - effect->temperature[1]);
        color.g = (effect->color[1].g * ((uint32_t)effect->temperature[2] - temp) + effect->color[2].g * ((uint32_t)temp - effect->temperature[1])) / (effect->temperature[2] - effect->temperature[1]);
        color.b = (effect->color[1].b * ((uint32_t)effect->temperature[2] - temp) + effect->color[2].b * ((uint32_t)temp - effect->temperature[1])) / (effect->temperature[2] - effect->temperature[1]);
    } else if (temp >= effect->temperature[2]) {
        color = effect->color[2];
    }

    color = lightIntensity(color, ledStrip[stripIndex].brightness);

    for(ledPosition = group->startIndex; ledPosition < group->stopIndex; ++ledPosition) {
        stripArray[stripIndex][ledPosition] = color;
    }
}




Error ledSetLedType(LedStripIndex stripIndex, LedType ledType)
{
    if (stripIndex != LED_STRIP_1 && stripIndex != LED_STRIP_2) {
        return RES_INVALID_PORT_INDEX;
    }

    if (ledType != LED_TYPE_WS2812B && ledType != LED_TYPE_HD59731B) {
        return RES_INVALID_ARGUMENT;
    }

#ifdef JOTUN
    if (stripIndex == LED_STRIP_1 && ledType != LED_TYPE_WS2812B) {
        // on Jotun first led strip type only LED_TYPE_WS2812B allowed
        return RES_INVALID_ARGUMENT;
    }
#endif

    ledStrip[stripIndex].ledType = ledType;
    return RES_OK;
}

Error ledSetLedCount(LedStripIndex stripIndex, uint8_t count)
{
    if (stripIndex != LED_STRIP_1 && stripIndex != LED_STRIP_2) {
        return RES_INVALID_PORT_INDEX;
    }

    if (stripIndex == LED_STRIP_1 && count > LED_STRIP_1_MAX_LED_COUNT) {
        return RES_INVALID_ARGUMENT;
    }

    if (stripIndex == LED_STRIP_2 && count > LED_STRIP_2_MAX_LED_COUNT) {
        return RES_INVALID_ARGUMENT;
    }

    ledStrip[stripIndex].ledsCount = count;
    return RES_OK;
}

Error ledSetLedBrightness(LedStripIndex stripIndex, uint8_t brightness)
{
    if (stripIndex != LED_STRIP_1 && stripIndex != LED_STRIP_2) {
        return RES_INVALID_PORT_INDEX;
    }

    if (brightness > LED_BRIGHTNESS_MAX) {
        return RES_INVALID_ARGUMENT;
    }

    ledStrip[stripIndex].brightness = brightness;
    return RES_OK;
}

Error ledStripSetGroupExternalTemperature(LedStripIndex stripIndex, uint8_t groupNumber, uint16_t temperature)
{
    if (stripIndex != LED_STRIP_1 && stripIndex != LED_STRIP_2) {
        return RES_INVALID_PORT_INDEX;
    }

    if ((groupNumber >= LED_GROUP_MAX_COUNT) ||                   // not valid group number
        (groupNumber >= ledStrip[stripIndex].groupCount) ||                     // group number > than count of wrote groups
        (ledStrip[stripIndex].group[groupNumber].effectType != EFFECT_TEMPERATURE)) // that group does not play temperature effect
    {
        return RES_INVALID_ARGUMENT;
    }

    ledGroupState[stripIndex][groupNumber].temperatureColor.temperature = temperature;
    ledGroupState[stripIndex][groupNumber].temperatureColor.timeStamp = timerGetTime();
    return RES_OK;
}

Error ledStripClear(LedStripIndex stripIndex)
{
    if (stripIndex != LED_STRIP_1 && stripIndex != LED_STRIP_2) {
        return RES_INVALID_PORT_INDEX;
    }

    ledStrip[stripIndex].groupCount = 0;
    playEffect = false;
    return RES_OK;
}

Error ledSetStripMode(LedStripIndex stripIndex, LedStripMode mode)
{
    if (stripIndex != LED_STRIP_1 && stripIndex != LED_STRIP_2) {
        return RES_INVALID_PORT_INDEX;
    }

    if (mode != LED_STRIP_MODE_OFF && mode != LED_STRIP_MODE_PRESET && mode != LED_STRIP_MODE_DIRECT) {
        return RES_INVALID_ARGUMENT;
    }

    ledStrip[stripIndex].mode = mode;
    return RES_OK;
}

static void ledSetEffectInitialState(LedGroupState *state, Effect effectType)
{
    switch (effectType) {
        case EFFECT_RAINBOW_WAVE:
            state->rainbowWaveSettings.wheelPosition = 0;
            break;

        case EFFECT_COLOR_SHIFT:
            state->colorShiftSettings.position = 0;
            break;

        case EFFECT_COLOR_PULSE:
            state->colorPulse.time = 0;
            break;

        case EFFECT_COLOR_WAVE:
            state->colorWave.colorIndex = 0;
            state->colorWave.time = 0;
            break;

        case EFFECT_STATIC:
            break;

        case EFFECT_TEMPERATURE:
            state->temperatureColor.temperature = BARBUDA_DEFAULT_TEMPERATURE;// default temperature
            state->temperatureColor.previousTemperature = BARBUDA_DEFAULT_TEMPERATURE;
            state->temperatureColor.timeStamp = timerGetTime();
            break;

        case EFFECT_VISOR:
            state->visor.time1 = 0;
            state->visor.time2 = 0;
            break;

        case EFFECT_MARQUEE:
            state->marquee.time = 0;
            break;

        case EFFECT_STROBING:
            state->strobing.time = 0;
            state->strobing.colorIndex = 0;
            break;

        case EFFECT_SEQUENTIAL:
            state->sequential.time = 0;
            memset(&state->sequential.backgroundColor, 0, sizeof(state->sequential.backgroundColor));
            break;

        case EFFECT_RAINBOW:
            state->rainbowSettings.wheelPosition = 0;
            break;
    }
}

Error ledStripAppendGroup(LedStripIndex stripIndex, LedGroup *ledGroup)
{
    LedStrip *strip;

    if (stripIndex != LED_STRIP_1 && stripIndex != LED_STRIP_2) {
        return RES_INVALID_PORT_INDEX;
    }

    strip = &ledStrip[stripIndex];

    if ((stripIndex >= LED_STRIP_COUNT) || (strip->groupCount >= LED_GROUP_MAX_COUNT)) {
        return RES_INVALID_ARGUMENT;
    }

    strip->group[strip->groupCount] = *ledGroup;
    // initialize effect state structure
    ledSetEffectInitialState(&ledGroupState[stripIndex][strip->groupCount], ledGroup->effectType);
    strip->groupCount++;
    return RES_OK;
}

void ledAllStripArrayClear(void)
{
    memset(stripArray[LED_STRIP_1], 0, sizeof(stripArray[LED_STRIP_1]));
    memset(stripArray[LED_STRIP_2], 0, sizeof(stripArray[LED_STRIP_2]));
}

void ledSetAllEffectsInitialState(void)
{
    LedStripIndex stripIndex;
    uint8_t groupIndex;

    for (stripIndex = 0; stripIndex < LED_STRIP_COUNT; ++stripIndex) {
        for (groupIndex = 0; groupIndex < ledStrip[stripIndex].groupCount; ++groupIndex) {
            ledSetEffectInitialState(&ledGroupState[stripIndex][groupIndex], ledStrip[stripIndex].group[groupIndex].effectType);
        }
    }
}

void ledInit(void)
{
    LedGroup ledGroup;
    uint8_t i;
    uint8_t offset = 0;

    ledControllerInit(NULL);

    for (i = 0; i < LED_STRIP_COUNT; ++i) {
        ledStrip[i].mode = LED_STRIP_MODE_PRESET;
        ledStrip[i].brightness = 100;
        ledStrip[i].ledsCount = (i == LED_STRIP_1) ? LED_STRIP_1_MAX_LED_COUNT : LED_STRIP_2_MAX_LED_COUNT;
        ledStrip[i].ledType = LED_TYPE_WS2812B;
        ledStrip[i].groupCount = 0;
    }

    memset(&ledGroup, 0, sizeof(ledGroup));
    /*
    ledGroup.effectType = EFFECT_RAINBOW_WAVE;
    ledGroup.effect.rainbowWave.speed = EFFECT_SPEED_MIDDLE;
    ledGroup.effect.rainbowWave.forwardDirection = 1;
    */
    /*
    ledGroup.effectType = EFFECT_COLOR_SHIFT;
    ledGroup.effect.colorShift.speed = EFFECT_SPEED_MIDDLE;
    ledGroup.effect.colorShift.randomColor = 0;
    {
        Color color[2] = {{0, 0xff, 0}, {0, 0, 0xff}};
        ledGroup.effect.colorShift.color[0] = color[0];
        ledGroup.effect.colorShift.color[1] = color[1];
    }
    */
    /*
    ledGroup.effectType = EFFECT_RAINBOW;
    ledGroup.effect.rainbow.speed = EFFECT_SPEED_MIDDLE;
    */
    /*
    ledGroup.effectType = EFFECT_STATIC;
    {
        Color color = {0, 0xff, 0};
        ledGroup.effect.staticEffect.color = color;
    }
    */
    /*
    ledGroup.effectType = EFFECT_STROBING;
    ledGroup.effect.strobing.speed = EFFECT_SPEED_MIDDLE;
    ledGroup.effect.strobing.randomColor = 1;
    {
        Color color = {0, 0xff, 0};
        ledGroup.effect.strobing.color = color;
    }
    */
    /*
    ledGroup.effectType = EFFECT_MARQUEE;
    ledGroup.effect.marquee.speed = EFFECT_SPEED_MIDDLE;
    {
        Color color = {0, 0xff, 0};
        ledGroup.effect.marquee.color = color;
    }
    */
    /*
    ledGroup.effectType = EFFECT_SEQUENTIAL;
    ledGroup.effect.sequential.speed = EFFECT_SPEED_MIDDLE;
    ledGroup.effect.sequential.randomColor = 0;
    ledGroup.effect.sequential.forwardDirection = 1;
    {
        Color color = {0, 0xff, 0};
        ledGroup.effect.sequential.color = color;
    }
    */
    /*
    ledGroup.effectType = EFFECT_VISOR;
    ledGroup.effect.visor.speed = EFFECT_SPEED_MIDDLE;
    ledGroup.effect.visor.randomColor = 0;
    {
        Color color[2] = {{0, 0xff, 0}, {0, 0, 0xff}};
        ledGroup.effect.visor.color[0] = color[0];
        ledGroup.effect.visor.color[1] = color[1];
    }
    */
    /*
    ledGroup.effectType = EFFECT_COLOR_PULSE;
    ledGroup.effect.colorPulse.speed = EFFECT_SPEED_MIDDLE;
    ledGroup.effect.colorPulse.randomColor = 1;
    {
        Color color[2] = {{0, 0xff, 0}, {0, 0, 0xff}};
        ledGroup.effect.colorPulse.color[0] = color[0];
        ledGroup.effect.colorPulse.color[1] = color[1];
    }
    */
    ledGroup.effectType = EFFECT_COLOR_WAVE;
    ledGroup.effect.colorWave.speed = EFFECT_SPEED_FAST;
    ledGroup.effect.colorWave.randomColor = 1;
    ledGroup.effect.colorWave.forwardDirection = 1;
    {
        Color color[2] = {{0, 0xff, 0}, {0, 0, 0xff}};
        ledGroup.effect.colorWave.color[0] = color[0];
        ledGroup.effect.colorWave.color[1] = color[1];
    }

#ifdef JOTUN
    ledGroup.startIndex = 0;
    ledGroup.stopIndex = 4;
    ledStripAppendGroup(LED_STRIP_1, &ledGroup);

    ledGroup.startIndex = 4;
    ledGroup.stopIndex = 22;
    ledStripAppendGroup(LED_STRIP_1, &ledGroup);

    offset = 22;
#endif

    for (i = 0; i < 6; ++i) {
        ledGroup.startIndex = i * 12 + offset;
        ledGroup.stopIndex = (i + 1) * 12 + offset;
        ledStripAppendGroup(LED_STRIP_1, &ledGroup);
    }

    for (i = 0; i < 6; ++i) {
        ledGroup.startIndex = i * 12;
        ledGroup.stopIndex = (i + 1) * 12;
        ledStripAppendGroup(LED_STRIP_2, &ledGroup);
    }

    playEffect = true;
}

uint8_t *ledGetConfigurationLocation(uint16_t *length)
{
    *length = sizeof(ledStrip);
    return (uint8_t *)ledStrip;
}

static void ledProcessGroupEffect(LedStripIndex stripIndex, uint8_t groupIndex)
{
    LedGroup *group = &ledStrip[stripIndex].group[groupIndex];

    switch (group->effectType) {
        case EFFECT_RAINBOW_WAVE:  rainbowWave(stripIndex, groupIndex);        break;
        case EFFECT_COLOR_SHIFT:   colorShift(stripIndex, groupIndex);         break;
        case EFFECT_COLOR_PULSE:   colorPulse(stripIndex, groupIndex);         break;
        case EFFECT_COLOR_WAVE:    colorWave(stripIndex, groupIndex);          break;
        case EFFECT_STATIC:        colorStatic(stripIndex, groupIndex);        break;
        case EFFECT_TEMPERATURE:   temperatureEffect(stripIndex, groupIndex);  break;
        case EFFECT_VISOR:         visor(stripIndex, groupIndex);              break;
        case EFFECT_MARQUEE:       marquee(stripIndex, groupIndex);            break;
        case EFFECT_STROBING:      strobing(stripIndex, groupIndex);           break;
        case EFFECT_SEQUENTIAL:    sequential(stripIndex, groupIndex);         break;
        case EFFECT_RAINBOW:       rainbow(stripIndex, groupIndex);            break;
        default:                                                               break;
    }
}

void ledEnablePlayEffect(void)
{
    playEffect = true;
}

void ledPlayEffects(void)
{
    LedStripIndex stripIndex;

    ledAllStripArrayClear();

    if (playEffect == true) {
        for (stripIndex = 0; stripIndex < LED_STRIP_COUNT; ++stripIndex) {
            if (ledStrip[stripIndex].mode == LED_STRIP_MODE_PRESET) {
                uint8_t groupIndex;

                for (groupIndex = 0; groupIndex < ledStrip[stripIndex].groupCount; ++groupIndex) {
                    ledProcessGroupEffect(stripIndex, groupIndex);
                }
            } else if (ledStrip[stripIndex].mode == LED_STRIP_MODE_DIRECT) {

            } else if (ledStrip[stripIndex].mode == LED_STRIP_MODE_OFF) {

            }

        }
    }
    ledControllerSetLedData(LED_STRIP_1, ledStrip[LED_STRIP_1].ledType, (const uint32_t *) &stripArray[LED_STRIP_1], LED_STRIP_1_MAX_LED_COUNT);
    ledControllerSetLedData(LED_STRIP_2, ledStrip[LED_STRIP_2].ledType, (const uint32_t *) &stripArray[LED_STRIP_2], LED_STRIP_2_MAX_LED_COUNT);
}

void ledAllStripClear(void)
{
    ledAllStripArrayClear();
    ledControllerSetLedData(LED_STRIP_1, ledStrip[LED_STRIP_1].ledType, (const uint32_t *) &stripArray[LED_STRIP_1], LED_STRIP_1_MAX_LED_COUNT);
    ledControllerSetLedData(LED_STRIP_2, ledStrip[LED_STRIP_2].ledType, (const uint32_t *) &stripArray[LED_STRIP_2], LED_STRIP_2_MAX_LED_COUNT);
}
