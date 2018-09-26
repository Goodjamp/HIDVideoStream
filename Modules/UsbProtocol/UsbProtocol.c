#include "UsbProtocol.h"
#include "FanSystem.h"
#include "FanController.h"
#include "AnalogMeasurementSystem.h"
#include "led.h"

#include <string.h>

#define UINT_16_FROM_BIG_ENDIAN(ptr) (uint16_t) ((*((uint8_t *) ptr) << 8) | *((uint8_t *) ptr))

typedef enum
{
    STATUS_OK                   = 0x00,
    STATUS_INVALID_COMMAND      = 0x01,
    STATUS_INVALID_PORT_INDEX   = 0x10,
    STATUS_SENSOR_ABSENT        = 0x11,
    STATUS_INVALID_ARGUMENT     = 0x12,
} BarbudaStatusCode;


typedef enum
{
    /* System commands */
    CMD_EMPTY                   = 0x00,
    CMD_READ_STATUS             = 0x01,
    CMD_READ_FIRMWARE_VERSION   = 0x02,
    CMD_READ_DEVICE_ID          = 0x03,
    CMD_WRITE_DEVICE_ID         = 0x04,
    CMD_START_FIRMWARE_UPDATE   = 0x05,
    CMD_READ_BOOTLOADER_VERSION = 0x06,
    CMD_WRITE_TEST_FLAG         = 0x07,

    /* Measurement commands */
    CMD_READ_TEMPERATURE_MASK   = 0x10,
    CMD_READ_TEMPERATURE_VALUE  = 0x11,
    CMD_READ_VOLTAGE_VALUE      = 0x12,
    /* FAN commands */
    CMD_READ_FAN_MASK           = 0x20,
    CMD_READ_FAN_SPEED          = 0x21,
    CMD_READ_FAN_POWER          = 0x22,
    CMD_WRITE_FAN_POWER         = 0x23,
    CMD_WRITE_FAN_SPEED         = 0x24,
    CMD_WRITE_FAN_CURVE         = 0x25,
    CMD_WRITE_FAN_EXTERNAL_TEMP = 0x26,
    CMD_WRITE_FAN_TEST_3        = 0x27,
    /* LED commands */
    CMD_READ_LED_STRIP_MASK     = 0x30,
    CMD_WRITE_LED_RGB_VALUE     = 0x31,
    CMD_WRITE_LED_COLOR_VALUES  = 0x32,
    CMD_WRITE_LED_TRIGGER       = 0x33,
    CMD_WRITE_LED_CLEAR         = 0x34,
    CMD_WRITE_LED_GROUP_SET     = 0x35,
    CMD_WRITE_LED_EXTERNAL_TEMP = 0x36,
    CMD_WRITE_LED_GROUPS_CLEAR  = 0x37,
    CMD_WRITE_LED_MODE          = 0x38,
    CMD_WRITE_LED_BRIGHTNESS    = 0x39,
    CMD_WRITE_LED_COUNT         = 0x3A,
    CMD_WRITE_LED_PORT_TYPE     = 0x3B,
    // PMBus
    CMD_PMBUS_WRITE             = 0x4A,
    CMD_PMBUS_READ              = 0x4B

} BarbudaCommandCode;


static void CMD_ReadStatus(uint8_t inReport[])
{
    inReport[0] = STATUS_OK;
}

static void CMD_GetFirmwareVersion(uint8_t inReport[])
{
    inReport[0] = STATUS_OK;
    //inReport[1] = BARBUDA_FIRMWARE_VERSION_MAJOR;
    //inReport[2] = BARBUDA_FIRMWARE_VERSION_MINOR;
    //inReport[3] = BARBUDA_FIRMWARE_VERSION_PATCH;
}

static void CMD_GetDeviceId(uint8_t inReport[])
{
    uint8_t deviceId[] = {1, 1, 1, 1};
    //systemGetDeviceId(&inReport[1]);
    inReport[0] = STATUS_OK;
    memcpy(&inReport[1], deviceId, sizeof(deviceId));
}

static void CMD_SetDeviceId(const uint8_t outReport[], uint8_t inReport[])
{
    //memcpy(deviceId, &outReport[1], sizeof(deviceId));
    /*
    if(!testFlag)
    {
        saveSystemInfo();
    }
    */
    //systemSetDeviceId(&outReport[1]);
    inReport[0] = STATUS_OK;

}

#if BOOTMODE
static void CMD_StartFirmwareUpdate(uint8_t inReport[])
{
    //scheduleEnterBootloader();
    inReport[0] = STATUS_OK;
}
#endif

static void CMD_GetBootloaderVersion(uint8_t inReport[])
{
    inReport[0] = STATUS_OK;
    //systemGetBootloaderVersion(&inReport[1]);
}

static void CMD_SetTestFlag(const uint8_t outReport[], uint8_t inReport[])
{
    /*
    if ((outReport[1] != 0) && (outReport[1] != 1))
    {
        inReport[0] = STATUS_INVALID_ARGUMENT;
    }
    else
    {
        //testFlag = outReport[1];
        inReport[0] = STATUS_OK;
    }
    */
}

static void CmdReadFanMask(uint8_t inReport[])
{
    inReport[0] = STATUS_OK;
    memset(&inReport[1], 2, 6);
}

static void CmdReadFanSpeed(const uint8_t outReport[], uint8_t inReport[])
{
    uint32_t fan = outReport[1];

    uint16_t speedRpm;
    inReport[0] = STATUS_OK;
    speedRpm = fanSystemGetMeanRpm(fan);

    if(inReport[0] == STATUS_OK)
    {
        inReport[1] = (uint8_t)(speedRpm >> 8);
        inReport[2] = (uint8_t)(speedRpm & 0xFF);
    }
}

static void CmdReadFanPower(const uint8_t outReport[], uint8_t inReport[])
{
    uint32_t fan = outReport[1];
    uint8_t power;
    inReport[0] = STATUS_OK;

    power = fanSystemGetPwm(fan) * 100 / fanControllerGetMaxPwmValue();

    if(inReport[0] == STATUS_OK)
    {
       inReport[1] = power;
    }

}

static void CMD_GetTemperatureMask(uint8_t inReport[])
{
    inReport[0] = STATUS_OK;
    inReport[1] = 1;
    inReport[2] = 1;
    inReport[3] = 0;
    inReport[4] = 0;
    inReport[5] = 0;
}

static void CMD_GetTemperatureValue(const uint8_t outReport[], uint8_t inReport[])
{
    uint8_t temperatureIndex = outReport[1];
    uint16_t temperature;

    temperature = analogMeasurementTemperatureFromAdcValue(temperatureGetInternal(temperatureIndex));

    inReport[0] = STATUS_OK;

    inReport[1] = (uint8_t)(temperature >> 8);
    inReport[2] = (uint8_t)(temperature & 0xFF);
}

static void CMD_GetVoltageValue(const uint8_t outReport[], uint8_t inReport[])
{

    uint8_t voltageIndex = outReport[1];
    uint16_t voltage;

    voltage = powerGetVoltage(voltageIndex);

    inReport[0] = STATUS_OK;
    inReport[1] = (uint8_t)(voltage >> 8);
    inReport[2] = (uint8_t)(voltage & 0xFF);
}

static void CMD_SetFanPower(const uint8_t outReport[], uint8_t inReport[])
{
    uint32_t fan = outReport[1];
    uint8_t fanPower = outReport[2];

    inReport[0] = STATUS_OK;
    fanSystemSetMode(fan, FAN_SYSTEM_MODE_PWM);
    fanSystemSetPwm(fan, fanPower * fanControllerGetMaxPwmValue() / 100);

}

static void CmdWriteFanSpeed(const uint8_t outReport[], uint8_t inReport[])
{
    uint32_t fan = outReport[1];
    uint16_t fanSpeed = ((uint16_t) outReport[2] << 8) | outReport[3];

    inReport[0] = STATUS_OK;
    fanSystemSetMode(fan, FAN_SYSTEM_MODE_RPM);
    fanSystemSetTargetRpm(fan, fanSpeed);
}

static void CMD_SetFanCurve(const uint8_t outReport[], uint8_t inReport[])
{
    uint8_t fanIndex = outReport[1];
    uint8_t temperatureIndex = outReport[2];

    FanSpeedTempCurve fanSpeedTempCurve = {
        .curvePoints = {
            {.rpm = UINT_16_FROM_BIG_ENDIAN(&outReport[15]), .temperature = UINT_16_FROM_BIG_ENDIAN(&outReport[3])},
            {.rpm = UINT_16_FROM_BIG_ENDIAN(&outReport[17]), .temperature = UINT_16_FROM_BIG_ENDIAN(&outReport[5])},
            {.rpm = UINT_16_FROM_BIG_ENDIAN(&outReport[19]), .temperature = UINT_16_FROM_BIG_ENDIAN(&outReport[7])},
            {.rpm = UINT_16_FROM_BIG_ENDIAN(&outReport[21]), .temperature = UINT_16_FROM_BIG_ENDIAN(&outReport[9])},
            {.rpm = UINT_16_FROM_BIG_ENDIAN(&outReport[23]), .temperature = UINT_16_FROM_BIG_ENDIAN(&outReport[11])},
            {.rpm = UINT_16_FROM_BIG_ENDIAN(&outReport[25]), .temperature = UINT_16_FROM_BIG_ENDIAN(&outReport[13])},
        },
        .curvePointCount = 6
    };

    inReport[0] = STATUS_OK;
    fanSystemSetMode(fanIndex, FAN_SYSTEM_MODE_CURVE);
    fanSystemSetCurveTemperatureIndex(fanIndex, temperatureIndex);
    fanSystemSetCurve(fanIndex, &fanSpeedTempCurve);
}

static void CMD_SetFanTest3(const uint8_t outReport[], uint8_t inReport[])
{
    //setFanToTest3(outReport[1]);
    inReport[0] = STATUS_OK;
}

static void CMD_SetFanExternalTemperature(const uint8_t outReport[], uint8_t inReport[])
{
    uint8_t fanIndex = outReport[1];

    uint16_t temperature = UINT_16_FROM_BIG_ENDIAN(&outReport[2]);
    inReport[0] = STATUS_OK;
    temperatureSetExternal(fanIndex, temperature);
}

static void cmdReadLedStripMask(uint8_t inReport[])
{
    inReport[0] = STATUS_OK;
    inReport[1] = 1;
    inReport[2] = 1;
}

static void cmdWriteLedRgbValue(const uint8_t outReport[], uint8_t inReport[])
{
    /*
    uint8_t ledStripIndex = outReport[1];
    uint8_t ledIndex = outReport[2];
    inReport[0] = checkLedStripStatus(ledStripIndex, ledIndex, 0);

    if (inReport[0] == STATUS_OK)
    {
        writeLEDValue(ledStripIndex, outReport);
    }
    */
}

static void cmdWriteLedColorValues(const uint8_t outReport[], uint8_t inReport[])
{
    /*
    uint8_t ledStripIndex = outReport[1];
    uint8_t startIndex = outReport[2];
    uint8_t lenght = outReport[3];
    inReport[0] = checkLedStripStatus(ledStripIndex, startIndex, lenght);

    if(inReport[0] == STATUS_OK)
    {
        writeLEDColorValues(ledStripIndex, outReport);
    }
    */
}

static void cmdWriteLedTrigger(const uint8_t outReport[], uint8_t inReport[])
{
    // Write led setting to eeprom
    //uint16_t length = 0;
    //uint8_t *settings;
    //settings = ledGetConfigurationLocation(&length);
    //systemStoreSettings(SYSTEM_LED_SETTINGS, settings, length);

    ledEnablePlayEffect();
    inReport[0] = STATUS_OK;
    /*
    uint8_t ledStripIndex = outReport[1];
    inReport[0] = checkLedStripStatus(ledStripIndex, 0,0);

    if(inReport[0] == STATUS_OK)
    {
        if(getLedMode(ledStripIndex) == LED_MODE_PRESET)
        {
            if(!testFlag)
            {
                saveSystemInfo();
                saveLedInfo(ledStripIndex);
            }
        }
        else
        {
            setTrig(ledStripIndex, TRIG_ON);
        }
    }
    */
}

static void cmdWriteLedClear(const uint8_t outReport[], uint8_t inReport[])
{
    /*
    uint8_t ledStripIndex = outReport[1];
    inReport[0] = checkLedStripStatus(ledStripIndex, 0,0);

    if(inReport[0] == STATUS_OK)
    {
        writeLEDClear(ledStripIndex);
    }
    */
}

static void cmdWriteLedGroupSet(const uint8_t outReport[], uint8_t inReport[])
{
    LedGroup ledGroup;
    LedStripIndex stripIndex = outReport[1];
    Color color[3];
    uint16_t temperature[3];
    EffectSpeed speed = outReport[5];
    uint8_t forwardDirection = outReport[6];
    uint8_t randomColor = outReport[7];
    uint8_t temperatureIndex = outReport[8];

    memset(&ledGroup, 0, sizeof(ledGroup));
    ledGroup.startIndex = outReport[2];
    ledGroup.stopIndex = outReport[2] + outReport[3];
    ledGroup.effectType = outReport[4];
    color[0].r = outReport[9];
    color[0].g = outReport[10];
    color[0].b = outReport[11];
    color[1].r = outReport[12];
    color[1].g = outReport[13];
    color[1].b = outReport[14];
    color[2].r = outReport[15];
    color[2].g = outReport[16];
    color[2].b = outReport[17];
    temperature[0] = ((uint16_t) outReport[18] << 8) | outReport[19];
    temperature[1] = ((uint16_t) outReport[20] << 8) | outReport[21];
    temperature[2] = ((uint16_t) outReport[22] << 8) | outReport[23];

    if (randomColor) {
        ledGenerateRandomColor(&color[0]);
        ledGenerateRandomColor(&color[1]);
    }

    switch (ledGroup.effectType) {
        case EFFECT_RAINBOW_WAVE:
            ledGroup.effect.rainbowWave.speed = speed;
            ledGroup.effect.rainbowWave.forwardDirection = forwardDirection;
            break;

        case EFFECT_COLOR_SHIFT:
            ledGroup.effect.colorShift.speed = speed;
            ledGroup.effect.colorShift.randomColor = randomColor;
            ledGroup.effect.colorShift.color[0] = color[0];
            ledGroup.effect.colorShift.color[1] = color[1];
            break;

        case EFFECT_COLOR_PULSE:
            ledGroup.effect.colorPulse.speed = speed;
            ledGroup.effect.colorPulse.randomColor = randomColor;
            ledGroup.effect.colorPulse.color[0] = color[0];
            ledGroup.effect.colorPulse.color[1] = color[1];
            break;

        case EFFECT_COLOR_WAVE:
            ledGroup.effect.colorWave.speed = speed;
            ledGroup.effect.colorWave.forwardDirection = forwardDirection;
            ledGroup.effect.colorWave.randomColor = randomColor;
            ledGroup.effect.colorWave.color[0] = color[0];
            ledGroup.effect.colorWave.color[1] = color[1];
            break;

        case EFFECT_STATIC:
            ledGroup.effect.staticEffect.color = color[0];
            break;

        case EFFECT_TEMPERATURE:
            ledGroup.effect.temperatureEffect.temperatureIndex = temperatureIndex;
            ledGroup.effect.temperatureEffect.color[0] = color[0];
            ledGroup.effect.temperatureEffect.color[1] = color[1];
            ledGroup.effect.temperatureEffect.color[2] = color[2];
            ledGroup.effect.temperatureEffect.temperature[0] = temperature[0];
            ledGroup.effect.temperatureEffect.temperature[1] = temperature[1];
            ledGroup.effect.temperatureEffect.temperature[2] = temperature[2];
            break;

        case EFFECT_VISOR:
            ledGroup.effect.visor.speed = speed;
            ledGroup.effect.visor.randomColor = randomColor;
            ledGroup.effect.visor.color[0] = color[0];
            ledGroup.effect.visor.color[1] = color[1];
            break;

        case EFFECT_MARQUEE:
            ledGroup.effect.marquee.speed = speed;
            ledGroup.effect.marquee.color = color[0];
            break;

        case EFFECT_STROBING:
            ledGroup.effect.strobing.speed = speed;
            ledGroup.effect.strobing.randomColor = randomColor;
            ledGroup.effect.strobing.color[0] = color[0];
            ledGroup.effect.strobing.color[1] = color[1];
            break;

        case EFFECT_SEQUENTIAL:
            ledGroup.effect.sequential.speed = speed;
            ledGroup.effect.sequential.forwardDirection = forwardDirection;
            ledGroup.effect.sequential.randomColor = randomColor;
            ledGroup.effect.sequential.color = color[0];
            break;

        case EFFECT_RAINBOW:
            ledGroup.effect.rainbow.speed = speed;
            break;

        default:
            inReport[0] = STATUS_INVALID_ARGUMENT;
            return;
    }

    inReport[0] = ledStripAppendGroup(stripIndex, &ledGroup);
}

static void cmdWriteLedMode(const uint8_t outReport[], uint8_t inReport[])
{
    LedStripIndex stripIndex = outReport[1];
    LedStripMode ledMode = outReport[2];
    inReport[0] = ledSetStripMode(stripIndex, ledMode);
}

static void cmdWriteLedGroupsClear(const uint8_t outReport[], uint8_t inReport[])
{
    LedStripIndex stripIndex = outReport[1];
    inReport[0] = ledStripClear(stripIndex);
    /*
    if (inReport[0] == STATUS_OK)
    {
        ledGroupsClear(stripIndex);
        setLedStripOnOff(stripIndex, 1);
        setLedMode(stripIndex, LED_MODE_OFF);
        setTrig(stripIndex, TRIG_OFF);
    }
    */
}

static void cmdWriteLedExternalTemp(const uint8_t outReport[], uint8_t inReport[])
{
    LedStripIndex stripIndex = outReport[1];
    uint8_t groupIndex = outReport[2];
    uint16_t temperature = ((uint16_t) outReport[3] << 8) | outReport[4];
    inReport[0] = ledStripSetGroupExternalTemperature(stripIndex, groupIndex, temperature);
}

static void cmdWriteLedBrightness(const uint8_t outReport[], uint8_t inReport[])
{
    LedStripIndex stripIndex = outReport[1];
    uint8_t brightness = outReport[2];
    inReport[0] = ledSetLedBrightness(stripIndex, brightness);
}

static void cmdWriteLedCount(const uint8_t outReport[], uint8_t inReport[])
{
    LedStripIndex stripIndex = outReport[1];
    uint8_t ledCount = outReport[2];
    inReport[0] = ledSetLedCount(stripIndex, ledCount);
    // Also this command clear all PRESET groups.
    ledStripClear(stripIndex);
}

static void cmdWriteLedPortType(const uint8_t outReport[], uint8_t inReport[])
{
    LedStripIndex stripIndex = outReport[1];
    LedType ledType = outReport[2] - 1;

    inReport[0] = ledSetLedType(stripIndex, ledType);
}

static void cmdPmbusWrite(const uint8_t outReport[], uint8_t inReport[])
{
    /*
    uint8_t addr = outReport[1];
    uint8_t cmdDataLength = outReport[2];
    uint8_t dataLength = outReport[3];
*/
    //pmBusWrite(addr, &outReport[4], cmdDataLength, &outReport[4 + cmdDataLength], dataLength);
    inReport[0] = 0xAB;
}

static void cmdPmbusRead(const uint8_t outReport[], uint8_t inReport[])
{
    /*
    uint8_t addr = outReport[1];
    uint8_t cmdDataLength = outReport[2];
    uint8_t dataLength = outReport[3];
*/
    //pmBusRead(addr, &outReport[4], cmdDataLength, &inReport[1], dataLength);
    inReport[0] = 0xBB;
}

// CMD processing
void HID_ProcessReport(const uint8_t outReport[], uint8_t inReport[])
{
    BarbudaCommandCode cmd = (BarbudaCommandCode)outReport[0];
    memset(inReport, 0, 16);

    switch (cmd)
    {
    case CMD_READ_STATUS:
        CMD_ReadStatus(inReport);
        break;

    case CMD_READ_FIRMWARE_VERSION:
        CMD_GetFirmwareVersion(inReport);
        break;

    case CMD_READ_DEVICE_ID:
        CMD_GetDeviceId(inReport);
        break;

    case CMD_WRITE_DEVICE_ID:
        CMD_SetDeviceId(outReport, inReport);
        break;

#if BOOTMODE
    case CMD_START_FIRMWARE_UPDATE:
        CMD_StartFirmwareUpdate(inReport);
        break;
#endif

    case CMD_READ_BOOTLOADER_VERSION:
        CMD_GetBootloaderVersion(inReport);
        break;

    case CMD_WRITE_TEST_FLAG:
        CMD_SetTestFlag(outReport, inReport);
        break;

    case CMD_READ_FAN_MASK:
        CmdReadFanMask(inReport);
        break;

    case CMD_READ_FAN_SPEED:
        CmdReadFanSpeed(outReport, inReport);
        break;

    case CMD_READ_FAN_POWER:
        CmdReadFanPower(outReport, inReport);
        break;

    case CMD_READ_TEMPERATURE_MASK:
        CMD_GetTemperatureMask(inReport);
        break;

    case CMD_READ_TEMPERATURE_VALUE:
        CMD_GetTemperatureValue(outReport, inReport);
        break;

    case CMD_READ_VOLTAGE_VALUE:
        CMD_GetVoltageValue(outReport, inReport);
        break;

    case CMD_WRITE_FAN_POWER:
        CMD_SetFanPower(outReport, inReport);
        break;

    case CMD_WRITE_FAN_SPEED:
        CmdWriteFanSpeed(outReport, inReport);
        break;

    case CMD_WRITE_FAN_CURVE:
        CMD_SetFanCurve(outReport, inReport);
        break;

    case CMD_WRITE_FAN_TEST_3:
        CMD_SetFanTest3(outReport, inReport);
        break;

    case CMD_WRITE_FAN_EXTERNAL_TEMP:
        CMD_SetFanExternalTemperature(outReport, inReport);
        break;

    case CMD_READ_LED_STRIP_MASK:
        cmdReadLedStripMask(inReport);
        break;

    case CMD_WRITE_LED_RGB_VALUE:
        cmdWriteLedRgbValue(outReport, inReport);
        break;

    case CMD_WRITE_LED_COLOR_VALUES:
        cmdWriteLedColorValues(outReport, inReport);
        break;

    case CMD_WRITE_LED_TRIGGER:
        cmdWriteLedTrigger(outReport, inReport);
        break;

    case CMD_WRITE_LED_CLEAR:
        cmdWriteLedClear(outReport, inReport);
        break;

    case CMD_WRITE_LED_GROUP_SET:
        cmdWriteLedGroupSet(outReport, inReport);
        break;

    case CMD_WRITE_LED_MODE:
        cmdWriteLedMode(outReport, inReport);
        break;

    case CMD_WRITE_LED_GROUPS_CLEAR:
        cmdWriteLedGroupsClear(outReport, inReport);
        break;

    case CMD_WRITE_LED_EXTERNAL_TEMP:
        cmdWriteLedExternalTemp(outReport, inReport);
        break;

    case CMD_WRITE_LED_BRIGHTNESS:
        cmdWriteLedBrightness(outReport, inReport);
        break;

    case CMD_WRITE_LED_COUNT:
        cmdWriteLedCount(outReport, inReport);
        break;

    case CMD_WRITE_LED_PORT_TYPE:
        cmdWriteLedPortType(outReport, inReport);
        break;

    case CMD_PMBUS_WRITE:
        cmdPmbusWrite(outReport, inReport);
        break;

    case CMD_PMBUS_READ:
        cmdPmbusRead(outReport, inReport);
        break;

    default:
        inReport[0] = STATUS_INVALID_COMMAND;
        break;
    }
}
