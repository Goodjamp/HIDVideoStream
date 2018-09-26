#include "FanTask.h"

#include "FreeRTOS.h"
#include "task.h"

#include "FanSystem.h"
#include "FanController.h"
#include "AnalogMeasurementSystem.h"

#include <stdint.h>

#define FAN_TASK_PRIORITY           1
#define FAN_TASK_STACK_SIZE         100
#define FAN_TASK_CALL_PERIOD        50

#define TEMPERATURE_INDEX_MAX       0xFF

static uint32_t fanGetTemparature(uint32_t temperatureIndex, uint32_t fanIndex)
{
    if (temperatureIndex == TEMPERATURE_INDEX_MAX)
        return temperatureGetExternal(fanIndex);
    else
        return temperatureGetInternal(temperatureIndex);
}

static uint32_t getTimestamp(void)
{
    return xTaskGetTickCount();
}

static void onFanSystemEvent(void)
{

}

static void fanTaskFunction(void *pvParameters)
{
    fanSystemInit(fanGetTemparature, getTimestamp, onFanSystemEvent);
    uint32_t previousTickCount = xTaskGetTickCount();
    fanSystemStartFanDetection();
    for (;;) {
        fanSystemTaskRun();
        vTaskDelayUntil(&previousTickCount, FAN_TASK_CALL_PERIOD);
    }
}

void fanTaskCreate(void)
{
    xTaskCreate(fanTaskFunction, "Fan", FAN_TASK_STACK_SIZE, NULL, FAN_TASK_PRIORITY, NULL);
}
