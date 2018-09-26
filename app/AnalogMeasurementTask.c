#include "AnalogMeasurementSystem.h"
#include "AnalogMeasurementHal.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include <stdint.h>

#define ANALOG_MEASUREMENT_TASK_PRIORITY                    1
#define ANALOG_MEASUREMENT_TASK_STACK_SIZE                  100
#define ANALOG_MEASUREMENT_TASK_PERIOD                      20

static void analogMeasurementTaskFunction(void *pvParameters)
{
    analogMeasurementSystemInit();
    for (;;) {
        analogMeasurementSystemStartConversion();
        vTaskDelay(ANALOG_MEASUREMENT_TASK_PERIOD);
    }
}

void analogMeasurementTaskCreate(void)
{
    xTaskCreate(analogMeasurementTaskFunction, "analog", ANALOG_MEASUREMENT_TASK_STACK_SIZE, NULL, ANALOG_MEASUREMENT_TASK_PRIORITY, NULL);
}
