#include "LightingTask.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "LedController.h"
#include "Led.h"

#include <stdint.h>


#define LIGHTING_TASK_PRIORITY      2
#define LIGHTING_TASK_STACK_SIZE    100

const uint32_t rgbData0[] = {
    0x000000FF, 0x000000FF, 0x000000FF, 0x000000FF, 0x000000FF,
    0x000000FF, 0x000000FF, 0x000000FF, 0x000000FF, 0x000000FF,
    0x000000FF, 0x000000FF, 0x000000FF, 0x000000FF, 0x000000FF,
    0x000000FF, 0x000000FF, 0x000000FF, 0x000000FF, 0x000000FF
};

const uint32_t rgbData1[] = {
    0x0000FF00, 0x0000FF00, 0x0000FF00, 0x0000FF00, 0x0000FF00,
    0x0000FF00, 0x0000FF00, 0x0000FF00, 0x0000FF00, 0x0000FF00,
    0x0000FF00, 0x0000FF00, 0x0000FF00, 0x0000FF00, 0x0000FF00,
    0x0000FF00, 0x0000FF00, 0x0000FF00, 0x0000FF00, 0x0000FF00
};

const uint32_t rgbData2[] = {
    0x00FF0000, 0x00FF0000, 0x00FF0000, 0x00FF0000, 0x00FF0000,
    0x00FF0000, 0x00FF0000, 0x00FF0000, 0x00FF0000, 0x00FF0000,
    0x00FF0000, 0x00FF0000, 0x00FF0000, 0x00FF0000, 0x00FF0000,
    0x00FF0000, 0x00FF0000, 0x00FF0000, 0x00FF0000, 0x00FF0000,
};

SemaphoreHandle_t ledDriverReadySemaphore = NULL;

void onLedDriverReady(void)
{
    static BaseType_t xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;

    xSemaphoreGiveFromISR( ledDriverReadySemaphore, &xHigherPriorityTaskWoken );

    if (xHigherPriorityTaskWoken != pdFALSE) {
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }

}

static void ligthingTaskFunction(void *pvParameters)
{
//    ledDriverReadySemaphore = xSemaphoreCreateBinary();

//    ledControllerInit(onLedDriverReady);
    ledInit();
    for (;;) {
        ledPlayEffects();
        vTaskDelay(40);
    }
}

void lightingTaskCreate(void)
{
    xTaskCreate(ligthingTaskFunction, "Light", LIGHTING_TASK_STACK_SIZE, NULL, LIGHTING_TASK_PRIORITY, NULL);
}
