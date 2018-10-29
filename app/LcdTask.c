#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "Lcd.h"

#include <stdint.h>

#include "./Anim/Anim.c"

#define LCD_TASK_PRIORITY      2
#define LCD_TASK_STACK_SIZE    200

static SemaphoreHandle_t lcdSemaphore = NULL;

struct
{
    uint8_t *frameBuff;
}frameProcessing;


void setNewFrame(uint8_t frameBuff[])
{
    frameProcessing.frameBuff = frameBuff;
    xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(lcdSemaphore, xHigherPriorityTaskWoken);
}


static void lcdTaskFunction(void *pvParameters)
{

    lcdInit();
    lcdSemaphore = xSemaphoreCreateBinary();
    for (;;) {
        if (xSemaphoreTake(lcdSemaphore, REASONABLE_LONG_TIME) == pdTRUE) {
            putPicture(image_data_imagesimage_data_images);
        }
    }

}


void lcdTaskCreate(void)
{
    xTaskCreate(lcdTaskFunction, "Lcd", LCD_TASK_STACK_SIZE, NULL, LCD_TASK_PRIORITY, NULL);
}
