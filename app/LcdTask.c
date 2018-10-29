#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "Lcd.h"

#include <stdint.h>

#include "./Anim/Anim.c"

#define LCD_TASK_PRIORITY      2
#define LCD_TASK_STACK_SIZE    200
#define START_IND_SUMBOL      "START"

#pragma pack(push,1)
typedef struct
{
    uint8_t commandMarker[sizeof(START_IND_SUMBOL) - 1];
    uint32_t size;
    uint8_t payload[];
} sendFrameCommandT;
#pragma pack(pop)


static SemaphoreHandle_t lcdSemaphore = NULL;

struct
{
    uint8_t *frameBuff;
}frameProcessing;


void setNewFrame(uint8_t frameBuff[])
{
    static BaseType_t xHigherPriorityTaskWoken;
    sendFrameCommandT *frameCommand =  (sendFrameCommandT*)frameBuff;
    frameProcessing.frameBuff = frameCommand->payload;
    xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(lcdSemaphore, &xHigherPriorityTaskWoken);
}


static void lcdTaskFunction(void *pvParameters)
{

    lcdInit();
    lcdSemaphore = xSemaphoreCreateBinary();
    putPicture(image_data_imagesimage_data_images);
    for (;;) {
        if (xSemaphoreTake(lcdSemaphore, portMAX_DELAY ) == pdTRUE) {
            //putPicture(image_data_imagesimage_data_images);
            putPicture(frameProcessing.frameBuff);
        }
    }

}


void lcdTaskCreate(void)
{
    xTaskCreate(lcdTaskFunction, "Lcd", LCD_TASK_STACK_SIZE, NULL, LCD_TASK_PRIORITY, NULL);
}
