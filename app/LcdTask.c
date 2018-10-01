#include "FreeRTOS.h"
#include "task.h"

#include "Lcd.h"

#include <stdint.h>

#include "./Anim/Anim.c"

#define LCD_TASK_PRIORITY      2
#define LCD_TASK_STACK_SIZE    200



static void lcdTaskFunction(void *pvParameters)
{

    lcdInit();
    for (;;) {
        //wait for next frame
        putPicture(image_data_imagesimage_data_images);
    }

}


void lcdTaskCreate(void)
{
    xTaskCreate(lcdTaskFunction, "Lcd", LCD_TASK_STACK_SIZE, NULL, LCD_TASK_PRIORITY, NULL);
}
