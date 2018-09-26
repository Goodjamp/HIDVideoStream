#include "FreeRTOS.h"
#include "task.h"

#include "Lcd.h"

#include <stdint.h>

#include "./Anim/Anim.c"

#define LCD_TASK_PRIORITY      2
#define LCD_TASK_STACK_SIZE    200

#define ANIME_3

#ifdef ANIME_1
uint8_t const *anime_1[] =
{
    image_data_01_0016_Layer1,
    image_data_01_0015_Layer2,
    image_data_01_0014_Layer3,
    image_data_01_0013_Layer4,
    image_data_01_0012_Layer5,
    image_data_01_0011_Layer6,
    image_data_01_0010_Layer7,
    //image_data_01_0009_Layer8,
    image_data_01_0008_Layer9,
    //image_data_01_0007_Layer10,
    image_data_01_0006_Layer11,
    //image_data_01_0005_Layer12,
    image_data_01_0004_Layer13,
   // image_data_01_0003_Layer14,
    image_data_01_0002_Layer15,
    image_data_01_0001_Layer16,
    image_data_01_0000_Layer17,
};
uint8_t numFrame = sizeof(anime_1) / sizeof(anime_1[0]);
#endif

#ifdef ANIME_2
uint8_t const *anime_2[] =
{
    image_data_02_0009_Layer1,
    image_data_02_0008_Layer2,
    image_data_02_0007_Layer3,
    image_data_02_0006_Layer4,
    image_data_02_0005_Layer5,
    image_data_02_0004_Layer6,
    image_data_02_0003_Layer7,
    image_data_02_0002_Layer8,
    image_data_02_0001_Layer9,
    image_data_02_0000_Layer10,
};
uint8_t numFrame = sizeof(anime_2) / sizeof(anime_2[0]);
#endif

#ifdef ANIME_3
uint8_t const *anime_3[] =
{
    image_data_03_0017_Layer1,
    //image_data_03_0016_Layer2,
    image_data_03_0015_Layer3,
    //image_data_03_0014_Layer4,
    image_data_03_0013_Layer5,
    image_data_03_0012_Layer6,
    image_data_03_0011_Layer7,
    image_data_03_0010_Layer8,
    image_data_03_0009_Layer9,
    //image_data_03_0008_Layer10,
    image_data_03_0007_Layer11,
    //image_data_03_0006_Layer12,
    image_data_03_0005_Layer13,
    //image_data_03_0004_Layer14,
    image_data_03_0003_Layer15,
    //image_data_03_0002_Layer16,
    image_data_03_0001_Layer17,
    image_data_03_0000_Layer18,
};
uint8_t numFrame = sizeof(anime_3) / sizeof(anime_3[0]);
#endif

uint8_t frameCnt;

const uint8_t zerroArray[32768] = {[0 ... (32768-1)] = 0};


static void lcdTaskFunction(void *pvParameters)
{

    lcdInit();
    for (;;) {
       // for(frameCnt = 0; frameCnt < numFrame; frameCnt++)
      //  {
            #ifdef ANIME_1
           // putPicture(anime_1[frameCnt]);
            #endif
            #ifdef ANIME_2
          //  putPicture(anime_2[frameCnt]);
            #endif
            #ifdef ANIME_3
          //  putPicture(anime_3[frameCnt]);
            #endif

         //   vTaskDelay(40);
      //  }
        putPicture(image_data_imagesimage_data_images);
        vTaskDelay(1000);
        putPicture(zerroArray);
        vTaskDelay(1);
        /*putPicture(image_data_1);
        vTaskDelay(2000);
        putPicture(image_data_2);
        vTaskDelay(2000);
        putPicture(image_data_3);
        vTaskDelay(2000);
        putPicture(image_data_4);
        vTaskDelay(2000);
        putPicture(image_data_5);
        vTaskDelay(2000);
        putPicture(image_data_6);
        vTaskDelay(2000);
        putPicture(image_data_7);
        vTaskDelay(2000);
        putPicture(image_data_8);
        vTaskDelay(2000);
        putPicture(image_data_9);
        vTaskDelay(2000);
        putPicture(image_data_10);
        vTaskDelay(2000);
        putPicture(image_data_11);
        vTaskDelay(2000);
        putPicture(image_data_12);
        vTaskDelay(2000);*/
    }

}

void lcdTaskCreate(void)
{
    xTaskCreate(lcdTaskFunction, "Lcd", LCD_TASK_STACK_SIZE, NULL, LCD_TASK_PRIORITY, NULL);
}
