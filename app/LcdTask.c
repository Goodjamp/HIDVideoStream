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



/*
static void lcdTaskFunction(void *pvParameters)
{

    lcdInit();
    lcdSemaphore = xSemaphoreCreateBinary();
    putPicture(image_data_02_0000_Layer10);
    for (;;) {
        if (xSemaphoreTake(lcdSemaphore, portMAX_DELAY ) == pdTRUE) {
            //putPicture(image_data_imagesimage_data_images);
            putPicture(frameProcessing.frameBuff);
        }
    }

}
*/

/*GET FRAME FROM FLASH FUNCTION PROTOTYPE*/

void readFlashFrame(uint16_t videoIndex, uint16_t videoFrameCnt);


typedef enum
{
    PLAY_FLASH,
    PLAY_DIRECT,
}playSourceT;


typedef enum
{
    PLAYER_PLAY,
    PLAYER_STOP,
}playerStateT;


struct playFlashStateT
{
    uint16_t frameCnt;
};


struct playDirectStateT
{
    uint16_t reserved;
};


struct
{
    playSourceT             playSource;
    playerStateT            playerState;
    struct playFlashStateT  playFlashState;
    struct playDirectStateT playDirectState;
}playerH =
{
    .playSource  = PLAY_DIRECT,
    .playerState = PLAYER_STOP,
};


void setNewFrame(uint8_t frameBuff[])
{
    static BaseType_t xHigherPriorityTaskWoken;
    sendFrameCommandT *frameCommand =  (sendFrameCommandT*)frameBuff;
    frameProcessing.frameBuff = frameCommand->payload;
    xHigherPriorityTaskWoken = pdFALSE;
    playerH.playerState = PLAYER_PLAY;
    xSemaphoreGiveFromISR(lcdSemaphore, &xHigherPriorityTaskWoken);
}



void playFlashProcessing(void)
{

}


void playDirectProcessing(void)
{
    putPicture(frameProcessing.frameBuff);
}



static void lcdTaskFunction(void *pvParameters)
{

    lcdInit();
    lcdSemaphore = xSemaphoreCreateBinary();
    putPicture(image_data_02_0000_Layer10);
    for (;;) {
       if (xSemaphoreTake(lcdSemaphore, portMAX_DELAY ) != pdTRUE)
       {
           continue;
       }
       if(playerH.playerState != PLAYER_PLAY)
       {
           continue;
       }

       switch(playerH.playSource)
       {
           case PLAY_FLASH:
               playFlashProcessing();
               break;
           case PLAY_DIRECT:
               playDirectProcessing();
               break;
       }
    }

}







void lcdTaskCreate(void)
{
    xTaskCreate(lcdTaskFunction, "Lcd", LCD_TASK_STACK_SIZE, NULL, LCD_TASK_PRIORITY, NULL);
}
