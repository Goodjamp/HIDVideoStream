#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "Lcd.h"
#include "HalIFlash.h"

#include <stdint.h>

#include "./Anim/Anim.c"

#define LCD_TASK_PRIORITY      2
#define LCD_TASK_STACK_SIZE    200
#define START_IND_SUMBOL      "START"
/*play Flash buffer description*/
#define SUB_BUFF_SIZE          1024
#define SUB_FRAME_QUANTITY     32
#define SUB_BUFF_QUANTITY      2
#define VIDEO_BASE_ADDRESS     4096
#define VIDEO_FRAME_SIZE       32768

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

void lcdProcessingCB(bool state);

uint8_t playFlashSubBuff[SUB_BUFF_QUANTITY][SUB_BUFF_SIZE];

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
    uint16_t frameNumber;
    uint16_t subFrameNumber;
    uint16_t subBuffNumber;
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
    .playFlashState.subFrameNumber = 0,
    .playFlashState.frameNumber    = 0,
    .playFlashState.subBuffNumber  = 0,
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


void lcdProcessingCB(bool state)
{
    //send Current Sub Frame
    lcdSendNextSubFrame(playFlashSubBuff[playerH.playFlashState.subBuffNumber], SUB_BUFF_SIZE, lcdProcessingCB);
    if(playerH.playFlashState.subBuffNumber >= (SUB_FRAME_QUANTITY - 1))
    {
        return; // sending full frame was completing
    }
    //calculate next buff number
    playerH.playFlashState.subBuffNumber++;
    playerH.playFlashState.subBuffNumber = playerH.playFlashState.subBuffNumber & (SUB_BUFF_QUANTITY - 1);
    //start read next sub frame (should be complete before complete send current buff in LCD !!)
    flashReadData(playFlashSubBuff[playerH.playFlashState.subBuffNumber],
                                   SUB_BUFF_SIZE,
                                   VIDEO_BASE_ADDRESS + playerH.playFlashState.frameNumber * VIDEO_FRAME_SIZE + playerH.playFlashState.subBuffNumber * SUB_BUFF_SIZE);
}


static void playFlashProcessing(void)
{
    //read first sub frame with blocking
    flashReadData(playFlashSubBuff[playerH.playFlashState.subBuffNumber],
                     SUB_BUFF_SIZE,
                     VIDEO_BASE_ADDRESS + playerH.playFlashState.frameNumber * VIDEO_FRAME_SIZE + playerH.playFlashState.subBuffNumber * SUB_BUFF_SIZE);
    // wait for read complete
    while(flashGetState() == FLASH_BUSSY){}
    if(flashGetState() != FLASH_OK)
    {
        return;
    }
    lcdSendFirstSubFrame(playFlashSubBuff[playerH.playFlashState.subBuffNumber], SUB_BUFF_SIZE, lcdProcessingCB);
    //calculate next buff number
    playerH.playFlashState.subBuffNumber++;
    playerH.playFlashState.subBuffNumber = playerH.playFlashState.subBuffNumber & (SUB_BUFF_QUANTITY - 1);
    //start read next sub frame (should be complete before complete send current buff in LCD !!)
    flashReadData(playFlashSubBuff[playerH.playFlashState.subBuffNumber],
                                   SUB_BUFF_SIZE,
                                   VIDEO_BASE_ADDRESS + playerH.playFlashState.frameNumber * VIDEO_FRAME_SIZE + playerH.playFlashState.subBuffNumber * SUB_BUFF_SIZE);
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
