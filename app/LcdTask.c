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
#define VIDEO_BLOCK_ADDRESS    5
#define FLASH_BLOCK_SIZE       32768
#define VIDEO_FRAME_SIZE       32768

#pragma pack(push,1)
typedef struct
{
    uint8_t commandMarker[sizeof(START_IND_SUMBOL)];
    uint32_t size;
    uint8_t payload[];
} sendFrameCommandT;
#pragma pack(pop)


static SemaphoreHandle_t lcdSemaphore = NULL;

struct
{
    uint8_t *frameBuff;
}frameProcessing;

void lcdProcessingCB(bool state);

uint8_t playFlashSubBuff[SUB_BUFF_QUANTITY][SUB_BUFF_SIZE + 4];

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
    if(playerH.playFlashState.subFrameNumber >= SUB_FRAME_QUANTITY )
    {
        return; // sending full frame was completing
    }
    putSubFrameNext(&playFlashSubBuff[playerH.playFlashState.subBuffNumber], SUB_BUFF_SIZE, lcdProcessingCB);
    if(playerH.playFlashState.subFrameNumber >= (SUB_FRAME_QUANTITY - 1) )
    {
        playerH.playFlashState.subFrameNumber++;
        return; // sending full frame was completing
    }
    //calculate next buff number
    playerH.playFlashState.subBuffNumber++;
    playerH.playFlashState.subBuffNumber = playerH.playFlashState.subBuffNumber & (SUB_BUFF_QUANTITY - 1);
    //start read next sub frame (should be complete before complete send current buff in LCD !!)
    if(playerH.playFlashState.subFrameNumber == (SUB_FRAME_QUANTITY - 2))
    {
        flashReadSubFrameLast(playFlashSubBuff[playerH.playFlashState.subBuffNumber],
                              SUB_BUFF_SIZE,
                              VIDEO_BLOCK_ADDRESS*FLASH_BLOCK_SIZE + playerH.playFlashState.frameNumber * VIDEO_FRAME_SIZE + playerH.playFlashState.subBuffNumber * SUB_BUFF_SIZE);
    }
    else
    {
        flashReadSubFrameNext(playFlashSubBuff[playerH.playFlashState.subBuffNumber],
                              SUB_BUFF_SIZE,
                              VIDEO_BLOCK_ADDRESS*FLASH_BLOCK_SIZE + playerH.playFlashState.frameNumber * VIDEO_FRAME_SIZE + playerH.playFlashState.subBuffNumber * SUB_BUFF_SIZE);
    }
    playerH.playFlashState.subFrameNumber++;
}


static void playFlashProcessing(void)
{
    //read first sub frame with blocking
    flashReadSubFrameStart(playFlashSubBuff[playerH.playFlashState.subBuffNumber],
                     SUB_BUFF_SIZE,
                     VIDEO_BLOCK_ADDRESS*FLASH_BLOCK_SIZE + playerH.playFlashState.frameNumber * VIDEO_FRAME_SIZE + playerH.playFlashState.subBuffNumber * SUB_BUFF_SIZE);

    putSubFrameStart(playFlashSubBuff[playerH.playFlashState.subBuffNumber], SUB_BUFF_SIZE, lcdProcessingCB);
    //calculate next buff number
    playerH.playFlashState.subBuffNumber++;
    playerH.playFlashState.subBuffNumber = playerH.playFlashState.subBuffNumber & (SUB_BUFF_QUANTITY - 1);
    //start read next sub frame (should be complete before complete send current buff in LCD !!)
    flashReadSubFrameNext(playFlashSubBuff[playerH.playFlashState.subBuffNumber],
                                   SUB_BUFF_SIZE,
                                   VIDEO_BLOCK_ADDRESS*FLASH_BLOCK_SIZE + playerH.playFlashState.frameNumber * VIDEO_FRAME_SIZE + playerH.playFlashState.subBuffNumber * SUB_BUFF_SIZE);
     playerH.playFlashState.subFrameNumber++;
}


void playDirectProcessing(void)
{
    putPicture(frameProcessing.frameBuff);
}


#define BLOCK_ADDRESS    5
#define IMAGE_ADDRESS   (BLOCK_ADDRESS*32768)


static void lcdTaskFunction(void *pvParameters)
{

    lcdInit();
    lcdSemaphore = xSemaphoreCreateBinary();
    putPicture(image_data_01_0006_Layer11);

    flashEraseBlock32(IMAGE_ADDRESS);
    flashWriteBlock(image_data_imagesimage_data_images, sizeof(image_data_imagesimage_data_images), IMAGE_ADDRESS);

    playFlashProcessing();
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
