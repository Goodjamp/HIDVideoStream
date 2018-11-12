#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "timers.h"

#include "Lcd.h"
#include "HalIFlash.h"

#include <stdint.h>
#include <string.h>

#include "./Anim/Anim.c"

#define LCD_TASK_PRIORITY      2
#define LCD_TASK_STACK_SIZE    200
#define PROTECT_STR           "VIDEO_IS_PRESSENT"
/*play Flash buffer description*/
#define SUB_BUFF_SIZE          1024
#define SUB_BUFF_QUANTITY      2
#define SUB_FRAME_QUANTITY     32
#define INFORM_BLOCK_ADDRESS   4
#define VIDEO_BLOCK_ADDRESS    5
#define FLASH_BLOCK_SIZE       32768
#define FRAME_SIZE             32768
#define FLASH_VIDEO_FPS        33

#define FRAME_ADDRESS(X)       ((VIDEO_BLOCK_ADDRESS + X) * 32768)
#define INFO_ADDRESS           (INFORM_BLOCK_ADDRESS * 32768)


void flashVideoPlayProcessingCB(bool state);

/*******************************VIDEO CONTROLL PROTOCOLL DESCRIPTION*************/
typedef enum
{
    DIRECT = (uint8_t)0,
    FLASH  = (uint8_t)1,
} fieldTargetT;

typedef enum
{
    FLASH_WRITE = (uint8_t)0,
    FLASH_PLAY  = (uint8_t)1,
    FLASH_STOP  = (uint8_t)2,
} fieldSubTargetT;

#pragma pack(push,1)
typedef struct
{
    uint8_t  target;
    uint8_t  subTarget;
    uint16_t frameNumber;
    uint16_t frameQuantity;
    uint32_t size;
    uint8_t  payload[];
} commandDescriptionT;
#pragma pack(pop)
/***********************************************************************************/

/********************************VIDEO INFORMATION**********************************/
#pragma pack(push, 1)
struct
{
    uint8_t  protectStr[sizeof(PROTECT_STR)];
    uint16_t frameQuantity;
}videoData;
#pragma pack(pop)
/**/

static SemaphoreHandle_t lcdSemaphore = NULL;
static TimerHandle_t flashVideoTimer;
static commandDescriptionT *rxCommand;
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
    uint16_t totalFrameNumber;
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


static void flashVideoUpdateInfo(uint16_t totalQuantity)
{
    memcpy(videoData.protectStr, PROTECT_STR, sizeof(videoData.protectStr));
    videoData.frameQuantity = totalQuantity;
    flashEraseBlock32(INFO_ADDRESS);
    flashWriteBlock((uint8_t*)&videoData, sizeof(videoData), INFO_ADDRESS);
}


static bool flashVideoReadInfo(uint16_t *totalQuantity)
{
    memset((uint8_t*)&videoData, (uint8_t)0, sizeof(videoData));
    flashReadData((uint8_t*)&videoData, sizeof(videoData), INFO_ADDRESS);
    if(memcmp(videoData.protectStr, PROTECT_STR, sizeof(PROTECT_STR)) == 0)
    {
        *totalQuantity = videoData.frameQuantity;
        return true;
    }
    return false;
}


static void flashVideoWriteFrame(uint8_t *frame, uint16_t number, uint16_t totalQuantity)
{
    if(number == 0) // first frame, save information about video
    {
        flashVideoUpdateInfo(totalQuantity);
    }
    flashEraseBlock32(FRAME_ADDRESS(number));
    flashWriteBlock(frame, FRAME_SIZE, FRAME_ADDRESS(number));
}

void flashVideoPlayProcessingCB(bool state)
{
    //send Current Sub Frame
    if(playerH.playFlashState.subFrameNumber >= SUB_FRAME_QUANTITY )
    {
        playerH.playFlashState.frameNumber++;
        return; // sending full frame was completing
    }
    putSubFrameNext(&playFlashSubBuff[playerH.playFlashState.subBuffNumber], SUB_BUFF_SIZE, flashVideoPlayProcessingCB);
    if(playerH.playFlashState.subFrameNumber >= (SUB_FRAME_QUANTITY - 1) )
    {
        playerH.playFlashState.subFrameNumber++;
        return; // sending full frame was completing
    }
    //calculate next sub buff number
    playerH.playFlashState.subBuffNumber++;
    playerH.playFlashState.subBuffNumber = playerH.playFlashState.subBuffNumber & (SUB_BUFF_QUANTITY - 1);
    //start read next sub frame (should be complete before complete send current buff in LCD !!)
    if(playerH.playFlashState.subFrameNumber == (SUB_FRAME_QUANTITY - 2))
    {
        flashReadSubFrameLast(playFlashSubBuff[playerH.playFlashState.subBuffNumber],
                              SUB_BUFF_SIZE,
                              FRAME_ADDRESS(playerH.playFlashState.frameNumber) + playerH.playFlashState.subBuffNumber * SUB_BUFF_SIZE);
    }
    else
    {
        flashReadSubFrameNext(playFlashSubBuff[playerH.playFlashState.subBuffNumber],
                              SUB_BUFF_SIZE,
                              FRAME_ADDRESS(playerH.playFlashState.frameNumber) + playerH.playFlashState.subBuffNumber * SUB_BUFF_SIZE);
    }
    playerH.playFlashState.subFrameNumber++;
}


static void flashVideoPlayProcessing(void)
{
    playerH.playFlashState.subBuffNumber  = 0;
    playerH.playFlashState.subFrameNumber = 0;
    //read first sub frame with blocking
    flashReadSubFrameStart(playFlashSubBuff[playerH.playFlashState.subBuffNumber],
                     SUB_BUFF_SIZE,
                     FRAME_ADDRESS(playerH.playFlashState.frameNumber) + playerH.playFlashState.subBuffNumber * SUB_BUFF_SIZE);

    putSubFrameStart(playFlashSubBuff[playerH.playFlashState.subBuffNumber], SUB_BUFF_SIZE, flashVideoPlayProcessingCB);
    //calculate next buff number
    playerH.playFlashState.subBuffNumber++;
    playerH.playFlashState.subBuffNumber = playerH.playFlashState.subBuffNumber & (SUB_BUFF_QUANTITY - 1);
    //start read next sub frame (should be complete before complete send current buff in LCD !!)
    flashReadSubFrameNext(playFlashSubBuff[playerH.playFlashState.subBuffNumber],
                                   SUB_BUFF_SIZE,
                                   FRAME_ADDRESS(playerH.playFlashState.frameNumber) + playerH.playFlashState.subBuffNumber * SUB_BUFF_SIZE);
    playerH.playFlashState.subFrameNumber++;
}


void flashVideoTimerCB( TimerHandle_t xTimer )
{
    if(playerH.playFlashState.frameNumber >= playerH.playFlashState.totalFrameNumber)
    {
        xTimerStop(flashVideoTimer, 2);
        return;
    }
    flashVideoPlayProcessing();
}


bool flashVideoPlay(void)
{
    if(!flashVideoReadInfo(&playerH.playFlashState.totalFrameNumber))
    {
        return false;
    }
    playerH.playFlashState.frameNumber   = 0;
    flashVideoPlayProcessing();
    xTimerStart(flashVideoTimer, 2);
    return true;
}


bool flashVideoStop(void)
{
    xTimerStop(flashVideoTimer, 2);
    return true;
}


static void directVideoPlayFrame(void)
{
    putPicture(rxCommand->payload);
}

void videoCommandProcessing(uint8_t frameBuff[])
{
    static BaseType_t xHigherPriorityTaskWoken;

    rxCommand = (commandDescriptionT*)frameBuff;
    xHigherPriorityTaskWoken = pdFALSE;
    playerH.playerState = PLAYER_PLAY;
    xSemaphoreGiveFromISR(lcdSemaphore, &xHigherPriorityTaskWoken);

}


#include "math.h"
extern uint8_t  rxCommandBuffer[34816];
#define FRAME_LEN  4
#define R_0        20.0F
#define R_1        18.0F
#define R_2        14.0F
#define R_3        12.0F
#define X0_1       18.0F
#define Y0_1       18.0F
#define X0_2       (128.0F-18.0F)
#define Y0_2       18.0F

#define COLOR      0X1f1b


static inline float cyrc_(float xIn, float rIn, float x0, float y0)
{
    return  y0 - sqrt(pow(rIn,2) - pow((xIn - X0_1), 2));
}


typedef struct{
    uint8_t x;
    uint8_t y;
    uint8_t alfa;
}pixelDescrT;

struct{
    uint16_t size;
    pixelDescrT borderDescr[R_0*R_0];
}enflDescription;

void createAngle(void)
{
    uint8_t x,  y;
    float   xF, yF;

    for(x = 0; x < R_OUT; x++)
    {
        xF = x + 0.5F;
        for(y = 0; y < R_OUT; y++)
        {
            yF = y + 0.5F;
            d = sqrt(pow((xF - X0_1), 2) + pow((yF - Y0_1), 2));
            if(d > R_0 && d < R_3)
            {
                continue;
            }
            else if(d < R_0 && d > R_1)
            {
                enflDescription.borderDescr[enflDescription.size].alfa = 50
            }
            else if(d < R_1 && d > R_2)
            {
                enflDescription.borderDescr[enflDescription.size].alfa = 100
            }
            else if(d < R_2 && d > R_3)
            {
                enflDescription.borderDescr[enflDescription.size].alfa = 50
            }
            enflDescription.borderDescr[enflDescription.size].x = x;
            enflDescription.borderDescr[enflDescription.size].y = y;
        }
    }
    enflDescription.size++;
}


void createCyrcle(void)
{
    uint16_t (*frame)[128][128] = rxCommandBuffer;
    memset(rxCommandBuffer, 0, sizeof(rxCommandBuffer));
}


/*
void createCyrcle(void)
{

    uint16_t (*frame)[128][128] = rxCommandBuffer;
    memset(rxCommandBuffer, 0, sizeof(rxCommandBuffer));
    uint16_t yUp, yDown, x;
    float xF;
    for(x = 0; x < FRAME_LEN; x++)
    {
        xF = x;// + 0.5F;
        yUp = cyrc_(xF, R_OUT, X0_1, Y0_1);
        for(; yUp < R_OUT; yUp++)
        {
           (*frame)[yUp][x] = COLOR;
        }
    }

    for(x = 0; x < R_OUT; x++)
    {
        xF = x;// + 0.5F;
        yUp = cyrc_(xF, R_OUT, X0_1, Y0_1);
        yDown = cyrc_(xF, R_IN, X0_1, Y0_1);
        for(; yUp < yDown; yUp++)
        {
           (*frame)[yUp][x] = COLOR;
        }
    }


    for(uint16_t cnt = 0; cnt < FRAME_LEN; cnt++)
    {
        R -= cnt;
        for(x = cnt; x < R1; x++ )
        {

            y = Y0_1 - sqrt(R*R - (x - X0_1)*(x - X0_1));
            (*frame)[y][x] = COLOR;
        }

    }

 putPicture(rxCommandBuffer);
}
*/


static void lcdTaskFunction(void *pvParameters)
{

    lcdInit();
    lcdSemaphore = xSemaphoreCreateBinary();
    flashVideoTimer = xTimerCreate("flashVideoPlayTimer",
                                       FLASH_VIDEO_FPS,
                                       pdTRUE,
                                       NULL,
                                       flashVideoTimerCB);
    // demo after power on
    createCyrcle();
    //flashVideoPlay();
    putPicture(rxCommandBuffer);
    for (;;)
    {
        if (xSemaphoreTake(lcdSemaphore, portMAX_DELAY ) != pdTRUE)
        {
            continue;
        }

        switch(rxCommand->target)
        {
        case DIRECT:
            directVideoPlayFrame();
            flashVideoStop();
            break;
        case FLASH:
            switch(rxCommand->subTarget)
            {
            case FLASH_WRITE:
                flashVideoWriteFrame(rxCommand->payload, rxCommand->frameNumber, rxCommand->frameQuantity);
                break;
            case FLASH_PLAY:
                flashVideoPlay();
                break;
            case FLASH_STOP:
                flashVideoStop();
                break;
            }
            break;
        }
    }
}


void lcdTaskCreate(void)
{
    xTaskCreate(lcdTaskFunction, "Lcd", LCD_TASK_STACK_SIZE, NULL, LCD_TASK_PRIORITY, NULL);
}
