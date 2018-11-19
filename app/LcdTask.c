#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "timers.h"
#include "fsl_ctimer.h"

#include "Lcd.h"
#include "HalIFlash.h"

#include <stdint.h>
#include <string.h>

#include "./Anim/Anim.c"
#include "screenSaverProcessing.h"

#define LCD_TASK_PRIORITY      2
#define LCD_TASK_STACK_SIZE    200
#define PROTECT_STR           "VIDEO_IS_PRESSENT"

/****Screen Saver paramiters***************************************/
#define SCRREN_SAVER_UPDATE_PER_MS   33
/******************************************************************/

/***Play Flash buffer description*********************************/
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
/******************************************************************/

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
/***********************************************************************************/
extern uint8_t  rxCommandBuffer[34816];

static SemaphoreHandle_t lcdSemaphore = NULL;
static TimerHandle_t flashVideoTimer;
static commandDescriptionT *rxCommand;
uint8_t playFlashSubBuff[SUB_BUFF_QUANTITY][SUB_BUFF_SIZE + 4];
typedef enum
{
    PLAY_FLASH,
    PLAY_DIRECT,
    PLAY_SCREEN_SAVER,
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

struct playScreenSaverStateT
{
    uint32_t lastTimeState;
};

struct
{
    playSourceT                  playSource;
    playerStateT                 playerState;
    struct playFlashStateT       playFlashState;
    struct playDirectStateT      playDirectState;
    struct playScreenSaverStateT playScreenSaverState;
}playerH =
{
    .playSource  = PLAY_SCREEN_SAVER,
    .playerState = PLAYER_PLAY,
    .playFlashState.subFrameNumber = 0,
    .playFlashState.frameNumber    = 0,
    .playFlashState.subBuffNumber  = 0,
};



/************************************************************/
/* user timer variable*/
static CTIMER_Type *TIMER_CNT = CTIMER4;

struct
{
    uint32_t rxTime_[320];
    uint16_t calcCnt;
}calcFrameTest;

static inline uint32_t getTimerCNT(void)
{
    return (uint32_t)TIMER_CNT->TC;
}

static inline void resetTimerCNT(void)
{
    TIMER_CNT->TC = 0;
}

void initTimerForMeTime(void)
{
    ctimer_config_t initStruct;

    initStruct.mode      = kCTIMER_TimerMode;
    SYSCON->ASYNCAPBCTRL = SYSCON_ASYNCAPBCTRL_ENABLE(1);
    initStruct.prescale  = CLOCK_GetAsyncApbClkFreq()/100000; // 10 us
    CTIMER_Init(TIMER_CNT, &initStruct);
    CTIMER_StartTimer(TIMER_CNT);
}
/************************************************************/


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


static void flashVideoPlayFrame(void)
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


void screenSaverPlayFrame(void)
{
    playerH.playScreenSaverState.lastTimeState = xTaskGetTickCount() + SCRREN_SAVER_UPDATE_PER_MS;
    xSemaphoreGive(lcdSemaphore);
}


void videoTimerCB( TimerHandle_t xTimer )
{
    switch(playerH.playSource)
    {
    case PLAY_FLASH:
        if(playerH.playFlashState.frameNumber >= playerH.playFlashState.totalFrameNumber)
        {
            xTimerStop(flashVideoTimer, 2);
            return;
        }
        flashVideoPlayFrame();
        break;
    case PLAY_DIRECT:
        break;
    case PLAY_SCREEN_SAVER:
        screenSaverPlayFrame();
        break;
    }
}


bool videoPlay(playSourceT playSource)
{
    switch(playerH.playSource)
    {
    case PLAY_FLASH:
        if(!flashVideoReadInfo(&playerH.playFlashState.totalFrameNumber))
        {
            return false;
        }
        playerH.playFlashState.frameNumber = 0;
        flashVideoPlayFrame();
        xTimerStart(flashVideoTimer, 2);
        break;
    case PLAY_DIRECT:
    break;
    case PLAY_SCREEN_SAVER:
        xTimerStart(flashVideoTimer, 2);
    break;
    }

    playerH.playSource  = playSource;
    return true;
}


static void lcdTaskFunction(void *pvParameters)
{
    initTimerForMeTime();
    lcdInit();
    lcdSemaphore = xSemaphoreCreateBinary();
    flashVideoTimer = xTimerCreate("flashVideoPlayTimer",
                                       FLASH_VIDEO_FPS,
                                       pdTRUE,
                                       NULL,
                                       videoTimerCB);

    memset(rxCommandBuffer, 0,32768);
    putPicture(rxCommandBuffer);
    vTaskDelay(500);

    initScreenSaver();
    videoPlay(PLAY_SCREEN_SAVER);

    for (;;)
    {
        if (xSemaphoreTake(lcdSemaphore, portMAX_DELAY ) != pdTRUE)
        {
            continue;
        }

        if(playerH.playSource = PLAY_SCREEN_SAVER )
        {
            putPicture(rxCommandBuffer);
            resetTimerCNT();

            calculateScreenSaverFrame(playerH.playScreenSaverState.lastTimeState, rxCommandBuffer, 32768);

            calcFrameTest.rxTime_[calcFrameTest.calcCnt++] = getTimerCNT();
            if(calcFrameTest.calcCnt >= (sizeof(calcFrameTest.rxTime_) / sizeof(calcFrameTest.rxTime_[0])))
            {
                calcFrameTest.calcCnt = 0;
            }
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
                videoPlay(PLAY_FLASH);
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








