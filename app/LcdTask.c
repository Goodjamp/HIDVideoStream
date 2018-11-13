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

extern const uint8_t image_data_logo_small[4050];
extern uint8_t  rxCommandBuffer[34816];

/***********************************BORDER CREATION CONFIGURATION*****/
#define R_0        18.0F
#define R_1        17.0F
#define R_2        15.0F
#define R_3        14.0F

#define MAX_ALFA   255.0F
#define K1         (MAX_ALFA/(R_1 - R_0))
#define B1         (-K1*R_0)
#define K2         (MAX_ALFA/(R_2 - R_3))
#define B2         (-K2*R_3)
/********************************************************************/

/*************************TOTAL PLAY CONFIGURATION***************/
#define SCRREN_SAVER_PERIOD_MS   7000
/*************************BORDER PLAY CONFIGURATION**************/
#define COLOR_QYANTITY            6
#define COLOR_1                   {91,  161,  87}
#define COLOR_2                   {249, 237,  56}
#define COLOR_3                   {200,  38,  43}
#define COLOR_4                   {198,  18, 132}
#define COLOR_5                   {67,   67, 146}
#define COLOR_5                   {95,  167, 229}

struct
{
    uint8_t  color[3];
    float    lineKoef[2];
    uint32_t stopTime;
}logoPlayDescription[] =
{
    {.color = COLOR_1},
    {.color = COLOR_2},
    {.color = COLOR_3},
    {.color = COLOR_4},
    {.color = COLOR_5},
    {.color = COLOR_6},
};

/********************************************************************/

#define GET_BLUE(X)   ((X & 0b1111100000000) >> 8)
#define GET_GREEN(X)  (((X & 0b111)>>10) | (X & 0b111))
#define GET_RED(X)    ((X & 0b11111000) >> 3)

#define SET_BLUE(X)   ( X << 8)
#define SET_GREEN(X)  (((X & 0b111000)<<10) | (X & 0b111))
#define SET_RED(X)    (X << 3)

// color format: RGB565: LSB0...4 - R, MSB0..2 LSB5...7 - G. MSB3...7 - B

#define COLOR      0b11111000
#define COLOR_B    0X1b


typedef struct{
    uint8_t x;
    uint8_t y;
    uint16_t alfa;
}pixelDescrT;

struct{
    uint16_t size;
    pixelDescrT pixel[100*100];
}border;


void createLine(uint16_t x0, uint16_t y0, uint16_t xStart, uint16_t xStop, uint16_t yStart, uint16_t yStop, bool isHorizontal)
{
    uint16_t x, y;
    float   xF, yF;
    float d;
    x = xStart;
    for(; x < xStop; x++)
    {
        xF = x + 0.5F;
        y = yStart;
        for(; y < yStop; y++)
        {
            yF = y + 0.5F;
            d = (isHorizontal) ? (sqrt(pow(y0 - yF, 2))):(sqrt(pow(x0 - xF, 2)));
            //d = sqrt(pow((xF - x), 2) + pow((yF - Y0_1), 2));
            if(d > R_0 || d < R_3)
            {
                continue;
            }
            else if(d < R_0 && d > R_1)
            {
                border.pixel[border.size].alfa = d * K1 + B1;
            }
            else if(d < R_1 && d > R_2)
            {
                border.pixel[border.size].alfa = MAX_ALFA;
            }
            else if(d < R_2 && d > R_3)
            {
                border.pixel[border.size].alfa = d * K2 + B2;;
            }
            border.pixel[border.size].x = x;
            border.pixel[border.size].y = y;
            border.size++;
        }
    }
}

void createAngle(uint16_t x0In, uint16_t y0In, uint16_t xStart, uint16_t xStop, uint16_t yStart, uint16_t yStop)
{
    uint16_t x, y;
    float   xF, yF;
    float d;
    x = xStart;
    for(; x < xStop; x++)
    {
        xF = x + 0.5F;
        y = yStart;
        for(; y < yStop; y++)
        {
            yF = y + 0.5F;
            d = sqrt(pow((xF - x0In), 2) + pow((yF - y0In), 2));
            if(d > R_0 || d < R_3)
            {
                continue;
            }
            else if(d < R_0 && d > R_1)
            {
                border.pixel[border.size].alfa = d * K1 + B1;
            }
            else if(d < R_1 && d > R_2)
            {
                border.pixel[border.size].alfa = MAX_ALFA;
            }
            else if(d < R_2 && d > R_3)
            {
                border.pixel[border.size].alfa = d * K2 + B2;;
            }
            border.pixel[border.size].x = x;
            border.pixel[border.size].y = y;
            border.size++;
        }
    }

}

void createBorder(void)
{
    createAngle(R_0, R_0, 0, R_0, 0, R_0);
    createAngle(128 - R_0, R_0, 128 - R_0, 128, 0, R_0);
    createAngle(128 - R_0, 128 - R_0, 128 - R_0, 128, 128 - R_0, 128);
    createAngle(R_0, 128 - R_0, 0, R_0, 128 - R_0, 128);
    createLine(R_0, R_0, R_0, 128 - R_0, 0, R_0, true);
    createLine(R_0, 128 - R_0, R_0, 128 - R_0, 128 - R_0, 128, true);
    createLine(R_0, R_0, 0, R_0, R_0, 128 - R_0, false);
    createLine(128 - R_0, R_0, 128 - R_0, 128, R_0, 128 - R_0, false);
}


void updateBorder(uint32_t cntMs)
{
    uint16_t red1, green1, blue1;
    uint16_t red2, green2, blue2;
    uint16_t cnt = 0;
    uint16_t (*frame)[128][128] = rxCommandBuffer;

    for(;cnt < border.size; cnt++)
    {

        red1   = GET_RED(  (*frame)[border.pixel[cnt].y][border.pixel[cnt].x]);
        green1 = GET_GREEN((*frame)[border.pixel[cnt].y][border.pixel[cnt].x]);
        blue1  = GET_BLUE( (*frame)[border.pixel[cnt].y][border.pixel[cnt].x]);

        red2   = GET_RED(  COLOR);
        green2 = GET_GREEN(COLOR);
        blue2  = GET_BLUE( COLOR);

        (*frame)[border.pixel[cnt].y][border.pixel[cnt].x] =  SET_RED(((255 - border.pixel[cnt].alfa)*red1   + (border.pixel[cnt].alfa)*red2)/255);
        (*frame)[border.pixel[cnt].y][border.pixel[cnt].x] |= SET_GREEN(((255 - border.pixel[cnt].alfa)*green1 + (border.pixel[cnt].alfa)*green2)/255);
        (*frame)[border.pixel[cnt].y][border.pixel[cnt].x] |= SET_BLUE(((255 - border.pixel[cnt].alfa)*blue1  + (border.pixel[cnt].alfa)*blue2)/255);
    }
}


void updateLogo(uint32_t cntMs)
{
    uint16_t (*frame)[128][128] = rxCommandBuffer;
    uint16_t (*logo)[45][45] =  image_data_logo_small;
    uint16_t imageShift = 128/2 - 45/2;
    for(uint16_t x = imageShift, xLogo = 0; xLogo < 45; x++, xLogo++ )
    {
        for(uint16_t y = imageShift, yLogo = 0; yLogo < 45; y++, yLogo++ )
        {
            (*frame)[y][x] = (*logo)[yLogo][xLogo];
        }
    }
}


void updateScreenSaver(uint32_t cntMs)
{
    memset(rxCommandBuffer, 0x00, sizeof(rxCommandBuffer));
    updateLogo(cntMs);
    updateBorder(cntMs);
    putPicture(rxCommandBuffer);
}


static void lcdTaskFunction(void *pvParameters)
{

    lcdInit();
    lcdSemaphore = xSemaphoreCreateBinary();
    flashVideoTimer = xTimerCreate("flashVideoPlayTimer",
                                       FLASH_VIDEO_FPS,
                                       pdTRUE,
                                       NULL,
                                       flashVideoTimerCB);
    //create border
    createBorder();
    //screen saver
    updateScreenSaver(0);
    putPicture(rxCommandBuffer);

    //flashVideoPlay();
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
