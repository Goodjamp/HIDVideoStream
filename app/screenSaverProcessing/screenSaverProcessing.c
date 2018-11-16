#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include "math.h"

#define SCREEN_HEIGHT_PIXEL    128
#define SCREEN_WIDTH_PIXEL     128

#define RB_RANGE               0b11111
#define G_RANGE                0b111111

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
#define SCRREN_SAVER_DURATION_MS   7000
/*************************BORDER LAYER CONFIGURATION**************/
#define COLOR_QYANTITY            6
#define COLOR_0                   {91,  161, 87}
#define COLOR_1                   {249, 237, 56}
#define COLOR_2                   {200, 38,  43}
#define COLOR_3                   {198, 18,  132}
#define COLOR_4                   {67,  67,  146}
#define COLOR_5                   {95,  167, 229}
// last color should be the same as first
#define COLOR_6                   {91,  161, 87}
/********************************************************************/

/*************************WAVE_1 LAYER CONFIGURATION**************/
#define waveBezier_p0x   0.0F
#define waveBezier_p0y   0.0F
#define waveBezier_p1x   0.36F
#define waveBezier_p1y   0.45F
#define waveBezier_p2x   0.63F
#define waveBezier_p2y   0.53F
#define waveBezier_p3x   1.0F
#define waveBezier_p3y   1.0F

/********************************************************************/

/*************************WAVE_2 LAYER CONFIGURATION**************/
#define waveBezierVertical_p0x   0.0F
#define waveBezierVertical_p0y   0.0F
#define waveBezierVertical_p1x   0.36F
#define waveBezierVertical_p1y   0.45F
#define waveBezierVertical_p2x   0.63F
#define waveBezierVertical_p2y   0.53F
#define waveBezierVertical_p3x   1.0F
#define waveBezierVertical_p3y   1.0F

#define waveBezierHorizontal_p0x  0.0F
#define waveBezierHorizontal_p0y  0.0F
#define waveBezierHorizontal_p1x  0.25F
#define waveBezierHorizontal_p1y  0.1F
#define waveBezierHorizontal_p2x  0.24F
#define waveBezierHorizontal_p2y  1.0F
#define waveBezierHorizontal_p3x  1.0F
#define waveBezierHorizontal_p3y  1.0F

#define verticalDelayMs           -125
#define horizontalDelayMs         -1250

#define verticalShift_faze_1

#define verticalShift             125
#define horizontalShift           1250

/********************************************************************/

/*************************LOGO LAYER CONFIGURATION**************/
#define logoBezier_p0x   0.0F
#define logoBezier_p0y   0.0F
#define logoBezier_p1x   0.75F
#define logoBezier_p1y   0.45F
#define logoBezier_p2x   0.63F
#define logoBezier_p2y   0.53F
#define logoBezier_p3x   1.0F
#define logoBezier_p3y   1.0F

/********************************************************************/

// color format: BGR565 (Big     Endians): ORIGINAL: 0...4 - B,    5...10 - G 11...15 - R
//                       Little  Endians):           0...2 -G(MSB) 3...7 - R, 0...4 - B, 5...7 -G(LSB)

#define RED_TO_565(X)   ((X) >> 3)
#define GREEN_TO_565(X) ((X) >> 2)
#define BLUE_TO_565(X)  ((X) >> 3)

#define GET_BLUE(X)   ((X & 0b1111100000000) >> 8)
#define GET_GREEN(X)  (((X & 0b111)>>10) | (X & 0b111))
#define GET_RED(X)    ((X & 0b11111000) >> 3)

#define SET_BLUE(X)   ( X << 8)
#define SET_GREEN(X)  ((X <<13) | (X >> 3))
#define SET_RED(X)    (X << 3)


typedef uint16_t (*frameT)[SCREEN_WIDTH_PIXEL][SCREEN_HEIGHT_PIXEL];

/********Border layer variables***************/
typedef enum
{
    RED   = 0,
    GREEN = 1,
    BLUE  = 2
}colorT;

typedef struct{
    uint8_t x;
    uint8_t y;
    uint16_t alfa;
}pixelDescrT;

static const uint8_t borderColor[][3] =
{
    COLOR_0,
    COLOR_1,
    COLOR_2,
    COLOR_3,
    COLOR_4,
    COLOR_5,
    COLOR_6,
};

struct
{
    float    lineKoef[3][2];
    uint32_t stopTime;
}borderLayerDescription[COLOR_QYANTITY];

//this variable should be calculate outside in future
struct{
    float kRB;
    float kG;
    float bRB;
    float bG;
}wave1LayerDescription;

//this variable should be calculate outside in future
struct{
    uint16_t size;
    pixelDescrT pixel[100*100];
}border;
/*******************************************/

/********Wave layer variables***************/
const uint8_t wave[] = {
    #include "wavePoints.h"
    };
uint16_t wave1LayerColor[128];
/*******************************************/

/********Logo layer variables***************/
extern const uint8_t image_data_logo_small[4050];
/*******************************************/

void calcLineKoef(float *k, float *b, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
    *k = ((float)y1 - (float)y2)/((float)x1 - (float)x2);
    *b = (float)y1 - (*k) * (float)x1;
}


static inline float calcLinePoint(float k, float b, float x)
{
    return k * x + b;
}


static inline float cubicBezier(float t, float p0, float p1, float p2, float p3)
{
    return pow((1.0F-t),3.0F)*p0+3.0F*pow((1.0F-t),2.0F)*t*p1+3.0F*(1.0F-t)*pow(t,2.0F)*p2+pow(t,3.0F)*p3;
}


static void createLine(uint16_t x0, uint16_t y0, uint16_t xStart, uint16_t xStop, uint16_t yStart, uint16_t yStop, bool isHorizontal)
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

static void createAngle(uint16_t x0In, uint16_t y0In, uint16_t xStart, uint16_t xStop, uint16_t yStart, uint16_t yStop)
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

static void createBorder(void)
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


static void updateBorderLayer(uint32_t cntMs, uint8_t *buff)
{
    uint16_t red1, green1, blue1;
    uint16_t index;
    frameT   frame = (frameT)buff;

    for(index = 0; index < COLOR_QYANTITY ; index++)
    {
        if(cntMs < borderLayerDescription[index].stopTime)
        {
            break;
        }
    }

    uint8_t red2   = (uint8_t)calcLinePoint(borderLayerDescription[index].lineKoef[RED][0],
                                            borderLayerDescription[index].lineKoef[RED][1],
                                            cntMs);
    uint8_t green2 = (uint8_t)calcLinePoint(borderLayerDescription[index].lineKoef[GREEN][0],
                                            borderLayerDescription[index].lineKoef[GREEN][1],
                                            cntMs);
    uint8_t blue2  = (uint8_t)calcLinePoint(borderLayerDescription[index].lineKoef[BLUE][0],
                                            borderLayerDescription[index].lineKoef[BLUE][1],
                                            cntMs);

    for(uint16_t cnt = 0; cnt < border.size; cnt++)
    {

        red1   = GET_RED(  (*frame)[border.pixel[cnt].y][border.pixel[cnt].x]);
        green1 = GET_GREEN((*frame)[border.pixel[cnt].y][border.pixel[cnt].x]);
        blue1  = GET_BLUE( (*frame)[border.pixel[cnt].y][border.pixel[cnt].x]);

        (*frame)[border.pixel[cnt].y][border.pixel[cnt].x] =  SET_RED(  ((255 - border.pixel[cnt].alfa)*red1   + (border.pixel[cnt].alfa)*red2  )/255);
        (*frame)[border.pixel[cnt].y][border.pixel[cnt].x] |= SET_GREEN(((255 - border.pixel[cnt].alfa)*green1 + (border.pixel[cnt].alfa)*green2)/255);
        (*frame)[border.pixel[cnt].y][border.pixel[cnt].x] |= SET_BLUE( ((255 - border.pixel[cnt].alfa)*blue1  + (border.pixel[cnt].alfa)*blue2 )/255);

        //(*frame)[border.pixel[cnt].y][border.pixel[cnt].x] = rezColor;
    }
}


static void updateLogoLayer(uint32_t cntMs, uint8_t *buff)
{
    uint16_t imageShift      = 128/2 - 45/2;
    uint16_t (*logo)[45][45] =  image_data_logo_small;
    frameT   frame           = (frameT)buff;

    for(uint16_t x = imageShift, xLogo = 0; xLogo < 45; x++, xLogo++ )
    {
        for(uint16_t y = imageShift, yLogo = 0; yLogo < 45; y++, yLogo++ )
        {
            (*frame)[y][x] = (*logo)[yLogo][xLogo];
        }
    }
}

static void updateWave1Layer(uint32_t cntMs, uint8_t *buff)
{
    const uint8_t (*wavePoints)[sizeof(wave)/2][2] =  (const uint8_t (*)[sizeof(wave)/2][2])wave;
    frameT   frame                = (frameT)buff;
    uint16_t xWave =  1600 * cubicBezier((float)cntMs/(float)SCRREN_SAVER_DURATION_MS,
                                      waveBezier_p0x,
                                      waveBezier_p1x,
                                      waveBezier_p2x,
                                      waveBezier_p3x);
    uint8_t color;
    uint8_t y;
    for(uint16_t xFrame = 0; xFrame < 128; xFrame++, xWave++)
    {
        if((*wavePoints)[xWave][0] > 127 )
        {
            continue;
        }
        if(xWave >= sizeof(wave)/2)
        {
            break;
        }
        y                    = (*wavePoints)[xWave][0] - 1;
        color                = (GET_RED(wave1LayerColor[y]) * (*wavePoints)[xWave][1]) / 10;
        (*frame)[y][xFrame]  = SET_RED(color);
        (*frame)[y][xFrame] |= SET_GREEN(color<<1);
        (*frame)[y][xFrame] |= SET_BLUE(color);
        for(uint16_t yFrame   = (*wavePoints)[xWave][0]; yFrame < 128; yFrame++)
        {
/*
            (*frame)[k][kF] = SET_RED((uint8_t)calcLinePoint(wave1LayerDescription.kRB,
                                                            wave1LayerDescription.bRB,
                                                            k));
            (*frame)[k][kF] |= SET_GREEN((uint8_t)calcLinePoint(wave1LayerDescription.kG,
                                                            wave1LayerDescription.bG,
                                                            k));
            (*frame)[k][kF] |= SET_BLUE((uint8_t)calcLinePoint(wave1LayerDescription.kRB,
                                                            wave1LayerDescription.bRB,
                                                            k));
 */
            (*frame)[yFrame][xFrame] = wave1LayerColor[yFrame];
        }

    }

}


static void initWave1Layer(void)
{
    float   kRB, kG;
    float   bRB, bG;
    uint8_t colorRB;
    uint8_t colorG;;
    calcLineKoef(&kRB, &bRB,
                 34,   RED_TO_565(53),
                 122,  RED_TO_565(27));
    calcLineKoef(&kG, &bG,
                 34,   GREEN_TO_565(53),
                 122,  GREEN_TO_565(27));
    for(uint8_t y = 0; y < sizeof(wave1LayerColor)/sizeof(wave1LayerColor[0]); y++ )
    {
        colorRB = calcLinePoint(kRB, bRB, y);
        colorG  = calcLinePoint(kG, bG, y);
        wave1LayerColor[y]  = SET_RED(colorRB);
        wave1LayerColor[y] |= SET_GREEN(colorG);
        wave1LayerColor[y] |= SET_BLUE(colorRB);
    }

}


static void initBorderLayer(void)
{
    uint16_t colorTimeStep = SCRREN_SAVER_DURATION_MS / COLOR_QYANTITY;
    uint16_t collorStopTime = 0;
    for(uint8_t cnt = 0; cnt < COLOR_QYANTITY ; cnt++ )
    {
        borderLayerDescription[cnt].stopTime = (cnt == (COLOR_QYANTITY - 1)) ? (SCRREN_SAVER_DURATION_MS) : (collorStopTime + colorTimeStep);
        // calculation line coefficient for interpolation red color
        calcLineKoef(&borderLayerDescription[cnt].lineKoef[RED][0],
                     &borderLayerDescription[cnt].lineKoef[RED][1],
                     collorStopTime,
                     RED_TO_565(borderColor[cnt][RED]),
                     collorStopTime + colorTimeStep,
                     RED_TO_565(borderColor[cnt + 1][RED]));
        // calculation line coefficient for interpolation green color
        calcLineKoef(&borderLayerDescription[cnt].lineKoef[GREEN][0],
                     &borderLayerDescription[cnt].lineKoef[GREEN][1],
                     collorStopTime,
                     GREEN_TO_565(borderColor[cnt][GREEN]),
                     collorStopTime + colorTimeStep,
                     GREEN_TO_565(borderColor[cnt + 1][GREEN]));
        // calculation line coefficient for interpolation blue color
        calcLineKoef(&borderLayerDescription[cnt].lineKoef[BLUE][0],
                     &borderLayerDescription[cnt].lineKoef[BLUE][1],
                     collorStopTime,
                     BLUE_TO_565(borderColor[cnt][BLUE]),
                     collorStopTime + colorTimeStep,
                     BLUE_TO_565(borderColor[cnt + 1][BLUE]));
        collorStopTime = collorStopTime + colorTimeStep;
    }
}



void calculateScreenSaverFrame(uint32_t cntMs, uint8_t *buff, uint32_t buffSize)
{
    cntMs = cntMs % SCRREN_SAVER_DURATION_MS;
    memset(buff, 0x00, buffSize);
    updateWave1Layer(cntMs, buff);
    updateLogoLayer(cntMs, buff);
    updateBorderLayer(cntMs, buff);
}


void initScreenSaver(void)
{
    /***Temporal function. Border should be calculate before !!!!!! ****/
    createBorder();
    /*******************************************************************/
    initBorderLayer();
    initWave1Layer();
}
