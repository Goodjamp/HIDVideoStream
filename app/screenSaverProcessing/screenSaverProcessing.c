#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include "math.h"

#include "border.h"

#define colorRBMax             0b11111
#define colorGMax              0b111111

#define SCREEN_HEIGHT_PIXEL    128
#define SCREEN_WIDTH_PIXEL     128

#define MAX_ALFA               255

/*Animation section description position**/
#define timeStart   0
#define timeStop    1
#define timeLength  2
#define posStart    3
#define posLength   4

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
/******************************************************************/

/*************************WAVE LAYER TOTAL CONFIGURATION***********/
#define waveLeftField    4
#define waveRightField   124
#define waveToptField    4
#define waveButtonField  124

/*****************************************************************/


/*************************WAVE_1 LAYER CONFIGURATION**************/
#define waveBezier_p0x   0.0F
#define waveBezier_p0y   0.0F
#define waveBezier_p1x   0.36F
#define waveBezier_p1y   0.45F
#define waveBezier_p2x   0.63F
#define waveBezier_p2y   0.53F
#define waveBezier_p3x   1.0F
#define waveBezier_p3y   1.0F

#define timePosV_0_0     0
#define posV_0_0         -10
#define timePosV_0_1     (SCRREN_SAVER_DURATION_MS / 2)
#define posV_0_1         20

/********************************************************************/

/*************************WAVE_2 LAYER CONFIGURATION**************/
//points for vertical shift of bezier curve
#define waveBezierV_p1y   0.1F
#define waveBezierV_p2y   1.0F

//points for horizontal shift of bizier curve
#define waveBezierH_p1y  0.45F
#define waveBezierH_p2y  0.53F

#define timePosV_0_0     0
#define posV_0_0         -10
#define timePosV_0_1     (SCRREN_SAVER_DURATION_MS / 2)
#define posV_0_1         20

#define timePosV_1_0     (SCRREN_SAVER_DURATION_MS / 2)
#define posV_1_0         20
#define timePosV_1_1     (SCRREN_SAVER_DURATION_MS)
#define posV_1_1         -10

#define timePosH_0_0     0
#define posH_0_0         0
#define timePosH_0_1     SCRREN_SAVER_DURATION_MS
#define posH_0_1         1600

#define delayVMs         1250
#define delayHMs         125

/********************************************************************/

/*************************LOGO LAYER CONFIGURATION**************/
#define logoBezier_p1y   0.45F
#define logoBezier_p2y   0.53F

#define logoTimeAlfa_0_0  0
#define logoAlfa_0_0      0
#define logoTimeAlfa_0_1  (2400)
#define logoAlfa_0_1      0

#define logoTimeAlfa_1_0  (2400)
#define logoAlfa_1_0      0
#define logoTimeAlfa_1_1  (SCRREN_SAVER_DURATION_MS / 2)
#define logoAlfa_1_1      MAX_ALFA

#define logoTimeAlfa_2_0  (SCRREN_SAVER_DURATION_MS / 2)
#define logoAlfa_2_0      MAX_ALFA
#define logoTimeAlfa_2_1  (SCRREN_SAVER_DURATION_MS )
#define logoAlfa_2_1      0

/********************************************************************/

// color format: BGR565 (Big     Endians): ORIGINAL: 0...4 - B,    5...10 - G 11...15 - R
//                       Little  Endians):           0...2 -G(MSB) 3...7 - R, 0...4 - B, 5...7 -G(LSB)

#define RED_TO_565(X)   ((X) >> 3)
#define GREEN_TO_565(X) ((X) >> 2)
#define BLUE_TO_565(X)  ((X) >> 3)

#define GET_BLUE(X)   ((X & 0b1111100000000) >> 8)
#define GET_GREEN(X)  ((X >>13) | ((X & 0b111)<<3))
#define GET_RED(X)    ((X & 0b11111000) >> 3)

#define SET_GRAYSCALE(RB, G) (( RB << 8) | (RB << 3) | (G <<13) | (G >> 3))
#define SET_COLOR(R, G, B)   (( B << 8)  | (R << 3)  | (G <<13) | (G >> 3))


typedef uint16_t (*FrameBuff)[SCREEN_WIDTH_PIXEL];

/********Border layer variables***************/
typedef enum ColorComponents {
    RED   = 0,
    GREEN = 1,
    BLUE  = 2
} ColorComponents;

typedef struct PixelDescription {
    uint8_t x;
    uint8_t y;
    uint16_t alfa;
} PixelDescription;

static const uint8_t borderColor[][3] = {
    COLOR_0,
    COLOR_1,
    COLOR_2,
    COLOR_3,
    COLOR_4,
    COLOR_5,
    COLOR_6,
};

struct {
    float    lineKoef[3][2];
    uint32_t stopTime;
}borderLayerDescription[COLOR_QYANTITY];

//this variable should be calculate outside in future
struct {
    float kRB;
    float kG;
    float bRB;
    float bG;
} wave1LayerDescription;


/********Wave layer variables***************/
const uint8_t wave[] = {
    #include "wavePoints.h"
    };
/********Wave layer 1 variables***************/
uint16_t wave1LayerColor[128];
/*******************************************/
/********Wave layer 2 variables***************/
uint8_t wave2LayerAlfa[128];

// time section start, time section length,  position section start, section length
const int32_t wave2TimeSectionHDesc[][5] ={
    {timePosH_0_0, timePosH_0_1, timePosH_0_1 - timePosH_0_0, posH_0_0, posH_0_1 - posH_0_0},
};


const int32_t wave2TimeSectionVDesc[][5] = {
    {timePosV_0_0, timePosV_0_1, timePosV_0_1 - timePosV_0_0, posV_0_0, posV_0_1 - posV_0_0},
    {timePosV_1_0, timePosV_1_1, timePosV_1_1 - timePosV_1_0, posV_1_0, posV_1_1 - posV_1_0},
};

const float wave2BizierH[2] = {
    waveBezierH_p1y,
    waveBezierH_p2y
};

const float wave2BizierV[2] = {
    waveBezierV_p1y,
    waveBezierV_p2y
};
/*******************************************/
/*******************************************/

/********Logo layer variables***************/
extern const uint8_t image_data_logo_small[4050];

const int32_t logoTimeSectionVDesc[][5] = {
    {logoTimeAlfa_0_0, logoTimeAlfa_0_1, logoTimeAlfa_0_1 - logoTimeAlfa_0_0, logoAlfa_0_0, logoAlfa_0_1 - logoAlfa_0_0},
    {logoTimeAlfa_1_0, logoTimeAlfa_1_1, logoTimeAlfa_1_1 - logoTimeAlfa_1_0, logoAlfa_1_0, logoAlfa_1_1 - logoAlfa_1_0},
    {logoTimeAlfa_2_0, logoTimeAlfa_2_1, logoTimeAlfa_2_1 - logoTimeAlfa_2_0, logoAlfa_2_0, logoAlfa_2_1 - logoAlfa_2_0},
};

const float logoBizier[2] = {
    logoBezier_p1y,
    logoBezier_p2y
};





void calcLineKoef(float *k, float *b, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
    *k = ((float)y1 - (float)y2)/((float)x1 - (float)x2);
    *b = (float)y1 - (*k) * (float)x1;
}

static inline float calcLinePoint(float k, float b, float x)
{
    return k * x + b;
}

static inline float calcCubicBezier(float t, float p0, float p1, float p2, float p3)
{
    return pow((1.0F-t),3.0F)*p0+3.0F*pow((1.0F-t),2.0F)*t*p1+3.0F*(1.0F-t)*pow(t,2.0F)*p2+pow(t,3.0F)*p3;
}

static inline int32_t calcCubicBezierPos(uint32_t cntTime, const float bezierPoints[], const int32_t timeSection[][5], const uint8_t quantitySection)
{
    uint8_t k = 0;

    for(;k < quantitySection; k++) {
        if(cntTime < timeSection[k][timeStop]) {
            break;
        }
    }

    float t = ((float)cntTime - (float)timeSection[k][timeStart]) / (float)timeSection[k][timeLength]; // relation time inside "k" section

    return (int32_t)(timeSection[k][posStart] + timeSection[k][posLength] *(3.0F*pow((1.0F-t),2.0F)*t*bezierPoints[0]+3.0F*(1.0F-t)*pow(t,2.0F)*bezierPoints[1]+pow(t,3.0F)));
}


static void updateBorderLayer(uint32_t cntMs, uint8_t buff[])
{
    #define POS_X      0
    #define POS_Y      1
    #define RADIUS_ALFA   2
    uint8_t (*borderRadius)[3] = (uint8_t(*)[3])borderRadiusDesc;
    uint16_t borderRadiusSize  = sizeof(borderRadiusDesc) / 3;

    uint8_t (*borderAngle)[2] = (uint8_t(*)[2])borderAngleDesc;
    uint16_t borderAngleSize  = sizeof(borderAngleDesc) / 2;

    uint8_t (*borderLine)[2] = (uint8_t(*)[2])borderLineDesc;
    uint16_t borderLineSize  = sizeof(borderLineDesc) / 2;

    uint8_t red, green, blue;
    uint8_t redA, greenA, blueA;
    uint8_t redB, greenB, blueB;
    uint16_t index;
    uint16_t k;
    uint16_t currentColor;

    FrameBuff   frame = (FrameBuff)buff;
    cntMs = cntMs % SCRREN_SAVER_DURATION_MS;

    for(index = 0; index < COLOR_QYANTITY; index++) {
        if(cntMs < borderLayerDescription[index].stopTime) {
            break;
        }
    }

    red   = (uint8_t)calcLinePoint(borderLayerDescription[index].lineKoef[RED][0],
                                   borderLayerDescription[index].lineKoef[RED][1],
                                   cntMs);
    green = (uint8_t)calcLinePoint(borderLayerDescription[index].lineKoef[GREEN][0],
                                   borderLayerDescription[index].lineKoef[GREEN][1],
                                   cntMs);
    blue  = (uint8_t)calcLinePoint(borderLayerDescription[index].lineKoef[BLUE][0],
                                   borderLayerDescription[index].lineKoef[BLUE][1],
                                   cntMs);

    currentColor = SET_COLOR(red, green, blue);

    for(k = 0; k < borderRadiusSize; k++) {
        if(borderRadius[k][RADIUS_ALFA] == MAX_ALFA) {
            frame[borderRadius[k][POS_Y]][borderRadius[k][POS_X]] = currentColor;
            continue;
        }

        redB   = GET_RED(   frame[borderRadius[k][POS_Y]][borderRadius[k][POS_X]]);
        greenB = GET_GREEN( frame[borderRadius[k][POS_Y]][borderRadius[k][POS_X]]);
        blueB  = GET_BLUE(  frame[borderRadius[k][POS_Y]][borderRadius[k][POS_X]]);

        redA   = ((MAX_ALFA - borderRadius[k][RADIUS_ALFA]) * redB + borderRadius[k][RADIUS_ALFA] * red)   / MAX_ALFA;
        greenA = ((MAX_ALFA - borderRadius[k][RADIUS_ALFA]) * greenB + borderRadius[k][RADIUS_ALFA] * green) / MAX_ALFA;
        blueA  = ((MAX_ALFA - borderRadius[k][RADIUS_ALFA]) * blueB + borderRadius[k][RADIUS_ALFA] * blue)  / MAX_ALFA;
        frame[borderRadius[k][POS_Y]][borderRadius[k][POS_X]] = SET_COLOR(redA, greenA, blueA);
    }

    for(k = 0; k < borderAngleSize; k++) {
        frame[borderAngle[k][POS_Y]][borderAngle[k][POS_X]] = 0x0000;
    }

    for(k = 0; k < borderLineSize; k++) {
        frame[borderLine[k][POS_Y]][borderLine[k][POS_X]] = currentColor;
    }

}

static void updateLogoLayer(uint32_t cntMs, uint8_t buff[])
{
    uint8_t colorRB;
    uint16_t imageShift  = 128/2 - 45/2;
    uint16_t (*logo)[45] = (uint16_t (*)[45])image_data_logo_small;
    FrameBuff   frame    = (FrameBuff)buff;
    int32_t  alfa;

    cntMs = cntMs % SCRREN_SAVER_DURATION_MS;
    alfa  = calcCubicBezierPos(cntMs, logoBizier, logoTimeSectionVDesc, sizeof(logoTimeSectionVDesc) / sizeof(logoTimeSectionVDesc[0]));

    if(alfa == 0) {
        return;
    }

    for(uint16_t x = imageShift, xLogo = 0; xLogo < 45; x++, xLogo++) {
        for(uint16_t y = imageShift, yLogo = 0; yLogo < 45; y++, yLogo++) {
            colorRB = GET_RED(logo[yLogo][xLogo]);
            colorRB = (alfa * colorRB)/MAX_ALFA;

            frame[y][x]  = SET_GRAYSCALE(colorRB, colorRB << 1);
        }
    }
}

static void updateWave1Layer(uint32_t cntMs, uint8_t buff[])
{
    const uint8_t (*wavePoints)[2] = (const uint8_t (*)[2])wave;
    uint16_t  xWave;
    FrameBuff frame;
    uint16_t xWaveSmooth;

    frame = (FrameBuff)buff;
    cntMs = cntMs % SCRREN_SAVER_DURATION_MS;
    xWave =  1600 * calcCubicBezier((float)cntMs/(float)SCRREN_SAVER_DURATION_MS,
                                      waveBezier_p0y,
                                      waveBezier_p1y,
                                      waveBezier_p2y,
                                      waveBezier_p3y);
    xWaveSmooth = xWave;

    for(uint16_t xFrame = waveLeftField; xFrame < waveRightField; xFrame++, xWave++) {
        if(wavePoints[xWave][0] > waveButtonField) {
            continue;
        }
        if(xWave >= sizeof(wave) / 2) {
            return;
        }
        for(uint16_t yFrame = wavePoints[xWave][0]; yFrame < waveButtonField; yFrame++) {
            frame[yFrame][xFrame] = wave1LayerColor[yFrame];
        }
    }

    //smoothing up of wave
    uint8_t rightBord = waveRightField - 1;
    uint8_t yTemp;
    uint32_t summRB;
    uint32_t summG;

    for(uint16_t xFrame = waveLeftField ; xFrame < waveRightField; xFrame++, xWaveSmooth++) {
        if((yTemp =  wavePoints[xWaveSmooth][0] ) > waveButtonField) {
            continue;
        }
        summRB = GET_RED(wave1LayerColor[yTemp]);
        summG  = GET_GREEN(wave1LayerColor[yTemp]);
        summRB *= wavePoints[xWaveSmooth][1];
        summRB /= 10;

        summG *= wavePoints[xWaveSmooth][1];
        summG /= 10;
        /*
        summRB = 0;
        summG = 0;

        //column 0
        xTemp = xFrame - 1;
        summRB += GET_RED(frame[yTemp][xTemp]);
        summG  += GET_GREEN(frame[yTemp][xTemp]);
        yTemp++;
        summRB += GET_RED(frame[yTemp][xTemp]);
        summG  += GET_GREEN(frame[yTemp][xTemp]);
        yTemp++;
        summRB += GET_RED(frame[yTemp][xTemp]);
        summG  += GET_GREEN(frame[yTemp][xTemp]);
        //column 1
        xTemp++;
        yTemp -= 2;
        summRB += GET_RED(frame[yTemp][xTemp]);
        summG  += GET_GREEN(frame[yTemp][xTemp]);
        yTemp++;
        summRB += GET_RED(frame[yTemp][xTemp]);
        summG  += GET_GREEN(frame[yTemp][xTemp]);
        yTemp++;
        summRB += GET_RED(frame[yTemp][xTemp]);
        summG  += GET_GREEN(frame[yTemp][xTemp]);
        //column 2
        xTemp++;
        yTemp -= 2;
        summRB += GET_RED(frame[yTemp][xTemp]);
        summG  += GET_GREEN(frame[yTemp][xTemp]);
        yTemp++;
        summRB += GET_RED(frame[yTemp][xTemp]);
        summG  += GET_GREEN(frame[yTemp][xTemp]);
        yTemp++;
        summRB += GET_RED(frame[yTemp][xTemp]);
        summG  += GET_GREEN(frame[yTemp][xTemp]);
        summRB /= 9;
        summG  /= 9;
        */
        frame[ wavePoints[xWaveSmooth][0] - 1][xFrame] = SET_GRAYSCALE(summRB, summG);
    }
}

static void updateWave2Layer(uint32_t cntMs, uint8_t buff[])
{
    const uint8_t (*wavePoints)[2] =  (const uint8_t (*)[2])wave;
    FrameBuff   frame  = (FrameBuff)buff;
    uint16_t colorRB, colorG;
    int32_t yFrame;
    uint32_t cntRel = (cntMs + delayVMs) % SCRREN_SAVER_DURATION_MS;
    int32_t  shiftY = calcCubicBezierPos(cntRel, wave2BizierV, wave2TimeSectionVDesc, sizeof(wave2TimeSectionVDesc) / sizeof(wave2TimeSectionVDesc[0]));
    cntRel = (cntMs  + delayHMs) % SCRREN_SAVER_DURATION_MS;
    int32_t  shiftX = calcCubicBezierPos(cntRel, wave2BizierH, wave2TimeSectionHDesc, sizeof(wave2TimeSectionHDesc) / sizeof(wave2TimeSectionHDesc[0]));

    for(int32_t xFrame = waveLeftField; xFrame < waveRightField; xFrame++, shiftX++) {
        yFrame = wavePoints[shiftX][0] + shiftY;
        if(waveButtonField > waveButtonField) {
            continue;
        }
        if(shiftX >= sizeof(wave) / 2) {
            break;
        }
        for(; yFrame < waveButtonField; yFrame++) {
             colorRB = GET_RED(  frame[yFrame][xFrame]);
             colorG  = GET_GREEN(frame[yFrame][xFrame]);
             colorRB = ((MAX_ALFA - wave2LayerAlfa[yFrame]) * colorRB + wave2LayerAlfa[yFrame] * colorRBMax)/MAX_ALFA;
             colorG  = ((MAX_ALFA - wave2LayerAlfa[yFrame]) * colorG  + wave2LayerAlfa[yFrame] * colorGMax) /MAX_ALFA;
             frame[yFrame][xFrame] = SET_GRAYSCALE(colorRB, colorG);
        }
    }
}

static void initWave1Layer(void)
{
    float   kRB, kG;
    float   bRB, bG;
    uint8_t colorRB;
    uint8_t colorG;
    calcLineKoef(&kRB, &bRB,
                 34,   RED_TO_565(53),
                 122,  RED_TO_565(27));
    calcLineKoef(&kG, &bG,
                 34,   GREEN_TO_565(53),
                 122,  GREEN_TO_565(27));
    for(uint8_t y = 0; y < sizeof(wave1LayerColor)/sizeof(wave1LayerColor[0]); y++) {
        colorRB = calcLinePoint(kRB, bRB, y);
        colorG  = calcLinePoint(kG, bG, y);

        wave1LayerColor[y] = SET_COLOR(colorRB, colorG, colorRB);
    }
}

static void initWave2Layer(void)
{
    float k;
    float b;
    calcLineKoef(&k, &b,
                 0,   150,
                 127, 20);
    for(uint8_t y = 0; y < sizeof(wave2LayerAlfa)/sizeof(wave2LayerAlfa[0]); y++ ) {
        wave2LayerAlfa[y] = calcLinePoint(k, b, y);
    }
}

static void initBorderLayer(void)
{
    uint16_t colorTimeStep = SCRREN_SAVER_DURATION_MS / COLOR_QYANTITY;
    uint16_t colorStopTime = 0;
    for(uint8_t cnt = 0; cnt < COLOR_QYANTITY ; cnt++ ) {
        borderLayerDescription[cnt].stopTime = (cnt == (COLOR_QYANTITY - 1)) ? (SCRREN_SAVER_DURATION_MS) : (colorStopTime + colorTimeStep);
        // calculation line coefficient for interpolation red color
        calcLineKoef(&borderLayerDescription[cnt].lineKoef[RED][0],
                     &borderLayerDescription[cnt].lineKoef[RED][1],
                     colorStopTime,
                     RED_TO_565(borderColor[cnt][RED]),
                     colorStopTime + colorTimeStep,
                     RED_TO_565(borderColor[cnt + 1][RED]));
        // calculation line coefficient for interpolation green color
        calcLineKoef(&borderLayerDescription[cnt].lineKoef[GREEN][0],
                     &borderLayerDescription[cnt].lineKoef[GREEN][1],
                     colorStopTime,
                     GREEN_TO_565(borderColor[cnt][GREEN]),
                     colorStopTime + colorTimeStep,
                     GREEN_TO_565(borderColor[cnt + 1][GREEN]));
        // calculation line coefficient for interpolation blue color
        calcLineKoef(&borderLayerDescription[cnt].lineKoef[BLUE][0],
                     &borderLayerDescription[cnt].lineKoef[BLUE][1],
                     colorStopTime,
                     BLUE_TO_565(borderColor[cnt][BLUE]),
                     colorStopTime + colorTimeStep,
                     BLUE_TO_565(borderColor[cnt + 1][BLUE]));
        colorStopTime = colorStopTime + colorTimeStep;
    }
}

void calculateScreenSaverFrame(uint32_t cntMs, uint8_t *buff, uint32_t buffSize)
{
    memset(buff, 0x00, buffSize);
    updateLogoLayer(cntMs, buff);
    updateWave1Layer(cntMs, buff);
    updateWave2Layer(cntMs, buff);
    updateBorderLayer(cntMs, buff);
}

void initScreenSaver(void)
{
    initBorderLayer();
    initWave1Layer();
    initWave2Layer();
}
