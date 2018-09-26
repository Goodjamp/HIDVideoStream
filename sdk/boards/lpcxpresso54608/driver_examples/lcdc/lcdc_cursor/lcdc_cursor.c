/*
 * The Clear BSD License
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided
 * that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*  Standard C Included Files */
#include <stdio.h>
#include <string.h>
#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_lcdc.h"

#include "pin_mux.h"
#include "fsl_sctimer.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define APP_LCD LCD
#define LCD_PANEL_CLK 9000000
#define LCD_PPL 480
#define LCD_HSW 2
#define LCD_HFP 8
#define LCD_HBP 43
#define LCD_LPP 272
#define LCD_VSW 10
#define LCD_VFP 4
#define LCD_VBP 12
#define LCD_POL_FLAGS kLCDC_InvertVsyncPolarity | kLCDC_InvertHsyncPolarity
#define IMG_HEIGHT 272
#define IMG_WIDTH 480
#define LCD_INPUT_CLK_FREQ CLOCK_GetFreq(kCLOCK_LCD)
#define APP_LCD_IRQHandler LCD_IRQHandler
#define APP_LCD_IRQn LCD_IRQn
#define APP_PIXEL_PER_BYTE 8

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

#if (defined(__CC_ARM) || defined(__GNUC__))
__attribute__((aligned(8)))
#elif defined(__ICCARM__)
#pragma data_alignment = 8
#else
#error Toolchain not support.
#endif
static uint8_t s_frameBufs[IMG_HEIGHT][IMG_WIDTH / APP_PIXEL_PER_BYTE];

/* Frame end flag. */
static volatile bool s_frameEndFlag;

/* Color palette. */
static const uint32_t palette[] = {0x0000001F};

/* 32x32 pixel cursor image. */
#if (defined(__CC_ARM) || defined(__GNUC__))
__attribute__((aligned(4)))
#elif defined(__ICCARM__)
#pragma data_alignment = 4
#else
#error Toolchain not support.
#endif
static const uint8_t cursor32Img0[] = {
    0xAA, 0xAA, 0xAA, 0xAA, 0xAA,
    0xAA, 0xAA, 0xAA, /* Line 1.             */
    0xAA, 0xAA, 0xAA, 0xAA, 0xAA,
    0xAA, 0xAA, 0xAA, /* Line 2.             */
    0xAA, 0xAA, 0xAA, 0xAA, 0xAA,
    0xAA, 0xAA, 0xAA, /* Line 3.             */
    0xAA, 0xAA, 0xAA, 0xAA, 0xAA,
    0xAA, 0xAA, 0xAA, /* Line 4.             */
    0xAA, 0xAA, 0xAA, 0xAA, 0xAA,
    0xAA, 0xAA, 0xAA, /* Line 5.             */
    0xAA, 0xAA, 0xAA, 0xFA, 0xAA,
    0xAA, 0xAA, 0xAA, /* Line 6.             */
    0xAA, 0xAA, 0xAB, 0xFE, 0xAA,
    0xAA, 0xAA, 0xAA, /* Line 7.             */
    0xAA, 0xAA, 0xAB, 0xFE, 0xAA,
    0xAA, 0xAA, 0xAA, /* Line 8.             */
    0xAA, 0xAA, 0xAB, 0xFE, 0xAA,
    0xAA, 0xAA, 0xAA, /* Line 9.             */
    0xAA, 0xAA, 0xAB, 0xFE, 0xAA,
    0xAA, 0xAA, 0xAA, /* Line 10.             */
    0xAA, 0xAA, 0xAB, 0xFF, 0xEA,
    0xAA, 0xAA, 0xAA, /* Line 11.             */
    0xAA, 0xAA, 0xAB, 0xFF, 0xFF,
    0xAA, 0xAA, 0xAA, /* Line 12.             */
    0xAA, 0xAA, 0xAB, 0xFF, 0xFF,
    0xFA, 0xAA, 0xAA, /* Line 13.             */
    0xAA, 0xAA, 0xAB, 0xFF, 0xFF,
    0xFE, 0xAA, 0xAA, /* Line 14.             */
    0xAA, 0xAB, 0xFB, 0xFF, 0xFF,
    0xFF, 0xAA, 0xAA, /* Line 15.             */
    0xAA, 0xAB, 0xFF, 0xFF, 0xFF,
    0xFF, 0xAA, 0xAA, /* Line 16.             */
    0xAA, 0xAB, 0xFF, 0xFF, 0xFF,
    0xFF, 0xAA, 0xAA, /* Line 17.             */
    0xAA, 0xAA, 0xFF, 0xFF, 0xFF,
    0xFF, 0xAA, 0xAA, /* Line 18.             */
    0xAA, 0xAA, 0xBF, 0xFF, 0xFF,
    0xFF, 0xAA, 0xAA, /* Line 19.             */
    0xAA, 0xAA, 0xBF, 0xFF, 0xFF,
    0xFF, 0xAA, 0xAA, /* Line 20.             */
    0xAA, 0xAA, 0xAF, 0xFF, 0xFF,
    0xFF, 0xAA, 0xAA, /* Line 21.             */
    0xAA, 0xAA, 0xAF, 0xFF, 0xFF,
    0xFE, 0xAA, 0xAA, /* Line 22.             */
    0xAA, 0xAA, 0xAB, 0xFF, 0xFF,
    0xFE, 0xAA, 0xAA, /* Line 23.             */
    0xAA, 0xAA, 0xAB, 0xFF, 0xFF,
    0xFE, 0xAA, 0xAA, /* Line 24.             */
    0xAA, 0xAA, 0xAA, 0xFF, 0xFF,
    0xFA, 0xAA, 0xAA, /* Line 25.             */
    0xAA, 0xAA, 0xAA, 0xFF, 0xFF,
    0xFA, 0xAA, 0xAA, /* Line 26.             */
    0xAA, 0xAA, 0xAA, 0xFF, 0xFF,
    0xFA, 0xAA, 0xAA, /* Line 27.             */
    0xAA, 0xAA, 0xAA, 0xAA, 0xAA,
    0xAA, 0xAA, 0xAA, /* Line 28.             */
    0xAA, 0xAA, 0xAA, 0xAA, 0xAA,
    0xAA, 0xAA, 0xAA, /* Line 29.             */
    0xAA, 0xAA, 0xAA, 0xAA, 0xAA,
    0xAA, 0xAA, 0xAA, /* Line 30.             */
    0xAA, 0xAA, 0xAA, 0xAA, 0xAA,
    0xAA, 0xAA, 0xAA, /* Line 31.             */
    0xAA, 0xAA, 0xAA, 0xAA, 0xAA,
    0xAA, 0xAA, 0xAA /* Line 32.             */
};

/*******************************************************************************
 * Code
 ******************************************************************************/
static void BOARD_InitPWM(void)
{
    sctimer_config_t config;
    sctimer_pwm_signal_param_t pwmParam;
    uint32_t event;

    CLOCK_AttachClk(kMCLK_to_SCT_CLK);

    CLOCK_SetClkDiv(kCLOCK_DivSctClk, 2, true);

    SCTIMER_GetDefaultConfig(&config);

    SCTIMER_Init(SCT0, &config);

    pwmParam.output = kSCTIMER_Out_5;
    pwmParam.level = kSCTIMER_HighTrue;
    pwmParam.dutyCyclePercent = 5;

    SCTIMER_SetupPwm(SCT0, &pwmParam, kSCTIMER_CenterAlignedPwm, 1000U, CLOCK_GetFreq(kCLOCK_Sct), &event);
}

void APP_LCD_IRQHandler(void)
{
    uint32_t intStatus = LCDC_GetEnabledInterruptsPendingStatus(APP_LCD);

    LCDC_ClearInterruptsStatus(APP_LCD, intStatus);

    if (intStatus & kLCDC_VerticalCompareInterrupt)
    {
        s_frameEndFlag = true;
    }
    __DSB();
    /* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F Store immediate overlapping
      exception return operation might vector to incorrect interrupt */
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
}

static void APP_FillBuffer(void *buffer)
{
    /*
     * Fill the frame buffer, show rectangle in the center.
     */
    uint8_t(*buf)[IMG_WIDTH / APP_PIXEL_PER_BYTE] = buffer;

    uint32_t i, j;

    for (i = 0; i < IMG_HEIGHT; i++)
    {
        for (j = 0; j < IMG_WIDTH / APP_PIXEL_PER_BYTE; j++)
        {
            buf[i][j] = 0x00U;
        }
    }

    for (i = IMG_HEIGHT / 4; i < IMG_HEIGHT * 3 / 4; i++)
    {
        for (j = IMG_WIDTH / APP_PIXEL_PER_BYTE / 4; j < IMG_WIDTH * 3 / APP_PIXEL_PER_BYTE / 4; j++)
        {
            buf[i][j] = 0xFFU;
        }
    }
}

int main(void)
{
    lcdc_config_t lcdConfig;
    lcdc_cursor_config_t cursorConfig;
    int32_t cursorPosX = 0;
    int32_t cursorPosY = 0;
    int8_t cursorIncX = 1;
    int8_t cursorIncY = 1;

    /* Route Main clock to LCD. */
    CLOCK_AttachClk(kMCLK_to_LCD_CLK);

    CLOCK_SetClkDiv(kCLOCK_DivLcdClk, 1, true);

    BOARD_InitPins();
    BOARD_BootClockFROHF48M();

    /* Set the back light PWM. */
    BOARD_InitPWM();

    APP_FillBuffer((void *)(s_frameBufs));

    /* Initialize the display. */
    LCDC_GetDefaultConfig(&lcdConfig);

    lcdConfig.panelClock_Hz = LCD_PANEL_CLK;
    lcdConfig.ppl = LCD_PPL;
    lcdConfig.hsw = LCD_HSW;
    lcdConfig.hfp = LCD_HFP;
    lcdConfig.hbp = LCD_HBP;
    lcdConfig.lpp = LCD_LPP;
    lcdConfig.vsw = LCD_VSW;
    lcdConfig.vfp = LCD_VFP;
    lcdConfig.vbp = LCD_VBP;
    lcdConfig.polarityFlags = LCD_POL_FLAGS;
    lcdConfig.upperPanelAddr = (uint32_t)s_frameBufs;
    lcdConfig.bpp = kLCDC_1BPP;
    lcdConfig.display = kLCDC_DisplayTFT;
    lcdConfig.swapRedBlue = false;

    LCDC_Init(APP_LCD, &lcdConfig, LCD_INPUT_CLK_FREQ);

    LCDC_SetPalette(APP_LCD, palette, ARRAY_SIZE(palette));

    /* Setup the Cursor. */
    LCDC_CursorGetDefaultConfig(&cursorConfig);

    cursorConfig.size = kLCDC_CursorSize32;
    cursorConfig.syncMode = kLCDC_CursorSync;
    cursorConfig.image[0] = (uint32_t *)cursor32Img0;

    LCDC_SetCursorConfig(APP_LCD, &cursorConfig);

    LCDC_ChooseCursor(APP_LCD, 0);
    LCDC_SetCursorPosition(APP_LCD, cursorPosX, cursorPosY);

    /* Trigger interrupt at start of every vertical back porch. */
    LCDC_SetVerticalInterruptMode(APP_LCD, kLCDC_StartOfBackPorch);
    LCDC_EnableInterrupts(APP_LCD, kLCDC_VerticalCompareInterrupt);
    NVIC_EnableIRQ(APP_LCD_IRQn);

    LCDC_EnableCursor(APP_LCD, true);

    LCDC_Start(APP_LCD);
    LCDC_PowerUp(APP_LCD);

    /*
     * The cursor will move in the range of x: [-31, IMG_WIDTH -1], y: [-31, IMG_HEIGHT -1]
     */
    while (1)
    {
        while (!s_frameEndFlag)
        {
        }

        s_frameEndFlag = false;

        /* Set cursor's new position. */
        LCDC_SetCursorPosition(APP_LCD, cursorPosX, cursorPosY);

        cursorPosX += cursorIncX;
        cursorPosY += cursorIncY;

        if (-31 == cursorPosX)
        {
            cursorIncX = 1;
        }
        else if (IMG_WIDTH - 1 == cursorPosX)
        {
            cursorIncX = -1;
        }

        if (-31 == cursorPosY)
        {
            cursorIncY = 1;
        }
        else if (IMG_HEIGHT - 1 == cursorPosY)
        {
            cursorIncY = -1;
        }
    }
}
