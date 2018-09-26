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

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include "fsl_debug_console.h"
#include "board.h"
#include "fsl_ctimer.h"

#include "pin_mux.h"
#include <stdbool.h>
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define CTIMER CTIMER3                  /* Timer 3 */
#define CTIMER_MAT0_OUT kCTIMER_Match_0 /* Match output 0 */
#define CTIMER_MAT1_OUT kCTIMER_Match_1 /* Match output 1 */
#define BUS_CLK_FREQ CLOCK_GetFreq(kCLOCK_AsyncApbClk)

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void ctimer_match0_callback(uint32_t flags);
void ctimer_match1_callback(uint32_t flags);

/* Array of function pointers for callback for each channel */
ctimer_callback_t ctimer_callback_table[] = {
    ctimer_match0_callback, ctimer_match1_callback, NULL, NULL, NULL, NULL, NULL, NULL};

/*******************************************************************************
* Variables
******************************************************************************/
/* Match Configuration for Channel 0 */
static ctimer_match_config_t matchConfig0;
/* Match Configuration for Channel 1 */
static ctimer_match_config_t matchConfig1;

/*******************************************************************************
 * Code
 ******************************************************************************/

void ctimer_match1_callback(uint32_t flags)
{
    static uint32_t count = 0;
    static uint32_t matchUpdateCount = 8;

    if (++count > matchUpdateCount)
    {
        count = 0;
        matchConfig1.matchValue >>= 1;
        matchUpdateCount <<= 1;
        if (matchUpdateCount == (1 << 8))
        {
            matchUpdateCount = 8;
            matchConfig1.matchValue = BUS_CLK_FREQ / 2;
        }
        CTIMER_SetupMatch(CTIMER, CTIMER_MAT1_OUT, &matchConfig1);
    }
}

void ctimer_match0_callback(uint32_t flags)
{
    static uint32_t count = 0;
    static uint32_t matchUpdateCount = 8;

    if (++count > matchUpdateCount)
    {
        count = 0;
        matchConfig0.matchValue >>= 1;
        matchUpdateCount <<= 1;
        if (matchUpdateCount == (1 << 8))
        {
            matchUpdateCount = 8;
            matchConfig0.matchValue = BUS_CLK_FREQ / 2;
        }
        CTIMER_SetupMatch(CTIMER, CTIMER_MAT0_OUT, &matchConfig0);
    }
}

/*!
 * @brief Main function
 */
int main(void)
{
    ctimer_config_t config;

    /* Init hardware*/
    /* attach 12 MHz clock to FLEXCOMM0 (debug console) */
    CLOCK_AttachClk(BOARD_DEBUG_UART_CLK_ATTACH);

    /* Enable the asynchronous bridge */
    SYSCON->ASYNCAPBCTRL = 1;

    /* Use 12 MHz clock for some of the Ctimers */
    CLOCK_AttachClk(kFRO12M_to_ASYNC_APB);

    BOARD_InitPins();
    BOARD_BootClockFROHF48M();
    BOARD_InitDebugConsole();

    PRINTF("CTimer match example to toggle the output. \r\n");
    PRINTF("This example uses interrupt to change the match period. \r\n");

    CTIMER_GetDefaultConfig(&config);

    CTIMER_Init(CTIMER, &config);

    /* Configuration 0 */
    matchConfig0.enableCounterReset = true;
    matchConfig0.enableCounterStop = false;
    matchConfig0.matchValue = BUS_CLK_FREQ / 2;
    matchConfig0.outControl = kCTIMER_Output_Toggle;
    matchConfig0.outPinInitState = false;
    matchConfig0.enableInterrupt = true;

    /* Configuration 1 */
    matchConfig1.enableCounterReset = true;
    matchConfig1.enableCounterStop = false;
    matchConfig1.matchValue = BUS_CLK_FREQ / 2;
    matchConfig1.outControl = kCTIMER_Output_Toggle;
    matchConfig1.outPinInitState = true;
    matchConfig1.enableInterrupt = true;

    CTIMER_RegisterCallBack(CTIMER, &ctimer_callback_table[0], kCTIMER_MultipleCallback);
    CTIMER_SetupMatch(CTIMER, CTIMER_MAT0_OUT, &matchConfig0);
    CTIMER_SetupMatch(CTIMER, CTIMER_MAT1_OUT, &matchConfig1);
    CTIMER_StartTimer(CTIMER);

    while (1)
    {
    }
}
