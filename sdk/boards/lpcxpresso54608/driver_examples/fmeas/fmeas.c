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

#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "fsl_inputmux.h"
#include "board.h"
#include "fsl_fmeas.h"

#include "pin_mux.h"
#include <stdbool.h>
#include "fsl_power.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define EXAMPLE_REFERENCE_CLOCK (kINPUTMUX_MainClkToFreqmeas)
#define EXAMPLE_TARGET_CLOCK_1 (kINPUTMUX_WdtOscToFreqmeas)
#define EXAMPLE_TARGET_CLOCK_2 (kINPUTMUX_32KhzOscToFreqmeas)
#define EXAMPLE_TARGET_CLOCK_3 (kINPUTMUX_Fro12MhzToFreqmeas)

#define EXAMPLE_REFERENCE_CLOCK_NAME "main clock"
#define EXAMPLE_TARGET_CLOCK_1_NAME "Watchdog oscillator"
#define EXAMPLE_TARGET_CLOCK_2_NAME "RTC32K oscillator"
#define EXAMPLE_TARGET_CLOCK_3_NAME "FRO oscillator"

#define EXAMPLE_REFERENCE_CLOCK_FREQUENCY (CLOCK_GetCoreSysClkFreq())
#define EXAMPLE_TARGET_CLOCK_1_FREQUENCY (CLOCK_GetWdtOscFreq())
#define EXAMPLE_TARGET_CLOCK_2_FREQUENCY (CLOCK_GetOsc32KFreq())
#define EXAMPLE_TARGET_CLOCK_3_FREQUENCY (CLOCK_GetFro12MFreq())

#define EXAMPLE_REFERENCE_CLOCK_REGISTRY_INDEX (0U)
#define EXAMPLE_TARGET_CLOCK_REGISTRY_INDEX (1U)


/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 * @brief    Measurement cycle with value display
 *
 * @param    srcName : Name of the measured target clock
 * @param    target  : Target (measured) clock
 * @param    freqRef : Reference frequency [Hz]
 * @param    freqExp : Expected frequency [Hz]
 *
 * @return   Measured frequency [Hz]
 */
static uint32_t MeasureDisplay(char *srcName, inputmux_connection_t target, uint32_t freqRef, uint32_t freqExp)
{
    uint32_t freqComp;

    /* Setup to measure the selected target */
    INPUTMUX_AttachSignal(INPUTMUX, EXAMPLE_TARGET_CLOCK_REGISTRY_INDEX, target);

    /* Start a measurement cycle and wait for it to complete. If the target
       clock is not running, the measurement cycle will remain active
       forever, so a timeout may be necessary if the target clock can stop */
    FMEAS_StartMeasure(SYSCON);
    while (!FMEAS_IsMeasureComplete(SYSCON))
    {
    }

    /* Get computed frequency */
    freqComp = FMEAS_GetFrequency(SYSCON, freqRef);

    /* Show the raw capture value and the compute frequency */
    PRINTF("Capture source: %s, reference frequency = %u Hz\r\n", srcName, freqRef);
    PRINTF("Computed frequency value = %u Hz\r\n", freqComp);
    PRINTF("Expected frequency value = %u Hz\r\n\r\n", freqExp);

    return freqComp;
}

/*!
 * @brief Main function
 */
int main(void)
{
    /* Init board hardware. */
    /* attach 12 MHz clock to FLEXCOMM0 (debug console) */
    CLOCK_AttachClk(BOARD_DEBUG_UART_CLK_ATTACH);

    /* enable clock for INPUTMUX */
    CLOCK_EnableClock(kCLOCK_InputMux);

    /* enable clock for IOCON */
    CLOCK_EnableClock(kCLOCK_Iocon);

    /* enable clock for watchdog */
    CLOCK_EnableClock(kCLOCK_Wwdt);

    /* Set watchdog oscilator to freq 1 MHz / 2 */
    /* TODO use API once available */
    SYSCON->WDTOSCCTRL = SYSCON_WDTOSCCTRL_DIVSEL(0x00) | SYSCON_WDTOSCCTRL_FREQSEL(0x05);
    /* Enable the bus clk for RTC */
    CLOCK_EnableClock(kCLOCK_Rtc);
    /* RTC oscillator power-on*/
    RTC->CTRL &= ~RTC_CTRL_RTC_OSC_PD_MASK;

    /* power up watchdog */
    POWER_DisablePD(kPDRUNCFG_PD_WDT_OSC);

    BOARD_InitPins();
    BOARD_BootClockFROHF48M();
    BOARD_InitDebugConsole();

    uint32_t freqRef = EXAMPLE_REFERENCE_CLOCK_FREQUENCY;

    INPUTMUX_AttachSignal(INPUTMUX, EXAMPLE_REFERENCE_CLOCK_REGISTRY_INDEX, EXAMPLE_REFERENCE_CLOCK);

    /* Start frequency measurement 1 and display results */
    MeasureDisplay(EXAMPLE_TARGET_CLOCK_1_NAME " (" EXAMPLE_REFERENCE_CLOCK_NAME " reference)", EXAMPLE_TARGET_CLOCK_1,
                   freqRef, EXAMPLE_TARGET_CLOCK_1_FREQUENCY);

    /* Start frequency measurement 2 and display results */
    MeasureDisplay(EXAMPLE_TARGET_CLOCK_2_NAME " (" EXAMPLE_REFERENCE_CLOCK_NAME " reference)", EXAMPLE_TARGET_CLOCK_2,
                   freqRef, EXAMPLE_TARGET_CLOCK_2_FREQUENCY);

    /* Start frequency measurement 3 and display results */
    MeasureDisplay(EXAMPLE_TARGET_CLOCK_3_NAME " (" EXAMPLE_REFERENCE_CLOCK_NAME " reference)", EXAMPLE_TARGET_CLOCK_3,
                   freqRef, EXAMPLE_TARGET_CLOCK_3_FREQUENCY);

    while (1)
    {
    }
}
