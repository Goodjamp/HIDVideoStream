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
#include "board.h"
#include "fsl_rng.h"
#include "fsl_debug_console.h"

#include "pin_mux.h"
#include <stdbool.h>
#include "fsl_clock.h"
#include "fsl_power.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define RNG_EXAMPLE_RANDOM_NUMBERS (4)
#define RNG_EXAMPLE_RANDOM_NUMBER_BITS (RNG_EXAMPLE_RANDOM_NUMBERS * 8 * sizeof(uint32_t))

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief RNG example.
 * @details Generate random numbers and print them to terminal.
 */

int main(void)
{
    uint32_t number;
    uint32_t skip;
    uint32_t data[RNG_EXAMPLE_RANDOM_NUMBERS];

    /* Init hardware*/
    /* attach 12 MHz clock to FLEXCOMM0 (debug console) */
    CLOCK_AttachClk(BOARD_DEBUG_UART_CLK_ATTACH);

    /* The clock & power for the random number generator are taken care by the ROM API */
    /*
     * POWER_DisablePD(kPDRUNCFG_PD_rng);
     * CLOCK_EnableClock(kCLOCK_Rng);
     * RESET_PeripheralReset(kRNG_RST_SHIFT_RSTn);
     */

    BOARD_InitPins();
    BOARD_BootClockFROHF96M();
    BOARD_InitDebugConsole();

    PRINTF("RNG Peripheral Driver Example\r\n\r\n");

    while (1)
    {
        PRINTF("Generate %u-bit random number: \r\n", RNG_EXAMPLE_RANDOM_NUMBER_BITS);

        /* Get Random data */
        for (number = 0; number < RNG_EXAMPLE_RANDOM_NUMBERS; number++)
        {
            data[number] = RNG_GetRandomData();

            /* Skip next 32 random numbers for better entropy */
            for (skip = 0; skip < 32; skip++)
            {
                RNG_GetRandomData();
            }
        }

        /* Print data */
        PRINTF("0x");
        for (number = 0; number < RNG_EXAMPLE_RANDOM_NUMBERS; number++)
        {
            PRINTF("%08X", data[number]);
        }
        PRINTF("\r\n");

        /* Print a note */
        PRINTF(" Press any key to continue... \r\n\r\n");
        GETCHAR();
    }
}
