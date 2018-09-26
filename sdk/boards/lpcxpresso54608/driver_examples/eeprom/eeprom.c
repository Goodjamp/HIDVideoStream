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
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "board.h"

#include "fsl_eeprom.h"

#include "pin_mux.h"
#include <stdbool.h>
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define EEPROM_SOURCE_CLOCK kCLOCK_BusClk
#define EEPROM_CLK_FREQ CLOCK_GetFreq(kCLOCK_BusClk)
#define EXAMPLE_EEPROM EEPROM

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
#define FSL_FEATURE_EEPROM_PAGE_SIZE (FSL_FEATURE_EEPROM_SIZE/FSL_FEATURE_EEPROM_PAGE_COUNT)
/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief Main function
 */
int main(void)
{
    uint32_t data[FSL_FEATURE_EEPROM_PAGE_SIZE/4];
    uint32_t pageNum = FSL_FEATURE_EEPROM_PAGE_COUNT;
    uint32_t i = 0, j = 0, err = 0;
    uint32_t eeprom_data = 0xFFFFFFFFU;
    eeprom_config_t config;
    uint32_t sourceClock_Hz = 0;
    
    /* attach 12 MHz clock to FLEXCOMM0 (debug console) */
    CLOCK_AttachClk(BOARD_DEBUG_UART_CLK_ATTACH);

    BOARD_InitPins();
    BOARD_BootClockFROHF48M();
    BOARD_InitDebugConsole();

    PRINTF("EEPROM example begins...\r\n");

    /* Init EEPROM */
    EEPROM_GetDefaultConfig(&config);
    sourceClock_Hz = EEPROM_CLK_FREQ;
    EEPROM_Init(EXAMPLE_EEPROM, &config, sourceClock_Hz);

    /* Prepare page data to write */
    for (i = 0; i < FSL_FEATURE_EEPROM_PAGE_SIZE/4; i++)
    {
        data[i] = i;
    }

    /* Write all data into eeprom */
    for (i = 0; i < pageNum - 1; i ++)
    {
        EEPROM_WritePage(EXAMPLE_EEPROM, i, data);
        for (j = 0; j < FSL_FEATURE_EEPROM_PAGE_SIZE/4; j++)
        {
            eeprom_data = *((uint32_t *)(FSL_FEATURE_EEPROM_BASE_ADDRESS + i * FSL_FEATURE_EEPROM_PAGE_SIZE + j * 4));
            if (eeprom_data != data[j])
            {
                err ++;
                PRINTF("Page %d offset %d is wrong, data is %x \r\n", i, (j*4), eeprom_data);
            }   
        }
        PRINTF("Page %d program finished!\r\n", i);
    }

    if (err == 0)
    {
        PRINTF("All data is correct! EEPROM example succeed!\r\n");
    }

    while(1)
    {
    }
}
