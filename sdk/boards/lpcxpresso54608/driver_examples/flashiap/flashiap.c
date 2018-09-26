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

#include "fsl_debug_console.h"
#include "board.h"

#include "fsl_flashiap.h"

#include "fsl_common.h"
#include "pin_mux.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/


/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

uint32_t page_buf[FSL_FEATURE_SYSCON_FLASH_PAGE_SIZE_BYTES / sizeof(uint32_t)];

/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
* @brief Main function
*/
int main(void)
{
    uint32_t i, status;

    /* Board pin, clock, debug console init */
    /* attach 12 MHz clock to FLEXCOMM0 (debug console) */
    CLOCK_AttachClk(BOARD_DEBUG_UART_CLK_ATTACH);

    BOARD_InitPins_Core0();
    BOARD_BootClockFROHF48M();
    BOARD_InitDebugConsole();

    /* Clear screen*/
    PRINTF("%c[2J", 27);
    /* Set cursor location at [0,0] */
    PRINTF("%c[0;0H", 27);
    PRINTF("\f\r\nFLASHIAP example\r\n");
    PRINTF("\r\nWriting flash sector 1\n");
    /* Erase sector before writing */
    FLASHIAP_PrepareSectorForWrite(1, 1);
    FLASHIAP_EraseSector(1, 1, SystemCoreClock);
    /* Generate data to be written to flash */
    for (i = 0; i < FSL_FEATURE_SYSCON_FLASH_PAGE_SIZE_BYTES; i++)
    {
        *(((uint8_t *)(&page_buf[0])) + i) = i;
    }
    /* Program sector */
    for (i = 0; i < (FSL_FEATURE_SYSCON_FLASH_SECTOR_SIZE_BYTES / FSL_FEATURE_SYSCON_FLASH_PAGE_SIZE_BYTES); i++)
    {
        FLASHIAP_PrepareSectorForWrite(1, 1);
        FLASHIAP_CopyRamToFlash(
            FSL_FEATURE_SYSCON_FLASH_SECTOR_SIZE_BYTES + (i * FSL_FEATURE_SYSCON_FLASH_PAGE_SIZE_BYTES), &page_buf[0],
            FSL_FEATURE_SYSCON_FLASH_PAGE_SIZE_BYTES, SystemCoreClock);
    }
    /* Verify sector contents */
    for (i = 0; i < (FSL_FEATURE_SYSCON_FLASH_SECTOR_SIZE_BYTES / FSL_FEATURE_SYSCON_FLASH_PAGE_SIZE_BYTES); i++)
    {
        status = FLASHIAP_Compare(
            FSL_FEATURE_SYSCON_FLASH_SECTOR_SIZE_BYTES + (i * FSL_FEATURE_SYSCON_FLASH_PAGE_SIZE_BYTES), &page_buf[0],
            FSL_FEATURE_SYSCON_FLASH_PAGE_SIZE_BYTES);

        if (status != kStatus_FLASHIAP_Success)
        {
            PRINTF("\r\nSector Verify failed\n");
            break;
        }
    }

    PRINTF("\r\nErasing flash sector 1\n");
    FLASHIAP_PrepareSectorForWrite(1, 1);
    FLASHIAP_EraseSector(1, 1, SystemCoreClock);
    status = FLASHIAP_BlankCheckSector(1, 1);
    if (status != kStatus_FLASHIAP_Success)
    {
        PRINTF("\r\nSector Erase failed");
    }

    PRINTF("\r\nErasing page 1 in flash sector 1\n");
    /* First write a page */
    FLASHIAP_PrepareSectorForWrite(1, 1);
    FLASHIAP_CopyRamToFlash(FSL_FEATURE_SYSCON_FLASH_SECTOR_SIZE_BYTES, &page_buf[0],
                            FSL_FEATURE_SYSCON_FLASH_PAGE_SIZE_BYTES, SystemCoreClock);
    /* Erase page */
    FLASHIAP_PrepareSectorForWrite(1, 1);
    FLASHIAP_ErasePage(FSL_FEATURE_SYSCON_FLASH_SECTOR_SIZE_BYTES / FSL_FEATURE_SYSCON_FLASH_PAGE_SIZE_BYTES,
                       FSL_FEATURE_SYSCON_FLASH_SECTOR_SIZE_BYTES / FSL_FEATURE_SYSCON_FLASH_PAGE_SIZE_BYTES,
                       SystemCoreClock);

    /* Fill page buffer with erased state value */
    for (i = 0; i < FSL_FEATURE_SYSCON_FLASH_PAGE_SIZE_BYTES; i++)
    {
        *(((uint8_t *)(&page_buf[0])) + i) = 0xFF;
    }
    /* Verify Erase */
    status = FLASHIAP_Compare(FSL_FEATURE_SYSCON_FLASH_SECTOR_SIZE_BYTES, (uint32_t *)(&page_buf[0]),
                              FSL_FEATURE_SYSCON_FLASH_PAGE_SIZE_BYTES);
    if (status != kStatus_FLASHIAP_Success)
    {
        PRINTF("\r\nPage Erase failed\n");
    }
    PRINTF("\r\nEnd of FLASHIAP Example \r\n");
    while (1)
    {
    }
}
