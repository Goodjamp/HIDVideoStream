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

#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_spifi_dma.h"

#include "pin_mux.h"
#include <stdbool.h>
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define EXAMPLE_SPIFI SPIFI0
#define PAGE_SIZE (256)
#define SECTOR_SIZE (4096)
#define EXAMPLE_SPI_BAUDRATE (96000000)
#define EXAMPLE_DMA DMA0
#define EXAMPLE_SPIFI_CHANNEL 18
#define COMMAND_NUM (6)
#define READ (0)
#define PROGRAM_PAGE (1)
#define GET_STATUS (2)
#define ERASE_SECTOR (3)
#define WRITE_ENABLE (4)
#define WRITE_REGISTER (5)

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static dma_handle_t s_DmaHandle;
static spifi_dma_handle_t handle;
static volatile bool finished = false;
/*******************************************************************************
 * Variables
 ******************************************************************************/
uint8_t g_buffer[PAGE_SIZE] = {0};

spifi_command_t command[COMMAND_NUM] = {
    {PAGE_SIZE, false, kSPIFI_DataInput, 1, kSPIFI_CommandDataQuad, kSPIFI_CommandOpcodeAddrThreeBytes, 0x6B},
    {PAGE_SIZE, false, kSPIFI_DataOutput, 0, kSPIFI_CommandOpcodeSerial, kSPIFI_CommandOpcodeAddrThreeBytes, 0x38},
    {4, false, kSPIFI_DataInput, 0, kSPIFI_CommandAllSerial, kSPIFI_CommandOpcodeOnly, 0x05},
    {0, false, kSPIFI_DataOutput, 0, kSPIFI_CommandAllSerial, kSPIFI_CommandOpcodeAddrThreeBytes, 0x20},
    {0, false, kSPIFI_DataOutput, 0, kSPIFI_CommandAllSerial, kSPIFI_CommandOpcodeOnly, 0x06},
    {4, false, kSPIFI_DataOutput, 0, kSPIFI_CommandAllSerial, kSPIFI_CommandOpcodeOnly, 0x01}};

/*******************************************************************************
 * Code
 ******************************************************************************/
void callback(SPIFI_Type *base, spifi_dma_handle_t *handle, status_t status, void *userData)
{
    finished = true;
}

void check_if_finish()
{
    uint32_t val = 0;
    /* Check WIP bit */
    do
    {
        SPIFI_SetCommand(EXAMPLE_SPIFI, &command[GET_STATUS]);
        while ((EXAMPLE_SPIFI->STAT & SPIFI_STAT_INTRQ_MASK) == 0U)
        {
        }
        val = SPIFI_ReadData(EXAMPLE_SPIFI);
    } while (val & 0x1);
}

void enable_quad_mode()
{
    /* Write enable */
    SPIFI_SetCommand(EXAMPLE_SPIFI, &command[WRITE_ENABLE]);

    /* Set write register command */
    SPIFI_SetCommand(EXAMPLE_SPIFI, &command[WRITE_REGISTER]);

    SPIFI_WriteData(EXAMPLE_SPIFI, 0x40);

    check_if_finish();
}

int main(void)
{
    spifi_config_t config = {0};
    spifi_transfer_t xfer = {0};
    uint32_t sourceClockFreq;
    uint32_t i = 0, page = 0, err = 0;
    uint8_t *val = (uint8_t *)FSL_FEATURE_SPIFI_START_ADDR;

    /* Init the boards */
    /* attach 12 MHz clock to FLEXCOMM0 (debug console) */
    CLOCK_AttachClk(BOARD_DEBUG_UART_CLK_ATTACH);

    /* attach 12 MHz clock to SPI3 */
    CLOCK_AttachClk(kFRO12M_to_FLEXCOMM9);

    BOARD_InitPins();
    BOARD_BootClockFROHF96M();
    BOARD_InitDebugConsole();
    PRINTF("SPIFI flash dma example started \r\n");

    DMA_Init(EXAMPLE_DMA);

    DMA_EnableChannel(EXAMPLE_DMA, EXAMPLE_SPIFI_CHANNEL);
    DMA_CreateHandle(&s_DmaHandle, EXAMPLE_DMA, EXAMPLE_SPIFI_CHANNEL);

    /* Set SPIFI clock source */
    CLOCK_AttachClk(kFRO_HF_to_SPIFI_CLK);
    sourceClockFreq = CLOCK_GetSpifiClkFreq();

    /* Set the clock divider */
    CLOCK_SetClkDiv(kCLOCK_DivSpifiClk, sourceClockFreq / EXAMPLE_SPI_BAUDRATE, false);

    /* Initialize SPIFI */
    SPIFI_GetDefaultConfig(&config);
    SPIFI_Init(EXAMPLE_SPIFI, &config);
    SPIFI_TransferRxCreateHandleDMA(EXAMPLE_SPIFI, &handle, callback, NULL, &s_DmaHandle);

    /* Enable Quad mode */
    enable_quad_mode();

    /* Setup memory command */
    SPIFI_SetMemoryCommand(EXAMPLE_SPIFI, &command[READ]);

    /* Set the buffer */
    for (i = 0; i < PAGE_SIZE; i++)
    {
        g_buffer[i] = i;
    }

    /* Reset the SPIFI to switch to command mode */
    SPIFI_ResetCommand(EXAMPLE_SPIFI);

    /* Write enable */
    SPIFI_SetCommand(EXAMPLE_SPIFI, &command[WRITE_ENABLE]);
    /* Set address */
    SPIFI_SetCommandAddress(EXAMPLE_SPIFI, FSL_FEATURE_SPIFI_START_ADDR);
    /* Erase sector */
    SPIFI_SetCommand(EXAMPLE_SPIFI, &command[ERASE_SECTOR]);

    /* Check if finished */
    check_if_finish();

    /* Program page */
    while (page < (SECTOR_SIZE / PAGE_SIZE))
    {
        SPIFI_SetCommand(EXAMPLE_SPIFI, &command[WRITE_ENABLE]);
        SPIFI_SetCommandAddress(EXAMPLE_SPIFI, FSL_FEATURE_SPIFI_START_ADDR + page * PAGE_SIZE);
        SPIFI_SetCommand(EXAMPLE_SPIFI, &command[PROGRAM_PAGE]);
        xfer.data = g_buffer;
        xfer.dataSize = PAGE_SIZE;
        SPIFI_TransferSendDMA(EXAMPLE_SPIFI, &handle, &xfer);
        while (!finished)
        {
        }
        finished = false;
        page++;
        check_if_finish();
    }

    /* Reset to memory command mode */
    SPIFI_ResetCommand(EXAMPLE_SPIFI);

    SPIFI_SetMemoryCommand(EXAMPLE_SPIFI, &command[READ]);

    for (i = 0; i < SECTOR_SIZE; i++)
    {
        val = (uint8_t *)(FSL_FEATURE_SPIFI_START_ADDR + i);
        if (*val != g_buffer[i % PAGE_SIZE])
        {
            PRINTF("Data error in address 0x%x, the value in memory is 0x%x\r\n", i, *val);
            err++;
        }
    }

    if (err == 0)
    {
        PRINTF("All data written is correct!\r\n");
    }

    PRINTF("SPIFI DMA example Finished!\r\n");
    while (1)
    {
    }
}
