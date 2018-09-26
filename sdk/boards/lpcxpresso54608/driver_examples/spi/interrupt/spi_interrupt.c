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

#include "fsl_spi.h"
#include "board.h"
#include "fsl_debug_console.h"

#include "pin_mux.h"
#include <stdbool.h>
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define EXAMPLE_SPI_MASTER SPI9
#define EXAMPLE_SPI_MASTER_IRQ FLEXCOMM9_IRQn
#define EXAMPLE_SPI_MASTER_CLK_SRC kCLOCK_Flexcomm9
#define EXAMPLE_SPI_MASTER_CLK_FREQ CLOCK_GetFreq(kCLOCK_Flexcomm9)
#define EXAMPLE_SPI_SLAVE SPI3
#define EXAMPLE_SPI_SLAVE_IRQ FLEXCOMM3_IRQn
#define EXAMPLE_SPI_SSEL 0
#define SPI_MASTER_IRQHandler FLEXCOMM9_IRQHandler
#define SPI_SLAVE_IRQHandler FLEXCOMM3_IRQHandler
#define EXAMPLE_SPI_SPOL kSPI_SpolActiveAllLow

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
#define BUFFER_SIZE (64)
static uint8_t srcBuff[BUFFER_SIZE];
static uint8_t destBuff[BUFFER_SIZE];
static uint32_t masterIndex = BUFFER_SIZE;
static uint32_t slaveIndex = BUFFER_SIZE;
static volatile bool masterFinished = false;
static volatile bool slaveFinished = false;

/*******************************************************************************
 * Code
 ******************************************************************************/

void SPI_MASTER_IRQHandler(void)
{
    /* read data to avoid rxOverflow */
    while (SPI_GetStatusFlags(EXAMPLE_SPI_MASTER) & kSPI_RxNotEmptyFlag)
    {
        SPI_ReadData(EXAMPLE_SPI_MASTER);
    }
    /* send data if buffer is not full */
    if (SPI_GetStatusFlags(EXAMPLE_SPI_MASTER) & kSPI_TxNotFullFlag)
    {
        if (masterIndex == 1)
        {
            /* need to disable interrupts before write last data */
            SPI_DisableInterrupts(EXAMPLE_SPI_MASTER, kSPI_TxLvlIrq);
            SPI_WriteData(EXAMPLE_SPI_MASTER, (uint16_t)(srcBuff[BUFFER_SIZE - masterIndex]), kSPI_FrameAssert);
        }
        else
        {
            SPI_WriteData(EXAMPLE_SPI_MASTER, (uint16_t)(srcBuff[BUFFER_SIZE - masterIndex]), 0);
        }
        masterIndex--;
    }
    if (masterIndex == 0U)
    {
        masterFinished = true;
    }
    /* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F Store immediate overlapping
      exception return operation might vector to incorrect interrupt */
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
}

void SPI_SLAVE_IRQHandler(void)
{
    /* fill txFIFO with dummy pattern */
    while (SPI_GetStatusFlags(EXAMPLE_SPI_SLAVE) & kSPI_TxNotFullFlag)
    {
        SPI_WriteData(EXAMPLE_SPI_SLAVE, 0xFFFF, 0);
    }
    /* read rxFIFO if is not empty */
    if (SPI_GetStatusFlags(EXAMPLE_SPI_SLAVE) & kSPI_RxNotEmptyFlag)
    {
        destBuff[BUFFER_SIZE - slaveIndex] = SPI_ReadData(EXAMPLE_SPI_SLAVE);
        slaveIndex--;
    }
    if (slaveIndex == 0U)
    {
        slaveFinished = true;
        SPI_DisableInterrupts(EXAMPLE_SPI_SLAVE, kSPI_RxLvlIrq | kSPI_TxLvlIrq);
    }
    /* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F Store immediate overlapping
      exception return operation might vector to incorrect interrupt */
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
}

int main(void)
{
    spi_master_config_t masterConfig = {0};
    spi_slave_config_t slaveConfig = {0};
    uint32_t sourceClock = 0U;
    uint32_t i = 0U;
    uint32_t err = 0U;

    /* Init the boards */
    /* attach 12 MHz clock to FLEXCOMM0 (debug console) */
    CLOCK_AttachClk(BOARD_DEBUG_UART_CLK_ATTACH);

    /* attach 12 MHz clock to SPI3 */
    CLOCK_AttachClk(kFRO12M_to_FLEXCOMM3);

    /* attach 12 MHz clock to SPI9 */
    CLOCK_AttachClk(kFRO12M_to_FLEXCOMM9);

    BOARD_InitPins();
    BOARD_BootClockFROHF48M();
    BOARD_InitDebugConsole();

    PRINTF("\r\nSPI one board interrupt example started!\r\n");

    /* Init SPI master */
    /*
     * masterConfig.enableLoopback = false;
     * masterConfig.enableMaster = true;
     * masterConfig.polarity = kSPI_ClockPolarityActiveHigh;
     * masterConfig.phase = kSPI_ClockPhaseFirstEdge;
     * masterConfig.direction = kSPI_MsbFirst;
     * masterConfig.baudRate_Bps = 500000U;
     */
    SPI_MasterGetDefaultConfig(&masterConfig);
    sourceClock = EXAMPLE_SPI_MASTER_CLK_FREQ;
    masterConfig.sselNum = (spi_ssel_t)EXAMPLE_SPI_SSEL;
    masterConfig.sselPol = (spi_spol_t)EXAMPLE_SPI_SPOL;
    SPI_MasterInit(EXAMPLE_SPI_MASTER, &masterConfig, sourceClock);
    /* Init SPI slave */
    /*
     * slaveConfig.enableSlave = true;
     * slaveConfig.polarity = kSPI_ClockPolarityActiveHigh;
     * slaveConfig.phase = kSPI_ClockPhaseFirstEdge;
     * slaveConfig.direction = kSPI_MsbFirst;
     */
    SPI_SlaveGetDefaultConfig(&slaveConfig);
    slaveConfig.sselPol = (spi_spol_t)EXAMPLE_SPI_SPOL;
    SPI_SlaveInit(EXAMPLE_SPI_SLAVE, &slaveConfig);
    /* Set priority, slave have higher priority */
    NVIC_SetPriority(EXAMPLE_SPI_MASTER_IRQ, 1U);
    NVIC_SetPriority(EXAMPLE_SPI_SLAVE_IRQ, 0U);

    /* Init source buffer */
    for (i = 0U; i < BUFFER_SIZE; i++)
    {
        srcBuff[i] = i;
    }

    /* Enable interrupt, first enable slave and then master. */
    EnableIRQ(EXAMPLE_SPI_MASTER_IRQ);
    EnableIRQ(EXAMPLE_SPI_SLAVE_IRQ);
    SPI_EnableInterrupts(EXAMPLE_SPI_SLAVE, kSPI_RxLvlIrq | kSPI_TxLvlIrq);
    SPI_EnableInterrupts(EXAMPLE_SPI_MASTER, kSPI_TxLvlIrq);

    while ((masterFinished != true) || (slaveFinished != true))
    {
    }

    /* Check the data received */
    for (i = 0U; i < BUFFER_SIZE; i++)
    {
        if (destBuff[i] != srcBuff[i])
        {
            PRINTF("\r\nThe %d data is wrong, the data received is %d \r\n", i, destBuff[i]);
            err++;
        }
    }
    if (err == 0U)
    {
        PRINTF("\r\nSPI transfer finished!\r\n");
    }

    while (1)
    {
    }
}
