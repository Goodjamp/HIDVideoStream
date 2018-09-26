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
#include "fsl_mcan.h"
#include "board.h"

#include "pin_mux.h"
#include <stdbool.h>
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define EXAMPLE_MCAN_IRQHandler CAN0_IRQ0_IRQHandler
#define EXAMPLE_MCAN_IRQn CAN0_IRQ0_IRQn
#define EXAMPLE_MCAN CAN0
#define MCAN_CLK_FREQ CLOCK_GetFreq(kCLOCK_MCAN0)
#define STDID_OFFSET 18U
#define MSG_RAM_BASE 0x20010000U
#define STD_FILTER_OFS 0x0
#define RX_FIFO0_OFS 0x10U
#define TX_BUFFER_OFS 0x20U

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
volatile bool rxComplete = false;
mcan_rx_buffer_frame_t rxFrame;

/*******************************************************************************
 * Code
 ******************************************************************************/

void CAN0_IRQ0_IRQHandler(void)
{
    MCAN_ClearStatusFlag(EXAMPLE_MCAN, CAN_IR_RF0N_MASK);
    MCAN_ReadRxFifo(EXAMPLE_MCAN, 0, &rxFrame);
    rxComplete = true;
    /* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F Store immediate overlapping
      exception return operation might vector to incorrect interrupt */
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
}

/*!
 * @brief Main function
 */
int main(void)
{
    mcan_config_t mcanConfig;
    mcan_frame_filter_config_t rxFilter;
    mcan_std_filter_element_config_t stdFilter;
    mcan_rx_fifo_config_t rxFifo0;
    mcan_tx_buffer_config_t txBuffer;
    mcan_tx_buffer_frame_t txFrame;

    /* Initialize board hardware. */
    /* attach 12 MHz clock to FLEXCOMM0 (debug console) */
    CLOCK_AttachClk(BOARD_DEBUG_UART_CLK_ATTACH);

    /* Set MCAN clock 48/6=8MHz. */
    CLOCK_SetClkDiv(kCLOCK_DivCan0Clk, 6U, true);

    BOARD_InitPins();
    BOARD_BootClockFROHF48M();
    BOARD_InitDebugConsole();

    PRINTF("\r\n==MCAN loopback functional example -- Start.==\r\n\r\n");

    MCAN_GetDefaultConfig(&mcanConfig);
    mcanConfig.enableLoopBackExt = true;
    MCAN_Init(EXAMPLE_MCAN, &mcanConfig, MCAN_CLK_FREQ);

    /* Set Message RAM base address and clear to avoid BEU/BEC error. */
    MCAN_SetMsgRAMBase(EXAMPLE_MCAN, MSG_RAM_BASE);
    uint32_t *p=(uint32_t *)(MSG_RAM_BASE);
    memset(p, 0, TX_BUFFER_OFS + 0x10U);

    /* STD filter config. */
    rxFilter.address = STD_FILTER_OFS;
    rxFilter.idFormat = kMCAN_FrameIDStandard;
    rxFilter.listSize = 1U;
    rxFilter.nmFrame = kMCAN_reject0;
    rxFilter.remFrame = kMCAN_rejectFrame;
    MCAN_SetFilterConfig(EXAMPLE_MCAN, &rxFilter);
    
    stdFilter.sfec = kMCAN_storeinFifo0;
    /* Classic filter mode, only filter matching ID. */
    stdFilter.sft = kMCAN_classic;
    stdFilter.sfid1 = 0x123U;
    stdFilter.sfid2 = 0x7FFU;
    MCAN_SetSTDFilterElement(EXAMPLE_MCAN, &rxFilter, &stdFilter, 0);

    /* RX fifo0 config. */
    rxFifo0.address = RX_FIFO0_OFS;
    rxFifo0.elementSize = 1U;
    rxFifo0.watermark = 0;
    rxFifo0.opmode = kMCAN_FifoBlocking;
    rxFifo0.datafieldSize = kMCAN_8ByteDatafield;
    MCAN_SetRxFifo0Config(EXAMPLE_MCAN, &rxFifo0);

    /* TX buffer config. */
    txBuffer.address = TX_BUFFER_OFS;
    txBuffer.dedicatedSize = 1U;
    txBuffer.fqSize = 0;
    txBuffer.datafieldSize = kMCAN_8ByteDatafield;
    MCAN_SetTxBufferConfig(EXAMPLE_MCAN, &txBuffer);

    /* Enable RX fifo0 new message interrupt using interrupt line 0. */
    MCAN_EnableInterrupts(EXAMPLE_MCAN, 0, CAN_IE_RF0NE_MASK);
    EnableIRQ(CAN0_IRQ0_IRQn);

    /* Enter normal mode. */
    MCAN_EnterNormalMode(EXAMPLE_MCAN);

    /* Config TX frame data. */
    uint8_t tx_data[8]={0x0,0x1U,0x2U,0x3U,0x4U,0x5U,0x6U,0x7U};
    txFrame.xtd = kMCAN_FrameIDStandard;
    txFrame.rtr = kMCAN_FrameTypeData;
    txFrame.fdf = 0;
    txFrame.brs = 0;
    txFrame.dlc = 8U;
    txFrame.id = 0x123U << STDID_OFFSET;
    txFrame.data = tx_data;
    txFrame.size = 8U;
    MCAN_TransferSendBlocking(EXAMPLE_MCAN, 0, &txFrame);

    while(!rxComplete)
    {
    }
    PRINTF("Received Frame ID: 0x%x\r\n", rxFrame.id >> STDID_OFFSET);
    PRINTF("Received Frame DATA: ");
    while(rxFrame.dlc--)
    {
        PRINTF("0x%x ", *(rxFrame.data++));
    }
    PRINTF("\r\n\r\n==MCAN loopback functional example -- Finish.==\r\n");

    while (1)
    {
    }
}
