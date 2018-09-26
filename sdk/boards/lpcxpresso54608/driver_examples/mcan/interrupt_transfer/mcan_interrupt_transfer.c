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
volatile bool txComplete = false;
volatile bool rxComplete = false;
mcan_tx_buffer_frame_t txFrame;
mcan_rx_buffer_frame_t rxFrame;
mcan_handle_t mcanHandle;
mcan_buffer_transfer_t txXfer;
mcan_fifo_transfer_t rxXfer;
uint32_t txIdentifier;
uint32_t rxIdentifier;

/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief MCAN Call Back function
 */
static void mcan_callback(CAN_Type *base, mcan_handle_t *handle, status_t status, uint32_t result, void *userData)
{
    switch (status)
    {
        case kStatus_MCAN_RxFifo0Idle:
            {
                rxComplete = true;
            }
            break;

        case kStatus_MCAN_TxIdle:
            {
                txComplete = true;
            }
            break;

        default:
            break;
    }
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
    uint8_t node_type;
    uint8_t count = 0;

    /* Initialize board hardware. */
    /* attach 12 MHz clock to FLEXCOMM0 (debug console) */
    CLOCK_AttachClk(BOARD_DEBUG_UART_CLK_ATTACH);

    /* Set MCAN clock 48/6=8MHz. */
    CLOCK_SetClkDiv(kCLOCK_DivCan0Clk, 6U, true);

    BOARD_InitPins();
    BOARD_BootClockFROHF48M();
    BOARD_InitDebugConsole();

    do
    {
        PRINTF("Please select local node as A or B:\r\n");
        PRINTF("Note: Node B should start first.\r\n");
        PRINTF("Node:");
        node_type = GETCHAR();
        PRINTF("%c", node_type);
        PRINTF("\r\n");
    } while ((node_type != 'A') && (node_type != 'B') && (node_type != 'a') && (node_type != 'b'));

    /* Select mailbox ID. */
    if ((node_type == 'A') || (node_type == 'a'))
    {
        txIdentifier = 0x321U;
        rxIdentifier = 0x123U;
    }
    else
    {
        txIdentifier = 0x123U;
        rxIdentifier = 0x321U;
    }

    MCAN_GetDefaultConfig(&mcanConfig);
    MCAN_Init(EXAMPLE_MCAN, &mcanConfig, MCAN_CLK_FREQ);

    /* Create MCAN handle structure and set call back function. */
    MCAN_TransferCreateHandle(EXAMPLE_MCAN, &mcanHandle, mcan_callback, NULL);

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
    stdFilter.sfid1 = rxIdentifier;
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

    /* Finish software initialization and enter normal mode, synchronizes to
       CAN bus, ready for communication */
    MCAN_EnterNormalMode(EXAMPLE_MCAN);

    if ((node_type == 'A') || (node_type == 'a'))
    {
        PRINTF("Press any key to trigger one-shot transmission\r\n\r\n");
    }
    else
    {
        PRINTF("Start to Wait data from Node A\r\n\r\n");
    }
    
    while(1)
    {
        if ((node_type == 'A') || (node_type == 'a'))
        {
            GETCHAR();
            /* Config TX frame data. */
            uint8_t tx_data[8]={0x0,0x1U,0x2U,0x3U,0x4U,0x5U,0x6U,0x7U};
            tx_data[0] += count++;
            txFrame.xtd = kMCAN_FrameIDStandard;
            txFrame.rtr = kMCAN_FrameTypeData;
            txFrame.fdf = 0;
            txFrame.brs = 0;
            txFrame.dlc = 8U;
            txFrame.id = txIdentifier << STDID_OFFSET;
            txFrame.data = tx_data;
            txFrame.size = 8U;
            txXfer.frame = &txFrame;
            txXfer.bufferIdx = 0;
            MCAN_TransferSendNonBlocking(EXAMPLE_MCAN, &mcanHandle, &txXfer);

            while (!txComplete)
            {
            }
            txComplete = false;

            /* Start receive data through Rx FIFO 0. */
            rxXfer.frame = &rxFrame;
            MCAN_TransferReceiveFifoNonBlocking(EXAMPLE_MCAN, 0, &mcanHandle, &rxXfer);

            /* Wait until message received. */
            while (!rxComplete)
            {
            }
            rxComplete = false;
            PRINTF("Received Frame ID: 0x%x\r\n", rxFrame.id >> STDID_OFFSET);
            PRINTF("Received Frame DATA: ");
            while(rxFrame.dlc--)
            {
                PRINTF("0x%x ", *(rxFrame.data++));
            }
            PRINTF("\r\n");
            PRINTF("Press any key to trigger the next transmission!\r\n\r\n");
        }
        else
        {
            rxXfer.frame = &rxFrame;
            MCAN_TransferReceiveFifoNonBlocking(EXAMPLE_MCAN, 0, &mcanHandle, &rxXfer);

            while (!rxComplete)
            {
            }
            rxComplete = false;
            PRINTF("Received Frame ID: 0x%x\r\n", rxFrame.id >> STDID_OFFSET);
            PRINTF("Received Frame DATA: ");

            count = 9U;
            while(--count)
            {
                PRINTF("0x%x ", *(rxFrame.data++));
            }
            while(++count < 9U)
            {
                rxFrame.data--;
            }
            PRINTF("\r\n");

            /* Copy received frame control field and data pointer to tx frame. */
            memcpy(&txFrame, &rxFrame, 12U);
            txFrame.size = 8U;
            txFrame.id = txIdentifier << STDID_OFFSET;
            txXfer.frame = &txFrame;
            txXfer.bufferIdx = 0;
            MCAN_TransferSendNonBlocking(EXAMPLE_MCAN, &mcanHandle, &txXfer);

            while (!txComplete)
            {
            }
            txComplete = false;
            PRINTF("Wait Node A to trigger the next transmission!\r\n\r\n");
        }
    }
}
