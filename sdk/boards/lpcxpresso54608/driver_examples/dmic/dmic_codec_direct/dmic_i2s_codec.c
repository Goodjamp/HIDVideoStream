/*
 * The Clear BSD License
 * Copyright (c) 2016, NXP Semiconductors.
 * All rights reserved.
 *
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
#include "board.h"
#include "fsl_dmic.h"
#include "fsl_i2c.h"
#include "fsl_i2s.h"
#include "fsl_wm8904.h"
#include <stdlib.h>
#include <string.h>

#include "pin_mux.h"
#include <stdbool.h>
/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define DEMO_I2C (I2C2)
#define DEMO_I2C_MASTER_CLOCK_FREQUENCY (12000000)
#define DEMO_I2S_TX (I2S1)
#define DEMO_I2S_RX (I2S0)

#define FIFO_DEPTH 0U

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
* Variables
******************************************************************************/

static i2s_config_t tx_config;

/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
* @brief Call back for DMIC0 Interrupt
*/
void DMIC0_Callback(void)
{
    /* In this example this interrupt is disabled */
}

/*!
* @brief Main function
*/

int main(void)
{
    dmic_channel_config_t dmic_channel_cfg;

    i2c_master_config_t i2cConfig;

    wm8904_config_t codecConfig;
    wm8904_handle_t codecHandle;

    /* Board pin, clock, debug console init */
    CLOCK_EnableClock(kCLOCK_InputMux);
    CLOCK_EnableClock(kCLOCK_Iocon);
    CLOCK_EnableClock(kCLOCK_Gpio0);
    CLOCK_EnableClock(kCLOCK_Gpio1);

    /* USART0 clock */
    CLOCK_AttachClk(BOARD_DEBUG_UART_CLK_ATTACH);

    /* I2C clock */
    CLOCK_AttachClk(kFRO12M_to_FLEXCOMM2);

    /* I2S clocks */
    CLOCK_AttachClk(kFRO_HF_to_FLEXCOMM6);
    CLOCK_AttachClk(kFRO_HF_to_FLEXCOMM7);

    /* DMIC uses 12MHz FRO clock */
    CLOCK_AttachClk(kFRO12M_to_DMIC);

    /*12MHz divided by 15 = 800KHz PDM clock */
    CLOCK_SetClkDiv(kCLOCK_DivDmicClk, 14, false);

    /* Enable high speed clock for I2S*/
    SYSCON->FROCTRL |= (uint32_t)SYSCON_FROCTRL_HSPDCLK(1U);
    SYSCON->MCLKDIV = SYSCON_MCLKDIV_DIV(1 - 1);
    SYSCON->MCLKIO = 1U;

    /* Attach high speed clock to MCLK for I2S */
    CLOCK_AttachClk(kFRO_HF_to_MCLK);

    /* reset FLEXCOMM for I2C */
    RESET_PeripheralReset(kFC2_RST_SHIFT_RSTn);

    /* reset FLEXCOMM for I2S */
    RESET_PeripheralReset(kFC6_RST_SHIFT_RSTn);
    RESET_PeripheralReset(kFC7_RST_SHIFT_RSTn);

    BOARD_InitPins();
    BOARD_BootClockFROHF48M();
    BOARD_InitDebugConsole();

    PRINTF("Configure I2C\r\n");

    /*
     * enableMaster = true;
     * baudRate_Hz = 100000U;
     * enableTimeout = false;
     */
    I2C_MasterGetDefaultConfig(&i2cConfig);
    i2cConfig.baudRate_Bps = WM8904_I2C_BITRATE;
    I2C_MasterInit(DEMO_I2C, &i2cConfig, DEMO_I2C_MASTER_CLOCK_FREQUENCY);

    PRINTF("Configure WM8904 codec\r\n");

    WM8904_GetDefaultConfig(&codecConfig);
    codecHandle.i2c = DEMO_I2C;
    codecConfig.format.sampleRate = kWM8904_SampleRate16kHz;
    if (WM8904_Init(&codecHandle, &codecConfig) != kStatus_Success)
    {
        PRINTF("WM8904_Init failed!\r\n");
    }

    /* Adjust it to your needs, 0x0006 for -51 dB, 0x0039 for 0 dB etc. */
    WM8904_SetVolume(&codecHandle, 0x0030, 0x0030);

    dmic_channel_cfg.divhfclk = kDMIC_PdmDiv1;
    dmic_channel_cfg.osr = 25U;
    dmic_channel_cfg.gainshft = 3U;
    dmic_channel_cfg.preac2coef = kDMIC_CompValueZero;
    dmic_channel_cfg.preac4coef = kDMIC_CompValueZero;
    dmic_channel_cfg.dc_cut_level = kDMIC_DcCut155;
    dmic_channel_cfg.post_dc_gain_reduce = 1U;
    dmic_channel_cfg.saturate16bit = 1U;
    dmic_channel_cfg.sample_rate = kDMIC_PhyFullSpeed;
    DMIC_Init(DMIC0);

    DMIC_ConfigIO(DMIC0, kDMIC_PdmDual);
    DMIC_Use2fs(DMIC0, true);
    DMIC_SetOperationMode(DMIC0, kDMIC_OperationModePoll);
    DMIC_ConfigChannel(DMIC0, kDMIC_Channel0, kDMIC_Left, &dmic_channel_cfg);
    DMIC_ConfigChannel(DMIC0, kDMIC_Channel1, kDMIC_Right, &dmic_channel_cfg);

    /* FIFO disabled */
    DMIC_FifoChannel(DMIC0, kDMIC_Channel0, FIFO_DEPTH, false, true);
    DMIC_FifoChannel(DMIC0, kDMIC_Channel1, FIFO_DEPTH, false, true);
    DMIC_EnableChannnel(DMIC0, (DMIC_CHANEN_EN_CH0(1) | DMIC_CHANEN_EN_CH1(1)));
    DMIC_DisableIntCallback(DMIC0, DMIC0_Callback);
    PRINTF("Configure I2S\r\n");

    /*
     * masterSlave = kI2S_MasterSlaveNormalMaster;
     * mode = kI2S_ModeI2sClassic;
     * rightLow = false;
     * leftJust = false;
     * pdmData = false;
     * sckPol = false;
     * wsPol = false;
     * divider = 1;
     * oneChannel = false;
     * dataLength = 16;
     * frameLength = 32;
     * position = 0;
     * fifoLevel = 4;
     */
    I2S_TxGetDefaultConfig(&tx_config);
    tx_config.divider = 16U; /* divide by 16*/
    tx_config.pdmData = true;
    /*
     * masterSlave = kI2S_MasterSlaveNormalSlave;
     * mode = kI2S_ModeI2sClassic;
     * rightLow = false;
     * leftJust = false;
     * pdmData = false;
     * sckPol = false;
     * wsPol = false;
     * divider = 1;
     * oneChannel = false;
     * dataLength = 16;
     * frameLength = 32;
     * position = 0;
     * fifoLevel = 4;
     */
    I2S_TxInit(DEMO_I2S_TX, &tx_config);
    I2S_Enable(DEMO_I2S_TX);

    while (1)
    {
    }
}
