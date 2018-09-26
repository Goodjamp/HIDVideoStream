/*
 * The Clear BSD License
 * Copyright (c) 2015-2016, Freescale Semiconductor, Inc.
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

#include "fsl_crc.h"

#include "pin_mux.h"
#include <stdbool.h>
/*******************************************************************************
 * Definitions
 ******************************************************************************/


/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
* @brief Init for CRC-16-CCITT.
* @details Init CRC peripheral module for CRC-16/CCITT-FALSE protocol:
*          width=16 poly=0x1021 init=0xffff refin=false refout=false xorout=0x0000 check=0x29b1
*          http://reveng.sourceforge.net/crc-catalogue/
* name="CRC-16/CCITT-FALSE"
*/
static void InitCrc16_CcittFalse(CRC_Type *base, uint32_t seed)
{
    crc_config_t config;

    /*
     * config.polynomial = kCRC_Polynomial_CRC_CCITT;
     * config.reverseIn = false;
     * config.complementIn = false;
     * config.reverseOut = false;
     * config.complementOut = false;
     * config.seed = 0xFFFFU;
     */
    CRC_GetDefaultConfig(&config);
    config.seed = seed;
    CRC_Init(base, &config);
}

/*!
 * @brief Init for CRC-16/ARC.
 * @details Init CRC peripheral module for CRC-16/ARC protocol.
 *          width=16 poly=0x8005 init=0x0000 refin=true refout=true xorout=0x0000 check=0xbb3d name="ARC"
 *          http://reveng.sourceforge.net/crc-catalogue/
 */
static void InitCrc16(CRC_Type *base, uint32_t seed)
{
    crc_config_t config;

    config.polynomial = kCRC_Polynomial_CRC_16;
    config.reverseIn = true;
    config.complementIn = false;
    config.reverseOut = true;
    config.complementOut = false;
    config.seed = seed;

    CRC_Init(base, &config);
}

/*!
 * @brief Init for CRC-32.
 * @details Init CRC peripheral module for CRC-32 protocol.
 *          width=32 poly=0x04c11db7 init=0xffffffff refin=true refout=true xorout=0xffffffff check=0xcbf43926
 *          name="CRC-32"
 *          http://reveng.sourceforge.net/crc-catalogue/
 */
static void InitCrc32(CRC_Type *base, uint32_t seed)
{
    crc_config_t config;

    config.polynomial = kCRC_Polynomial_CRC_32;
    config.reverseIn = true;
    config.complementIn = false;
    config.reverseOut = true;
    config.complementOut = true;
    config.seed = seed;

    CRC_Init(base, &config);
}

/*!
 * @brief Main function
 */
int main(void)
{
    char testData[] = "123456789";
    const uint16_t checkCcittFalseCrc16 = 0x29b1u;
    const uint16_t checkCrc16Arc = 0xbb3du;
    const uint32_t checkCrc32 = 0xcbf43926u;

    CRC_Type *base = CRC_ENGINE;
    uint16_t checksum16;
    uint32_t checksum32;

    /* Init hardware */
    /* attach 12 MHz clock to FLEXCOMM0 (debug console) */
    CLOCK_AttachClk(BOARD_DEBUG_UART_CLK_ATTACH);

    BOARD_InitPins();
    BOARD_BootClockFROHF48M();
    BOARD_InitDebugConsole();

    PRINTF("CRC Peripheral Driver Example\r\n\r\n");

    /* ***************
    * CRC-16/CCITT-FALSE *
    *************** */
    InitCrc16_CcittFalse(base, 0xFFFFU);
    CRC_WriteData(base, (uint8_t *)&testData[0], sizeof(testData) - 1);
    checksum16 = CRC_Get16bitResult(base);

    PRINTF("Test string: %s\r\n", testData);
    PRINTF("CRC-16 CCITT FALSE: 0x%x\r\n", checksum16);
    if (checksum16 != checkCcittFalseCrc16)
    {
        PRINTF("...Check fail. Expected: 0x%x\r\n", checkCcittFalseCrc16);
    }

    /* ***************
    * CRC-16/ARC *
    *************** */
    InitCrc16(base, 0x0U);
    CRC_WriteData(base, (uint8_t *)&testData[0], sizeof(testData) - 1);
    checksum16 = CRC_Get16bitResult(base);

    PRINTF("CRC-16 ARC: 0x%x\r\n", checksum16);
    if (checksum16 != checkCrc16Arc)
    {
        PRINTF("...Check fail. Expected: 0x%x\r\n\r\n", checkCrc16Arc);
    }

    /* ***************
    * CRC-32 *
    *************** */
    InitCrc32(base, 0xFFFFFFFFU);
    CRC_WriteData(base, (uint8_t *)&testData[0], sizeof(testData) - 1);
    checksum32 = CRC_Get32bitResult(base);

    PRINTF("CRC-32: 0x%x\r\n\r\n", checksum32);
    if (checksum32 != checkCrc32)
    {
        PRINTF("...Check fail. Expected: 0x%x\r\n\r\n", checkCrc32);
    }

    while (1)
    {
    }
}
