/*
 * The Clear BSD License
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
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

/*
 * How to set up clock using clock driver functions:
 *
 * 1. Setup clock sources.
 *
 * 2. Setup voltage for the fastest of the clock outputs
 *
 * 3. Set up wait states of the flash.
 *
 * 4. Set up all dividers.
 *
 * 5. Set up all selectors to provide selected clocks.
 */

/* TEXT BELOW IS USED AS SETTING FOR THE CLOCKS TOOL *****************************
!!ClocksProfile
product: Clocks v1.0
processor: LPC54608J512
package_id: LPC54608J512ET180
mcu_data: ksdk2_0
processor_version: 0.0.0
board: LPCXpresso54608
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR THE CLOCKS TOOL **/

#include "fsl_power.h"
#include "fsl_clock.h"
#include "clock_config.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
/* System clock frequency. */
extern uint32_t SystemCoreClock;

/*******************************************************************************
 ********************* Configuration BOARD_BootClockFRO12M ***********************
 ******************************************************************************/
/* TEXT BELOW IS USED AS SETTING FOR THE CLOCKS TOOL *****************************
!!Configuration
name: BOARD_BootClockFRO12M
outputs:
- {id: System_clock.outFreq, value: 12 MHz}
settings:
- {id: SYSCON.EMCCLKDIV.scale, value: '1', locked: true}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR THE CLOCKS TOOL **/

/*******************************************************************************
 * Variables for BOARD_BootClockFRO12M configuration
 ******************************************************************************/
/*******************************************************************************
 * Code for BOARD_BootClockFRO12M configuration
 ******************************************************************************/
void BOARD_BootClockFRO12M(void)
{
    /*!< Set up the clock sources */
    /*!< Set up FRO */
    POWER_DisablePD(kPDRUNCFG_PD_FRO_EN); /*!< Ensure FRO is on  */
    CLOCK_AttachClk(
        kFRO12M_to_MAIN_CLK); /*!< Switch to FRO 12MHz first to ensure we can change voltage without accidentally
                                   being below the voltage for current speed */
    CLOCK_SetupFROClocking(12000000U); /*!< Set up FRO to the 12 MHz, just for sure */
    POWER_SetVoltageForFreq(
        12000000U); /*!< Set voltage for the one of the fastest clock outputs: System clock output */
    CLOCK_SetFLASHAccessCyclesForFreq(12000000U); /*!< Set FLASH wait states for core */

    /*!< Set up dividers */
    CLOCK_SetClkDiv(kCLOCK_DivAhbClk, 1U, false); /*!< Reset divider counter and set divider to value 1 */

    /*!< Set up clock selectors - Attach clocks to the peripheries */
    CLOCK_AttachClk(kFRO12M_to_MAIN_CLK); /*!< Switch MAIN_CLK to FRO12M */
    /*!< Set SystemCoreClock variable. */
    SystemCoreClock = BOARD_BOOTCLOCKFRO12M_CORE_CLOCK;
}

/*******************************************************************************
 ********************** Configuration BOARD_BootClockFROHF48M ***********************
 ******************************************************************************/
/* TEXT BELOW IS USED AS SETTING FOR THE CLOCKS TOOL *****************************
!!Configuration
name: BOARD_BootClockFROHF48M
outputs:
- {id: System_clock.outFreq, value: 48 MHz}
settings:
- {id: SYSCON.MAINCLKSELA.sel, value: SYSCON.fro_hf}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR THE CLOCKS TOOL **/

/*******************************************************************************
 * Variables for BOARD_BootClockFROHF48M configuration
 ******************************************************************************/
/*******************************************************************************
 * Code for BOARD_BootClockFROHF48M configuration
 ******************************************************************************/
void BOARD_BootClockFROHF48M(void)
{
    /*!< Set up the clock sources */
    /*!< Set up FRO */
    POWER_DisablePD(kPDRUNCFG_PD_FRO_EN); /*!< Ensure FRO is on  */
    CLOCK_AttachClk(
        kFRO12M_to_MAIN_CLK); /*!< Switch to FRO 12MHz first to ensure we can change voltage without accidentally
                                   being below the voltage for current speed */
    POWER_SetVoltageForFreq(
        48000000U); /*!< Set voltage for the one of the fastest clock outputs: System clock output */
    CLOCK_SetFLASHAccessCyclesForFreq(48000000U); /*!< Set FLASH wait states for core */

    CLOCK_SetupFROClocking(48000000U); /*!< Set up high frequency FRO output to selected frequency */

    /*!< Set up dividers */
    CLOCK_SetClkDiv(kCLOCK_DivAhbClk, 1U, false); /*!< Reset divider counter and set divider to value 1 */

    /*!< Set up clock selectors - Attach clocks to the peripheries */
    CLOCK_AttachClk(kFRO_HF_to_MAIN_CLK); /*!< Switch MAIN_CLK to FRO_HF */
    /*!< Set SystemCoreClock variable. */
    SystemCoreClock = BOARD_BOOTCLOCKFROHF48M_CORE_CLOCK;
}

/*******************************************************************************
 ********************* Configuration BOARD_BootClockFROHF96M **********************
 ******************************************************************************/
/* TEXT BELOW IS USED AS SETTING FOR THE CLOCKS TOOL *****************************
!!Configuration
name: BOARD_BootClockFROHF96M
outputs:
- {id: System_clock.outFreq, value: 96 MHz}
settings:
- {id: SYSCON.MAINCLKSELA.sel, value: SYSCON.fro_hf}
sources:
- {id: SYSCON.fro_hf.outFreq, value: 96 MHz}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR THE CLOCKS TOOL **/

/*******************************************************************************
 * Variables for BOARD_BootClockFROHF96M configuration
 ******************************************************************************/
/*******************************************************************************
 * Code for BOARD_BootClockFROHF96M configuration
 ******************************************************************************/
void BOARD_BootClockFROHF96M(void)
{
    /*!< Set up the clock sources */
    /*!< Set up FRO */
    POWER_DisablePD(kPDRUNCFG_PD_FRO_EN); /*!< Ensure FRO is on  */
    CLOCK_AttachClk(kFRO12M_to_MAIN_CLK); /*!< Switch to FRO 12MHz first to ensure we can change voltage without accidentally
                                   being below the voltage for current speed */
    POWER_SetVoltageForFreq(96000000U); /*!< Set voltage for the one of the fastest clock outputs: System clock output */
    CLOCK_SetFLASHAccessCyclesForFreq(96000000U); /*!< Set FLASH wait states for core */

    CLOCK_SetupFROClocking(96000000U); /*!< Set up high frequency FRO output to selected frequency */

    /*!< Set up dividers */
    CLOCK_SetClkDiv(kCLOCK_DivAhbClk, 1U, false); /*!< Reset divider counter and set divider to value 1 */

    /*!< Set up clock selectors - Attach clocks to the peripheries */
    CLOCK_AttachClk(kFRO_HF_to_MAIN_CLK); /*!< Switch MAIN_CLK to FRO_HF */
    /*!< Set SystemCoreClock variable. */
    SystemCoreClock = BOARD_BOOTCLOCKFROHF96M_CORE_CLOCK;
}

/*******************************************************************************
 ********************* Configuration BOARD_BootClockPLL180M **********************
 ******************************************************************************/
/* TEXT BELOW IS USED AS SETTING FOR THE CLOCKS TOOL *****************************
!!Configuration
name: BOARD_BootClockPLL180M
outputs:
- {id: FRO12M_clock.outFreq, value: 12 MHz}
- {id: FROHF_clock.outFreq, value: 48 MHz}
- {id: SYSPLL_clock.outFreq, value: 180 MHz}
- {id: System_clock.outFreq, value: 180 MHz}
settings:
- {id: SYSCON.M_MULT.scale, value: '30', locked: true}
- {id: SYSCON.N_DIV.scale, value: '1', locked: true}
- {id: SYSCON.PDEC.scale, value: '2', locked: true}
- {id: SYSCON_PDRUNCFG0_PDEN_SYS_PLL_CFG, value: Power_up}
sources:
- {id: SYSCON._clk_in.outFreq, value: 12 MHz, enabled: true}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR THE CLOCKS TOOL **/

/*******************************************************************************
 * Variables for BOARD_BootClockPLL180M configuration
 ******************************************************************************/
/*******************************************************************************
 * Code for BOARD_BootClockPLL180M configuration
 ******************************************************************************/
void BOARD_BootClockPLL180M(void)
{
 /*!< Set up the clock sources */
    /*!< Set up FRO */
    POWER_DisablePD(kPDRUNCFG_PD_FRO_EN);                   /*!< Ensure FRO is on  */
    CLOCK_AttachClk(kFRO12M_to_MAIN_CLK);                  /*!< Switch to FRO 12MHz first to ensure we can change voltage without accidentally
                                                                being below the voltage for current speed */
    POWER_DisablePD(kPDRUNCFG_PD_SYS_OSC);          /*!< Enable System Oscillator Power */
    SYSCON->SYSOSCCTRL = ((SYSCON->SYSOSCCTRL & ~SYSCON_SYSOSCCTRL_FREQRANGE_MASK) | SYSCON_SYSOSCCTRL_FREQRANGE(0U)); /*!< Set system oscillator range */
    POWER_SetVoltageForFreq(180000000U);             /*!< Set voltage for the one of the fastest clock outputs: System clock output */
    CLOCK_SetFLASHAccessCyclesForFreq(180000000U);    /*!< Set FLASH wait states for core */


    /*!< Set up SYS PLL */
    const pll_setup_t pllSetup = {
        .pllctrl =  SYSCON_SYSPLLCTRL_SELI(32U) | SYSCON_SYSPLLCTRL_SELP(16U) | SYSCON_SYSPLLCTRL_SELR(0U),
        .pllmdec = (SYSCON_SYSPLLMDEC_MDEC(8191U)),
        .pllndec = (SYSCON_SYSPLLNDEC_NDEC(770U)),
        .pllpdec = (SYSCON_SYSPLLPDEC_PDEC(98U)),
        .pllRate = 180000000U,
        .flags =  PLL_SETUPFLAG_WAITLOCK | PLL_SETUPFLAG_POWERUP
    };
    CLOCK_AttachClk(kEXT_CLK_to_SYS_PLL);        /*!< Set sys pll clock source*/
    CLOCK_SetPLLFreq(&pllSetup);                 /*!< Configure PLL to the desired value */
    /*!< Set up USB PLL */
    /*
    const usb_pll_setup_t usb_pllSetup = {
        .msel = 15U,
        .nsel = 0U,
        .psel = 1U,
        .direct = false,
        .bypass = false,
        .fbsel = false,
        .inputRate = 12000000U,
    };
    CLOCK_SetUsbPLLFreq(&usb_pllSetup);

    CLOCK_SetClkDiv(kCLOCK_DivUsb1Clk,1U, false);
    CLOCK_AttachClk(kUSB_PLL_to_USB1_CLK);

*/
    CLOCK_SetupFROClocking(48000000U);              /*!< Set up high frequency FRO output to selected frequency */

    /*!< Set up dividers */
    CLOCK_SetClkDiv(kCLOCK_DivAhbClk, 1U, false);                  /*!< Reset divider counter and set divider to value 1 */
    CLOCK_SetClkDiv(kCLOCK_DivClkOut, 0U, true);                  /*!< Reset CLKOUTDIV divider counter and halt it */
    CLOCK_SetClkDiv(kCLOCK_DivClkOut, 1U, false);                  /*!< Set CLKOUTDIV divider to value 1 */
    CLOCK_SetClkDiv(kCLOCK_DivAdcAsyncClk, 0U, true);                  /*!< Reset ADCCLKDIV divider counter and halt it */
    CLOCK_SetClkDiv(kCLOCK_DivAdcAsyncClk, 256U, false);                  /*!< Set ADCCLKDIV divider to value 256 */
    CLOCK_SetClkDiv(kCLOCK_DivSctClk, 0U, true);                  /*!< Reset SCTCLKDIV divider counter and halt it */
    CLOCK_SetClkDiv(kCLOCK_DivSctClk, 12U, false);                  /*!< Set SCTCLKDIV divider to value 12 */

    /*!< Set up clock selectors - Attach clocks to the peripheries */
    CLOCK_AttachClk(kSYS_PLL_to_MAIN_CLK);                  /*!< Switch MAIN_CLK to SYS_PLL */
    CLOCK_AttachClk(kFRO_HF_to_ADC_CLK);                  /*!< Switch ADC_CLK to FRO_HF */
    SYSCON->FROHFCLKDIV = ((SYSCON->FROHFCLKDIV & ~SYSCON_FROHFCLKDIV_DIV_MASK) | SYSCON_FROHFCLKDIV_DIV(0U)); /*!< Set FROHF CLKDIV  to value 0 */
    CLOCK_AttachClk(kFRO_HF_to_FLEXCOMM0);                      /*!< Switch FLEXCOMM0 to FRO_HF */
    CLOCK_AttachClk(kFRO12M_to_FLEXCOMM1);                  /*!< Switch FLEXCOMM1 to FRO12M */
    CLOCK_AttachClk(kFRO12M_to_FLEXCOMM2);                  /*!< Switch FLEXCOMM2 to FRO12M */
    CLOCK_AttachClk(kFRO_HF_to_FLEXCOMM4);                      /*!< Switch FLEXCOMM3 to FRO_HF */
    CLOCK_AttachClk(kFRO_HF_to_SCT_CLK);                  /*!< Switch SCT_CLK to FRO_HF */
    CLOCK_AttachClk(kOSC32K_OSC_to_CLKOUT);                  /*!< Switch CLKOUT to OSC32K_OSC */
    SYSCON->MAINCLKSELA = ((SYSCON->MAINCLKSELA & ~SYSCON_MAINCLKSELA_SEL_MASK) | SYSCON_MAINCLKSELA_SEL(0U)); /*!< Switch MAINCLKSELA to FRO12M even it is not used for MAINCLKSELB */
    /* Set SystemCoreClock variable. */
    SystemCoreClock = BOARD_BOOTCLOCKPLL180M_CORE_CLOCK;
}
