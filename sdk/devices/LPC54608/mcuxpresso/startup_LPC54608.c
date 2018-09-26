/* File: startup_ARMCM4.c
 * Purpose: startup file for Cortex-M4 devices.
 *          Should be used with GCC 'GNU Tools ARM Embedded'
 * Version: V1.01
 * Date: 12 June 2014
 *
 */
/* Copyright (c) 2011 - 2014 ARM LIMITED

   All rights reserved.
   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:
   - Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   - Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.
   - Neither the name of ARM nor the names of its contributors may be used
     to endorse or promote products derived from this software without
     specific prior written permission.
   *
   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
   ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
   LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
   SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
   POSSIBILITY OF SUCH DAMAGE.
   ---------------------------------------------------------------------------*/

#include <stdint.h>
#include "fsl_device_registers.h"

/*----------------------------------------------------------------------------
  Linker generated Symbols
 *----------------------------------------------------------------------------*/
extern uint32_t __etext;
extern uint32_t __data_start__;
extern uint32_t __data_end__;
extern uint32_t __copy_table_start__;
extern uint32_t __copy_table_end__;
extern uint32_t __zero_table_start__;
extern uint32_t __zero_table_end__;
extern uint32_t __bss_start__;
extern uint32_t __bss_end__;
extern uint32_t __StackTop;

/*----------------------------------------------------------------------------
  Exception / Interrupt Handler Function Prototype
 *----------------------------------------------------------------------------*/
typedef void( *pFunc )( void );


/*----------------------------------------------------------------------------
  External References
 *----------------------------------------------------------------------------*/
#ifndef __START
extern void  _start(void) __attribute__((noreturn));    /* PreeMain (C library entry point) */
#else
extern int  __START(void) __attribute__((noreturn));    /* main entry point */
#endif

#ifndef __NO_SYSTEM_INIT
extern void SystemInit (void);            /* CMSIS System Initialization      */
#endif


/*----------------------------------------------------------------------------
  Internal References
 *----------------------------------------------------------------------------*/
void Default_Handler(void);                          /* Default empty handler */
void Reset_Handler(void);                            /* Reset Handler */


/*----------------------------------------------------------------------------
  User Initial Stack & Heap
 *----------------------------------------------------------------------------*/
#ifndef __STACK_SIZE
  #define	__STACK_SIZE  0x00000200
#endif
static uint8_t stack[__STACK_SIZE] __attribute__ ((aligned(8), used, section(".stack")));

#ifndef __HEAP_SIZE
  #define	__HEAP_SIZE   0x00000000
#endif
#if __HEAP_SIZE > 0
static uint8_t heap[__HEAP_SIZE]   __attribute__ ((aligned(8), used, section(".heap")));
#endif


/*----------------------------------------------------------------------------
  Exception / Interrupt Handler
 *----------------------------------------------------------------------------*/
/* Cortex-M4 Processor Exceptions */
void __attribute__ ((weak)) NMI_Handler                                (void) { while(1); };
void __attribute__ ((weak)) HardFault_Handler                          (void) { while(1); };
void __attribute__ ((weak)) MemManage_Handler                          (void) { while(1); };
void __attribute__ ((weak)) BusFault_Handler                           (void) { while(1); };
void __attribute__ ((weak)) UsageFault_Handler                         (void) { while(1); };
void __attribute__ ((weak)) SVC_Handler                                (void) { while(1); };
void __attribute__ ((weak)) DebugMon_Handler                           (void) { while(1); };
void __attribute__ ((weak)) PendSV_Handler                             (void) { while(1); };
void __attribute__ ((weak)) SysTick_Handler                            (void) { while(1); };

/* External interrupts */
extern void WDT_BOD_DriverIRQHandler            (void) __attribute__ ((weak, alias("Default_Handler")));
extern void DMA0_DriverIRQHandler               (void) __attribute__ ((weak, alias("Default_Handler")));
extern void GINT0_DriverIRQHandler              (void) __attribute__ ((weak, alias("Default_Handler")));
extern void GINT1_DriverIRQHandler              (void) __attribute__ ((weak, alias("Default_Handler")));
extern void PIN_INT0_DriverIRQHandler           (void) __attribute__ ((weak, alias("Default_Handler")));
extern void PIN_INT1_DriverIRQHandler           (void) __attribute__ ((weak, alias("Default_Handler")));
extern void PIN_INT2_DriverIRQHandler           (void) __attribute__ ((weak, alias("Default_Handler")));
extern void PIN_INT3_DriverIRQHandler           (void) __attribute__ ((weak, alias("Default_Handler")));
extern void UTICK0_DriverIRQHandler             (void) __attribute__ ((weak, alias("Default_Handler")));
extern void MRT0_DriverIRQHandler               (void) __attribute__ ((weak, alias("Default_Handler")));
extern void CTIMER0_DriverIRQHandler            (void) __attribute__ ((weak, alias("Default_Handler")));
extern void CTIMER1_DriverIRQHandler            (void) __attribute__ ((weak, alias("Default_Handler")));
extern void SCT0_DriverIRQHandler               (void) __attribute__ ((weak, alias("Default_Handler")));
extern void CTIMER3_DriverIRQHandler            (void) __attribute__ ((weak, alias("Default_Handler")));
extern void FLEXCOMM0_DriverIRQHandler          (void) __attribute__ ((weak, alias("Default_Handler")));
extern void FLEXCOMM1_DriverIRQHandler          (void) __attribute__ ((weak, alias("Default_Handler")));
extern void FLEXCOMM2_DriverIRQHandler          (void) __attribute__ ((weak, alias("Default_Handler")));
extern void FLEXCOMM3_DriverIRQHandler          (void) __attribute__ ((weak, alias("Default_Handler")));
extern void FLEXCOMM4_DriverIRQHandler          (void) __attribute__ ((weak, alias("Default_Handler")));
extern void FLEXCOMM5_DriverIRQHandler          (void) __attribute__ ((weak, alias("Default_Handler")));
extern void FLEXCOMM6_DriverIRQHandler          (void) __attribute__ ((weak, alias("Default_Handler")));
extern void FLEXCOMM7_DriverIRQHandler          (void) __attribute__ ((weak, alias("Default_Handler")));
extern void ADC0_SEQA_DriverIRQHandler          (void) __attribute__ ((weak, alias("Default_Handler")));
extern void ADC0_SEQB_DriverIRQHandler          (void) __attribute__ ((weak, alias("Default_Handler")));
extern void ADC0_THCMP_DriverIRQHandler         (void) __attribute__ ((weak, alias("Default_Handler")));
extern void DMIC0_DriverIRQHandler              (void) __attribute__ ((weak, alias("Default_Handler")));
extern void HWVAD0_DriverIRQHandler             (void) __attribute__ ((weak, alias("Default_Handler")));
extern void USB0_NEEDCLK_DriverIRQHandler       (void) __attribute__ ((weak, alias("Default_Handler")));
extern void USB0_DriverIRQHandler               (void) __attribute__ ((weak, alias("Default_Handler")));
extern void RTC_DriverIRQHandler                (void) __attribute__ ((weak, alias("Default_Handler")));
extern void Reserved46_DriverIRQHandler         (void) __attribute__ ((weak, alias("Default_Handler")));
extern void Reserved47_DriverIRQHandler         (void) __attribute__ ((weak, alias("Default_Handler")));
extern void PIN_INT4_DriverIRQHandler           (void) __attribute__ ((weak, alias("Default_Handler")));
extern void PIN_INT5_DriverIRQHandler           (void) __attribute__ ((weak, alias("Default_Handler")));
extern void PIN_INT6_DriverIRQHandler           (void) __attribute__ ((weak, alias("Default_Handler")));
extern void PIN_INT7_DriverIRQHandler           (void) __attribute__ ((weak, alias("Default_Handler")));
extern void CTIMER2_DriverIRQHandler            (void) __attribute__ ((weak, alias("Default_Handler")));
extern void CTIMER4_DriverIRQHandler            (void) __attribute__ ((weak, alias("Default_Handler")));
extern void RIT_DriverIRQHandler                (void) __attribute__ ((weak, alias("Default_Handler")));
extern void SPIFI0_DriverIRQHandler             (void) __attribute__ ((weak, alias("Default_Handler")));
extern void FLEXCOMM8_DriverIRQHandler          (void) __attribute__ ((weak, alias("Default_Handler")));
extern void FLEXCOMM9_DriverIRQHandler          (void) __attribute__ ((weak, alias("Default_Handler")));
extern void SDIO_DriverIRQHandler               (void) __attribute__ ((weak, alias("Default_Handler")));
extern void CAN0_IRQ0_DriverIRQHandler          (void) __attribute__ ((weak, alias("Default_Handler")));
extern void CAN0_IRQ1_DriverIRQHandler          (void) __attribute__ ((weak, alias("Default_Handler")));
extern void CAN1_IRQ0_DriverIRQHandler          (void) __attribute__ ((weak, alias("Default_Handler")));
extern void CAN1_IRQ1_DriverIRQHandler          (void) __attribute__ ((weak, alias("Default_Handler")));
extern void USB1_DriverIRQHandler               (void) __attribute__ ((weak, alias("Default_Handler")));
extern void USB1_NEEDCLK_DriverIRQHandler       (void) __attribute__ ((weak, alias("Default_Handler")));
extern void ETHERNET_DriverIRQHandler           (void) __attribute__ ((weak, alias("Default_Handler")));
extern void ETHERNET_PMT_DriverIRQHandler       (void) __attribute__ ((weak, alias("Default_Handler")));
extern void ETHERNET_MACLP_DriverIRQHandler     (void) __attribute__ ((weak, alias("Default_Handler")));
extern void EEPROM_DriverIRQHandler             (void) __attribute__ ((weak, alias("Default_Handler")));
extern void LCD_DriverIRQHandler                (void) __attribute__ ((weak, alias("Default_Handler")));
extern void SHA_DriverIRQHandler                (void) __attribute__ ((weak, alias("Default_Handler")));
extern void SMARTCARD0_DriverIRQHandler         (void) __attribute__ ((weak, alias("Default_Handler")));
extern void SMARTCARD1_DriverIRQHandler         (void) __attribute__ ((weak, alias("Default_Handler")));

__attribute__ ((weak)) void WDT_BOD_IRQHandler        (void) { WDT_BOD_DriverIRQHandler();        }
__attribute__ ((weak)) void DMA0_IRQHandler           (void) { DMA0_DriverIRQHandler();           }
__attribute__ ((weak)) void GINT0_IRQHandler          (void) { GINT0_DriverIRQHandler();          }
__attribute__ ((weak)) void GINT1_IRQHandler          (void) { GINT1_DriverIRQHandler();          }
__attribute__ ((weak)) void PIN_INT0_IRQHandler       (void) { PIN_INT0_DriverIRQHandler();       }
__attribute__ ((weak)) void PIN_INT1_IRQHandler       (void) { PIN_INT1_DriverIRQHandler();       }
__attribute__ ((weak)) void PIN_INT2_IRQHandler       (void) { PIN_INT2_DriverIRQHandler();       }
__attribute__ ((weak)) void PIN_INT3_IRQHandler       (void) { PIN_INT3_DriverIRQHandler();       }
__attribute__ ((weak)) void UTICK0_IRQHandler         (void) { UTICK0_DriverIRQHandler();         }
__attribute__ ((weak)) void MRT0_IRQHandler           (void) { MRT0_DriverIRQHandler();           }
__attribute__ ((weak)) void CTIMER0_IRQHandler        (void) { CTIMER0_DriverIRQHandler();        }
__attribute__ ((weak)) void CTIMER1_IRQHandler        (void) { CTIMER1_DriverIRQHandler();        }
__attribute__ ((weak)) void SCT0_IRQHandler           (void) {
    SCT0_DriverIRQHandler();           }
__attribute__ ((weak)) void CTIMER3_IRQHandler        (void) { CTIMER3_DriverIRQHandler();        }
__attribute__ ((weak)) void FLEXCOMM0_IRQHandler      (void) { FLEXCOMM0_DriverIRQHandler();      }
__attribute__ ((weak)) void FLEXCOMM1_IRQHandler      (void) { FLEXCOMM1_DriverIRQHandler();      }
__attribute__ ((weak)) void FLEXCOMM2_IRQHandler      (void) { FLEXCOMM2_DriverIRQHandler();      }
__attribute__ ((weak)) void FLEXCOMM3_IRQHandler      (void) { FLEXCOMM3_DriverIRQHandler();      }
__attribute__ ((weak)) void FLEXCOMM4_IRQHandler      (void) { FLEXCOMM4_DriverIRQHandler();      }
__attribute__ ((weak)) void FLEXCOMM5_IRQHandler      (void) { FLEXCOMM5_DriverIRQHandler();      }
__attribute__ ((weak)) void FLEXCOMM6_IRQHandler      (void) { FLEXCOMM6_DriverIRQHandler();      }
__attribute__ ((weak)) void FLEXCOMM7_IRQHandler      (void) { FLEXCOMM7_DriverIRQHandler();      }
__attribute__ ((weak)) void ADC0_SEQA_IRQHandler      (void) { ADC0_SEQA_DriverIRQHandler();      }
__attribute__ ((weak)) void ADC0_SEQB_IRQHandler      (void) { ADC0_SEQB_DriverIRQHandler();      }
__attribute__ ((weak)) void ADC0_THCMP_IRQHandler     (void) { ADC0_THCMP_DriverIRQHandler();     }
__attribute__ ((weak)) void DMIC0_IRQHandler          (void) { DMIC0_DriverIRQHandler();          }
__attribute__ ((weak)) void HWVAD0_IRQHandler         (void) { HWVAD0_DriverIRQHandler();         }
__attribute__ ((weak)) void USB0_NEEDCLK_IRQHandler   (void) { USB0_NEEDCLK_DriverIRQHandler();   }
__attribute__ ((weak)) void USB0_IRQHandler           (void) { USB0_DriverIRQHandler();           }
__attribute__ ((weak)) void RTC_IRQHandler            (void) { RTC_DriverIRQHandler();            }
__attribute__ ((weak)) void Reserved46_IRQHandler     (void) { Reserved46_DriverIRQHandler();     }
__attribute__ ((weak)) void Reserved47_IRQHandler     (void) { Reserved47_DriverIRQHandler();     }
__attribute__ ((weak)) void PIN_INT4_IRQHandler       (void) { PIN_INT4_DriverIRQHandler();       }
__attribute__ ((weak)) void PIN_INT5_IRQHandler       (void) { PIN_INT5_DriverIRQHandler();       }
__attribute__ ((weak)) void PIN_INT6_IRQHandler       (void) { PIN_INT6_DriverIRQHandler();       }
__attribute__ ((weak)) void PIN_INT7_IRQHandler       (void) { PIN_INT7_DriverIRQHandler();       }
__attribute__ ((weak)) void CTIMER2_IRQHandler        (void) { CTIMER2_DriverIRQHandler();        }
__attribute__ ((weak)) void CTIMER4_IRQHandler        (void) { CTIMER4_DriverIRQHandler();        }
__attribute__ ((weak)) void RIT_IRQHandler            (void) { RIT_DriverIRQHandler();            }
__attribute__ ((weak)) void SPIFI0_IRQHandler         (void) { SPIFI0_DriverIRQHandler();         }
__attribute__ ((weak)) void FLEXCOMM8_IRQHandler      (void) { FLEXCOMM8_DriverIRQHandler();      }
__attribute__ ((weak)) void FLEXCOMM9_IRQHandler      (void) { FLEXCOMM9_DriverIRQHandler();      }
__attribute__ ((weak)) void SDIO_IRQHandler           (void) { SDIO_DriverIRQHandler();           }
__attribute__ ((weak)) void CAN0_IRQ0_IRQHandler      (void) { CAN0_IRQ0_DriverIRQHandler();      }
__attribute__ ((weak)) void CAN0_IRQ1_IRQHandler      (void) { CAN0_IRQ1_DriverIRQHandler();      }
__attribute__ ((weak)) void CAN1_IRQ0_IRQHandler      (void) { CAN1_IRQ0_DriverIRQHandler();      }
__attribute__ ((weak)) void CAN1_IRQ1_IRQHandler      (void) { CAN1_IRQ1_DriverIRQHandler();      }
__attribute__ ((weak)) void USB1_IRQHandler           (void) { USB1_DriverIRQHandler();           }
__attribute__ ((weak)) void USB1_NEEDCLK_IRQHandler   (void) { USB1_NEEDCLK_DriverIRQHandler();   }
__attribute__ ((weak)) void ETHERNET_IRQHandler       (void) { ETHERNET_DriverIRQHandler();       }
__attribute__ ((weak)) void ETHERNET_PMT_IRQHandler   (void) { ETHERNET_PMT_DriverIRQHandler();   }
__attribute__ ((weak)) void ETHERNET_MACLP_IRQHandler (void) { ETHERNET_MACLP_DriverIRQHandler(); }
__attribute__ ((weak)) void EEPROM_IRQHandler         (void) { EEPROM_DriverIRQHandler();         }
__attribute__ ((weak)) void LCD_IRQHandler            (void) { LCD_DriverIRQHandler();            }
__attribute__ ((weak)) void SHA_IRQHandler            (void) { SHA_DriverIRQHandler();            }
__attribute__ ((weak)) void SMARTCARD0_IRQHandler     (void) { SMARTCARD0_DriverIRQHandler();     }
__attribute__ ((weak)) void SMARTCARD1_IRQHandler     (void) { SMARTCARD1_DriverIRQHandler();     }

/*----------------------------------------------------------------------------
  Exception / Interrupt Vector table
 *----------------------------------------------------------------------------*/
const pFunc __Vectors[] __attribute__ ((section(".vectors"))) = {
  /* Cortex-M4 Exceptions Handler */
  (pFunc)&__StackTop,           /*      Initial Stack Pointer     */
  Reset_Handler,                /*      Reset Handler             */
  NMI_Handler,                  /*      NMI Handler               */
  HardFault_Handler,            /*      Hard Fault Handler        */
  MemManage_Handler,            /*      MPU Fault Handler         */
  BusFault_Handler,             /*      Bus Fault Handler         */
  UsageFault_Handler,           /*      Usage Fault Handler       */
  0,                            /*      Reserved                  */
  0,                            /*      Reserved                  */
  0,                            /*      Reserved                  */
  0,                            /*      Reserved                  */
  SVC_Handler,                  /*      SVCall Handler            */
  DebugMon_Handler,             /*      Debug Monitor Handler     */
  0,                            /*      Reserved                  */
  PendSV_Handler,               /*      PendSV Handler            */
  SysTick_Handler,              /*      SysTick Handler           */

  /* External interrupts */
  WDT_BOD_IRQHandler,           /* Windowed watchdog timer, Brownout detect */
  DMA0_IRQHandler,              /* DMA controller */
  GINT0_IRQHandler,             /* GPIO group 0 */
  GINT1_IRQHandler,             /* GPIO group 1 */
  PIN_INT0_IRQHandler,          /* Pin interrupt 0 or pattern match engine slice 0 */
  PIN_INT1_IRQHandler,          /* Pin interrupt 1or pattern match engine slice 1 */
  PIN_INT2_IRQHandler,          /* Pin interrupt 2 or pattern match engine slice 2 */
  PIN_INT3_IRQHandler,          /* Pin interrupt 3 or pattern match engine slice 3 */
  UTICK0_IRQHandler,            /* Micro-tick Timer */
  MRT0_IRQHandler,              /* Multi-rate timer */
  CTIMER0_IRQHandler,           /* Standard counter/timer CTIMER0 */
  CTIMER1_IRQHandler,           /* Standard counter/timer CTIMER1 */
  SCT0_IRQHandler,              /* SCTimer/PWM */
  CTIMER3_IRQHandler,           /* Standard counter/timer CTIMER3 */
  FLEXCOMM0_IRQHandler,         /* Flexcomm Interface 0 (USART, SPI, I2C, FLEXCOMM) */
  FLEXCOMM1_IRQHandler,         /* Flexcomm Interface 1 (USART, SPI, I2C, FLEXCOMM) */
  FLEXCOMM2_IRQHandler,         /* Flexcomm Interface 2 (USART, SPI, I2C, FLEXCOMM) */
  FLEXCOMM3_IRQHandler,         /* Flexcomm Interface 3 (USART, SPI, I2C, FLEXCOMM) */
  FLEXCOMM4_IRQHandler,         /* Flexcomm Interface 4 (USART, SPI, I2C, FLEXCOMM) */
  FLEXCOMM5_IRQHandler,         /* Flexcomm Interface 5 (USART, SPI, I2C,, FLEXCOMM) */
  FLEXCOMM6_IRQHandler,         /* Flexcomm Interface 6 (USART, SPI, I2C, I2S,, FLEXCOMM) */
  FLEXCOMM7_IRQHandler,         /* Flexcomm Interface 7 (USART, SPI, I2C, I2S,, FLEXCOMM) */
  ADC0_SEQA_IRQHandler,         /* ADC0 sequence A completion. */
  ADC0_SEQB_IRQHandler,         /* ADC0 sequence B completion. */
  ADC0_THCMP_IRQHandler,        /* ADC0 threshold compare and error. */
  DMIC0_IRQHandler,             /* Digital microphone and DMIC subsystem */
  HWVAD0_IRQHandler,            /* Hardware Voice Activity Detector */
  USB0_NEEDCLK_IRQHandler,      /* USB Activity Wake-up Interrupt */
  USB0_IRQHandler,              /* USB device */
  RTC_IRQHandler,               /* RTC alarm and wake-up interrupts */
  Reserved46_IRQHandler,        /* Reserved interrupt */
  Reserved47_IRQHandler,        /* Reserved interrupt */
  PIN_INT4_IRQHandler,          /* Pin interrupt 4 or pattern match engine slice 4 int */
  PIN_INT5_IRQHandler,          /* Pin interrupt 5 or pattern match engine slice 5 int */
  PIN_INT6_IRQHandler,          /* Pin interrupt 6 or pattern match engine slice 6 int */
  PIN_INT7_IRQHandler,          /* Pin interrupt 7 or pattern match engine slice 7 int */
  CTIMER2_IRQHandler,           /* Standard counter/timer CTIMER2 */
  CTIMER4_IRQHandler,           /* Standard counter/timer CTIMER4 */
  RIT_IRQHandler,               /* Repetitive Interrupt Timer */
  SPIFI0_IRQHandler,            /* SPI flash interface */
  FLEXCOMM8_IRQHandler,         /* Flexcomm Interface 8 (USART, SPI, I2C, FLEXCOMM) */
  FLEXCOMM9_IRQHandler,         /* Flexcomm Interface 9 (USART, SPI, I2C, FLEXCOMM) */
  SDIO_IRQHandler,              /* SD/MMC */
  CAN0_IRQ0_IRQHandler,         /* CAN0 interrupt0 */
  CAN0_IRQ1_IRQHandler,         /* CAN0 interrupt1 */
  CAN1_IRQ0_IRQHandler,         /* CAN1 interrupt0 */
  CAN1_IRQ1_IRQHandler,         /* CAN1 interrupt1 */
  USB1_IRQHandler,              /* USB1 interrupt */
  USB1_NEEDCLK_IRQHandler,      /* USB1 activity */
  ETHERNET_IRQHandler,          /* Ethernet */
  ETHERNET_PMT_IRQHandler,      /* Ethernet power management interrupt */
  ETHERNET_MACLP_IRQHandler,    /* Ethernet MAC interrupt */
  EEPROM_IRQHandler,            /* EEPROM interrupt */
  LCD_IRQHandler,               /* LCD interrupt */
  SHA_IRQHandler,               /* SHA interrupt */
  SMARTCARD0_IRQHandler,        /* Smart card 0 interrupt */
  SMARTCARD1_IRQHandler,        /* Smart card 1 interrupt */
};


/*----------------------------------------------------------------------------
  Reset Handler called on controller reset
 *----------------------------------------------------------------------------*/
    void Reset_Handler(void) {
  uint32_t *pSrc, *pDest;
  uint32_t *pTable __attribute__((unused));

  __disable_irq();

  /* Enable SRAM clock used by Stack */
  SYSCON->AHBCLKCTRLSET[0] = SYSCON_AHBCLKCTRL_SRAM1_MASK | SYSCON_AHBCLKCTRL_SRAM2_MASK | SYSCON_AHBCLKCTRL_SRAM3_MASK;

/*  Firstly it copies data from read only memory to RAM. There are two schemes
 *  to copy. One can copy more than one sections. Another can only copy
 *  one section.  The former scheme needs more instructions and read-only
 *  data to implement than the latter.
 *  Macro __STARTUP_COPY_MULTIPLE is used to choose between two schemes.  */

#ifdef __STARTUP_COPY_MULTIPLE
/*  Multiple sections scheme.
 *
 *  Between symbol address __copy_table_start__ and __copy_table_end__,
 *  there are array of triplets, each of which specify:
 *    offset 0: LMA of start of a section to copy from
 *    offset 4: VMA of start of a section to copy to
 *    offset 8: size of the section to copy. Must be multiply of 4
 *
 *  All addresses must be aligned to 4 bytes boundary.
 */
  pTable = &__copy_table_start__;

  for (; pTable < &__copy_table_end__; pTable = pTable + 3) {
		pSrc  = (uint32_t*)*(pTable + 0);
		pDest = (uint32_t*)*(pTable + 1);
		for (; pDest < (uint32_t*)(*(pTable + 1) + *(pTable + 2)) ; ) {
      *pDest++ = *pSrc++;
		}
	}
#else
/*  Single section scheme.
 *
 *  The ranges of copy from/to are specified by following symbols
 *    __etext: LMA of start of the section to copy from. Usually end of text
 *    __data_start__: VMA of start of the section to copy to
 *    __data_end__: VMA of end of the section to copy to
 *
 *  All addresses must be aligned to 4 bytes boundary.
 */
  pSrc  = &__etext;
  pDest = &__data_start__;

  for ( ; pDest < &__data_end__ ; ) {
    *pDest++ = *pSrc++;
  }
#endif /*__STARTUP_COPY_MULTIPLE */

/*  This part of work usually is done in C library startup code. Otherwise,
 *  define this macro to enable it in this startup.
 *
 *  There are two schemes too. One can clear multiple BSS sections. Another
 *  can only clear one section. The former is more size expensive than the
 *  latter.
 *
 *  Define macro __STARTUP_CLEAR_BSS_MULTIPLE to choose the former.
 *  Otherwise efine macro __STARTUP_CLEAR_BSS to choose the later.
 */
#ifdef __STARTUP_CLEAR_BSS_MULTIPLE
/*  Multiple sections scheme.
 *
 *  Between symbol address __copy_table_start__ and __copy_table_end__,
 *  there are array of tuples specifying:
 *    offset 0: Start of a BSS section
 *    offset 4: Size of this BSS section. Must be multiply of 4
 */
  pTable = &__zero_table_start__;

  for (; pTable < &__zero_table_end__; pTable = pTable + 2) {
		pDest = (uint32_t*)*(pTable + 0);
		for (; pDest < (uint32_t*)(*(pTable + 0) + *(pTable + 1)) ; ) {
      *pDest++ = 0;
		}
	}
#elif defined (__STARTUP_CLEAR_BSS)
/*  Single BSS section scheme.
 *
 *  The BSS section is specified by following symbols
 *    __bss_start__: start of the BSS section.
 *    __bss_end__: end of the BSS section.
 *
 *  Both addresses must be aligned to 4 bytes boundary.
 */
  pDest = &__bss_start__;

  for ( ; pDest < &__bss_end__ ; ) {
    *pDest++ = 0ul;
  }
#endif /* __STARTUP_CLEAR_BSS_MULTIPLE || __STARTUP_CLEAR_BSS */

#ifndef __NO_SYSTEM_INIT
	SystemInit();
#endif

  __enable_irq();

#ifndef __START
#define __START _start
#endif
	extern int main(void);
	main();

}


/*----------------------------------------------------------------------------
  Default Handler for Exceptions / Interrupts
 *----------------------------------------------------------------------------*/
void Default_Handler(void) {
    uint32_t ulCurrentInterrupt;

    /* Obtain the number of the currently executing interrupt. */
    __asm volatile( "mrs %0, ipsr" : "=r"( ulCurrentInterrupt ) );
	while(1);
}
