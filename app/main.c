#include "Rtos.h"
#include "UsbDeviceTask.h"
#include "LightingTask.h"
#include "FanTask.h"
#include "PmBusCommunicationTask.h"
#include "AnalogMeasurementTask.h"
#include "HalCommon.h"
#include "PmBusHal.h"
#include "LcdTask.h"

int main(void)
{
    halCommonInit();
    usbDeviceTaskCreate();
    lightingTaskCreate();
    fanTaskCreate();
    analogMeasurementTaskCreate();
    lcdTaskCreate();
    rtosStartScheduler();


    for (;;) {

    }

	return 0;
}

void hard_fault_handler_c(unsigned int * hardfault_args)
{
    unsigned int stacked_r0;
    unsigned int stacked_r1;
    unsigned int stacked_r2;
    unsigned int stacked_r3;
    unsigned int stacked_r12;
    unsigned int stacked_lr;
    unsigned int stacked_pc;
    unsigned int stacked_psr;

    stacked_r0 = ((unsigned long) hardfault_args[0]);
    stacked_r1 = ((unsigned long) hardfault_args[1]);
    stacked_r2 = ((unsigned long) hardfault_args[2]);
    stacked_r3 = ((unsigned long) hardfault_args[3]);

    stacked_r12 = ((unsigned long) hardfault_args[4]);
    stacked_lr = ((unsigned long) hardfault_args[5]);
    stacked_pc = ((unsigned long) hardfault_args[6]);
    stacked_psr = ((unsigned long) hardfault_args[7]);

    for (;;) {
		__asm volatile ( "NOP" );
	}
	(void) stacked_r0;
    (void) stacked_r1;
    (void) stacked_r2;
    (void) stacked_r3;
    (void) stacked_r12;
    (void) stacked_lr;
    (void) stacked_pc;
    (void) stacked_psr;
}

void HardFault_Handler(void)
{
    asm volatile(
        "movs r0, #4\t\n"
        "mov  r1, lr\t\n"
        "tst  r0, r1\t\n" /* Check EXC_RETURN[2] */
        "beq 1f\t\n"
        "mrs r0, psp\t\n"
        "ldr r1,=hard_fault_handler_c\t\n"
        "bx r1\t\n"
        "1:mrs r0,msp\t\n"
        "ldr r1,=hard_fault_handler_c\t\n"
        : /* no output */
        : /* no input */
        : "r0" /* clobber */
    );
}
