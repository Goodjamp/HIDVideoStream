#include "Rtos.h"

#include "FreeRTOS.h"
#include "task.h"

#include <string.h>

void rtosStartScheduler(void)
{
    vTaskStartScheduler();
}

void vApplicationMallocFailedHook(void)
{
	taskDISABLE_INTERRUPTS();
	for (;;);
}

void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)
{
	taskDISABLE_INTERRUPTS();
	for (;;);

    (void) pcTaskName;
	(void) pxTask;
}

void vApplicationIdleHook(void)
{

}

void vApplicationTickHook( void )
{

}
