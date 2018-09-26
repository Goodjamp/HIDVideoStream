#include "UsbDeviceTask.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "UsbHidDevice.h"

#include "UsbProtocol.h"

#include "LPC54608.h"
#include "fsl_ctimer.h"
#include "fsl_clock.h"

#include <stdint.h>
#include <string.h>

#define REASONABLE_LONG_TIME            0xFFFF

#define USB_DEVICE_TASK_PRIORITY        3
#define USB_DEVICE_TASK_STACK_SIZE      256

SemaphoreHandle_t xSemaphore = NULL;

static uint8_t inputHidReport[200];
//static uint8_t outputHidReport[200];

#define SIZE_PACKET      (512)
#define NUMBER_OF_PACKET (32)
#define SIZE_RX_BUFFER   (SIZE_PACKET * NUMBER_OF_PACKET)
static   uint8_t  rxPacets[SIZE_RX_BUFFER] = {[0 ... (SIZE_RX_BUFFER - 1)] = 0};
static   uint32_t cntRx = 0;
uint32_t rezTime;
uint32_t packetCnt;

static CTIMER_Type *TIMER_CNT = CTIMER4;
struct{
    uint32_t time[NUMBER_OF_PACKET];
    uint16_t cnt;
}rxPacketTime;


static inline uint32_t getTimerCNT(void)
{
    return (uint32_t)TIMER_CNT->TC;
}

static inline void resrtTimerCNT(void)
{
    TIMER_CNT->TC = 0;
}

void initTimerForMeTime(void)
{
    ctimer_config_t initStruct;

    initStruct.mode      = kCTIMER_TimerMode;
    SYSCON->ASYNCAPBCTRL = SYSCON_ASYNCAPBCTRL_ENABLE(1);
    initStruct.prescale  = CLOCK_GetAsyncApbClkFreq()/1000000; // 1 us
    CTIMER_Init(TIMER_CNT, &initStruct);
}


void hidIntOutputReport(const uint8_t reportData[], size_t reportSize)
{
    static BaseType_t xHigherPriorityTaskWoken;
    static uint32_t lastTime;
    if(packetCnt)
    {
       rxPacketTime.time[rxPacketTime.cnt++] = getTimerCNT() - lastTime;
    }
    lastTime = getTimerCNT();
    if(packetCnt == 0  )
    {
        resrtTimerCNT();
        CTIMER_StartTimer(TIMER_CNT);
    }
    packetCnt++;
    xHigherPriorityTaskWoken = pdFALSE;
    memcpy(&rxPacets[cntRx], reportData, reportSize);
    cntRx += reportSize;
    if(cntRx >= SIZE_RX_BUFFER)
    {
        cntRx = 0;
    }
    if(packetCnt >= NUMBER_OF_PACKET)
    {
        CTIMER_StopTimer(TIMER_CNT);
        rezTime =  getTimerCNT();
        rxPacketTime.cnt = 0;
        memset(&rxPacketTime.time, 0, sizeof(rxPacketTime.time));
        packetCnt = 0;
    }


    xSemaphoreGiveFromISR(xSemaphore, &xHigherPriorityTaskWoken);

    if (xHigherPriorityTaskWoken != pdFALSE) {
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void hidIntInputReport(void)
{

}

static void usbDeviceTaskFunction(void *pvParameters)
{
    xSemaphore = xSemaphoreCreateBinary();

    for (;;) {
        if (xSemaphoreTake(xSemaphore, REASONABLE_LONG_TIME) == pdTRUE) {
            //HID_ProcessReport(outputHidReport, inputHidReport);
            while (!usbHidDeviceSendReport(inputHidReport)) {

            }
        }
    }
}

void usbDeviceTaskCreate(void)
{
    initTimerForMeTime();
    usbHidDeviceSetCallbacks(hidIntInputReport, hidIntOutputReport);
    xTaskCreate(usbDeviceTaskFunction,   "UsbComm", USB_DEVICE_TASK_STACK_SIZE, NULL, USB_DEVICE_TASK_PRIORITY, NULL);
    xTaskCreate(usbHidDeviceTaskFunction, "UsbDrv", USB_DEVICE_TASK_STACK_SIZE, NULL, USB_DEVICE_TASK_PRIORITY, NULL);
}

