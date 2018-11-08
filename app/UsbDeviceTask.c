#include "UsbDeviceTask.h"
#include "LcdTask.h"

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

//static uint8_t outputHidReport[200];

#define SIZE_PACKET      (1024)
#define NUMBER_OF_PACKET ((32768 + SIZE_PACKET)/SIZE_PACKET + 1)
#define SIZE_RX_BUFFER   (SIZE_PACKET * NUMBER_OF_PACKET)
/* user timer variable*/
static CTIMER_Type *TIMER_CNT = CTIMER4;

#pragma pack(push,1)
typedef struct
{
    uint16_t quantityPacket;
    uint16_t packetNumber;
    uint16_t rest;
    uint8_t  payload[];
}packetT;
#pragma pack(pop)


struct
{
    uint16_t nextPacketNumber;
    uint32_t rxTime;
}rxPacketState =
{
    .nextPacketNumber = 0,
    .rxTime = 0,
};

const uint16_t packetPayloadSize = SIZE_PACKET - sizeof(packetT);

static uint8_t  rxCommandBuffer[SIZE_RX_BUFFER] = {[0 ... (SIZE_RX_BUFFER - 1)] = 0};

uint16_t rxTime_[300];



static inline uint32_t getTimerCNT(void)
{
    return (uint32_t)TIMER_CNT->TC;
}

static inline void resetTimerCNT(void)
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
    CTIMER_StartTimer(TIMER_CNT);
}


static inline void rxPacketProcessing(const uint8_t buffer[], uint16_t bufferSize)
{
    static BaseType_t xHigherPriorityTaskWoken;
    packetT *rxPacket = (packetT*)buffer;
    if(rxPacket->packetNumber != rxPacketState.nextPacketNumber)
    {
        rxPacketState.nextPacketNumber = 0;
        return;
    }
    if(rxPacketState.nextPacketNumber == 0)
    {
        rxPacketState.rxTime = getTimerCNT();
    }
    rxTime_[rxPacketState.nextPacketNumber] =  getTimerCNT() - rxPacketState.rxTime;
    rxPacketState.rxTime = getTimerCNT();

    memcpy(&rxCommandBuffer[rxPacket->packetNumber * packetPayloadSize], rxPacket->payload, ((rxPacket->quantityPacket - 1) == rxPacket->packetNumber) ?
                                                 (rxPacket->rest):
                                                 (packetPayloadSize));
    if(rxPacket->packetNumber == (rxPacket->quantityPacket - 1))
    {
        //rxComplite, start processing command layer
        rxPacketState.rxTime = getTimerCNT() - rxPacketState.rxTime;
        rxPacketState.nextPacketNumber = 0;
        // unblock USB task
        xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(xSemaphore, &xHigherPriorityTaskWoken);
        if (xHigherPriorityTaskWoken != pdFALSE)
        {
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
        return;
    }
    rxPacketState.nextPacketNumber++;
}




void hidIntOutputReport(const uint8_t reportData[], size_t reportSize)
{

    rxPacketProcessing(reportData, reportSize);
    /*
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

*/
}

void hidIntInputReport(void)
{

}

static void usbDeviceTaskFunction(void *pvParameters)
{
    xSemaphore = xSemaphoreCreateBinary();
    //xSemaphoreTake(xSemaphore, REASONABLE_LONG_TIME);

    for (;;) {
        if (xSemaphoreTake(xSemaphore, REASONABLE_LONG_TIME) == pdTRUE)
        {
            videoCommandProcessing(rxCommandBuffer);

            /*HID_ProcessReport(outputHidReport, inputHidReport);
            while (!usbHidDeviceSendReport(inputHidReport)) {

            }
            */
        }
    }
}

#include "HalIFlash.h"
void usbDeviceTaskCreate(void)
{
    halSpiFlashInit();
    initTimerForMeTime();
    usbHidDeviceSetCallbacks(hidIntInputReport, hidIntOutputReport);
    xTaskCreate(usbDeviceTaskFunction,   "UsbComm", USB_DEVICE_TASK_STACK_SIZE, NULL, USB_DEVICE_TASK_PRIORITY, NULL);
    xTaskCreate(usbHidDeviceTaskFunction, "UsbDrv", USB_DEVICE_TASK_STACK_SIZE, NULL, USB_DEVICE_TASK_PRIORITY, NULL);
}

