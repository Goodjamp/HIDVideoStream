#include "PmBusHal.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <math.h>

#include <stdint.h>

#define PM_BUS_COMMUNICATION_TASK_PRIORITY                  1
#define PM_BUS_COMMUNICATION_TASK_STACK_SIZE                200

static const uint8_t pmBusSetupPacket[] = {0x00, 0x11, 0x02, 0x64};
//static uint8_t pmBusReceiveBuffer[100] = {0x03};
static uint8_t pmBusWriteBuffer[] = {0x02, 0x8B};
//static uint8_t pmBusWriteBuffer1[] = {0x02, 0x3B, 100};
static bool result;

void pmBusOnTransferComplete(bool isOk)
{
    result = isOk;
}

double voltage(uint8_t buff[])
{
    int8_t n = ((int8_t)buff[1]) >> 3;
    int16_t y = ((int16_t)((((buff[1] & 0x07) << 8) | buff[0]) << 5)) >> 5;
    return y * pow(2, n);
}

static void pmBusCommunicationTaskFunction(void *pvParameters)
{
    //pmBusInit();

    for (;;) {
        vTaskDelay(200);
        uint8_t data[10];
        //pmBusRead(1, &pmBusWriteBuffer[1], 1, data, 2);
    }

    /*
    pmBusInit(pmBusOnTransferComplete);
//    pmBusWrite(pmBusSetupPacket, sizeof(pmBusSetupPacket) / sizeof(*pmBusSetupPacket));
//
    //pmBusWrite(pmBusWriteBuffer, 2);
    vTaskDelay(200);

    pmBusWrite(pmBusWriteBuffer1, 3);

    for (;;) {
        vTaskDelay(200);
        pmBusReceiveBuffer[0] = 0x03;

        //pmBusWrite(pmBusWriteBufferPage1, 3);
        //vTaskDelay(200);

        result = false;
        pmBusWrite(pmBusWriteBuffer, 2);

        while (result == false)
            { };

        result = false;
        pmBusRead(pmBusReceiveBuffer, 2);

        while (result == false)
            { };


        //int8_t n = ((int8_t)pmBusReceiveBuffer[1]) >> 3;
        //int16_t y = ((int16_t)((((pmBusReceiveBuffer[1] & 0x07) << 8) | pmBusReceiveBuffer[0]) << 5)) >> 5;

        double v1 = voltage(pmBusReceiveBuffer);
        double v2 = voltage(&pmBusReceiveBuffer[2]);
        volatile int ccc;

        ccc++;
    }
    */
    for (;;) {

    }
}

void pmBusCommunicationTaskCreate(void)
{
    xTaskCreate(pmBusCommunicationTaskFunction, "pmbus", PM_BUS_COMMUNICATION_TASK_STACK_SIZE, NULL, PM_BUS_COMMUNICATION_TASK_PRIORITY, NULL);
}
