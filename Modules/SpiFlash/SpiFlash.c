#include "SpiFlash.h"

#include <string.h>

#include "halSpi.h"
#include "halGpio.h"

volatile bool spiFlashBusy = false;


static void spiFlashTransferCallback(bool isOk)
{
    spiFlashBusy = false;
}

void spiFlashInit(void)
{
    halGpioPinInit(halGpio.spiFlashCs);
    halGpioPinInit(halGpio.spiFlashMiso);
    halGpioPinInit(halGpio.spiFlashMosi);
    halGpioPinInit(halGpio.spiFlashSck);
}

bool spiFlashIsDetected(void)
{
    uint8_t txData[] = { 0x5A, // read SFDP command
                         0x00, 0x00, 0x00, //address
                         0x00, //dummy byte
                         0x00, 0x00, 0x00, 0x00 };
    uint8_t rxData[sizeof(txData)];
    spiFlashBusy = true;
    halSpiTransfer(txData, rxData, sizeof(txData), spiFlashTransferCallback);

    while (spiFlashBusy == true) {

    }

    char sfdpId[] = "SFDP";

    if (!memcmp(&rxData[5], sfdpId, strlen(sfdpId))) {
        return true;
    }

    return false;
}
