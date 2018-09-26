#ifndef __HAL_SPI_H__
#define __HAL_SPI_H__

#include <stdint.h>
#include <stdbool.h>


typedef void(*HalSpiTransferDoneCallback)(bool isOk);


void halSpiInit(void);
bool halSpiTransfer(const uint8_t txData[], uint8_t rxData[], uint32_t transferDataLength, HalSpiTransferDoneCallback callback);

#endif
