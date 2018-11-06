#ifndef __HAL_SPI_H__
#define __HAL_SPI_H__

#include <stdint.h>
#include <stdbool.h>

typedef void(*HalSpiTransferDoneCallback)(bool isOk);


void lcdInit(void);
void putPicture(const uint8_t data[]);
void lcdSendFirstSubFrame(uint8_t *buff, uint16_t buffSize, HalSpiTransferDoneCallback callback);
void lcdSendNextSubFrame(uint8_t *buff,  uint16_t buffSize, HalSpiTransferDoneCallback callback);

#endif
