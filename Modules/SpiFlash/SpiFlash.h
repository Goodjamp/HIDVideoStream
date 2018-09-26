#ifndef __SPI_FLASH_H__
#define __SPI_FLASH_H__

#include <stdint.h>
#include <stdbool.h>

void spiFlashInit(void);
bool spiFlashIsDetected(void);

#endif
