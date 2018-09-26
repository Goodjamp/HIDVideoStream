#ifndef __HAL_SPI_H__
#define __HAL_SPI_H__

#include <stdint.h>
#include <stdbool.h>

void lcdInit(void);
void putPicture(const uint8_t data[]);

#endif
