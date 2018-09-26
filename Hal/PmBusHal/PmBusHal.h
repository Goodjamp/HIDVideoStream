#ifndef __PM_BUS_HAL_H__
#define __PM_BUS_HAL_H__

#include <stdint.h>
#include <stdbool.h>

typedef void (*PmBusTranferCompleteCb)(bool isSuccessful);

void pmBusInit(void);
//void pmBusWrite(const uint8_t buff[], uint32_t bufferSize);
//void pmBusRead(uint8_t buff[], uint32_t bytesToRead);

bool pmBusWrite(uint8_t addr, uint8_t cmdData[], uint8_t cmdDataLength, uint8_t data[], uint8_t dataLength);
bool pmBusRead(uint8_t addr, uint8_t cmdData[], uint8_t cmdDataLength, uint8_t data[], uint8_t dataLength);

#endif
