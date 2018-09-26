#ifndef __HAL_I2C_H__
#define __HAL_I2C_H__

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

typedef void (*HalI2cEventCallback)(bool isTransferOk);

typedef enum I2cMaster {
    I2C_MASTER_0,
    I2C_MASTER_1,
    I2C_MASTER_COUNT
} I2cMaster;

void halI2cInit(void);

bool halI2cReadBuff(I2cMaster I2cMaster, uint8_t slaveAddress, uint8_t regAddress, uint8_t dataBuffer[], size_t bufferLength, HalI2cEventCallback halI2cEventCb);
bool halI2cWriteBuff(I2cMaster I2cMaster, uint8_t slaveAddress, uint8_t regAddress, const uint8_t dataBuffer[], size_t bufferLength, HalI2cEventCallback halI2cEventCb);

void halI2cDeinit(void);

#endif
