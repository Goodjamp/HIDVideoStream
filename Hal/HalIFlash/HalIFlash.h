#ifndef HALFLASH_H_
#define HALFLASH_H_

typedef enum{
    FLASH_OK,
    FLASH_BUSSY,
    FLASH_ERROR,
}flashStatT;


void halSpiFlashInit(void);
void flashReadData(uint8_t *buff,uint32_t dataSize, uint32_t address);
flashStatT flashGetState(void);

#endif
