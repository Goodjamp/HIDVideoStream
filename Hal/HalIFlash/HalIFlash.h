#ifndef HALFLASH_H_
#define HALFLASH_H_

typedef enum{
    FLASH_OK,
    FLASH_BUSSY,
    FLASH_ERROR,
}flashStatT;


void halSpiFlashInit(void);
flashStatT flashGetState(void);
flashStatT flashGetWrite(void);
flashStatT flashEraseBlock32(uint32_t blockNumber);
flashStatT flashWriteEnable(void);
flashStatT flashWriteBlock(uint8_t *buff, uint32_t dataSize, uint32_t address);
flashStatT flashReadData(uint8_t *buff, uint32_t dataSize, uint32_t address);
bool isFlashBussy(void);


flashStatT flashReadSubFrameStart(uint8_t *buff, uint32_t dataSize, uint32_t address);
flashStatT flashReadSubFrameNext(uint8_t *buff, uint32_t dataSize, uint32_t address);
flashStatT flashReadSubFrameLast(uint8_t *buff, uint32_t dataSize, uint32_t address);


#endif
