#ifndef __FLASHMEM_H__
#define __FLASHMEM_H__

#include <stdint.h>
#include <stdbool.h>

/* Memory chip definitions */
#define FLASH_MEM_PAGE_SIZE                 (0x100)
#define FLASH_MEM_SECTOR_SIZE               (0x1000)
#define FLASH_MEM_SIZE                      (0x800000)
#define FLASH_MEM_EMULATED_SECTOR_SIZE      (0x200)
#define FLASH_MEM_MSD_SECTOR_SIZE           (0x400)

#define FLASH_MEM_SECTOR_COUNT              (2048)
#define FLASH_MEM_EMULATED_SECTOR_COUNT     (16384)
#define FLASH_MEM_MSD_SECTOR_COUNT          (8192)
#define FLASH_MEM_PAGE_CCOUNT               (32768)

/* External variables */
extern uint8_t flashMemWriteBuffer[FLASH_MEM_SECTOR_SIZE];

/* External functions */
void flashMemInit(void);
uint8_t flashMemReadStatusReg0(void);
uint8_t flashMemGetTransactionStatus(void);
void flashMemReadIds(uint8_t id[3]);
void flashMemReadPage(uint32_t pageAddress, uint8_t *dataBuffer);
void flashMemWritePage(uint32_t pageAddress, const uint8_t *dataBuffer);
void flashMemEraseSector(uint32_t sectorAddress);
bool flashMemTest(void);
void flashMemGetUniqueId(uint8_t id[8]);


#endif  // __FLASHMEM_H__
