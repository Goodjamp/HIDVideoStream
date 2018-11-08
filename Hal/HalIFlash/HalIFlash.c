#include "stdint.h"
#include "stdbool.h"
#include "stddef.h"

#include "LPC54608.H"

#include "fsl_dma.h"
#include "fsl_spi.h"
#include "fsl_spi_dma.h"

#include "HalIFlash.h"

#include "HalGpio.h"

/*
14 Flexcomm Interface 7 RX / I2C Slave [1] DMA_ITRIG_INMUX14
15 Flexcomm Interface 7 TX / I2C Master [1] DMA_ITRIG_INMUX15

*/
/**FLASH COMMAND DESCRIPTION**/
#define  WRITE_ENABLE       0x6
#define  V_WRITE_ENABLE     0x50
#define  READ_STATUS_REG1   0x5
#define  READ_STATUS_REG2   0x35
#define  READ_STATUS_REG3   0x15
#define  WRITE_STATUS_REG1  0x1
#define  WRITE_STATUS_REG2  0x31
#define  WRITE_STATUS_REG3  0x11
#define  READ_DATA          0x3
#define  PROGRAM_PAGE       0x2
#define  SECTOR_ERASE       0x20
#define  BLOCK_ERASE32      0x52
#define  BLOCK_ERASE64      0xD8

#pragma pack(push, 1)
typedef struct
{
    uint8_t command;
    uint8_t payload[];
}commanDescriptionT;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct
{
    uint8_t BUSY:1; //0
    uint8_t  WEL:1; //1
    uint8_t  BP0:1; //2
    uint8_t  BP1:1; //3
    uint8_t  BP2:1; //4
    uint8_t   TB:1; //5
    uint8_t  SEC:1; //6
    uint8_t SPR0:1; //7
}flashStatusReg1T;
#pragma pack(pop)


#define SPI_FLASH                SPI4
#define DMA_SPI_FLASH            DMA0
#define SPI_FLASH_FRQ            48000000
#define COMMAND_BUFF_SIZE        6
#define DMA_SPI_FLASH_RX_CHANEL  8
#define DMA_SPI_FLASH_TX_CHANEL  9

volatile static struct flashProcessingT
{
    bool csSet;
    bool flashBussy;
}flashProcessing;


dma_handle_t  dmaSpiFlashRxHandle;
dma_handle_t  dmaSpiFlashTxHandle;

dma_descriptor_t dmaSpiFlashRxDescriptor;
dma_descriptor_t dmaSpiFlashTxDescriptor;

/*SPI-DMA variables*/
spi_dma_handle_t spiDmaFlashHandl;
static spi_transfer_t masterXfer;

extern const HalGpio halGpio;


uint8_t commandBuffTx[COMMAND_BUFF_SIZE];
uint8_t commandBuffRx[COMMAND_BUFF_SIZE];

void halSpiFlashInit(void)
{
    DMA_Init(DMA0);

    halGpioPinInit(halGpio.spiFlashWp);
    halGpioPinInit(halGpio.spiFlashHold);
    halGpioPinInit(halGpio.spiFlashMosi);
    halGpioPinInit(halGpio.spiFlashMiso);
    halGpioPinInit(halGpio.spiFlashSck);
    halGpioPinInit(halGpio.spiFlashCs);
    /*set cs pin*/
    halGpioSetPin(halGpio.spiFlashCs,   true);
    halGpioSetPin(halGpio.spiFlashHold, true);
    halGpioSetPin(halGpio.spiFlashWp,   true);

    // attach 12 MHz clock to SPI3
    CLOCK_AttachClk(kFRO_HF_to_FLEXCOMM4);
    // SPI init
    uint32_t srcClock_Hz;
    spi_master_config_t masterConfig;

    srcClock_Hz = CLOCK_GetFreq(kCLOCK_Flexcomm4);
    // Master config
    SPI_MasterGetDefaultConfig(&masterConfig);
    masterConfig.sselNum = (spi_ssel_t)0;
    masterConfig.sselPol = (spi_spol_t)kSPI_SpolActiveAllLow;
    masterConfig.baudRate_Bps = SPI_FLASH_FRQ;
    SPI_MasterInit(SPI_FLASH, &masterConfig, srcClock_Hz);

    DMA_EnableChannel(DMA0, DMA_SPI_FLASH_RX_CHANEL);
    DMA_EnableChannel(DMA0, DMA_SPI_FLASH_TX_CHANEL);
    DMA_SetChannelPriority(DMA0, DMA_SPI_FLASH_RX_CHANEL, kDMA_ChannelPriority2);
    DMA_SetChannelPriority(DMA0, DMA_SPI_FLASH_TX_CHANEL, kDMA_ChannelPriority2);
}


void spiDmaCB(SPI_Type *base, spi_dma_handle_t *handle, status_t status, void *userData)
{
    halGpioSetPin(halGpio.spiFlashCs,
                  (((struct flashProcessingT*)userData)->csSet) ? (true) : (false));
    ((struct flashProcessingT*)userData)->flashBussy = false;
}


static status_t flashTxRxProcessing(uint8_t *bufferTx, uint8_t *bufferRx, uint32_t dataSize)
{
    /*pull down CS*/
    /*start send command*/
    /*wait for completing send command*/
    /*start receive data*/
    halGpioSetPin(halGpio.spiFlashCs, false);

    DMA_CreateHandle(&dmaSpiFlashRxHandle, DMA0, DMA_SPI_FLASH_RX_CHANEL);
    DMA_CreateHandle(&dmaSpiFlashTxHandle, DMA0, DMA_SPI_FLASH_TX_CHANEL);

    // Set up spi master
    /*
    status_t SPI_MasterTransferCreateHandleDMA(SPI_Type *base,
                                           spi_dma_handle_t *handle,
                                           spi_dma_callback_t callback,
                                           void *userData,
                                           dma_handle_t *txHandle,
                                           dma_handle_t *rxHandle);
    */
    flashProcessing.flashBussy = true;
    SPI_MasterTransferCreateHandleDMA(SPI_FLASH, &spiDmaFlashHandl, spiDmaCB, &flashProcessing, &dmaSpiFlashTxHandle, &dmaSpiFlashRxHandle);
    // Start master transfer
    masterXfer.txData   = bufferTx; // if Tx only transfer - txData pointer should be NULL
    masterXfer.rxData   = bufferRx; // if Rx only transfer - rxData pointer should be NULL
    masterXfer.dataSize = dataSize;
    masterXfer.configFlags |= kSPI_FrameAssert;

    return SPI_MasterTransferDMA(SPI_FLASH, &spiDmaFlashHandl, &masterXfer);
}


flashStatT flashGetState(void)
{
    ((commanDescriptionT*)commandBuffTx)->command = READ_STATUS_REG1;
    // send command
    flashProcessing.csSet      = false;
    flashProcessing.flashBussy = true;
    halGpioSetPin(halGpio.spiFlashCs, false);
    flashTxRxProcessing(commandBuffTx, NULL, 1);
    while(flashProcessing.flashBussy){}
    // read data
    flashProcessing.csSet      = true;
    flashProcessing.flashBussy = true;
    flashTxRxProcessing(NULL, commandBuffRx, 1);
    while(flashProcessing.flashBussy){};

    return (((flashStatusReg1T*)commandBuffRx)->BUSY) ? (FLASH_BUSSY) : (FLASH_OK);
}


flashStatT flashGetWrite(void)
{
    ((commanDescriptionT*)commandBuffTx)->command = READ_STATUS_REG1;
    // send command
    flashProcessing.csSet      = false;
    flashProcessing.flashBussy = true;
    halGpioSetPin(halGpio.spiFlashCs, false);
    flashTxRxProcessing(commandBuffTx, NULL, 1);
    while(flashProcessing.flashBussy){}
    // read data
    flashProcessing.csSet      = true;
    flashProcessing.flashBussy = true;
    flashTxRxProcessing(NULL, commandBuffRx, 1);
    while(flashProcessing.flashBussy){};

    return (((flashStatusReg1T*)commandBuffRx)->WEL) ? (FLASH_OK) : (FLASH_BUSSY);
}

bool isFlashBussy(void)
{
    return flashProcessing.flashBussy;
}


flashStatT flashReadData(uint8_t *buff, uint32_t dataSize, uint32_t address)
{
    while(flashGetState() == FLASH_BUSSY) {};
    // prepare command buff
    ((commanDescriptionT*)commandBuffTx)->command    = READ_DATA;
    ((commanDescriptionT*)commandBuffTx)->payload[2] = address         & 0xFF;
    ((commanDescriptionT*)commandBuffTx)->payload[1] = (address >> 8)  & 0xFF;
    ((commanDescriptionT*)commandBuffTx)->payload[0] = (address >> 16) & 0xFF;
    // send command
    flashProcessing.csSet      = false;
    flashProcessing.flashBussy = true;
    halGpioSetPin(halGpio.spiFlashCs, false);
    flashTxRxProcessing(commandBuffTx, NULL, 4);
    while(flashProcessing.flashBussy) {}
    // read data
    flashProcessing.csSet      = true;
    flashProcessing.flashBussy = true;
    flashTxRxProcessing(commandBuffTx, buff, dataSize );
    while(flashProcessing.flashBussy) {}
    return FLASH_OK;
}


flashStatT flashReadSubFrameStart(uint8_t *buff, uint32_t dataSize, uint32_t address)
{
    while(flashGetState() == FLASH_BUSSY) {};
    // prepare command buff
    ((commanDescriptionT*)commandBuffTx)->command    = READ_DATA;
    ((commanDescriptionT*)commandBuffTx)->payload[2] = address         & 0xFF;
    ((commanDescriptionT*)commandBuffTx)->payload[1] = (address >> 8)  & 0xFF;
    ((commanDescriptionT*)commandBuffTx)->payload[0] = (address >> 16) & 0xFF;
    // send command
    flashProcessing.csSet      = false;
    flashProcessing.flashBussy = true;
    halGpioSetPin(halGpio.spiFlashCs, false);
    flashTxRxProcessing(commandBuffTx, NULL, 4);
    while(flashProcessing.flashBussy) {}
    // read data
    flashProcessing.csSet      = false;
    flashProcessing.flashBussy = true;
    flashTxRxProcessing(NULL, buff, dataSize );
    while(flashProcessing.flashBussy) {}
    return FLASH_OK;
}


flashStatT flashReadSubFrameNext(uint8_t *buff, uint32_t dataSize, uint32_t address)
{
    // read data
    flashProcessing.csSet      = false;
    flashProcessing.flashBussy = true;
    flashTxRxProcessing(NULL, buff, dataSize );
    return FLASH_OK;
}

flashStatT flashReadSubFrameLast(uint8_t *buff, uint32_t dataSize, uint32_t address)
{
    // read data
    flashProcessing.csSet      = true;
    flashProcessing.flashBussy = true;
    flashTxRxProcessing(NULL, buff, dataSize );
    return FLASH_OK;
}





flashStatT flashWriteBlock(uint8_t *buff, uint32_t dataSize, uint32_t address)
{
    if(dataSize == 0)
    {
        return FLASH_OK;
    }
    uint16_t lastBlockQuantity  = dataSize % 256;
    uint16_t writeBlockQuantity = (lastBlockQuantity == 0) ? (dataSize/256 - 1) : (dataSize/256 );
    lastBlockQuantity           = (lastBlockQuantity == 0) ? (256) : (lastBlockQuantity);
    // prepare command buff

    for(uint16_t cnt = 0; cnt <= writeBlockQuantity; cnt++)
    {
        while(flashGetState() == FLASH_BUSSY) {};
        flashWriteEnable();
        while(flashGetWrite() == FLASH_BUSSY) {};

        halGpioSetPin(halGpio.spiFlashCs, false);
        // prepare command buff
        ((commanDescriptionT*)commandBuffTx)->command    = PROGRAM_PAGE;
        ((commanDescriptionT*)commandBuffTx)->payload[2] = address         & 0xFF;
        ((commanDescriptionT*)commandBuffTx)->payload[1] = (address >> 8)  & 0xFF;
        ((commanDescriptionT*)commandBuffTx)->payload[0] = (address >> 16) & 0xFF;
        address += 256;
        // send command
        flashProcessing.csSet = false;
        flashTxRxProcessing(commandBuffTx, NULL, 4);
        while(flashProcessing.flashBussy) {}
        // send data
        flashProcessing.csSet = true;
        flashTxRxProcessing(buff + 256 * cnt, NULL, (writeBlockQuantity == cnt) ? (lastBlockQuantity) : (256));
        while(flashProcessing.flashBussy) {}
    }

    return FLASH_OK;
}


flashStatT flashEraseBlock32(uint32_t blockNumber)
{
    flashWriteEnable();
    ((commanDescriptionT*)commandBuffTx)->command = BLOCK_ERASE32;
    ((commanDescriptionT*)commandBuffTx)->payload[2] = blockNumber & 0xFF;
    blockNumber >>= 8;
    ((commanDescriptionT*)commandBuffTx)->payload[1] = blockNumber & 0xFF;
    blockNumber >>= 8;
    ((commanDescriptionT*)commandBuffTx)->payload[0] = blockNumber & 0xFF;
    // send command
    flashProcessing.csSet      = true;
    flashProcessing.flashBussy = true;
    halGpioSetPin(halGpio.spiFlashCs, false);
    flashTxRxProcessing(commandBuffTx, NULL, 4);
    while(flashProcessing.flashBussy){}
    // wait for complete erasing
    while(flashGetState() == FLASH_BUSSY){}
    return FLASH_OK;
}


flashStatT flashWriteEnable(void)
{
    ((commanDescriptionT*)commandBuffTx)->command = WRITE_ENABLE;
    // send command
    flashProcessing.csSet      = true;
    flashProcessing.flashBussy = true;
    halGpioSetPin(halGpio.spiFlashCs, false);
    flashTxRxProcessing(commandBuffTx, NULL, 1);
    while(flashProcessing.flashBussy){}
    // read status
    return FLASH_OK;
}
