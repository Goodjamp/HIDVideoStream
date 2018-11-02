#include "stdint.h"
#include "stdbool.h"

#include "LPC54608.H"

#include "fsl_dma.h"
#include "fsl_spi.h"
#include "fsl_spi_dma.h"

#include "HalGpio.h"

/*
14 Flexcomm Interface 7 RX / I2C Slave [1] DMA_ITRIG_INMUX14
15 Flexcomm Interface 7 TX / I2C Master [1] DMA_ITRIG_INMUX15

*/

#define SPI_FLASH        SPI4
#define DMA_SPI_FLASH    DMA0
#define SPI_FLASH_FRQ    1000000

#define DMA_SPI_FLASH_RX_CHANEL  8
#define DMA_SPI_FLASH_TX_CHANEL  9

dma_handle_t  dmaSpiFlashRxHandle;
dma_handle_t  dmaSpiFlashTxHandle;

dma_descriptor_t dmaSpiFlashRxDescriptor;
dma_descriptor_t dmaSpiFlashTxDescriptor;

/*SPI-DMA variables*/
spi_dma_handle_t spiDmaFlashHandl;
static spi_transfer_t masterXfer;

extern const HalGpio halGpio;

static bool csSet;

#define TX_RX_BUFF_SIZE    7
uint8_t buffTx[TX_RX_BUFF_SIZE] = {0x9F,0,0,0,0,0,0};
uint8_t buffRx[TX_RX_BUFF_SIZE];
status_t readData(uint32_t address, uint8_t *bufferTx, uint8_t *bufferRx, uint32_t dataSize);


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
    /**/
    readData(0, buffTx, buffRx, sizeof(buffTx));
}


void spiDmaCB(SPI_Type *base, spi_dma_handle_t *handle, status_t status, void *userData)
{
    bool *inDtat = userData;
    halGpioSetPin(halGpio.spiFlashCs, (*inDtat) ? (true) : (false));
    readData(0, buffTx, buffRx, sizeof(buffTx));
}


status_t readData(uint32_t address, uint8_t *bufferTx, uint8_t *bufferRx, uint32_t dataSize)
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
    csSet = true;
    SPI_MasterTransferCreateHandleDMA(SPI_FLASH, &spiDmaFlashHandl, spiDmaCB, &csSet, &dmaSpiFlashTxHandle, &dmaSpiFlashRxHandle);

    // Start master transfer
    masterXfer.txData   = bufferTx;
    masterXfer.rxData   = bufferRx;
    masterXfer.dataSize = dataSize;
    masterXfer.configFlags |= kSPI_FrameAssert;

    return SPI_MasterTransferDMA(SPI_FLASH, &spiDmaFlashHandl, &masterXfer);
}


void spiFlashTransferCompliteCB(void)
{

}


void initSPIDMAFlash(void)
{
    //
}
