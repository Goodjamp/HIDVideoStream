#include "stdint.h"

#include "LPC54608.H"

#include "fsl_dma.h"
#include "fsl_spi.h"
#include "fsl_spi_dma.h"

/*
14 Flexcomm Interface 7 RX / I2C Slave [1] DMA_ITRIG_INMUX14
15 Flexcomm Interface 7 TX / I2C Master [1] DMA_ITRIG_INMUX15

*/

#define SPI_FLASH                 SPI7
#define DMA_SPI_FLASH             DMA0


#define DMA_SPIF_FLASH_RX_CHANEL  14
#define DMA_SPIF_FLASH_TX_CHANEL  15

dma_handle_t  dmaSpiFlashRxHandle;
dma_handle_t  dmaSpiFlashTxHandle;

dma_descriptor_t dmaSpiFlashRxDescriptor;
dma_descriptor_t dmaSpiFlashTxDescriptor;

void initSPIFlash(void)
{
    DMA_Init(DMA0);

    DMA_EnableChannel(DMA0, DMA_SPIF_FLASH_RX_CHANEL);
    DMA_EnableChannel(DMA0, DMA_SPIF_FLASH_TX_CHANEL);

    DMA_SetChannelPriority(DMA0, DMA_SPIF_FLASH_RX_CHANEL, kDMA_ChannelPriority2);
    DMA_SetChannelPriority(DMA0, DMA_SPIF_FLASH_TX_CHANEL, kDMA_ChannelPriority2);

}


void sendCommand()


void readData(uint32_t address, uint8_t *buffer, uint32_t dataSize)
{
    /*pull down CS*/
    /*start send command*/
    /*wait for completing send command*/
    /*start receive data*/

    halSpiTransferDoneCallback = callback;

    DMA_CreateHandle(&masterTxHandle, SPI_DMA, EXAMPLE_SPI_MASTER_TX_CHANNEL);
    DMA_CreateHandle(&masterRxHandle, SPI_DMA, EXAMPLE_SPI_MASTER_RX_CHANNEL);

    // Set up spi master
    SPI_MasterTransferCreateHandleDMA(EXAMPLE_SPI_MASTER, &masterHandle, SPI_MasterUserCallback, NULL, &masterTxHandle, &masterRxHandle);

    // Start master transfer
    masterXfer.txData = (uint8_t *) txData;
    masterXfer.rxData = rxData;
    masterXfer.dataSize = transferDataLength;
    masterXfer.configFlags |= kSPI_FrameAssert;

    return (kStatus_Success == SPI_MasterTransferDMA(EXAMPLE_SPI_MASTER, &masterHandle, &masterXfer));
}



}


void spiFlashTransferCompliteCB(void)
{

}


void initSPIDMAFlash(void)
{
    //
}
