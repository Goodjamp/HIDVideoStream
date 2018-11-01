#include "Halspi.h"

#include "fsl_spi_dma.h"

#define SPI_DMA DMA0

#define EXAMPLE_SPI_MASTER SPI3
#define EXAMPLE_SPI_SSEL 1
#define EXAMPLE_SPI_MASTER_CLK_FREQ CLOCK_GetFreq(kCLOCK_Flexcomm3)
#define EXAMPLE_MASTER_SPI_SPOL kSPI_SpolActiveAllLow

#define EXAMPLE_SPI_MASTER_RX_CHANNEL 6
#define EXAMPLE_SPI_MASTER_TX_CHANNEL 7

static dma_handle_t masterTxHandle;
static dma_handle_t masterRxHandle;

static spi_dma_handle_t masterHandle;

static spi_transfer_t masterXfer;

static HalSpiTransferDoneCallback halSpiTransferDoneCallback = NULL;


static void SPI_MasterUserCallback(SPI_Type *base, spi_dma_handle_t *handle, status_t status, void *userData)
{
    if (halSpiTransferDoneCallback != NULL) {
        halSpiTransferDoneCallback(status == kStatus_Success);
    }
}

void halSpiInit(void)
{
    // attach 12 MHz clock to SPI3
    CLOCK_AttachClk(kFRO12M_to_FLEXCOMM3);
    // SPI init
    uint32_t srcClock_Hz;
    spi_master_config_t masterConfig;

    srcClock_Hz = EXAMPLE_SPI_MASTER_CLK_FREQ;
    // Master config
    SPI_MasterGetDefaultConfig(&masterConfig);
    masterConfig.sselNum = (spi_ssel_t)EXAMPLE_SPI_SSEL;
    masterConfig.sselPol = (spi_spol_t)EXAMPLE_MASTER_SPI_SPOL;
    SPI_MasterInit(EXAMPLE_SPI_MASTER, &masterConfig, srcClock_Hz);

    DMA_EnableChannel(SPI_DMA, EXAMPLE_SPI_MASTER_TX_CHANNEL);
    DMA_EnableChannel(SPI_DMA, EXAMPLE_SPI_MASTER_RX_CHANNEL);
    DMA_SetChannelPriority(SPI_DMA, EXAMPLE_SPI_MASTER_TX_CHANNEL, kDMA_ChannelPriority3);
    DMA_SetChannelPriority(SPI_DMA, EXAMPLE_SPI_MASTER_RX_CHANNEL, kDMA_ChannelPriority2);
}


bool halSpiTransfer(uint8_t txData[], uint8_t rxData[], uint32_t transferDataLength, HalSpiTransferDoneCallback callback)
{
    halSpiTransferDoneCallback = callback;

    DMA_CreateHandle(&masterTxHandle, SPI_DMA, EXAMPLE_SPI_MASTER_TX_CHANNEL);
    DMA_CreateHandle(&masterRxHandle, SPI_DMA, EXAMPLE_SPI_MASTER_RX_CHANNEL);

    // Set up spi master
    SPI_MasterTransferCreateHandleDMA(EXAMPLE_SPI_MASTER, &masterHandle, SPI_MasterUserCallback, NULL, &masterTxHandle, &masterRxHandle);

    // Start master transfer
    masterXfer.txData = txData;
    masterXfer.rxData = rxData;
    masterXfer.dataSize = transferDataLength;
    masterXfer.configFlags |= kSPI_FrameAssert;

    return (kStatus_Success == SPI_MasterTransferDMA(EXAMPLE_SPI_MASTER, &masterHandle, &masterXfer));
}

