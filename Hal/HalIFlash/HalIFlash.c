#include "stdint.h"

#include "LPC54608.H"

#include "fsl_dma.h"
#include "fsl_spi.h"
#include "fsl_spi_dma.h"

/*
14 Flexcomm Interface 7 RX / I2C Slave [1] DMA_ITRIG_INMUX14
15 Flexcomm Interface 7 TX / I2C Master [1] DMA_ITRIG_INMUX15

*/

#define DMA_SPIF_LASH_RX_CHANEL  14
#define DMA_SPIF_LASH_TX_CHANEL  15



void initSPIFlash(void)
{
    DMA_Init(DMA0);

    DMA_EnableChannelInterrupts(DMA0, DMA_SPIF_LASH_RX_CHANEL);
    DMA_EnableChannelInterrupts(DMA0, DMA_SPIF_LASH_TX_CHANEL);

    DMA_EnableChannel(DMA0, DMA_SPIF_LASH_RX_CHANEL);
    DMA_EnableChannel(DMA0, DMA_SPIF_LASH_TX_CHANEL);

    DMA_EnableChannelPeriphRq(DMA0, DMA_SPIF_LASH_RX_CHANEL);
    DMA_EnableChannelPeriphRq(DMA0, DMA_SPIF_LASH_TX_CHANEL);
}

void initSPIDMAFlash(void)
{

}
