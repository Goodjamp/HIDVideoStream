//#include "Halspi.h"
#include "fsl_gpio.h"
#include "fsl_iocon.h"
#include "fsl_spi_dma.h"

#include  "lcd.h"

#define SPI_DMA DMA0

#define EXAMPLE_SPI_MASTER SPI3
#define EXAMPLE_SPI_SSEL 0
#define EXAMPLE_SPI_MASTER_CLK_FREQ CLOCK_GetFreq(kCLOCK_Flexcomm3)
#define EXAMPLE_MASTER_SPI_SPOL kSPI_SpolActiveAllLow

#define EXAMPLE_SPI_MASTER_RX_CHANNEL 6
#define EXAMPLE_SPI_MASTER_TX_CHANNEL 7

#define GPIO_CONFIG_OUT_LOGIC_0 { .pinDirection = kGPIO_DigitalOutput, .outputLogic = 0 }
#define GPIO_CONFIG_OUT_LOGIC_1 { .pinDirection = kGPIO_DigitalOutput, .outputLogic = 1 }
#define GPIO_CONFIG_INPUT       { .pinDirection = kGPIO_DigitalInput, .outputLogic = 0 }

#define DIGITAL_WITH_PULLUP (IOCON_MODE_PULLUP | IOCON_DIGITAL_EN | IOCON_INPFILT_OFF)


typedef struct HalGpioPinStruct {
    iocon_group_t iocon;
    gpio_pin_config_t config;
} HalGpioPinStruct;

static dma_handle_t masterTxHandle;
static dma_handle_t masterRxHandle;

static spi_dma_handle_t masterHandle;

static spi_transfer_t masterXfer;

static HalSpiTransferDoneCallback halSpiTransferDoneCallback = NULL;

static volatile bool isTxDone = false;

typedef struct HalGpioPinStruct * HalGpioPin;

typedef struct HalGpio {
    HalGpioPin spiOledDc;
    HalGpioPin spiOledMosi;
    HalGpioPin spiOledSck;
    HalGpioPin spiOledCs;
    HalGpioPin spiOledRst;
} HalGpio;

static void halGpioPinInit(HalGpioPin pin);
static void halGpioSetPin(HalGpioPin pin, bool value);

static const HalGpio halGpioPin = {
   .spiOledDc = &(HalGpioPinStruct) {
        .iocon = {
            .port = 2,
            .pin = 20,
            .modefunc = IOCON_FUNC0 | DIGITAL_WITH_PULLUP
        },
        .config	= GPIO_CONFIG_OUT_LOGIC_1
    },
    .spiOledMosi = &(HalGpioPinStruct) {
        .iocon = {
            .port = 2,
            .pin = 18,
            .modefunc = IOCON_FUNC2 | DIGITAL_WITH_PULLUP
        },
        .config = GPIO_CONFIG_OUT_LOGIC_0
    },
    .spiOledSck = &(HalGpioPinStruct) {
        .iocon = {
            .port = 2,
            .pin = 26,
            .modefunc = IOCON_FUNC3 | DIGITAL_WITH_PULLUP
        },
        .config = GPIO_CONFIG_OUT_LOGIC_0
    },
    .spiOledCs = &(HalGpioPinStruct) {
        .iocon = {
            .port = 2,
            .pin = 21,
            .modefunc = IOCON_FUNC2 | DIGITAL_WITH_PULLUP
        },
        .config = GPIO_CONFIG_OUT_LOGIC_1
    },
    .spiOledRst = &(HalGpioPinStruct) {
        .iocon = {
            .port = 2,
            .pin = 19,
            .modefunc = IOCON_FUNC0 | DIGITAL_WITH_PULLUP
        },
        .config = GPIO_CONFIG_OUT_LOGIC_0
    },
};

static void halGpioPinInit(HalGpioPin pin)
{
    IOCON_PinMuxSet(IOCON, pin->iocon.port, pin->iocon.pin, pin->iocon.modefunc);
    GPIO_PinInit(GPIO, pin->iocon.port, pin->iocon.pin, &pin->config);
}

static void halGpioSetPin(HalGpioPin pin, bool value)
{
    GPIO_WritePinOutput(GPIO, pin->iocon.port, pin->iocon.pin, value);
}

static void SPI_MasterUserCallback(SPI_Type *base, spi_dma_handle_t *handle, status_t status, void *userData)
{
    isTxDone = true;
    if (halSpiTransferDoneCallback != NULL) {
        halSpiTransferDoneCallback(status == kStatus_Success);
    }
}

bool halSpiTransfer(const uint8_t txData[], uint8_t rxData[], uint32_t transferDataLength, HalSpiTransferDoneCallback callback)
{
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

void sendCommand(uint8_t data)
{
    uint8_t receiveData;
    halGpioSetPin(halGpioPin.spiOledDc, false);
    isTxDone = false;
    halSpiTransfer(&data, &receiveData, sizeof(data), NULL);
    while (isTxDone == false) { }
}

void sendData(uint8_t data)
{
    while (isTxDone == false) {

    }

    uint8_t receiveData;
    halGpioSetPin(halGpioPin.spiOledDc, true);
    isTxDone = false;
    halSpiTransfer(&data, &receiveData, sizeof(data), NULL);
}

void putPicture(const uint8_t data[])
{
    sendCommand(0x0C);
    halGpioSetPin(halGpioPin.spiOledDc, true);
    uint32_t a = 0;
    for (a = 0; a < 32; a++) {
        isTxDone = false;
        halSpiTransfer(data + a * 1024, NULL, 1024, NULL);
        while (isTxDone == false) {

        }
    }
}



void putSubFrameStart(const uint8_t data[], uint16_t buffSize, HalSpiTransferDoneCallback callback)
{
    sendCommand(0x0C);
    halGpioSetPin(halGpioPin.spiOledDc, true);
    isTxDone = false;
    halSpiTransfer(data, NULL, buffSize, callback);
}


void putSubFrameNext(const uint8_t data[], uint16_t buffSize, HalSpiTransferDoneCallback callback)
{
    isTxDone = false;
    halSpiTransfer(data, NULL, buffSize, callback);
}


void lcdInit(void)
{
    DMA_Init(DMA0);
    halGpioPinInit(halGpioPin.spiOledRst);
    halGpioPinInit(halGpioPin.spiOledCs);
    halGpioPinInit(halGpioPin.spiOledDc);
    halGpioPinInit(halGpioPin.spiOledMosi);
    halGpioPinInit(halGpioPin.spiOledSck);
    // attach 12 MHz clock to SPI3
    CLOCK_AttachClk(kFRO_HF_to_FLEXCOMM3);
    // SPI init
    uint32_t srcClock_Hz;
    spi_master_config_t masterConfig;

    srcClock_Hz = EXAMPLE_SPI_MASTER_CLK_FREQ;
    // Master config
    SPI_MasterGetDefaultConfig(&masterConfig);
    masterConfig.sselNum = (spi_ssel_t)EXAMPLE_SPI_SSEL;
    masterConfig.sselPol = (spi_spol_t)EXAMPLE_MASTER_SPI_SPOL;
    masterConfig.baudRate_Bps = 24000000;
    SPI_MasterInit(EXAMPLE_SPI_MASTER, &masterConfig, srcClock_Hz);

    DMA_EnableChannel(SPI_DMA, EXAMPLE_SPI_MASTER_TX_CHANNEL);
    DMA_EnableChannel(SPI_DMA, EXAMPLE_SPI_MASTER_RX_CHANNEL);
    DMA_SetChannelPriority(SPI_DMA, EXAMPLE_SPI_MASTER_TX_CHANNEL, kDMA_ChannelPriority3);
    DMA_SetChannelPriority(SPI_DMA, EXAMPLE_SPI_MASTER_RX_CHANNEL, kDMA_ChannelPriority2);
    volatile uint32_t delayCntr = 0;
    for (delayCntr = 0; delayCntr < 1000000; delayCntr++) {

    }

    halGpioSetPin(halGpioPin.spiOledRst, true);

    for (delayCntr = 0; delayCntr < 1000000; delayCntr++) {

    }

    sendCommand(0x01);

    sendCommand(0x03);
    sendData(0x00);

    sendCommand(0x04);
    sendData(0x04);

    sendCommand(0x05);
    sendData(0x00);

    sendCommand(0x06);
    sendData(0x00);

    sendCommand(0x07);
    sendData(0x00);
    sendData(0x00);
    sendData(0x07);
    sendData(0x0F);
    sendData(0x00);
    sendData(0x00);
    sendData(0x07);
    sendData(0x0F);

    sendCommand(0x08);
    sendData(0x01);

    sendCommand(0x09);
    sendData(0x07);

    sendCommand(0x0A);
    sendData(0x00);
    sendData(0x00);
    sendData(0x07);
    sendData(0x0F);
    sendData(0x00);
    sendData(0x00);
    sendData(0x07);
    sendData(0x0F);

    sendCommand(0x0B);
    sendData(0x00);
    sendData(0x00);
    sendData(0x00);
    sendData(0x00);

    sendCommand(0x0E);
    sendData(0x03);
    sendData(0x0A);
    sendData(0x03);
    sendData(0x0A);
    sendData(0x03);
    sendData(0x0A);

    sendCommand(0x0F);
    sendData(0x14);
    sendData(0x14);
    sendData(0x14);

    sendCommand(0x1C);
    sendData(0x08);

    sendCommand(0x1D);
    sendData(0x0A);
    sendData(0x0A);
    sendData(0x0A);

    sendCommand(0x1E);
    sendData(0x00);

    sendCommand(0x1F);
    sendData(0x00);

    sendCommand(0x30);
    sendData(0x12);

    sendCommand(0x02);
    sendData(0x01);

    sendCommand(0x3C);
    sendData(0x01);

    sendCommand(0x3A); //Set Gamma Correction Table

    static const uint8_t Gray_Scale_TB1[64] = {
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x02,0x00,0x03,0x00,0x04,0x00,0x06,
        0x00,0x08,0x00,0x0A,0x00,0x0D,0x00,0x0F,0x01,0x03,0x01,0x06,0x01,0x09,0x01,0x0D,
        0x02,0x01,0x02,0x06,0x02,0x0A,0x02,0x0F,0x03,0x04,0x03,0x0A,0x03,0x0F,0x04,0x05,
        0x04,0x0C,0x05,0x02,0x05,0x09,0x06,0x00,0x06,0x07,0x06,0x0F,0x07,0x06,0x07,0x0F
    };
    static const uint8_t Gray_Scale_TB2[128] = {
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x01,
        0x00,0x02,0x00,0x02,0x00,0x03,0x00,0x03,0x00,0x04,0x00,0x05,0x00,0x06,0x00,0x07,
        0x00,0x08,0x00,0x09,0x00,0x0A,0x00,0x0B,0x00,0x0C,0x00,0x0E,0x00,0x0F,0x01,0x00,
        0x01,0x02,0x01,0x03,0x01,0x05,0x01,0x07,0x01,0x09,0x01,0x0A,0x01,0x0C,0x01,0x0E,
        0x02,0x00,0x02,0x02,0x02,0x04,0x02,0x07,0x02,0x09,0x02,0x0B,0x02,0x0E,0x03,0x00,
        0x03,0x03,0x03,0x05,0x03,0x08,0x03,0x0B,0x03,0x0D,0x04,0x00,0x04,0x03,0x04,0x06,
        0x04,0x09,0x04,0x0C,0x04,0x0F,0x05,0x03,0x05,0x06,0x05,0x09,0x05,0x0D,0x06,0x00,
        0x06,0x04,0x06,0x07,0x06,0x0B,0x06,0x0F,0x07,0x03,0x07,0x07,0x07,0x0B,0x07,0x0F
    };
    uint32_t cntr = 0;
    for (cntr = 0; cntr < 64; cntr++) {
        sendData(Gray_Scale_TB1[cntr]);
    } //Red

    for(cntr = 0; cntr < 128; cntr++) {
        sendData(Gray_Scale_TB2[cntr]);
    } //Green

    for(cntr = 0; cntr < 64; cntr++) {
        sendData(Gray_Scale_TB1[cntr]);
    } //Blue
}

