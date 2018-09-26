#include "HalI2c.h"
#include "fsl_i2c.h"
#include "fsl_i2c_dma.h"
#include "fsl_iocon.h"

#define I2C_DMA                         DMA0
#define HAL_I2C_DMA_CHANNEL_0           3
#define HAL_I2C_DMA_CHANNEL_1           5

#define I2C_MASTER_0_BASE               ((I2C_Type *) I2C1_BASE)
#define I2C_MASTER_1_BASE               ((I2C_Type *) I2C2_BASE)

#define I2C_BAUDRATE                    (100000)


#define HAL_I2C_SDA_0_PORT              0
#define HAL_I2C_SCL_0_PORT              0
#define HAL_I2C_SDA_1_PORT              1
#define HAL_I2C_SCL_1_PORT              1

#define HAL_I2C_SDA_0_PIN               13
#define HAL_I2C_SCL_0_PIN               14
#define HAL_I2C_SDA_1_PIN               26
#define HAL_I2C_SCL_1_PIN               27


static i2c_master_dma_handle_t masterDmaHandle[I2C_MASTER_COUNT];
static dma_handle_t dmaHandle[I2C_MASTER_COUNT];
static i2c_master_transfer_t masterXfer[I2C_MASTER_COUNT];

static HalI2cEventCallback halI2cEventCallback[I2C_MASTER_COUNT];
static const I2cMaster i2cMasterUnits[I2C_MASTER_COUNT] = {I2C_MASTER_0, I2C_MASTER_1};
static const uint32_t i2cMasterDmaChannel[I2C_MASTER_COUNT] = {HAL_I2C_DMA_CHANNEL_0, HAL_I2C_DMA_CHANNEL_1};
static const I2C_Type *i2cMasterBase[I2C_MASTER_COUNT] = {I2C_MASTER_0_BASE, I2C_MASTER_1_BASE};
static volatile bool isTransferCompleted[I2C_MASTER_COUNT] = {true, true};

static void halI2cDmaCallback(I2C_Type *base, i2c_master_dma_handle_t *handle, status_t status, void *userData)
{
    if (userData != NULL) {
        isTransferCompleted[*((I2cMaster *) userData)] = true;
        if (halI2cEventCallback[*((I2cMaster *) userData)] != NULL) {
            halI2cEventCallback[*((I2cMaster *) userData)](status == kStatus_Success);
        }
    }
}

static const iocon_group_t halI2cPinMux[] = {
    {HAL_I2C_SDA_0_PORT, HAL_I2C_SDA_0_PIN, IOCON_FUNC1 | IOCON_MODE_PULLUP | IOCON_DIGITAL_EN | IOCON_INPFILT_OFF},
    {HAL_I2C_SCL_0_PORT, HAL_I2C_SCL_0_PIN, IOCON_FUNC1 | IOCON_MODE_PULLUP | IOCON_DIGITAL_EN | IOCON_INPFILT_OFF},
    {HAL_I2C_SDA_1_PORT, HAL_I2C_SDA_1_PIN, IOCON_FUNC1 | IOCON_MODE_PULLUP | IOCON_DIGITAL_EN | IOCON_INPFILT_OFF},
    {HAL_I2C_SCL_1_PORT, HAL_I2C_SCL_1_PIN, IOCON_FUNC1 | IOCON_MODE_PULLUP | IOCON_DIGITAL_EN | IOCON_INPFILT_OFF},
};

static void halI2cPinsInit(void)
{
    IOCON_SetPinMuxing(IOCON, halI2cPinMux, sizeof(halI2cPinMux) / sizeof(*halI2cPinMux));
}

void halI2cInit(void)
{
    i2c_master_config_t masterConfig;

    // FC2_I2C FC1_I2C
    CLOCK_AttachClk(kFRO12M_to_FLEXCOMM2);
    CLOCK_AttachClk(kFRO12M_to_FLEXCOMM1);

    /* reset FLEXCOMM for I2C */
    RESET_PeripheralReset(kFC2_RST_SHIFT_RSTn);
    RESET_PeripheralReset(kFC1_RST_SHIFT_RSTn);

    halI2cPinsInit();
    DMA_Init(I2C_DMA);

    I2C_MasterGetDefaultConfig(&masterConfig);

    /* Change the default baudrate configuration */
    masterConfig.baudRate_Bps = I2C_BAUDRATE;
    masterConfig.enableTimeout = true;

    /* Initialize the I2C master peripheral */
    I2C_MasterInit((I2C_Type *) i2cMasterBase[I2C_MASTER_0], &masterConfig, CLOCK_GetFreq(kCLOCK_Flexcomm2));
    I2C_MasterInit((I2C_Type *) i2cMasterBase[I2C_MASTER_1], &masterConfig, CLOCK_GetFreq(kCLOCK_Flexcomm1));
}

bool halI2cReadBuff(I2cMaster i2cMaster, uint8_t slaveAddress, uint8_t regAddress, uint8_t dataBuffer[], size_t bufferLength, HalI2cEventCallback halI2cEventCb)
{
    if (isTransferCompleted[i2cMaster] == false) {
        return false;
    }

    halI2cEventCallback[i2cMaster] = halI2cEventCb;
    memset(&masterDmaHandle[i2cMaster], 0, sizeof(masterDmaHandle[i2cMaster]));
    memset(&masterXfer[i2cMaster], 0, sizeof(masterXfer[i2cMaster]));

    masterXfer[i2cMaster].slaveAddress = slaveAddress;
    masterXfer[i2cMaster].direction = kI2C_Read;
    masterXfer[i2cMaster].subaddress = regAddress;
    masterXfer[i2cMaster].subaddressSize = sizeof(regAddress);
    masterXfer[i2cMaster].data = dataBuffer;
    masterXfer[i2cMaster].dataSize = bufferLength;
    masterXfer[i2cMaster].flags = kI2C_TransferDefaultFlag;

    DMA_EnableChannel(I2C_DMA, i2cMasterDmaChannel[i2cMaster]);
    DMA_CreateHandle(&dmaHandle[i2cMaster], I2C_DMA, i2cMasterDmaChannel[i2cMaster]);

    isTransferCompleted[i2cMaster] = false;
    I2C_MasterTransferCreateHandleDMA((I2C_Type *) i2cMasterBase[i2cMaster], &masterDmaHandle[i2cMaster], halI2cDmaCallback, (I2cMaster *) &i2cMasterUnits[i2cMaster], &dmaHandle[i2cMaster]);
    status_t status = I2C_MasterTransferDMA((I2C_Type *) i2cMasterBase[i2cMaster], &masterDmaHandle[i2cMaster], &masterXfer[i2cMaster]);

    return status == kStatus_Success;
}

bool halI2cWriteBuff(I2cMaster i2cMaster, uint8_t slaveAddress, uint8_t regAddress, const uint8_t dataBuffer[], size_t bufferLength, HalI2cEventCallback halI2cEventCb)
{
    if (isTransferCompleted[i2cMaster] == false) {
        return false;
    }

    halI2cEventCallback[i2cMaster] = halI2cEventCb;
    memset(&masterDmaHandle[i2cMaster], 0, sizeof(masterDmaHandle[i2cMaster]));
    memset(&masterXfer[i2cMaster], 0, sizeof(masterXfer[i2cMaster]));

    masterXfer[i2cMaster].slaveAddress = slaveAddress;
    masterXfer[i2cMaster].direction = kI2C_Write;
    masterXfer[i2cMaster].subaddress = regAddress;
    masterXfer[i2cMaster].subaddressSize = sizeof(regAddress);
    masterXfer[i2cMaster].data = (uint8_t *) dataBuffer;
    masterXfer[i2cMaster].dataSize = bufferLength;
    masterXfer[i2cMaster].flags = kI2C_TransferDefaultFlag;

    DMA_EnableChannel(I2C_DMA, i2cMasterDmaChannel[i2cMaster]);
    DMA_CreateHandle(&dmaHandle[i2cMaster], I2C_DMA, i2cMasterDmaChannel[i2cMaster]);

    isTransferCompleted[i2cMaster] = false;
    I2C_MasterTransferCreateHandleDMA((I2C_Type *) i2cMasterBase[i2cMaster], &masterDmaHandle[i2cMaster], halI2cDmaCallback, (I2cMaster *) &i2cMasterUnits[i2cMaster], &dmaHandle[i2cMaster]);
    status_t status = I2C_MasterTransferDMA((I2C_Type *) i2cMasterBase[i2cMaster], &masterDmaHandle[i2cMaster], &masterXfer[i2cMaster]);

    return status == kStatus_Success;
}

void halI2cDeinit(void)
{
    I2C_MasterDeinit((I2C_Type *) i2cMasterBase[I2C_MASTER_0]);
    I2C_MasterDeinit((I2C_Type *) i2cMasterBase[I2C_MASTER_1]);
}
