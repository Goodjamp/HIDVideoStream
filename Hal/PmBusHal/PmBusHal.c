#include "PmBusHal.h"

#include "chip.h"
#include <string.h>
/* HW definitions */
#define PM_BUS_I2C_HW_UNIT                      I2C0
#define PM_BUS_I2C_PERIPH_RESET                 RESET_I2C0

// I2C0 chips
#define PM_BUS_I2C_SDA_PIO_NUM                  0
#define PM_BUS_I2C_SDA_PIN_NUM                  5

#define PM_BUS_I2C_SCL_PIO_NUM                  0
#define PM_BUS_I2C_SCL_PIN_NUM                  4

/* Settings definitions */
#define I2C_SPEED_400KHZ                        (400000)
#define I2C_SPEED_1MHZ                          (1000000)
#define I2C_SPEED_100KHZ                        (100000)

#define I2C0_BITRATE                            I2C_SPEED_100KHZ

#define I2C_RD_CMD_BIT                          (0x01)

#define I2C_FASTPLUS_BIT                        (0)

#if (I2C_BITRATE > SPEED_400KHZ)

#undef  I2C_FASTPLUS_BIT
#define I2C_FASTPLUS_BIT                IOCON_FASTI2C_EN

#endif

#define I2C_MASTER_HANDLE_MEM 0x20
static uint32_t i2c0MasterHandleMEM[I2C_MASTER_HANDLE_MEM];
static I2C_HANDLE_T *i2c0HandleMaster;

static I2C_PARAM_T param;
static I2C_RESULT_T result;

bool transferComplete = false;

static void i2cOperationCompleteCb(uint32_t errorCode, uint32_t n)
{
    if (errorCode != LPC_OK) {
        transferComplete = true;
    } else if (result.n_bytes_sent == param.num_bytes_send && result.n_bytes_recd == param.num_bytes_rec) {
        transferComplete = true;
    }
}

static bool i2cIsBusOk(void)
{
    bool scl = Chip_GPIO_GetPinState(LPC_GPIO, PM_BUS_I2C_SCL_PIO_NUM, PM_BUS_I2C_SCL_PIN_NUM);
    bool sda = Chip_GPIO_GetPinState(LPC_GPIO, PM_BUS_I2C_SDA_PIO_NUM, PM_BUS_I2C_SDA_PIN_NUM);

    if (scl == false || sda == false) {
            volatile int i = 0;
    }
    return (scl && sda);
}

static bool i2cReadBuffer(uint8_t buff[], uint32_t bytesToRead)
{
    transferComplete = false;
	//I2C_PARAM_T param;
	/* Setup I2C parameters for number of bytes with stop - appears as follows on bus:
	   Start - address7 or address10upper - ack
	   (10 bits addressing only) address10lower - ack
	   value 1 (read) - ack
	   value 2 read) - ack - stop */
	param.num_bytes_send = 0;
	param.num_bytes_rec = bytesToRead;
	param.buffer_ptr_rec = buff;
	param.stop_flag = 1;
	param.func_pt = i2cOperationCompleteCb;

    result.n_bytes_recd = 0;
    result.n_bytes_sent = 0;

	/* Function is non-blocking, returned error should be LPC_OK, but isn't checked here */
	LPC_I2CD_API->i2c_master_receive_intr(i2c0HandleMaster, &param, &result);
    return true;
}

static bool i2cWriteBuffer(const uint8_t buff[], uint32_t bufferSize)
{
    transferComplete = false;
	//I2C_RESULT_T result;
	/* Setup I2C parameters for number of bytes with stop - appears as follows on bus:
	   Start - address7 or address10upper - ack
	   (10 bits addressing only) address10lower - ack
	   value 1 - ack
	   value 2 - ack - stop */
	param.num_bytes_send = bufferSize;
	param.buffer_ptr_send = (uint8_t *) buff;
	param.num_bytes_rec = 0;
	param.stop_flag = 1;
	param.func_pt = i2cOperationCompleteCb;

    result.n_bytes_recd = 0;
    result.n_bytes_sent = 0;

	/* Function is non-blocking, returned error should be LPC_OK, but isn't checked here */
	uint32_t res = LPC_I2CD_API->i2c_master_transmit_intr(i2c0HandleMaster, &param, &result);
	return true;
}

void pmBusInit(void)
{
    // pin muxing
    Chip_SYSCTL_PeriphReset(PM_BUS_I2C_PERIPH_RESET);
    Chip_IOCON_PinMuxSet(LPC_IOCON, PM_BUS_I2C_SCL_PIO_NUM, PM_BUS_I2C_SCL_PIN_NUM, (IOCON_FUNC1 | I2C_FASTPLUS_BIT) | IOCON_DIGMODE_EN);
    Chip_IOCON_PinMuxSet(LPC_IOCON, PM_BUS_I2C_SDA_PIO_NUM, PM_BUS_I2C_SDA_PIN_NUM, (IOCON_FUNC1 | I2C_FASTPLUS_BIT) | IOCON_DIGMODE_EN);
    // i2c setup
    Chip_I2C_Init(PM_BUS_I2C_HW_UNIT);

    if (LPC_I2CD_API->i2c_get_mem_size() > sizeof(i2c0MasterHandleMEM)) {
        return;
    }

    /* Setup the I2C handle */
    i2c0HandleMaster = LPC_I2CD_API->i2c_setup(LPC_I2C0_BASE, i2c0MasterHandleMEM);

    if (i2c0HandleMaster == NULL) {
        return;
    }

    /* Set I2C bitrate */
    if (LPC_I2CD_API->i2c_set_bitrate(i2c0HandleMaster, Chip_Clock_GetSystemClockRate(), I2C0_BITRATE) != LPC_OK)
        return;

    NVIC_EnableIRQ(I2C0_IRQn);
}

void I2C0_IRQHandler(void)
{
	/* Call I2C ISR function in ROM with the I2C handle */
	LPC_I2CD_API->i2c_isr_handler(i2c0HandleMaster);
}

bool pmBusWrite(uint8_t addr, uint8_t cmdData[], uint8_t cmdDataLength, uint8_t data[], uint8_t dataLength)
{
    if (i2cIsBusOk() == false) {
        return false;
    }

    uint8_t writeData[65];
    writeData[0] = addr << 1;

    memcpy(&writeData[1], cmdData, cmdDataLength);

    i2cWriteBuffer(writeData, cmdDataLength + 1);

    while (transferComplete == false)
        {};

    if (dataLength) {
        memcpy(&writeData[1], data, dataLength);

        i2cWriteBuffer(writeData, dataLength + 1);

        while (transferComplete == false)
            {};
    }
    return true;
}

bool pmBusRead(uint8_t addr, uint8_t cmdData[], uint8_t cmdDataLength, uint8_t data[], uint8_t dataLength)
{
    if (i2cIsBusOk() == false) {
        return false;
    }

    uint8_t writeData[65];

    writeData[0] = addr << 1;
    memcpy(&writeData[1], cmdData, cmdDataLength);

    i2cWriteBuffer(writeData, cmdDataLength + 1);

    while (transferComplete == false)
        {};

    data[0] = (addr << 1) | I2C_RD_CMD_BIT;

    i2cReadBuffer(data, dataLength);

    while (transferComplete == false)
        {};

    return true;
}
