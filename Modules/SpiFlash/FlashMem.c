#include "chip.h"

#include "FlashMem.h"

/* Settings definitions */
#define FLASH_MEM_USE_TRANSFER_IRQ      (0)
#define FLASH_MEM_SSP_BIT_RATE          (24000000)

/* HW definitions */
#define FLASH_MEM_SSP_HW_UNIT           LPC_SSP0

#define FLASH_MEM_WP_PIO_NUM            2
#define FLASH_MEM_WP_PIN_NUM            22
#define FLASH_MEM_SSEL_PIO_NUM          0
#define FLASH_MEM_SSEL_PIN_NUM          2

#define FLASH_MEM_MISO_PIO_NUM          0
#define FLASH_MEM_MISO_PIN_NUM          8

#define FLASH_MEM_SCK_PIO_NUM           0
#define FLASH_MEM_SCK_PIN_NUM           6

#define FLASH_MEM_MOSI_PIO_NUM          1
#define FLASH_MEM_MOSI_PIN_NUM          12

#define FLASH_MEM_SSP_IRQN              SSP0_IRQn
#define FLASH_MEM_SSP_IRQ_HANDLER       SSP0_IRQHandler

/* Flash chip command definitions */
#define FLASH_MEM_WRITE_ENABLE_CMD      0x06
#define FLASH_MEM_WRITE_DISABLE_CMD     0x04
#define FLASH_MEM_WRITE_PAGE_CMD        0x02
#define FLASH_MEM_READ_PAGE_CMD         0x03
#define FLASH_MEM_READ_STATUS_REG_CMD   0x05
#define FLASH_MEM_ERASE_SECTOR_CMD      0x20
#define FLASH_MEM_READ_IDS_CMD          0x9F
#define FLASH_MEM_READ_SPDF_UID_CMD     0x5A

#define FLASH_MEM_SECTOR_ADDRESS_MASK   0x7FF000
#define FLASH_MEM_PAGE_ADDRESS_MASK     0x7FFF00

/* Local definitions */
#define SSP_MODE_MASTER                 (true)

#if defined(__GNUC__)
#define SSP_SSEL_DEASSERTION_DELAY()    __asm__ __volatile__ (  ".rept  2   \n\t" \
                                                                "nop \n\t" \
                                                                ".endr      \n\t" )  // Check this delay for frequencies higher than 48MHz (actual value 130nS)
#elif defined(__arm)
#define SSP_SSEL_DEASSERTION_DELAY()    __asm   {   NOP; \
                                                    NOP; }
#else
#define SSP_SSEL_DEASSERTION_DELAY()
#endif

#define FLASH_MEM_SECTOR_ADDRESS_THIRD_BUTE(address)  ((uint8_t) ((address & FLASH_MEM_SECTOR_ADDRESS_MASK) >> 16))
#define FLASH_MEM_SECTOR_ADDRESS_SECOND_BUTE(address) ((uint8_t) ((address & FLASH_MEM_SECTOR_ADDRESS_MASK) >> 8))
#define FLASH_MEM_SECTOR_ADDRESS_FIRST_BUTE(address)  ((uint8_t) ((address & FLASH_MEM_SECTOR_ADDRESS_MASK) >> 0))

#define FLASH_MEM_PAGE_ADDRESS_THIRD_BUTE(address)  ((uint8_t) ((address & FLASH_MEM_PAGE_ADDRESS_MASK) >> 16))
#define FLASH_MEM_PAGE_ADDRESS_SECOND_BUTE(address) ((uint8_t) ((address & FLASH_MEM_PAGE_ADDRESS_MASK) >> 8))
#define FLASH_MEM_PAGE_ADDRESS_FIRST_BUTE(address)  ((uint8_t) ((address & FLASH_MEM_PAGE_ADDRESS_MASK) >> 0))

#define SSP_STATUS_BUSY_BIT_MASK        (0x01)

uint8_t flashMemWriteBuffer[FLASH_MEM_SECTOR_SIZE];

static inline void flashMemSetWp(bool isHigh)
{
    Chip_GPIO_SetPinState(LPC_GPIO, FLASH_MEM_WP_PIO_NUM, FLASH_MEM_WP_PIN_NUM, isHigh);
}


static inline void flashMemSetSsel(bool isHigh)
{
    Chip_GPIO_SetPinState(LPC_GPIO, FLASH_MEM_SSEL_PIO_NUM, FLASH_MEM_SSEL_PIN_NUM, isHigh);
}

uint8_t flashMemReadStatusReg0()
{
#define DUMMY_SPI_VALUE 0x00

    uint8_t receivedData;
    flashMemSetSsel(false);
    SSP_SSEL_DEASSERTION_DELAY();

    Chip_SSP_SendFrame(FLASH_MEM_SSP_HW_UNIT, FLASH_MEM_READ_STATUS_REG_CMD);
    while (Chip_SSP_GetStatus(FLASH_MEM_SSP_HW_UNIT, SSP_STAT_BSY));

    receivedData = (uint8_t) Chip_SSP_ReceiveFrame(FLASH_MEM_SSP_HW_UNIT);

    Chip_SSP_SendFrame(FLASH_MEM_SSP_HW_UNIT, DUMMY_SPI_VALUE);
    while (Chip_SSP_GetStatus(FLASH_MEM_SSP_HW_UNIT, SSP_STAT_BSY));

    receivedData = (uint8_t) Chip_SSP_ReceiveFrame(FLASH_MEM_SSP_HW_UNIT);

    SSP_SSEL_DEASSERTION_DELAY();
    flashMemSetSsel(true);

    return receivedData;
#undef DUMMY_SPI_VALUE
}


uint8_t flashMemGetTransactionStatus()
{
    return flashMemReadStatusReg0() & SSP_STATUS_BUSY_BIT_MASK;
}

static const PINMUX_GRP_T flashMemPinMux[] = {
    {FLASH_MEM_SSEL_PIO_NUM, FLASH_MEM_SSEL_PIN_NUM,  (IOCON_FUNC0 | IOCON_MODE_INACT | IOCON_DIGMODE_EN)},
    {FLASH_MEM_SCK_PIO_NUM, FLASH_MEM_SCK_PIN_NUM,  (IOCON_FUNC2 | IOCON_MODE_INACT | IOCON_DIGMODE_EN)},
    {FLASH_MEM_MISO_PIO_NUM, FLASH_MEM_MISO_PIN_NUM,  (IOCON_FUNC1 | IOCON_MODE_INACT | IOCON_DIGMODE_EN)},
    {FLASH_MEM_MOSI_PIO_NUM, FLASH_MEM_MOSI_PIN_NUM,  (IOCON_FUNC1 | IOCON_MODE_INACT | IOCON_DIGMODE_EN)},
    {FLASH_MEM_WP_PIO_NUM, FLASH_MEM_WP_PIN_NUM, (IOCON_FUNC0 | IOCON_MODE_INACT | IOCON_DIGMODE_EN)},
};

void flashMemInit()
{
    SSP_ConfigFormat sspFormatConfigStruct;

    Chip_IOCON_SetPinMuxing(LPC_IOCON, flashMemPinMux, sizeof(flashMemPinMux) / sizeof(PINMUX_GRP_T));

    Chip_SSP_Init(FLASH_MEM_SSP_HW_UNIT);

    Chip_GPIO_SetPinDIROutput(LPC_GPIO, FLASH_MEM_WP_PIO_NUM, FLASH_MEM_WP_PIN_NUM);
    Chip_GPIO_SetPinDIROutput(LPC_GPIO, FLASH_MEM_SSEL_PIO_NUM, FLASH_MEM_SSEL_PIN_NUM);
    flashMemSetSsel(true);
    flashMemSetWp(false);

    sspFormatConfigStruct.frameFormat = SSP_FRAMEFORMAT_SPI;
    sspFormatConfigStruct.bits = SSP_BITS_8;
    sspFormatConfigStruct.clockMode = SSP_CLOCK_MODE0;
    Chip_SSP_SetBitRate(FLASH_MEM_SSP_HW_UNIT, FLASH_MEM_SSP_BIT_RATE);
    Chip_SSP_SetFormat(FLASH_MEM_SSP_HW_UNIT, sspFormatConfigStruct.bits, sspFormatConfigStruct.frameFormat, sspFormatConfigStruct.clockMode);
    Chip_SSP_SetMaster(FLASH_MEM_SSP_HW_UNIT, SSP_MODE_MASTER);
    Chip_SSP_Enable(FLASH_MEM_SSP_HW_UNIT);


#if (USE_FLASH_TRANSFER_IRQ)
    NVIC_EnableIRQ(FLASH_MEM_SSP_IRQN);
#endif

}


void flashMemReadIds(uint8_t id[3])
{
#define DUMMY_SPI_VALUE 0x00

    while (flashMemGetTransactionStatus());
    flashMemSetSsel(false);
    SSP_SSEL_DEASSERTION_DELAY();

    Chip_SSP_SendFrame(FLASH_MEM_SSP_HW_UNIT, FLASH_MEM_READ_IDS_CMD);
    while (Chip_SSP_GetStatus(FLASH_MEM_SSP_HW_UNIT, SSP_STAT_BSY));

    id[0] = (uint8_t) Chip_SSP_ReceiveFrame(FLASH_MEM_SSP_HW_UNIT);

    Chip_SSP_SendFrame(FLASH_MEM_SSP_HW_UNIT, DUMMY_SPI_VALUE);
    while (Chip_SSP_GetStatus(FLASH_MEM_SSP_HW_UNIT, SSP_STAT_BSY));

    id[0] = (uint8_t) Chip_SSP_ReceiveFrame(FLASH_MEM_SSP_HW_UNIT);

    Chip_SSP_SendFrame(FLASH_MEM_SSP_HW_UNIT, DUMMY_SPI_VALUE);
    while (Chip_SSP_GetStatus(FLASH_MEM_SSP_HW_UNIT, SSP_STAT_BSY));

    id[1] = (uint8_t) Chip_SSP_ReceiveFrame(FLASH_MEM_SSP_HW_UNIT);

    Chip_SSP_SendFrame(FLASH_MEM_SSP_HW_UNIT, DUMMY_SPI_VALUE);
    while (Chip_SSP_GetStatus(FLASH_MEM_SSP_HW_UNIT, SSP_STAT_BSY));

    id[2] = (uint8_t) Chip_SSP_ReceiveFrame(FLASH_MEM_SSP_HW_UNIT);

    SSP_SSEL_DEASSERTION_DELAY();
    flashMemSetSsel(true);

#undef DUMMY_SPI_VALUE
}


void flashMemReadPage(uint32_t pageAddress, uint8_t *dataBuffer)
{
#define DUMMY_SPI_VALUE 0x00

    uint8_t receivedData;
    uint8_t *endAddress = dataBuffer + FLASH_MEM_PAGE_SIZE;
    while (flashMemGetTransactionStatus());
    flashMemSetSsel(false);

    Chip_SSP_SendFrame(FLASH_MEM_SSP_HW_UNIT, FLASH_MEM_READ_PAGE_CMD);
    while (Chip_SSP_GetStatus(FLASH_MEM_SSP_HW_UNIT, SSP_STAT_BSY));
    receivedData = (uint8_t) Chip_SSP_ReceiveFrame(FLASH_MEM_SSP_HW_UNIT);

    Chip_SSP_SendFrame(FLASH_MEM_SSP_HW_UNIT, FLASH_MEM_PAGE_ADDRESS_THIRD_BUTE(pageAddress));
    while (Chip_SSP_GetStatus(FLASH_MEM_SSP_HW_UNIT, SSP_STAT_BSY));
    receivedData = (uint8_t) Chip_SSP_ReceiveFrame(FLASH_MEM_SSP_HW_UNIT);

    Chip_SSP_SendFrame(FLASH_MEM_SSP_HW_UNIT, FLASH_MEM_PAGE_ADDRESS_SECOND_BUTE(pageAddress));
    while (Chip_SSP_GetStatus(FLASH_MEM_SSP_HW_UNIT, SSP_STAT_BSY));
    receivedData = (uint8_t) Chip_SSP_ReceiveFrame(FLASH_MEM_SSP_HW_UNIT);

    Chip_SSP_SendFrame(FLASH_MEM_SSP_HW_UNIT, FLASH_MEM_PAGE_ADDRESS_FIRST_BUTE(pageAddress));
    while (Chip_SSP_GetStatus(FLASH_MEM_SSP_HW_UNIT, SSP_STAT_BSY));
    receivedData = (uint8_t) Chip_SSP_ReceiveFrame(FLASH_MEM_SSP_HW_UNIT);

    for (; dataBuffer < endAddress; ) {
        Chip_SSP_SendFrame(FLASH_MEM_SSP_HW_UNIT, DUMMY_SPI_VALUE);
        while (Chip_SSP_GetStatus(FLASH_MEM_SSP_HW_UNIT, SSP_STAT_BSY));
        *(dataBuffer++) = (uint8_t) Chip_SSP_ReceiveFrame(FLASH_MEM_SSP_HW_UNIT);
    }

    (void) receivedData;
    SSP_SSEL_DEASSERTION_DELAY();
    flashMemSetSsel(true);
#undef DUMMY_SPI_VALUE
}


void flashMemWritePage(uint32_t pageAddress, const uint8_t *dataBuffer)
{
    uint8_t receivedData;
    const uint8_t *endAddress = dataBuffer + FLASH_MEM_PAGE_SIZE;

    while (flashMemGetTransactionStatus());

    flashMemSetWp(true);
    flashMemSetSsel(false);

    Chip_SSP_SendFrame(FLASH_MEM_SSP_HW_UNIT, FLASH_MEM_WRITE_ENABLE_CMD);
    while (Chip_SSP_GetStatus(FLASH_MEM_SSP_HW_UNIT, SSP_STAT_BSY));
    receivedData = (uint8_t) Chip_SSP_ReceiveFrame(FLASH_MEM_SSP_HW_UNIT);

    SSP_SSEL_DEASSERTION_DELAY();
    flashMemSetSsel(true);
    SSP_SSEL_DEASSERTION_DELAY();
    flashMemSetSsel(false);

    Chip_SSP_SendFrame(FLASH_MEM_SSP_HW_UNIT, FLASH_MEM_WRITE_PAGE_CMD);
    while (Chip_SSP_GetStatus(FLASH_MEM_SSP_HW_UNIT, SSP_STAT_BSY));
    receivedData = (uint8_t) Chip_SSP_ReceiveFrame(FLASH_MEM_SSP_HW_UNIT);

    Chip_SSP_SendFrame(FLASH_MEM_SSP_HW_UNIT, FLASH_MEM_PAGE_ADDRESS_THIRD_BUTE(pageAddress));
    while (Chip_SSP_GetStatus(FLASH_MEM_SSP_HW_UNIT, SSP_STAT_BSY));
    receivedData = (uint8_t) Chip_SSP_ReceiveFrame(FLASH_MEM_SSP_HW_UNIT);

    Chip_SSP_SendFrame(FLASH_MEM_SSP_HW_UNIT, FLASH_MEM_PAGE_ADDRESS_SECOND_BUTE(pageAddress));
    while (Chip_SSP_GetStatus(FLASH_MEM_SSP_HW_UNIT, SSP_STAT_BSY));
    receivedData = (uint8_t) Chip_SSP_ReceiveFrame(FLASH_MEM_SSP_HW_UNIT);

    Chip_SSP_SendFrame(FLASH_MEM_SSP_HW_UNIT, FLASH_MEM_PAGE_ADDRESS_FIRST_BUTE(pageAddress));
    while (Chip_SSP_GetStatus(FLASH_MEM_SSP_HW_UNIT, SSP_STAT_BSY));
    receivedData = (uint8_t) Chip_SSP_ReceiveFrame(FLASH_MEM_SSP_HW_UNIT);

    for (; dataBuffer < endAddress; dataBuffer++) {
        Chip_SSP_SendFrame(FLASH_MEM_SSP_HW_UNIT, *dataBuffer);
        while (!Chip_SSP_GetStatus(FLASH_MEM_SSP_HW_UNIT, SSP_STAT_TNF));
    }
    while (Chip_SSP_GetStatus(FLASH_MEM_SSP_HW_UNIT, SSP_STAT_BSY));

    while (Chip_SSP_GetStatus(FLASH_MEM_SSP_HW_UNIT, SSP_STAT_RNE)) {
        receivedData = (uint8_t) Chip_SSP_ReceiveFrame(FLASH_MEM_SSP_HW_UNIT);
    }

    (void) receivedData;

    SSP_SSEL_DEASSERTION_DELAY();
    flashMemSetSsel(true);
    SSP_SSEL_DEASSERTION_DELAY();

    flashMemSetWp(false);
}


void flashMemEraseSector(uint32_t sectorAddress)
{
    uint8_t receivedData;

    while (flashMemGetTransactionStatus());

    flashMemSetWp(true);
    flashMemSetSsel(false);

    Chip_SSP_SendFrame(FLASH_MEM_SSP_HW_UNIT, FLASH_MEM_WRITE_ENABLE_CMD);
    while (Chip_SSP_GetStatus(FLASH_MEM_SSP_HW_UNIT, SSP_STAT_BSY));
    receivedData = (uint8_t) Chip_SSP_ReceiveFrame(FLASH_MEM_SSP_HW_UNIT);

    SSP_SSEL_DEASSERTION_DELAY();
    flashMemSetSsel(true);
    SSP_SSEL_DEASSERTION_DELAY();
    flashMemSetSsel(false);

    Chip_SSP_SendFrame(FLASH_MEM_SSP_HW_UNIT, FLASH_MEM_ERASE_SECTOR_CMD);
    while (Chip_SSP_GetStatus(FLASH_MEM_SSP_HW_UNIT, SSP_STAT_BSY));
    receivedData = (uint8_t) Chip_SSP_ReceiveFrame(FLASH_MEM_SSP_HW_UNIT);

    Chip_SSP_SendFrame(FLASH_MEM_SSP_HW_UNIT, FLASH_MEM_SECTOR_ADDRESS_THIRD_BUTE(sectorAddress));
    while (Chip_SSP_GetStatus(FLASH_MEM_SSP_HW_UNIT, SSP_STAT_BSY));
    receivedData = (uint8_t) Chip_SSP_ReceiveFrame(FLASH_MEM_SSP_HW_UNIT);

    Chip_SSP_SendFrame(FLASH_MEM_SSP_HW_UNIT, FLASH_MEM_SECTOR_ADDRESS_SECOND_BUTE(sectorAddress));
    while (Chip_SSP_GetStatus(FLASH_MEM_SSP_HW_UNIT, SSP_STAT_BSY));
    receivedData = (uint8_t) Chip_SSP_ReceiveFrame(FLASH_MEM_SSP_HW_UNIT);

    Chip_SSP_SendFrame(FLASH_MEM_SSP_HW_UNIT, FLASH_MEM_SECTOR_ADDRESS_FIRST_BUTE(sectorAddress));
    while (Chip_SSP_GetStatus(FLASH_MEM_SSP_HW_UNIT, SSP_STAT_BSY));
    receivedData = (uint8_t) Chip_SSP_ReceiveFrame(FLASH_MEM_SSP_HW_UNIT);

    SSP_SSEL_DEASSERTION_DELAY();
    flashMemSetSsel(true);
    SSP_SSEL_DEASSERTION_DELAY();

    (void) receivedData;
    flashMemSetWp(false);
}

#if (USE_FLASH_TRANSFER_IRQ)
void FLASH_MEM_SSP_IRQ_HANDLER()
{

}
#endif


static const uint32_t pageAdressToTest[] = {
    0 * FLASH_MEM_PAGE_SIZE, 1 * FLASH_MEM_PAGE_SIZE, 2 * FLASH_MEM_PAGE_SIZE, 3 * FLASH_MEM_PAGE_SIZE,
    4 * FLASH_MEM_PAGE_SIZE, 5 * FLASH_MEM_PAGE_SIZE, 6 * FLASH_MEM_PAGE_SIZE, 7 * FLASH_MEM_PAGE_SIZE,

    8 * FLASH_MEM_PAGE_SIZE, 9 * FLASH_MEM_PAGE_SIZE, 10 * FLASH_MEM_PAGE_SIZE, 11 * FLASH_MEM_PAGE_SIZE,
    12 * FLASH_MEM_PAGE_SIZE, 13 * FLASH_MEM_PAGE_SIZE, 14 * FLASH_MEM_PAGE_SIZE, 15 * FLASH_MEM_PAGE_SIZE,

    16 * FLASH_MEM_PAGE_SIZE, 17 * FLASH_MEM_PAGE_SIZE, 18 * FLASH_MEM_PAGE_SIZE, 19 * FLASH_MEM_PAGE_SIZE,
    20 * FLASH_MEM_PAGE_SIZE, 21 * FLASH_MEM_PAGE_SIZE, 22 * FLASH_MEM_PAGE_SIZE, 23 * FLASH_MEM_PAGE_SIZE,

    24 * FLASH_MEM_PAGE_SIZE, 25 * FLASH_MEM_PAGE_SIZE, 26 * FLASH_MEM_PAGE_SIZE, 27 * FLASH_MEM_PAGE_SIZE,
    28 * FLASH_MEM_PAGE_SIZE, 29 * FLASH_MEM_PAGE_SIZE, 30 * FLASH_MEM_PAGE_SIZE, 31 * FLASH_MEM_PAGE_SIZE,

    32 * FLASH_MEM_PAGE_SIZE, 33 * FLASH_MEM_PAGE_SIZE, 34 * FLASH_MEM_PAGE_SIZE, 35 * FLASH_MEM_PAGE_SIZE,
    36 * FLASH_MEM_PAGE_SIZE, 37 * FLASH_MEM_PAGE_SIZE, 38 * FLASH_MEM_PAGE_SIZE, 39 * FLASH_MEM_PAGE_SIZE,

    40 * FLASH_MEM_PAGE_SIZE, 41 * FLASH_MEM_PAGE_SIZE, 42 * FLASH_MEM_PAGE_SIZE, 43 * FLASH_MEM_PAGE_SIZE,
    44 * FLASH_MEM_PAGE_SIZE, 45 * FLASH_MEM_PAGE_SIZE, 46 * FLASH_MEM_PAGE_SIZE, 47 * FLASH_MEM_PAGE_SIZE,

    48 * FLASH_MEM_PAGE_SIZE, 49 * FLASH_MEM_PAGE_SIZE, 50 * FLASH_MEM_PAGE_SIZE, 51 * FLASH_MEM_PAGE_SIZE,
    52 * FLASH_MEM_PAGE_SIZE, 53 * FLASH_MEM_PAGE_SIZE, 54 * FLASH_MEM_PAGE_SIZE, 55 * FLASH_MEM_PAGE_SIZE,

    56 * FLASH_MEM_PAGE_SIZE, 57 * FLASH_MEM_PAGE_SIZE, 58 * FLASH_MEM_PAGE_SIZE, 59 * FLASH_MEM_PAGE_SIZE,
    60 * FLASH_MEM_PAGE_SIZE, 61 * FLASH_MEM_PAGE_SIZE, 62 * FLASH_MEM_PAGE_SIZE, 63 * FLASH_MEM_PAGE_SIZE,

    ((FLASH_MEM_PAGE_CCOUNT / 2) - 16) * FLASH_MEM_PAGE_SIZE, ((FLASH_MEM_PAGE_CCOUNT / 2) - 15) * FLASH_MEM_PAGE_SIZE,
    ((FLASH_MEM_PAGE_CCOUNT / 2) - 14) * FLASH_MEM_PAGE_SIZE, ((FLASH_MEM_PAGE_CCOUNT / 2) - 13) * FLASH_MEM_PAGE_SIZE,
    ((FLASH_MEM_PAGE_CCOUNT / 2) - 12) * FLASH_MEM_PAGE_SIZE, ((FLASH_MEM_PAGE_CCOUNT / 2) - 11) * FLASH_MEM_PAGE_SIZE,
    ((FLASH_MEM_PAGE_CCOUNT / 2) - 10) * FLASH_MEM_PAGE_SIZE, ((FLASH_MEM_PAGE_CCOUNT / 2) - 9) * FLASH_MEM_PAGE_SIZE,

    ((FLASH_MEM_PAGE_CCOUNT / 2) - 8) * FLASH_MEM_PAGE_SIZE, ((FLASH_MEM_PAGE_CCOUNT / 2) - 7) * FLASH_MEM_PAGE_SIZE,
    ((FLASH_MEM_PAGE_CCOUNT / 2) - 6) * FLASH_MEM_PAGE_SIZE, ((FLASH_MEM_PAGE_CCOUNT / 2) - 5) * FLASH_MEM_PAGE_SIZE,
    ((FLASH_MEM_PAGE_CCOUNT / 2) - 4) * FLASH_MEM_PAGE_SIZE, ((FLASH_MEM_PAGE_CCOUNT / 2) - 3) * FLASH_MEM_PAGE_SIZE,
    ((FLASH_MEM_PAGE_CCOUNT / 2) - 2) * FLASH_MEM_PAGE_SIZE, ((FLASH_MEM_PAGE_CCOUNT / 2) - 1) * FLASH_MEM_PAGE_SIZE,

    (FLASH_MEM_PAGE_CCOUNT - 16) * FLASH_MEM_PAGE_SIZE, (FLASH_MEM_PAGE_CCOUNT - 15) * FLASH_MEM_PAGE_SIZE,
    (FLASH_MEM_PAGE_CCOUNT - 14) * FLASH_MEM_PAGE_SIZE, (FLASH_MEM_PAGE_CCOUNT - 13) * FLASH_MEM_PAGE_SIZE,
    (FLASH_MEM_PAGE_CCOUNT - 12) * FLASH_MEM_PAGE_SIZE, (FLASH_MEM_PAGE_CCOUNT - 11) * FLASH_MEM_PAGE_SIZE,
    (FLASH_MEM_PAGE_CCOUNT - 10) * FLASH_MEM_PAGE_SIZE, (FLASH_MEM_PAGE_CCOUNT - 9) * FLASH_MEM_PAGE_SIZE,

    (FLASH_MEM_PAGE_CCOUNT - 8) * FLASH_MEM_PAGE_SIZE, (FLASH_MEM_PAGE_CCOUNT - 7) * FLASH_MEM_PAGE_SIZE,
    (FLASH_MEM_PAGE_CCOUNT - 6) * FLASH_MEM_PAGE_SIZE, (FLASH_MEM_PAGE_CCOUNT - 5) * FLASH_MEM_PAGE_SIZE,
    (FLASH_MEM_PAGE_CCOUNT - 4) * FLASH_MEM_PAGE_SIZE, (FLASH_MEM_PAGE_CCOUNT - 3) * FLASH_MEM_PAGE_SIZE,
    (FLASH_MEM_PAGE_CCOUNT - 2) * FLASH_MEM_PAGE_SIZE, (FLASH_MEM_PAGE_CCOUNT - 1) * FLASH_MEM_PAGE_SIZE,
};


#define TEST_PAGE_COUNT sizeof(pageAdressToTest) / sizeof(pageAdressToTest[0])

bool flashMemTest()
{
#define DUMMY_SPI_VALUE 0x00

    uint32_t pageCounter, byteCounter;
    uint8_t receivedData;
    bool result = true;
    for (pageCounter = 0; pageCounter < TEST_PAGE_COUNT; ++pageCounter) {

        if (!(pageAdressToTest[pageCounter] % FLASH_MEM_SECTOR_SIZE)) {
            flashMemEraseSector(pageAdressToTest[pageCounter]);
        }

        while (flashMemGetTransactionStatus());

        flashMemSetWp(true);
        flashMemSetSsel(false);

        Chip_SSP_SendFrame(FLASH_MEM_SSP_HW_UNIT, FLASH_MEM_WRITE_ENABLE_CMD);
        while (Chip_SSP_GetStatus(FLASH_MEM_SSP_HW_UNIT, SSP_STAT_BSY));
        receivedData = (uint8_t) Chip_SSP_ReceiveFrame(FLASH_MEM_SSP_HW_UNIT);

        SSP_SSEL_DEASSERTION_DELAY();
        flashMemSetSsel(true);
        SSP_SSEL_DEASSERTION_DELAY();
        flashMemSetSsel(false);

        Chip_SSP_SendFrame(FLASH_MEM_SSP_HW_UNIT, FLASH_MEM_WRITE_PAGE_CMD);
        while (Chip_SSP_GetStatus(FLASH_MEM_SSP_HW_UNIT, SSP_STAT_BSY));
        receivedData = (uint8_t) Chip_SSP_ReceiveFrame(FLASH_MEM_SSP_HW_UNIT);

        Chip_SSP_SendFrame(FLASH_MEM_SSP_HW_UNIT, FLASH_MEM_PAGE_ADDRESS_THIRD_BUTE(pageAdressToTest[pageCounter]));
        while (Chip_SSP_GetStatus(FLASH_MEM_SSP_HW_UNIT, SSP_STAT_BSY));
        receivedData = (uint8_t) Chip_SSP_ReceiveFrame(FLASH_MEM_SSP_HW_UNIT);

        Chip_SSP_SendFrame(FLASH_MEM_SSP_HW_UNIT, FLASH_MEM_PAGE_ADDRESS_SECOND_BUTE(pageAdressToTest[pageCounter]));
        while (Chip_SSP_GetStatus(FLASH_MEM_SSP_HW_UNIT, SSP_STAT_BSY));
        receivedData = (uint8_t) Chip_SSP_ReceiveFrame(FLASH_MEM_SSP_HW_UNIT);

        Chip_SSP_SendFrame(FLASH_MEM_SSP_HW_UNIT, FLASH_MEM_PAGE_ADDRESS_FIRST_BUTE(pageAdressToTest[pageCounter]));
        while (Chip_SSP_GetStatus(FLASH_MEM_SSP_HW_UNIT, SSP_STAT_BSY));
        receivedData = (uint8_t) Chip_SSP_ReceiveFrame(FLASH_MEM_SSP_HW_UNIT);

        for (byteCounter = 0; byteCounter < FLASH_MEM_PAGE_SIZE; byteCounter++) {
            Chip_SSP_SendFrame(FLASH_MEM_SSP_HW_UNIT, byteCounter);
            while (!Chip_SSP_GetStatus(FLASH_MEM_SSP_HW_UNIT, SSP_STAT_TNF));
        }
        while (Chip_SSP_GetStatus(FLASH_MEM_SSP_HW_UNIT, SSP_STAT_BSY));

        while (Chip_SSP_GetStatus(FLASH_MEM_SSP_HW_UNIT, SSP_STAT_RNE)) {
            receivedData = (uint8_t) Chip_SSP_ReceiveFrame(FLASH_MEM_SSP_HW_UNIT);
        }

        SSP_SSEL_DEASSERTION_DELAY();
        flashMemSetSsel(true);
        SSP_SSEL_DEASSERTION_DELAY();

        flashMemSetWp(false);

        while (flashMemGetTransactionStatus());
        flashMemSetSsel(false);

        Chip_SSP_SendFrame(FLASH_MEM_SSP_HW_UNIT, FLASH_MEM_READ_PAGE_CMD);
        while (Chip_SSP_GetStatus(FLASH_MEM_SSP_HW_UNIT, SSP_STAT_BSY));
        receivedData = (uint8_t) Chip_SSP_ReceiveFrame(FLASH_MEM_SSP_HW_UNIT);

        Chip_SSP_SendFrame(FLASH_MEM_SSP_HW_UNIT, FLASH_MEM_PAGE_ADDRESS_THIRD_BUTE(pageAdressToTest[pageCounter]));
        while (Chip_SSP_GetStatus(FLASH_MEM_SSP_HW_UNIT, SSP_STAT_BSY));
        receivedData = (uint8_t) Chip_SSP_ReceiveFrame(FLASH_MEM_SSP_HW_UNIT);

        Chip_SSP_SendFrame(FLASH_MEM_SSP_HW_UNIT, FLASH_MEM_PAGE_ADDRESS_SECOND_BUTE(pageAdressToTest[pageCounter]));
        while (Chip_SSP_GetStatus(FLASH_MEM_SSP_HW_UNIT, SSP_STAT_BSY));
        receivedData = (uint8_t) Chip_SSP_ReceiveFrame(FLASH_MEM_SSP_HW_UNIT);

        Chip_SSP_SendFrame(FLASH_MEM_SSP_HW_UNIT, FLASH_MEM_PAGE_ADDRESS_FIRST_BUTE(pageAdressToTest[pageCounter]));
        while (Chip_SSP_GetStatus(FLASH_MEM_SSP_HW_UNIT, SSP_STAT_BSY));
        receivedData = (uint8_t) Chip_SSP_ReceiveFrame(FLASH_MEM_SSP_HW_UNIT);

        for (byteCounter = 0; byteCounter < FLASH_MEM_PAGE_SIZE; byteCounter++) {
            Chip_SSP_SendFrame(FLASH_MEM_SSP_HW_UNIT, DUMMY_SPI_VALUE);
            while (Chip_SSP_GetStatus(FLASH_MEM_SSP_HW_UNIT, SSP_STAT_BSY));
            if ((uint8_t) Chip_SSP_ReceiveFrame(FLASH_MEM_SSP_HW_UNIT) != byteCounter) {
                result = false;
            }
        }

        (void) receivedData;
        SSP_SSEL_DEASSERTION_DELAY();
        flashMemSetSsel(true);

#undef DUMMY_SPI_VALUE
    }
    return result;
}


void flashMemGetUniqueId(uint8_t id[8])
{
#define UID_ADDRESS_BYTE_0          0xF8
#define UID_ADDRESS_BYTE_1_2_DUMMY  0x00
#define UID_SIZE                    0x08
    uint8_t receivedData;
    uint8_t byteCnt;

    if (!id) {
        return;
    }

    while (flashMemGetTransactionStatus());
    flashMemSetSsel(false);

    Chip_SSP_SendFrame(FLASH_MEM_SSP_HW_UNIT, FLASH_MEM_READ_SPDF_UID_CMD);
    while (Chip_SSP_GetStatus(FLASH_MEM_SSP_HW_UNIT, SSP_STAT_BSY));
    receivedData = (uint8_t) Chip_SSP_ReceiveFrame(FLASH_MEM_SSP_HW_UNIT);

    Chip_SSP_SendFrame(FLASH_MEM_SSP_HW_UNIT, UID_ADDRESS_BYTE_1_2_DUMMY);
    while (Chip_SSP_GetStatus(FLASH_MEM_SSP_HW_UNIT, SSP_STAT_BSY));
    receivedData = (uint8_t) Chip_SSP_ReceiveFrame(FLASH_MEM_SSP_HW_UNIT);

    Chip_SSP_SendFrame(FLASH_MEM_SSP_HW_UNIT, UID_ADDRESS_BYTE_1_2_DUMMY);
    while (Chip_SSP_GetStatus(FLASH_MEM_SSP_HW_UNIT, SSP_STAT_BSY));
    receivedData = (uint8_t) Chip_SSP_ReceiveFrame(FLASH_MEM_SSP_HW_UNIT);

    Chip_SSP_SendFrame(FLASH_MEM_SSP_HW_UNIT, UID_ADDRESS_BYTE_0);
    while (Chip_SSP_GetStatus(FLASH_MEM_SSP_HW_UNIT, SSP_STAT_BSY));
    receivedData = (uint8_t) Chip_SSP_ReceiveFrame(FLASH_MEM_SSP_HW_UNIT);

    Chip_SSP_SendFrame(FLASH_MEM_SSP_HW_UNIT, UID_ADDRESS_BYTE_1_2_DUMMY);
    while (Chip_SSP_GetStatus(FLASH_MEM_SSP_HW_UNIT, SSP_STAT_BSY));
    receivedData = (uint8_t) Chip_SSP_ReceiveFrame(FLASH_MEM_SSP_HW_UNIT);

    for (byteCnt = 0; byteCnt < UID_SIZE; byteCnt++) {
        Chip_SSP_SendFrame(FLASH_MEM_SSP_HW_UNIT, UID_ADDRESS_BYTE_1_2_DUMMY);
        while (Chip_SSP_GetStatus(FLASH_MEM_SSP_HW_UNIT, SSP_STAT_BSY));
        id[byteCnt] = (uint8_t) Chip_SSP_ReceiveFrame(FLASH_MEM_SSP_HW_UNIT);
    }

    SSP_SSEL_DEASSERTION_DELAY();
    flashMemSetSsel(true);
    SSP_SSEL_DEASSERTION_DELAY();

    (void) receivedData;

#undef UID_ADDRESS_BYTE_0
#undef UID_ADDRESS_BYTE_1_2_DUMMY
#undef UID_SIZE
}
