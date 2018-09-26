#include "HalCommon.h"

#include "clock_config.h"
#include "fsl_iocon.h"
#include "fsl_gpio.h"
#include "fsl_inputmux.h"

void halCommonInit(void)
{
    BOARD_BootClockFROHF96M();
    CLOCK_EnableClock(kCLOCK_Iocon);
    INPUTMUX_Init(INPUTMUX);
    CLOCK_EnableClock(kCLOCK_Gpio0);
    CLOCK_EnableClock(kCLOCK_Gpio1);
    CLOCK_EnableClock(kCLOCK_Gpio2);
    CLOCK_EnableClock(kCLOCK_Gpio3);
    CLOCK_EnableClock(kCLOCK_Gpio4);
    CLOCK_EnableClock(kCLOCK_Gpio5);

    SystemCoreClockUpdate();
}
