#include <CH58x_common.h>
#include <CH58xBLE_LIB.h>

/* code hardcoded for GPIO bank B at the moment */
#define BLINKY_GPIO_PIN  GPIO_Pin_4

/*
int main(void)
{
    SetSysClock(CLK_SOURCE_PLL_60MHz);

    GPIOB_SetBits(BLINKY_GPIO_PIN);
    GPIOB_ModeCfg(BLINKY_GPIO_PIN,
        GPIO_ModeOut_PP_20mA
    );
    while(1) {
        DelayMs(1000);
        GPIOB_InverseBits(BLINKY_GPIO_PIN);
    }
}
*/