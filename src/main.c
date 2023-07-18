#include <CH58x_common.h>
#include <CH58xBLE_LIB.h>
#include <stdio.h>
#include "usbd_core.h"

/* code hardcoded for GPIO bank A at the moment */
#define BLINKY_GPIO_PIN  GPIO_Pin_8

int main(void)
{
    SetSysClock(CLK_SOURCE_PLL_60MHz);

    GPIOA_SetBits(BLINKY_GPIO_PIN);
    GPIOA_ModeCfg(BLINKY_GPIO_PIN,
        GPIO_ModeOut_PP_20mA
    );

    extern void cdc_acm_init(void);
    cdc_acm_init();

    // Wait until configured
    while (!usb_device_is_configured()) {
    }

    DelayMs(2000);
    printf("CherryUSB device cdc acm example\n");

    // Everything is interrupt driven so just loop here
    while (1) {
        printf("Hello world!\n");
        DelayMs(1000);
        GPIOA_InverseBits(BLINKY_GPIO_PIN);
    }
    return 0;
}
