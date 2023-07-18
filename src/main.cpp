#include <CH58x_common.h>
#include <CH58xBLE_LIB.h>
#include <stdio.h>
#include "usbd_core.h"
#include <SPIdev.h>

/* code hardcoded for GPIO bank A at the moment */
#define BLINKY_GPIO_PIN  GPIO_Pin_8

extern "C" void cdc_acm_init(void);

#include "sensor/bmi160sensor.h"
BMI160Sensor sensor(0);


FusionReport rpt;

int main(void)
{
    SetSysClock(CLK_SOURCE_PLL_60MHz);

    GPIOA_SetBits(BLINKY_GPIO_PIN);
    GPIOA_ModeCfg(BLINKY_GPIO_PIN,
        GPIO_ModeOut_PP_20mA
    );

    cdc_acm_init();

    // Wait until configured
    while (!usb_device_is_configured()) {
    }

    SPIdev::initialize();

    DelayMs(4000);
    PRINT("CherryUSB device cdc acm example\n");
    PRINT("Start @ChipID=%02X\n", R8_CHIP_ID);

    sensor.motionSetup();

    // Everything is interrupt driven so just loop here
    while (1) {
        // PRINT("Hello world!\n");
        DelayMs(10);
        GPIOA_InverseBits(BLINKY_GPIO_PIN);
        extern volatile uint8_t dtr_enable;
        if (dtr_enable) {
            sensor.motionLoop();
            sensor.fusion.getResult(&rpt);
            printf("Fr: %6d %6d %6d %6d %6d %6d %6d\n", 
                        rpt.q[0], rpt.q[1], rpt.q[2], rpt.q[3], 
                        rpt.a[0], rpt.a[1], rpt.a[2]
                    );
        }
    }
    return 0;
}
