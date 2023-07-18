#include "CH58x_common.h"

void usb_dc_low_level_init(void)
{
    extern void USB_IRQHandler(void);
    PFIC_EnableIRQ(USB_IRQn);
    PFIC_EnableFastINT0(USB_IRQn, (uint32_t)(void *)USB_IRQHandler);
}

void usb_hc_low_level_init(void)
{
    PFIC_EnableIRQ(USB2_IRQn);
}
