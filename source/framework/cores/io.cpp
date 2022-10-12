#include "io.h"
#include "board_gpio.h"


void gpio_set_mode(uint8 pin, WiringPinMode mode)
{
    stc_port_init_t stcPortInit;

    MEM_ZERO_STRUCT(stcPortInit);

    switch(mode) {

        case OUTPUT:
            stcPortInit.enPinMode = Pin_Mode_Out;
            stcPortInit.enPullUp  = Disable;
        break;

        case INPUT:
        case INPUT_PULLDOWN:
            stcPortInit.enPinMode = Pin_Mode_In;
            stcPortInit.enPullUp  = Disable;
        break;

        case INPUT_PULLUP:
            stcPortInit.enPinMode = Pin_Mode_In;
            stcPortInit.enPullUp  = Enable;
        break;

        default:
        break;
    }

    PORT_InitMapp(pin, &stcPortInit);
}

void pinMode(uint8 pin, WiringPinMode mode)
{
    gpio_set_mode(pin, mode);
}

