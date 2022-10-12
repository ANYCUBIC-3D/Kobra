#define _BOARD_GPIO_C_

#include "startup.h"
#include "fastio.h"
#include "board_gpio.h"
#include "bsp_init.h"


extern const cfg_pin_info PIN_MAP[BOARD_NR_GPIO_PINS] = {

    {0,  PortA, Pin00, &adc1,  ADC1_IN0,         Func_Gpio},
    {1,  PortA, Pin01, &adc1,  ADC1_IN1,         Func_Gpio},
    {2,  PortA, Pin02, &adc1,  ADC1_IN2,         Func_Usart2_Tx},
    {3,  PortA, Pin03, &adc1,  ADC1_IN3,         Func_Usart2_Rx},
    {4,  PortA, Pin04, &adc1,  ADC12_IN4,        Func_Gpio},
    {5,  PortA, Pin05, &adc1,  ADC12_IN5,        Func_Gpio},
    {6,  PortA, Pin06, &adc1,  ADC12_IN6,        Func_Gpio},
    {7,  PortA, Pin07, &adc1,  ADC12_IN7,        Func_Gpio},
    {8,  PortA, Pin08, NULL,   ADC_PIN_INVALID,  Func_Gpio},
    {9,  PortA, Pin09, NULL,   ADC_PIN_INVALID,  Func_Usart1_Tx},
    {10, PortA, Pin10, NULL,   ADC_PIN_INVALID,  Func_Sdio},
    {11, PortA, Pin11, NULL,   ADC_PIN_INVALID,  Func_I2c1_Sda},
    {12, PortA, Pin12, NULL,   ADC_PIN_INVALID,  Func_I2c1_Scl},
    {13, PortA, Pin13, NULL,   ADC_PIN_INVALID,  Func_Gpio},
    {14, PortA, Pin14, NULL,   ADC_PIN_INVALID,  Func_Gpio},
    {15, PortA, Pin15, NULL,   ADC_PIN_INVALID,  Func_Usart1_Rx},

    {0,  PortB, Pin00, &adc1,  ADC12_IN8,        Func_Gpio},
    {1,  PortB, Pin01, &adc1,  ADC12_IN9,        Func_Gpio},
    {2,  PortB, Pin02, NULL,   ADC_PIN_INVALID,  Func_Gpio},
    {3,  PortB, Pin03, NULL,   ADC_PIN_INVALID,  Func_Gpio},
    {4,  PortB, Pin04, NULL,   ADC_PIN_INVALID,  Func_Gpio},
    {5,  PortB, Pin05, NULL,   ADC_PIN_INVALID,  Func_Gpio},
    {6,  PortB, Pin06, NULL,   ADC_PIN_INVALID,  Func_Gpio},
    {7,  PortB, Pin07, NULL,   ADC_PIN_INVALID,  Func_Gpio},
    {8,  PortB, Pin08, NULL,   ADC_PIN_INVALID,  Func_Gpio},
    {9,  PortB, Pin09, NULL,   ADC_PIN_INVALID,  Func_Gpio},
    {10, PortB, Pin10, NULL,   ADC_PIN_INVALID,  Func_Usart3_Tx},
    {11, PortB, Pin11, NULL,   ADC_PIN_INVALID,  Func_Gpio},
    {12, PortB, Pin12, NULL,   ADC_PIN_INVALID,  Func_Gpio},
    {13, PortB, Pin13, NULL,   ADC_PIN_INVALID,  Func_Gpio},
    {14, PortB, Pin14, NULL,   ADC_PIN_INVALID,  Func_Gpio},
    {15, PortB, Pin15, NULL,   ADC_PIN_INVALID,  Func_Gpio},

    {0,  PortC, Pin00, &adc1,  ADC12_IN10,       Func_Gpio},
    {1,  PortC, Pin01, &adc1,  ADC12_IN11,       Func_Gpio},
    {2,  PortC, Pin02, &adc1,  ADC1_IN12,        Func_Gpio},
    {3,  PortC, Pin03, &adc1,  ADC1_IN13,        Func_Gpio},
    {4,  PortC, Pin04, &adc1,  ADC1_IN14,        Func_Gpio},
    {5,  PortC, Pin05, &adc1,  ADC1_IN15,        Func_Gpio},
    {6,  PortC, Pin06, NULL,   ADC_PIN_INVALID,  Func_Gpio},
    {7,  PortC, Pin07, NULL,   ADC_PIN_INVALID,  Func_Gpio},
    {8,  PortC, Pin08, NULL,   ADC_PIN_INVALID,  Func_Sdio},
    {9,  PortC, Pin09, NULL,   ADC_PIN_INVALID,  Func_Sdio},
    {10, PortC, Pin10, NULL,   ADC_PIN_INVALID,  Func_Sdio},
    {11, PortC, Pin11, NULL,   ADC_PIN_INVALID,  Func_Sdio},
    {12, PortC, Pin12, NULL,   ADC_PIN_INVALID,  Func_Sdio},
    {13, PortC, Pin13, NULL,   ADC_PIN_INVALID,  Func_Gpio},
    {14, PortC, Pin14, NULL,   ADC_PIN_INVALID,  Func_Gpio},
    {15, PortC, Pin15, NULL,   ADC_PIN_INVALID,  Func_Gpio},

    {0,  PortD, Pin00, NULL,   ADC_PIN_INVALID,  Func_Gpio},
    {1,  PortD, Pin01, NULL,   ADC_PIN_INVALID,  Func_Gpio},
    {2,  PortD, Pin02, NULL,   ADC_PIN_INVALID,  Func_Sdio},
    {3,  PortD, Pin03, NULL,   ADC_PIN_INVALID,  Func_Gpio},
    {4,  PortD, Pin04, NULL,   ADC_PIN_INVALID,  Func_Gpio},
    {5,  PortD, Pin05, NULL,   ADC_PIN_INVALID,  Func_Gpio},
    {6,  PortD, Pin06, NULL,   ADC_PIN_INVALID,  Func_Gpio},
    {7,  PortD, Pin07, NULL,   ADC_PIN_INVALID,  Func_Gpio},
    {8,  PortD, Pin08, NULL,   ADC_PIN_INVALID,  Func_Gpio},
    {9,  PortD, Pin09, NULL,   ADC_PIN_INVALID,  Func_Gpio},
    {10, PortD, Pin10, NULL,   ADC_PIN_INVALID,  Func_Gpio},
    {11, PortD, Pin11, NULL,   ADC_PIN_INVALID,  Func_Gpio},
    {12, PortD, Pin12, NULL,   ADC_PIN_INVALID,  Func_Gpio},
    {13, PortD, Pin13, NULL,   ADC_PIN_INVALID,  Func_Gpio},
    {14, PortD, Pin14, NULL,   ADC_PIN_INVALID,  Func_Gpio},
    {15, PortD, Pin15, NULL,   ADC_PIN_INVALID,  Func_Gpio},

    {0,  PortE, Pin00, NULL,   ADC_PIN_INVALID,  Func_Gpio},
    {1,  PortE, Pin01, NULL,   ADC_PIN_INVALID,  Func_Gpio},
    {2,  PortE, Pin02, NULL,   ADC_PIN_INVALID,  Func_Gpio},
    {3,  PortE, Pin03, NULL,   ADC_PIN_INVALID,  Func_Gpio},
    {4,  PortE, Pin04, NULL,   ADC_PIN_INVALID,  Func_Usart3_Rx},
    {5,  PortE, Pin05, NULL,   ADC_PIN_INVALID,  Func_Usart3_Tx},
    {6,  PortE, Pin06, NULL,   ADC_PIN_INVALID,  Func_Gpio},
    {7,  PortE, Pin07, NULL,   ADC_PIN_INVALID,  Func_Gpio},
    {8,  PortE, Pin08, NULL,   ADC_PIN_INVALID,  Func_Gpio},
    {9,  PortE, Pin09, NULL,   ADC_PIN_INVALID,  Func_Gpio},
    {10, PortE, Pin10, NULL,   ADC_PIN_INVALID,  Func_Gpio},
    {11, PortE, Pin11, NULL,   ADC_PIN_INVALID,  Func_Gpio},
    {12, PortE, Pin12, NULL,   ADC_PIN_INVALID,  Func_Gpio},
    {13, PortE, Pin13, NULL,   ADC_PIN_INVALID,  Func_Gpio},
    {14, PortE, Pin14, NULL,   ADC_PIN_INVALID,  Func_Sdio},
    {15, PortE, Pin15, NULL,   ADC_PIN_INVALID,  Func_Gpio},

    {0,  PortH, Pin00, NULL,   ADC_PIN_INVALID,  Func_Gpio},
    {1,  PortH, Pin01, NULL,   ADC_PIN_INVALID,  Func_Gpio},
    {2,  PortH, Pin02, NULL,   ADC_PIN_INVALID,  Func_Usart3_Rx},
};


/*  Basically everything that is defined having ADC */
extern const uint8_t boardADCPins[BOARD_NR_ADC_PINS] = {
    PA0,PA1,PA2,PA3,PA4,PA5,PA6,PA7,PB0,PB1,PC0,PC1,PC2,PC3,PC4,PC5
};

extern void setup_gpio(void )
{  
    stc_port_init_t stcPortInit;
    MEM_ZERO_STRUCT(stcPortInit);

    PORT_DebugPortSetting(0x1C,Disable);
    
     /*initiallize LED port*/
    stcPortInit.enPinMode = Pin_Mode_Out;
    stcPortInit.enExInt = Disable;
    stcPortInit.enPullUp = Disable;
    /* LED0 and LED1 Port/Pin initialization */
    //PORT_InitMapp(LED, &stcPortInit);
}

HardwareSerial Serial1(USART1_CH);
HardwareSerial Serial2(USART2_CH);
HardwareSerial Serial3(USART3_CH);
HardwareSerial Serial4(USART4_CH);

adc_dev adc1;
struct adc_dev *ADC1;

//DEFINE_HWSERIAL(Serial1, 1);
//DEFINE_HWSERIAL(Serial2, 2);
//DEFINE_HWSERIAL(Serial3, 3);

//HardwareSerial MSerial(LPUART1);
//HardwareSerial MotorUart2(LPUART2);
//HardwareSerial MotorUart8(LPUART8);
//DEFINE_HWSERIAL(Serial4, 4);

#undef _BOARD_GPIO_C_
/************end of file********************/

