#ifndef _BOARD_GPIO_H
#define  _BOARD_GPIO_H


#include "hc32_common.h"
#include "board_adc.h"
#include "HardwareSerial.h"


#ifdef __cplusplus
extern "C"{
#endif

typedef struct hdsc_pin_info{
	uint8_t gpio_bit;			 /**< Pin's GPIO port bit. */
	__IO en_port_t gpio_port;
	__IO en_pin_t gpio_pin;
	adc_dev *adc_device;  
	__IO uint8_t adc_channel;
	__IO en_port_func_t FuncSel;
}cfg_pin_info;

extern const cfg_pin_info PIN_MAP[];
extern const uint8_t boardPWMPins[];


#define BOARD_NR_GPIO_PINS      83
#define BOARD_NR_ADC_PINS       16



#define BOARD_NR_SPI            3
#define BOARD_SPI1_NSS_PIN      PA4
#define BOARD_SPI1_SCK_PIN      PA5
#define BOARD_SPI1_MISO_PIN     PA6
#define BOARD_SPI1_MOSI_PIN     PA7

#define BOARD_SPI2_NSS_PIN      PB12
#define BOARD_SPI2_SCK_PIN      PB13
#define BOARD_SPI2_MISO_PIN     PB14
#define BOARD_SPI2_MOSI_PIN     PB15

#define BOARD_SPI3_NSS_PIN      PA15
#define BOARD_SPI3_SCK_PIN      PB3
#define BOARD_SPI3_MISO_PIN     PB4
#define BOARD_SPI3_MOSI_PIN     PB5


/**
 * @brief Feature test: nonzero iff the board has SerialUSB.
 */
 //Roger Clark. Change so that BOARD_HAVE_SERIALUSB is always true, so that it can be controller by -DSERIAL_USB
#define BOARD_HAVE_SERIALUSB 1

/*(defined(BOARD_USB_DISC_DEV) && defined(BOARD_USB_DISC_BIT))*/

//
// Pins Definitions
//
enum {
PA0,PA1,PA2,PA3,PA4,PA5,PA6,PA7,PA8,PA9,PA10,PA11,PA12,PA13,PA14,PA15,
PB0,PB1,PB2,PB3,PB4,PB5,PB6,PB7,PB8,PB9,PB10,PB11,PB12,PB13,PB14,PB15,
PC0,PC1,PC2,PC3,PC4,PC5,PC6,PC7,PC8,PC9,PC10,PC11,PC12,PC13,PC14,PC15,
PD0,PD1,PD2,PD3,PD4,PD5,PD6,PD7,PD8,PD9,PD10,PD11,PD12,PD13,PD14,PD15,
PE0,PE1,PE2,PE3,PE4,PE5,PE6,PE7,PE8,PE9,PE10,PE11,PE12,PE13,PE14,PE15,
PH0,PH1,PH2,
};

//
// Pins Definitions
//
#define PA0  0x00
#define PA1  0x01
#define PA2  0x02
#define PA3  0x03
#define PA4  0x04
#define PA5  0x05
#define PA6  0x06
#define PA7  0x07
#define PA8  0x08
#define PA9  0x09
#define PA10 0x0A
#define PA11 0x0B
#define PA12 0x0C
#define PA13 0x0D
#define PA14 0x0E
#define PA15 0x0F

#define PB0  0x10
#define PB1  0x11
#define PB2  0x12
#define PB3  0x13
#define PB4  0x14
#define PB5  0x15
#define PB6  0x16
#define PB7  0x17 // 36 pins (F103T)
#define PB8  0x18
#define PB9  0x19
#define PB10 0x1A
#define PB11 0x1B
#define PB12 0x1C
#define PB13 0x1D
#define PB14 0x1E
#define PB15 0x1F

#define PC0  0x20
#define PC1  0x21
#define PC2  0x22
#define PC3  0x23
#define PC4  0x24
#define PC5  0x25
#define PC6  0x26
#define PC7  0x27
#define PC8  0x28
#define PC9  0x29
#define PC10 0x2A
#define PC11 0x2B
#define PC12 0x2C
#define PC13 0x2D
#define PC14 0x2E
#define PC15 0x2F

#define PD0  0x30
#define PD1  0x31
#define PD2  0x32 // 64 pins (F103R)
#define PD3  0x33
#define PD4  0x34
#define PD5  0x35
#define PD6  0x36
#define PD7  0x37
#define PD8  0x38
#define PD9  0x39
#define PD10 0x3A
#define PD11 0x3B
#define PD12 0x3C
#define PD13 0x3D
#define PD14 0x3E
#define PD15 0x3F

#define PE0  0x40
#define PE1  0x41
#define PE2  0x42
#define PE3  0x43
#define PE4  0x44
#define PE5  0x45
#define PE6  0x46
#define PE7  0x47
#define PE8  0x48
#define PE9  0x49
#define PE10 0x4A
#define PE11 0x4B
#define PE12 0x4C
#define PE13 0x4D
#define PE14 0x4E
#define PE15 0x4F // 100 pins (F103V)

#define PH0  0x50
#define PH1  0x51
#define PH2  0x52


extern HardwareSerial Serial1;
extern HardwareSerial Serial2;
extern HardwareSerial Serial3;
extern HardwareSerial Serial4;


extern inline en_result_t PORT_SetFuncMapp(uint8_t PinNum,en_functional_state_t enSubFunc){
	if(PinNum>BOARD_NR_GPIO_PINS)return(Error);
	return (PORT_SetFunc(PIN_MAP[PinNum].gpio_port, PIN_MAP[PinNum].gpio_pin, PIN_MAP[PinNum].FuncSel, enSubFunc));
}

extern inline en_result_t PORT_InitMapp(uint8_t PinNum,const stc_port_init_t *pstcPortInit){
	if(PinNum>BOARD_NR_GPIO_PINS)return(Error);
	return (PORT_Init(PIN_MAP[PinNum].gpio_port, PIN_MAP[PinNum].gpio_pin,pstcPortInit));
}

extern inline en_result_t PORT_ToggleMapp(uint8_t PinNum){
	if(PinNum>BOARD_NR_GPIO_PINS)return(Error);
	return (PORT_Toggle(PIN_MAP[PinNum].gpio_port, PIN_MAP[PinNum].gpio_pin));
}

extern inline en_result_t PORT_SetBitsMapp(uint8_t PinNum){
	if(PinNum>BOARD_NR_GPIO_PINS)return(Error);
	return (PORT_SetBits(PIN_MAP[PinNum].gpio_port, PIN_MAP[PinNum].gpio_pin));
}

extern inline en_result_t PORT_ResetBitsMapp(uint8_t PinNum){
	if(PinNum>BOARD_NR_GPIO_PINS)return(Error);
	return (PORT_ResetBits(PIN_MAP[PinNum].gpio_port, PIN_MAP[PinNum].gpio_pin));
}

extern inline uint8_t PORT_GetBitMapp(uint8_t PinNum){
	en_flag_status_t getbit = Reset;
	if(PinNum>BOARD_NR_GPIO_PINS)return(false);
	getbit  = PORT_GetBit(PIN_MAP[PinNum].gpio_port, PIN_MAP[PinNum].gpio_pin);
	return (getbit==Reset?false:true);
}

extern void setup_gpio(void);

#ifdef __cplusplus
} // extern "C"
#endif
#endif

