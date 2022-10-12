#ifndef __BSP_INIT_H__
#define __BSP_INIT_H__


#include "hc32_ddl.h"


#ifdef __cplusplus
extern "C"
{
#endif


#define LED0_PORT                       (PortA)
#define LED0_PIN                        (Pin01)

#define LED0_ON()                       (PORT_SetBits(LED0_PORT, LED0_PIN))
#define LED0_OFF()                      (PORT_ResetBits(LED0_PORT, LED0_PIN))
#define LED0_TOGGLE()                   (PORT_Toggle(LED0_PORT, LED0_PIN))

#define LED0_ON()                       (PORT_SetBits(LED0_PORT, LED0_PIN))
#define LED0_OFF()                      (PORT_ResetBits(LED0_PORT, LED0_PIN))
#define LED0_TOGGLE()                   (PORT_Toggle(LED0_PORT, LED0_PIN))

#if 0


// Endstops
#define X_MIN_PORT                      (PortA)
#define X_MIN_PIN                       (Pin06)

#define Y_MIN_PORT                      (PortC)
#define Y_MIN_PIN                       (Pin05)

#define Z_MIN_PORT                      (PortA)
#define Z_MIN_PIN                       (Pin08)

#define E0_MIN_PORT                     (PortC)
#define E0_MIN_PIN                      (Pin13)

#define Z_PROBE_PORT                    (PortB)
#define Z_PROBE_PIN                     (Pin08)

//
// Steppers
//

// X stepper
#define X_ENABLE_PORT                   (PortC)
#define X_ENABLE_PIN                    (Pin03)

#define X_STEP_PORT                     (PortA)
#define X_STEP_PIN                      (Pin05)

#define X_DIR_PORT                      (PortA)
#define X_DIR_PIN                       (Pin04)

#define X_STALL_PORT                    X_MIN_PORT
#define X_STALL_PIN                     X_MIN_PIN

// Y stepper
#define Y_ENABLE_PORT                   X_ENABLE_PORT
#define Y_ENABLE_PIN                    X_ENABLE_PIN

#define Y_STEP_PORT                     (PortC)
#define Y_STEP_PIN                      (Pin04)

#define Y_DIR_PORT                      (PortA)
#define Y_DIR_PIN                       (Pin07)

#define Y_STALL_PORT                    Y_MIN_PORT
#define Y_STALL_PIN                     Y_MIN_PIN

// Z stepper
#define Z_ENABLE_PORT                   X_ENABLE_PORT
#define Z_ENABLE_PIN                    X_ENABLE_PIN

#define Z_STEP_PORT                     (PortC)
#define Z_STEP_PIN                      (Pin07)

#define Z_DIR_PORT                      (PortC)
#define Z_DIR_PIN                       (Pin06)

#define Z_STALL_PORT                    Z_MIN_PORT
#define Z_STALL_PIN                     Z_MIN_PIN


// E0 stepper
#define E0_ENABLE_PORT                  X_ENABLE_PORT
#define E0_ENABLE_PIN                   X_ENABLE_PIN

#define E0_STEP_PORT                    (PortC)
#define E0_STEP_PIN                     (Pin14)

#define E0_DIR_PORT                     (PortC)
#define E0_DIR_PIN                      (Pin15)

#define E0_STALL_PORT                   E0_MIN_PORT
#define E0_STALL_PIN                    E0_MIN_PIN


//
// Temperature Sensors
//
#define TEMP_BED_PORT                   (PortC)
#define TEMP_BED_PIN                    (Pin00)

#define TEMP_0_PORT                     (PortC)
#define TEMP_0_PIN                      (Pin01)

#define TEMP_1_PORT                     -1
#define TEMP_1_PIN                      -1

#define POWER_CHECK_PORT                (PortC)
#define POWER_CHECK_PIN                 (Pin02)

#define POWER_MONITOR_VOLTAGE_PIN       PC2


//
// Heaters
//
#define HEATER_BED_PORT                 (PortA)
#define HEATER_BED_PIN                  (Pin00)

#define HEATER_0_PORT                   (PortA)
#define HEATER_0_PIN                    (Pin01)

#define HEATER_1_PORT                   -1
#define HEATER_1_PIN                    -1

#define FAN_0_PORT                       (PortB)
#define FAN_0_PIN                        (Pin04)

#define FAN_0_ON()                       (PORT_SetBits(FAN_0_PORT, FAN_0_PIN))
#define FAN_0_OFF()                      (PORT_ResetBits(FAN_0_PORT, FAN_0_PIN))
#define FAN_0_TOGGLE()                   (PORT_Toggle(FAN_0_PORT, FAN_0_PIN))

#define FAN_1_PORT                       (PortB)
#define FAN_1_PIN                        (Pin03)

#define FAN_1_ON()                       (PORT_SetBits(FAN_1_PORT, FAN_1_PIN))
#define FAN_1_OFF()                      (PORT_ResetBits(FAN_1_PORT, FAN_1_PIN))
#define FAN_1_TOGGLE()                   (PORT_Toggle(FAN_1_PORT, FAN_1_PIN))

#define FAN_2_PORT                       (PortB)
#define FAN_2_PIN                        (Pin09)

#define FAN_2_ON()                       (PORT_SetBits(FAN_2_PORT, FAN_2_PIN))
#define FAN_2_OFF()                      (PORT_ResetBits(FAN_2_PORT, FAN_2_PIN))
#define FAN_2_TOGGLE()                   (PORT_Toggle(FAN_2_PORT, FAN_2_PIN))

#endif


// UART1
#define USART1_PWC_PERIPH_CLK          PWC_FCG1_PERIPH_USART1
#define USART1_CH                      (M4_USART1)
#define USART1_BAUDRATE                (115200ul)
#define USART1_INT_RI                  INT_USART1_RI
#define USART1_INT_EI                  INT_USART1_EI
#define USART1_INT_TI                  INT_USART1_TI
#define USART1_INT_TCI                 INT_USART1_TCI

#define USART1_TX_PORT                 (PortA)  // Func_Grp1
#define USART1_TX_PIN                  (Pin09)
#define USART1_TX_FUNC                 (Func_Usart1_Tx)

#define USART1_RX_PORT                 (PortA)  // Func_Grp1
#define USART1_RX_PIN                  (Pin15)
#define USART1_RX_FUNC                 (Func_Usart1_Rx)

// UART2
#define USART2_PWC_PERIPH_CLK          PWC_FCG1_PERIPH_USART2
#define USART2_CH                      (M4_USART2)
#define USART2_BAUDRATE                (115200ul)
#define USART2_INT_RI                  INT_USART2_RI
#define USART2_INT_EI                  INT_USART2_EI
#define USART2_INT_TI                  INT_USART2_TI
#define USART2_INT_TCI                 INT_USART2_TCI

#define USART2_TX_PORT                 (PortA)  // Func_Grp1
#define USART2_TX_PIN                  (Pin02)
#define USART2_TX_FUNC                 (Func_Usart2_Tx)

#define USART2_RX_PORT                 (PortA)  // Func_Grp1
#define USART2_RX_PIN                  (Pin03)
#define USART2_RX_FUNC                 (Func_Usart2_Rx)

// UART3
#define USART3_PWC_PERIPH_CLK          PWC_FCG1_PERIPH_USART3
#define USART3_CH                      (M4_USART3)
#define USART3_BAUDRATE                (115200ul)
#define USART3_INT_RI                  INT_USART3_RI
#define USART3_INT_EI                  INT_USART3_EI
#define USART3_INT_TI                  INT_USART3_TI
#define USART3_INT_TCI                 INT_USART3_TCI

#define USART3_TX_PORT                 (PortA)  // Func_Grp1
#define USART3_TX_PIN                  (Pin11)
#define USART3_TX_FUNC                 (Func_Usart3_Tx)

#define USART3_RX_PORT                 (PortA)  // Func_Grp1
#define USART3_RX_PIN                  (Pin12)
#define USART3_RX_FUNC                 (Func_Usart3_Rx)

// UART4
#define USART4_PWC_PERIPH_CLK          PWC_FCG1_PERIPH_USART4
#define USART4_CH                      (M4_USART4)
#define USART4_BAUDRATE                (115200ul)
#define USART4_INT_RI                  INT_USART4_RI
#define USART4_INT_EI                  INT_USART4_EI
#define USART4_INT_TI                  INT_USART4_TI
#define USART4_INT_TCI                 INT_USART4_TCI

#define USART4_TX_PORT                 (PortB)  // Func_Grp2
#define USART4_TX_PIN                  (Pin10)
#define USART4_TX_FUNC                 (Func_Usart4_Tx)

#define USART4_RX_PORT                 (PortH)  // Func_Grp2
#define USART4_RX_PIN                  (Pin02)
#define USART4_RX_FUNC                 (Func_Usart4_Rx)



void clock_init(void);
void get_all_clock(void);

void led_pin_init(void);

void endstop_pin_init(void);

void stepper_pin_init(void);

void heater_pin_init(void);

void fan_pin_init(void);

void uart1_init();
void uart2_init();
void uart3_init();
void uart4_init(void);


#ifdef __cplusplus
}
#endif


#endif

