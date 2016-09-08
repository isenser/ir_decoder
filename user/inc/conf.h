#include "stm8s.h"
#include "stm8s_gpio.h"
#include "stm8s_exti.h"
#include "stm8s_adc1.h" 
#include "stm8s_tim1.h"
#include "stm8s_tim2.h"
#include "stm8s_tim4.h"
#include "stm8s_uart1.h"
#include "stm8s_i2c.h"
#include "stm8s_iwdg.h"

void CLK_Configuration(void);
void GPIOA1_Int(void);
void GPIOA3_Int(void);
void GPIOD6_Int(void);
void GPIO_Configuration(void);
void ADC_Config(void);
void TIM1_Config(void);
void TIM2_Config(void);
void TIM4_Config(void);
void UART1_Config(void);
void IrDA_Config(void);
void Beep_Config(void);
void I2C_Config(void);
void I2C_Send(uint8_t Data);
void IWDG_Config(void);
void IWDG_Reset(void);
