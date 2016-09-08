#include "stm8s.h"
#include "stm8s_gpio.h"
#include "stm8s_exti.h"
#include "stm8s_adc1.h" 
#include "stm8s_tim1.h"
#include "stm8s_tim2.h"
#include "stm8s_tim4.h"
#include "stm8s_uart1.h"

#include "conf.h"

void CLK_Configuration(void)
{
  /* Fmaster = 16MHz */
  CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV1);
}

void GPIOA1_Int(void) {
  //GPIO_Init(GPIOA, GPIO_PIN_1, GPIO_MODE_IN_PU_IT);/*!< Input pull-up, external interrupt */
  GPIO_Init(GPIOA, GPIO_PIN_1, GPIO_MODE_IN_FL_IT);
  EXTI_DeInit();
  EXTI_SetExtIntSensitivity(EXTI_PORT_GPIOA, EXTI_SENSITIVITY_RISE_FALL); 
}

void GPIOA3_Int(void) {
  GPIO_Init(GPIOA, GPIO_PIN_3, GPIO_MODE_IN_PU_IT);/*!< Input pull-up, external interrupt */
  EXTI_DeInit();
  EXTI_SetExtIntSensitivity(EXTI_PORT_GPIOA, EXTI_SENSITIVITY_RISE_FALL); 
}

void GPIOD6_Int(void) {
  //GPIO_Init(GPIOD, GPIO_PIN_6, GPIO_MODE_IN_FL_IT);/*!< Input pull-up, external interrupt */
  GPIO_Init(GPIOD, GPIO_PIN_6, GPIO_MODE_IN_PU_IT);
  EXTI_DeInit();
  EXTI_SetExtIntSensitivity(EXTI_PORT_GPIOD, EXTI_SENSITIVITY_RISE_FALL); 
}

void GPIO_Configuration(void)
{
  GPIO_DeInit(GPIOA);
  GPIO_DeInit(GPIOB);
  GPIO_DeInit(GPIOC);
  GPIO_DeInit(GPIOD);
  
  //LED
  GPIO_Init(GPIOC, GPIO_PIN_3, GPIO_MODE_OUT_PP_LOW_SLOW);
   
}

void ADC_Config(void) {
  //PD3 - ADC Analog input 4
  GPIO_Init(GPIOD, GPIO_PIN_3, GPIO_MODE_IN_FL_NO_IT);
  //PC4 - ADC Analog input 2
  GPIO_Init(GPIOC, GPIO_PIN_4, GPIO_MODE_IN_FL_NO_IT);
  
  //ADC_Init 
  ADC1_DeInit();
  ADC1_Init(ADC1_CONVERSIONMODE_SINGLE, 
               ADC1_CHANNEL_4,
               ADC1_PRESSEL_FCPU_D4, 
               ADC1_EXTTRIG_TIM,DISABLE,
               ADC1_ALIGN_RIGHT, 
               ADC1_SCHMITTTRIG_ALL, 
               DISABLE); 
  ADC1_Init(ADC1_CONVERSIONMODE_SINGLE, 
               ADC1_CHANNEL_2,
               ADC1_PRESSEL_FCPU_D4, 
               ADC1_EXTTRIG_TIM,DISABLE,
               ADC1_ALIGN_RIGHT, 
               ADC1_SCHMITTTRIG_ALL, 
               DISABLE);
  ADC1_Cmd(ENABLE);
  //ADC1_StartConversion();
  
  //ADC1_ITConfig(ADC1_IT_EOCIE,ENABLE);

}
//Configure TIM1 PWM
void TIM1_Config(void) {

   TIM1_DeInit();

  /* Time Base configuration
   TIM1_Prescaler = 0
   TIM1_CounterMode = TIM1_COUNTERMODE_UP
   TIM1_Period = 4095
   TIM1_RepetitionCounter = 0
  */
  TIM1_TimeBaseInit(16, TIM1_COUNTERMODE_UP, 255, 0);

  /* Channel 1, 2,3 and 4 Configuration in PWM mode */
  
  /*
  TIM1_OCMode = TIM1_OCMODE_PWM2
  TIM1_OutputState = TIM1_OUTPUTSTATE_ENABLE
  TIM1_OutputNState = TIM1_OUTPUTNSTATE_ENABLE
  TIM1_Pulse = CCR1_Val
  TIM1_OCPolarity = TIM1_OCPOLARITY_LOW
  TIM1_OCNPolarity = TIM1_OCNPOLARITY_HIGH
  TIM1_OCIdleState = TIM1_OCIDLESTATE_SET
  TIM1_OCNIdleState = TIM1_OCIDLESTATE_RESET 
  */
//  TIM1_OC1Init(TIM1_OCMODE_PWM1,
//               TIM1_OUTPUTSTATE_ENABLE, TIM1_OUTPUTNSTATE_DISABLE,
//               pwm,
//               TIM1_OCPOLARITY_LOW,  TIM1_OCNPOLARITY_HIGH,
//               TIM1_OCIDLESTATE_SET, TIM1_OCNIDLESTATE_RESET);
//
//  TIM1_OC2Init(TIM1_OCMODE_PWM1,
//               TIM1_OUTPUTSTATE_ENABLE, TIM1_OUTPUTNSTATE_DISABLE,
//               pwm,
//               TIM1_OCPOLARITY_LOW,  TIM1_OCNPOLARITY_HIGH,
//               TIM1_OCIDLESTATE_SET, TIM1_OCNIDLESTATE_RESET);
//
//  TIM1_OC3Init(TIM1_OCMODE_PWM2,
//               TIM1_OUTPUTSTATE_ENABLE, TIM1_OUTPUTNSTATE_DISABLE,
//               pwm,
//               TIM1_OCPOLARITY_LOW,  TIM1_OCNPOLARITY_HIGH,
//               TIM1_OCIDLESTATE_SET, TIM1_OCNIDLESTATE_RESET);

  TIM1_OC4Init(TIM1_OCMODE_PWM2, TIM1_OUTPUTSTATE_ENABLE, 
               0, 
               TIM1_OCPOLARITY_LOW,
               TIM1_OCIDLESTATE_RESET);

  /* TIM1 counter enable */
  TIM1_Cmd(ENABLE);
  /* TIM1 Main Output Enable */
  TIM1_CtrlPWMOutputs(ENABLE);
  
//  /* Set the Pulse value */
//  TIM1->CCR1H = (uint8_t)(pwm >> 8);
//  TIM1->CCR1L = (uint8_t)(pwm);
//  
//  TIM1->CCR2H = (uint8_t)(pwm >> 8);
//  TIM1->CCR2L = (uint8_t)(pwm);
//  
//  TIM1->CCR3H = (uint8_t)(pwm >> 8);
//  TIM1->CCR3L = (uint8_t)(pwm);
//  
//  TIM1->CCR4H = (uint8_t)(pwm >> 8);
//  TIM1->CCR4L = (uint8_t)(pwm);
} //TIM1_Config()

//Configure TIM2 to generate an update interrupt each 1ms 
void TIM2_Config(void) {
    //1ms
    #define TIM2_PRESCALER TIM2_PRESCALER_128
    #define TIM2_PERIOD 124 
    /* Time base configuration */
    TIM2_TimeBaseInit(TIM2_PRESCALER, TIM2_PERIOD);
    /* Clear TIM2 update flag */
    TIM2_ClearFlag(TIM2_FLAG_UPDATE);
    /* Enable update interrupt */
    TIM2_ITConfig(TIM2_IT_UPDATE, ENABLE);
    /* Enable TIM2 */
    TIM2_Cmd(ENABLE);
} //TIM2_Config()

//Configure TIM4 to generate an update interrupt each 1ms 
void TIM4_Config(void) //8-bit basic timer
{
    /* TIM4 configuration:*/
    //0,01ms
    //#define TIM4_PRESCALER TIM4_PRESCALER_2
    //#define TIM4_PERIOD 79 
  
//    //0.012625ms
//    #define TIM4_PRESCALER TIM4_PRESCALER_2
//    #define TIM4_PERIOD 100 //0xdb 
  
    //0.1ms
    #define TIM4_PRESCALER TIM4_PRESCALER_16
    #define TIM4_PERIOD 99 //0x63
  
    /* Time base configuration */
    /* Set the Prescaler value */
    TIM4->PSCR = (uint8_t)(TIM4_PRESCALER);
    /* Set the Autoreload value */
    TIM4->ARR = (uint8_t)(TIM4_PERIOD);
    /* Clear TIM4 update flag */
    /* Clear the flags (rc_w0) clear this bit by writing 0. Writing ‘1’ has no effect*/
    TIM4->SR1 = (uint8_t)(~TIM4_FLAG_UPDATE);
    /* Enable the Interrupt sources */
    TIM4->IER |= (uint8_t)TIM4_IT_UPDATE;
    /* Enable TIM4 */
    TIM4->CR1 |= TIM4_CR1_CEN;
}

void UART1_Config(void) {
    //GPIO BEEP
    GPIO_DeInit(GPIOD);
    //Tx
    GPIO_Init(GPIOD, GPIO_PIN_5, GPIO_MODE_OUT_PP_LOW_FAST);
    //GPIO BEEP
    UART1_DeInit();
    UART1_Init(115200, UART1_WORDLENGTH_8D, 
                UART1_STOPBITS_1, UART1_PARITY_NO, 
                UART1_SYNCMODE_CLOCK_DISABLE, UART1_MODE_TXRX_ENABLE);
    //UART1_ClearFlag(UART1_FLAG_RXNE); // /*!< Read Data Register Not Empty flag */
    //UART1_ITConfig(UART1_IT_RXNE, ENABLE); ///*!< Receive interrupt */
    UART1_Cmd(ENABLE);
}


void IrDA_Config(void) {
  UART1_IrDAConfig(UART1_IRDAMODE_NORMAL); //UART1_IrDAMode specifies the IrDA mode.
  UART1_IrDACmd(ENABLE); //Enables or disables the UART’s IrDA interface.
//  UART1->CR3 |= (uint8_t)UART1_CR3_LINEN; // /*!< Alternate Function output mask */
}

void Beep_Config(void) {
  //GPIO BEEP
  GPIO_DeInit(GPIOD);
  GPIO_Init(GPIOD, GPIO_PIN_4, GPIO_MODE_OUT_PP_LOW_FAST);
  //GPIO BEEP
  BEEP_DeInit();
  //SET OUTPUT FREQ 64kHz
  BEEP->CSR &= (uint8_t)(~BEEP_CSR_BEEPDIV); /* Clear bits */
  BEEP->CSR = 0x00;
  //SET OUTPUT FREQ 64kHz
  BEEP_Init(BEEP_FREQUENCY_4KHZ);
}

#define I2C_SPEED      100000
#define SLAVE_ADDRESS  0x20
void I2C_Config(void) {
  
  GPIO_Init(GPIOB, GPIO_PIN_4, GPIO_MODE_OUT_PP_LOW_FAST);
  GPIO_Init(GPIOB, GPIO_PIN_5, GPIO_MODE_OUT_PP_LOW_FAST);
  
  I2C_DeInit();
  I2C_Init(I2C_SPEED,        SLAVE_ADDRESS, 
           I2C_DUTYCYCLE_2,  I2C_ACK_CURR, 
           I2C_ADDMODE_7BIT, 16);
}

void I2C_Send(uint8_t Data) {
  /*Wait while the bus is busy */
  while (I2C_GetFlagStatus(I2C_FLAG_BUSBUSY));
  /*Send Data*/
  I2C_SendData(Data);
}

void IWDG_Config(void) {
  /* Enable IWDG (the LSI oscillator will be enabled by hardware) */
  IWDG_Enable();
  
  /* IWDG timeout equal to 250 ms (the timeout may varies due to LSI frequency
     dispersion) */
  /* Enable write access to IWDG_PR and IWDG_RLR registers */
  IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
  
  /* IWDG counter clock: LSI/128 */
  IWDG_SetPrescaler(IWDG_Prescaler_128);
  
  /* Set counter reload value to obtain 250ms IWDG Timeout.
    Counter Reload Value = 250ms/IWDG counter clock period
                         = 250ms / (LSI/128)
                         = 0.25s / (LsiFreq/128)
                         = LsiFreq/(128 * 4)
                         = LsiFreq/512
   */
  IWDG_SetReload(0xff);
  
  /* Reload IWDG counter */
  IWDG_ReloadCounter();
}

void IWDG_Reset(void) {
  /* Reload IWDG counter */
    IWDG_ReloadCounter();  
}

