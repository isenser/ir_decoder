#include "stm8s.h"
#include <stdbool.h>
#include "conf.h"
#include "stm8s_tim2.h"
#include "stm8s_tim4.h"

/* Private function prototypes -----------------------------------------------*/
void delay_ms(unsigned int n);
INTERRUPT void TIM2_OVF_IRQHandler(void); /* TIM2 UPD/OVF */
INTERRUPT void TIM4_OVF_IRQHandler(void); /* TIM4 UPD/OVF */
INTERRUPT_HANDLER (ITC_IRQ_PORTAHandler, 3);
void usend(u8 msg);
/* Private functions ---------------------------------------------------------*/
/* Global variables ----------------------------------------------------------*/

#define LED_DELAY 1000
volatile u16  delay=0;
volatile u16  led_delay=0;

#define TSOP_IN (GPIO_ReadInputPin(GPIOD,GPIO_PIN_6))

#define LED0 GPIO_WriteLow(GPIOA,GPIO_PIN_3);
#define LED1 GPIO_WriteHigh(GPIOA,GPIO_PIN_3);

// пороговое значение дл€ сравнени€ длинн импульсов и пауз
static const u8 Ir_Thershold = 11;// = 1.1 msec
// определ€ет таймаут приема посылки 
// и ограничивает максимальную длину импульса и паузы
static const u8 IR_Timer_Stop = 40; // 4ms

volatile struct ir_t {
        bool rx_started;
        u8 pulse_width;
        uint32_t code, rx_buffer;
} ir;

void main(void) {
  CLK_Configuration(); 
  GPIO_Configuration();
  GPIOA1_Int();
  //TIM2_Config(); //1ms
  TIM4_Config(); //0.11ms
  UART1_Config();
  
  /* enable interrupts */
  enableInterrupts();
  //IWDG_Config();
  
while (1) {
  //IWDG_Reset();
//  if (led_delay==0) {
//    GPIO_WriteReverse(GPIOC,GPIO_PIN_3);
//    led_delay = 200;
//  }
  if   (ir.code) {
            // конвертируем код в строку и выводим на дисплей
            usend((uint8_t)(ir.code>>24)); 
            usend((uint8_t)(ir.code>>16));
            usend((uint8_t)(ir.code>>8));
            usend((uint8_t)(ir.code));
            usend((uint8_t)(0x0D)); //CR
            usend((uint8_t)(0x0A)); //LF
    ir.code=0x00;
  } //if ir.code
       
  }//while (1)
}//main

void usend(u8 msg) {
  while(!(UART1->SR & UART1_FLAG_TC));
  UART1_SendData8(msg);
}

void delay_ms(unsigned int n) {
  delay=n;
  while (delay!=0) {IWDG_Reset();;}
}

 INTERRUPT_HANDLER(TIM2_OVF_IRQHandler, 13) //1ms
{
  if (delay!=0)       {delay--;} 
  if (led_delay!=0)   {led_delay--;}
  /* Clear the IT pending Bit */
  TIM2->SR1 = (uint8_t)(~TIM2_IT_UPDATE);
}

 INTERRUPT_HANDLER(TIM4_OVF_IRQHandler, 23) //0,1ms
{
  if (++ir.pulse_width>IR_Timer_Stop) {
    ir.code        = ir.rx_buffer;
    ir.rx_buffer   = 0;
    ir.rx_started  = 0;
    ir.pulse_width = 0;
    TIM4->CR1 &= (uint8_t)(~TIM4_CR1_CEN);
  }
    /* Clear the IT pending Bit */
    TIM4->SR1 = (uint8_t)(~TIM4_IT_UPDATE);
}

 INTERRUPT_HANDLER(ITC_IRQ_PORTAHandler, 3) //PA1 Interrupt
{
  //uint8_t delta;
  GPIO_WriteReverse(GPIOC,GPIO_PIN_3);
  if(ir.rx_started)
  {
          // если длительность импульса/паузы больше пороговой 
          // сдвигаем в буфер единицу иначе ноль.
          ir.rx_buffer <<= 1;
          if(ir.pulse_width > Ir_Thershold) ir.rx_buffer |= 1;
          ir.pulse_width = 0;
  }
  else{
          ir.rx_started = 1;
          ir.pulse_width = 0;
          TIM4->CR1 |= TIM4_CR1_CEN;
  }
}


/****************** (c) 2016  isenser@ya.ru ******************************/
