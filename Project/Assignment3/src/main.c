/*
**
**                           Main.c
**
**
**********************************************************************/
/*
   Last committed:     $Revision: 00 $
   Last changed by:    $Author: $
   Last changed date:  $Date:  $
   ID:                 $Id:  $

**********************************************************************/
#include "stm32f30x_conf.h"
#include "30021_io.h"

void init_PWM()
{
	// Configure TIM16 as channel 4 PWM output on pin PB11
	RCC->APB2ENR|= RCC_APB2ENR_TIM16EN;					// Enable clock for TIM16
	TIM16->CR1	|= TIM_CR1_ARPE;						// Enable the ARR preload register
	TIM16->CCMR1|= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 |TIM_CCMR1_OC1PE; // Enable channel 1 in PWM mode 1, enable ch1 output cc preload register

	//TIM16->DIER	|= (TIM_DIER_CC1IE |TIM_DIER_UIE);		// Enable interrupts on ch1 compare, update event
	TIM16->CCER |= TIM_CCER_CC1E;						// Enable Channel 1
	TIM16->PSC 	 = 25-1;							    // Load prescaler value - 64 MHz/25 = 2.56MHz
	TIM16->ARR 	 = 256-1;							    // Load signal period - 2.56 MHz/256 = 10kHz
	TIM16->CCR1  = 126;							        // Load PWM dutycycle to ch1 cr
	TIM16->EGR 	|= TIM_EGR_UG;							// Generate an event to update shadow registers
	TIM16->BDTR |= TIM_BDTR_MOE;                        // Main output enable
	TIM16->CR1  |= TIM_CR1_CEN;                         // Enable clock for TIM16

	//NVIC_SetPriority(TIM1_UP_TIM16_IRQn, NVIC_EncodePriority(0, 1, 1));
	//NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);
}


void init_AF()
 {

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA,ENABLE); // Enable clock for GPIO Port A

    // Connect Alternative function AF_1 to PA6
    GPIO_InitTypeDef GPIO_InitStructAll; // Define typedef struct for setting pins

    GPIO_StructInit(&GPIO_InitStructAll);
    GPIO_InitStructAll.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructAll.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructAll.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_InitStructAll.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructAll);

    //Sets pin y at port x to alternative function z
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource6,GPIO_AF_1);
 }

int main(void)
{
    SystemInit();

    init_usb_uart(9600);
    init_AF();  // Connect pin PA6 to TIM16 channel 1
    init_PWM(); // Init PWM using TIM16 on channel 1

  while(1)
  {
      for(int i = 0; i < 200000; i++);
  }
}
