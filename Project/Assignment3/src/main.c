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
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_TIM16,ENABLE); //APB2 due to it being timer 16
    TIM_TimeBaseInitTypeDef TIM_InitStructure;

    TIM_TimeBaseStructInit(&TIM_InitStructure);
    TIM_InitStructure.TIM_ClockDivision = 0;
    TIM_InitStructure.TIM_Period = 0xFFFFFFFF;
    //TIM_InitStructure.TIM_Period = 0xFE;
    TIM_InitStructure.TIM_Prescaler = 6400;   // Will count at 10 KHz
    TIM_TimeBaseInit(TIM16, &TIM_InitStructure);


    /*
    TIM16->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1PE;
    TIM16->CCER |= TIM_CCER_CC1E;
    TIM16->ARR = 6500;
    TIM16->CCR1 = 3000;
    TIM16->EGR |= TIM_EGR_UG;
    TIM16->CR1 |= TIM_CR1_ARPE |TIM_CR1_CEN;
    */

    //PWM init
    TIM_OCInitTypeDef TIM_OCInitStruct;
    TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC1Init(TIM16, &TIM_OCInitStruct);
    TIM_OC1PreloadConfig(TIM16, TIM_OCPreload_Enable);

    TIM_CtrlPWMOutputs(TIM16, ENABLE);
    TIM_SetCompare1(TIM16, 127);
    TIM_Cmd(TIM16, ENABLE);
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
    init_usb_uart(9600);
    init_AF();
    init_PWM();

  while(1)
  {
    printf("Test");

    for(int i = 0; i < 2000000; i++);
  }
}
