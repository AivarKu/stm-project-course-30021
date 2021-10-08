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
#include "delay.h"
#include "30021_io.h"


volatile int upcount = 0;
volatile float distance = 0;
volatile char done_flag = 1;

void set_up()
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC,ENABLE); //Enable CLK for PORTC

    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC,EXTI_PinSource0);

    GPIO_InitTypeDef GPIO_InitStructAll; // Define typedef struct for setting pins

    //set PC1 to trig (out)
    GPIO_StructInit(&GPIO_InitStructAll); // Initialize GPIO struct
    GPIO_InitStructAll.GPIO_Mode = GPIO_Mode_OUT; // Set as input
    GPIO_InitStructAll.GPIO_PuPd = GPIO_PuPd_DOWN; // Set as pull down
    GPIO_InitStructAll.GPIO_Pin = GPIO_Pin_1; // Set so the configuration is on pin 0
    GPIO_Init(GPIOC, &GPIO_InitStructAll); // Setup of GPIO with the settings chosen
    /*
        //set PC12 as input 4 echo (Input)
        GPIO_StructInit(&GPIO_InitStructAll); // Initialize GPIO struct
        GPIO_InitStructAll.GPIO_Mode = GPIO_Mode_IN; // Set as input
        GPIO_InitStructAll.GPIO_PuPd = GPIO_PuPd_NOPULL; // Set as pull down
        GPIO_InitStructAll.GPIO_Pin = GPIO_Pin_12; // Set so the configuration is on pin 0
        GPIO_Init(GPIOC, &GPIO_InitStructAll); // Setup of GPIO with the settings chosen
    */

    EXTI_InitTypeDef EXTI_InitStructure;

    EXTI_InitStructure.EXTI_Line = EXTI_Line0;  //due to using PC0
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_Init(&EXTI_InitStructure);
}

void timer2_setup()
{
    // Timer
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);

    TIM_TimeBaseInitTypeDef TIM_InitStructure;
    TIM_TimeBaseStructInit(&TIM_InitStructure);
    TIM_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_InitStructure.TIM_Period = 10;
    TIM_InitStructure.TIM_Prescaler = 64000;
    TIM_TimeBaseInit(TIM2,&TIM_InitStructure);




    // NVIC for timer
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn; //timer2
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;  //eksternt interrupt
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_Init(&NVIC_InitStructure);

//   TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);

    //TIM_Cmd(TIM2,ENABLE);
}

void EXTI0_IRQHandler(void)
{
    distance = upcount/58;

    done_flag = 1;
    EXTI_ClearITPendingBit(EXTI_Line0);
    TIM_ITConfig(TIM2,TIM_IT_Update,DISABLE);
    TIM_Cmd(TIM2,DISABLE);
    upcount = 0;
}

void TIM2_IRQHandler(void)
{
    upcount++;
}

int main(void)
{
    set_up();
    init_usb_uart( 9600 );
    timer2_setup();
    while(1)
    {

        printf("a");
        if (done_flag)    //The if is made to avoid the sensor reciving more than one distance mesuarment
        {
            GPIO_SetBits(GPIOC, GPIO_Pin_1); //set the trig high.
            for (int i = 0; i < 200000; i++); //delay to keep trig high for more than 10us. RE-Evaluate later.
            GPIO_ResetBits(GPIOC, GPIO_Pin_1);//Det skal teknisk set først sættes højt efter 8 clk. Hvordan jeg sikre mig, at de 8 clk går før timer2 sætter igang
            TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);
            TIM_Cmd(TIM2,ENABLE);
            done_flag = 0;
        }
        printf("upcount \n");
        printf("distance");
    }
}
