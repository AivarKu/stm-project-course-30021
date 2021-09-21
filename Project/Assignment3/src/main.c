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
#include "lcd.h"

///////////////////////////////////////////////////////////////////////////
//
// Exe 3.1 - PWM
//
///////////////////////////////////////////////////////////////////////////
#if 0
void init_PWM()
{
	// Configure TIM16 as channel 4 PWM output on pin PB6
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
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC,ENABLE);
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
void initPins ()
{
    GPIO_InitTypeDef GPIO_InitStructAll;
        // Set PC2 as output
    GPIO_StructInit(&GPIO_InitStructAll); // Initialize GPIO struct
    GPIO_InitStructAll.GPIO_Mode = GPIO_Mode_OUT; // Set as input
    GPIO_InitStructAll.GPIO_PuPd = GPIO_OType_PP; // OUT-put no pull
    GPIO_InitStructAll.GPIO_Pin = GPIO_Pin_2; // Set so the configuration is on pin 2
    GPIO_Init(GPIOC, &GPIO_InitStructAll); // Setup of GPIO with the settings chosen

            // Set PC2 as output
    GPIO_StructInit(&GPIO_InitStructAll); // Initialize GPIO struct
    GPIO_InitStructAll.GPIO_Mode = GPIO_Mode_OUT; // Set as input
    GPIO_InitStructAll.GPIO_PuPd = GPIO_OType_PP; // OUT-put no pull
    GPIO_InitStructAll.GPIO_Pin = GPIO_Pin_3; // Set so the configuration is on pin 3
    GPIO_Init(GPIOC, &GPIO_InitStructAll); // Setup of GPIO with the settings chosen

            // Set PC2 as output
    GPIO_StructInit(&GPIO_InitStructAll); // Initialize GPIO struct
    GPIO_InitStructAll.GPIO_Mode = GPIO_Mode_OUT; // Set as input
    GPIO_InitStructAll.GPIO_PuPd = GPIO_OType_PP; // OUT-put no pull
    GPIO_InitStructAll.GPIO_Pin = GPIO_Pin_15; // Set so the configuration is on pin 3
    GPIO_Init(GPIOC, &GPIO_InitStructAll); // Setup of GPIO with the settings chosen

            // Set PC2 as output
    GPIO_StructInit(&GPIO_InitStructAll); // Initialize GPIO struct
    GPIO_InitStructAll.GPIO_Mode = GPIO_Mode_OUT; // Set as input
    GPIO_InitStructAll.GPIO_PuPd = GPIO_OType_PP; // OUT-put no pull
    GPIO_InitStructAll.GPIO_Pin = GPIO_Pin_14; // Set so the configuration is on pin 2
    GPIO_Init(GPIOC, &GPIO_InitStructAll); // Setup of GPIO with the settings chosen
}


void controlstep (char word)
{
    if(word == 0b0101)
    {
        GPIO_ResetBits(GPIOC, GPIO_Pin_3);
        GPIO_SetBits(GPIOC, GPIO_Pin_2);
        GPIO_ResetBits(GPIOC, GPIO_Pin_15);
        GPIO_SetBits(GPIOC, GPIO_Pin_14);
    }
    else if (word == 0b1001)
    {
        GPIO_SetBits(GPIOC, GPIO_Pin_3);
        GPIO_ResetBits(GPIOC, GPIO_Pin_2);
        GPIO_ResetBits(GPIOC, GPIO_Pin_15);
        GPIO_SetBits(GPIOC, GPIO_Pin_14);
    }
    else if (word ==  0b1010)
    {
        GPIO_SetBits(GPIOC, GPIO_Pin_3);
        GPIO_ResetBits(GPIOC, GPIO_Pin_2);
        GPIO_SetBits(GPIOC, GPIO_Pin_15);
        GPIO_ResetBits(GPIOC, GPIO_Pin_14);
    }
    else if (word ==  0b0110)
    {
        GPIO_ResetBits(GPIOC, GPIO_Pin_3);
        GPIO_SetBits(GPIOC, GPIO_Pin_2);
        GPIO_SetBits(GPIOC, GPIO_Pin_15);
        GPIO_ResetBits(GPIOC, GPIO_Pin_14);
    }
    else
    {
        GPIO_ResetBits(GPIOC, GPIO_Pin_3);
        GPIO_ResetBits(GPIOC, GPIO_Pin_2);
        GPIO_ResetBits(GPIOC, GPIO_Pin_15);
        GPIO_ResetBits(GPIOC, GPIO_Pin_14);
    }

}

int main(void)
{
    SystemInit();

    init_usb_uart(9600);
    init_AF();  // Connect pin PA6 to TIM16 channel 1
    init_PWM(); // Init PWM using TIM16 on channel 1
    initPins();
    char position[4] = {0b0101, 0b1001, 0b1010, 0b0110};
    char word;


  while(1)
  {
      switch(uart_getc())
      {
          case '1': putchar('1');
          word = position[2];
          controlstep(word);
          break;



          case '2': putchar('2');
          word = position[3];
          controlstep(word);
           break;



          case '3': putchar('3');
          word = position[0];
          controlstep(word);
          break;


          case '4': putchar('4');
          word = position[1];
          controlstep(word);
           break;
      }
  }
}
    GPIO_InitStructAll.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructAll);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource3,GPIO_AF_1); // TIM2 channel 2

    // Set-up pin PB11
    // Connect Alternative function AF_1 to PB11
    GPIO_StructInit(&GPIO_InitStructAll);
    GPIO_InitStructAll.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructAll.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructAll.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_InitStructAll.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructAll);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource11,GPIO_AF_1); // TIM2 channel 4

	// Configure TIM2 as channel 2 and 4 PWM output on pin PB3 and PB11
	RCC->APB1ENR|= RCC_APB1ENR_TIM2EN;					// Enable clock for TIM2
	TIM2->CR1	|= TIM_CR1_ARPE;						// Enable the ARR preload register
	TIM2->CCMR1 |= TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1 |TIM_CCMR1_OC2PE; // Enable channel 2 in PWM mode 1, enable ch2 output cc preload register
    TIM2->CCMR2 |= TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1 |TIM_CCMR2_OC4PE; // Enable channel 4 in PWM mode 1, enable ch4 output cc preload register
	TIM2->CCER  |= TIM_CCER_CC2E | TIM_CCER_CC4E;		// Enable Channel 2 and 4
	TIM2->PSC 	 = 625-1;							    // Load prescaler value - 64 MHz/625 = 102.4kHz
	TIM2->ARR 	 = 2048-2;							    // Load signal period - 102.4kHz/2048 = 50Hz
	TIM2->CCR2   = 126;							        // Load PWM dutycycle to ch2 cr
	TIM2->CCR4   = 126;							        // Load PWM dutycycle to ch4 cr
	TIM2->EGR   |= TIM_EGR_UG;						    // Generate an event to update shadow registers
	TIM2->CR1   |= TIM_CR1_CEN;                         // Enable clock for TIM2
}

void ADC_setup_PA()
{
    // Enable clocks for peripherals
    RCC_ADCCLKConfig(RCC_ADC12PLLCLK_Div4);             // Set ADC clk to sys_clk/8
    RCC_AHBPeriphClockCmd(ADC1, ENABLE);                // Enable clocking for ADC1
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE); // Enable clocking for GPIOA

    GPIO_InitTypeDef GPIO_InitStructAll; // Define typedef struct for setting pins

    // Configure pin PA0 as analog input
    GPIO_StructInit(&GPIO_InitStructAll);
    GPIO_InitStructAll.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStructAll.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructAll.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_InitStructAll.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructAll);

    // Configure pin PA1 as analog input
    GPIO_StructInit(&GPIO_InitStructAll);
    GPIO_InitStructAll.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStructAll.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructAll.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_InitStructAll.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructAll);

    // Configure ADC1 peripheral
    ADC_InitTypeDef  ADC_InitStruct;
    ADC_StructInit(&ADC_InitStruct);
    ADC_InitStruct.ADC_ContinuousConvMode = ADC_ContinuousConvMode_Disable;
    ADC_InitStruct.ADC_Resolution = ADC_Resolution_12b;
    ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_Init(ADC1, &ADC_InitStruct);
}

void ADC_SelfCalibrate()
{
    // Calibrate ADC
    ADC_VoltageRegulatorCmd(ADC1,ENABLE);   // set internal reference voltage source and wait
    for(uint32_t i = 0; i<10000;i++);       //Wait for at least 10uS before continuing...

    ADC_SelectCalibrationMode(ADC1,ADC_CalibrationMode_Single);
    ADC_StartCalibration(ADC1);

    while(ADC_GetCalibrationStatus(ADC1)){}
    for(uint32_t i = 0; i<100;i++);

    // Enable ADC
    ADC_Cmd(ADC1,ENABLE);
    while(!ADC_GetFlagStatus(ADC1,ADC_FLAG_RDY)){}
}

uint16_t ADC_measure_PA(uint8_t channel)
{
    ADC_RegularChannelConfig(ADC1, channel, 1, ADC_SampleTime_1Cycles5); //  Select ADC channel
    ADC_StartConversion(ADC1); // Start ADC read
    while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == 0); // Wait for ADC read
    return ADC_GetConversionValue(ADC1); // Read the ADC value
}

void printLCD_ADC_int(uint8_t* fbuffer)
{
    uint16_t PA0_val = 0;
    uint16_t PA1_val = 0;
    char line1 [128];
    char line2 [128];

    PA0_val = ADC_measure_PA(ADC_Channel_1);    // Read PA0 ADC
    PA1_val = ADC_measure_PA(ADC_Channel_2);    // Read PA1 ADC

    // Write to LCD ADC values
    sprintf(line1, "PA0 = %d", PA0_val);
    sprintf(line2, "PA1 = %d", PA1_val);
    lcd_write_string((uint8_t*)line1, fbuffer, 0, 0);
    lcd_write_string((uint8_t*)line2, fbuffer, 0, 1);
    lcd_push_buffer(fbuffer);
}

void controlServos()
{
    uint16_t PA0_val = 0;
    uint16_t PA1_val = 0;

    PA0_val = ADC_measure_PA(ADC_Channel_1);    // Read PA0 ADC
    PA1_val = ADC_measure_PA(ADC_Channel_2);    // Read PA1 ADC

    // 1000us width is 1000e-6/(1/50/2048) ~= 102 units
    uint16_t servo_1_CMD = 102 + (uint16_t)((float)PA0_val/4096*102);
    uint16_t servo_2_CMD = 102 + (uint16_t)((float)PA1_val/4096*102);

    // Set servo PWM values
    TIM2->CCR2 = servo_1_CMD;
    TIM2->CCR4 = servo_2_CMD;
}

int main(void)
{
    SystemInit();

    init_usb_uart(9600);
    init_ServoPWM(); // Init PWM using TIM16 on channel 1 and channel
    ADC_setup_PA();
    ADC_SelfCalibrate();
    init_spi_lcd();

    uint8_t fbuffer[LCD_BUFF_SIZE];         // Buffer to store LCD display
    memset(fbuffer, 0x00, LCD_BUFF_SIZE);   // Initialize array

    while(1)
    {
        for(int i = 0; i < 200000; i++);

        printLCD_ADC_int(&fbuffer);
        controlServos();
    }
}
#endif
