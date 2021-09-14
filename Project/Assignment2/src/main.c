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
#include "stm32f30x_conf.h" // STM32 config
#include "30021_io.h" // LCD library
#include "lcd.h"
#include <string.h>

///////////////////////////////////////////////////////////////////////////
//
// Exe 2.1 - Write to LCD
//
///////////////////////////////////////////////////////////////////////////
#if 0
int main(void)
{
    init_usb_uart( 9600 ); // Initialize USB serial emulation at 9600 baud
    printf("LCD!\n"); // Show the world you are alive!

    // Configure LCD
    init_spi_lcd();

    uint8_t fbuffer[LCD_BUFF_SIZE];         // Buffer to store LCD display
    memset(fbuffer, 0xAA, LCD_BUFF_SIZE);   // Initialize array

    lcd_push_buffer(fbuffer);               // Write to LCD

    while(1)
    {}
}
#endif

///////////////////////////////////////////////////////////////////////////
//
// Exe 2.2 - Write strings to LCD
//
///////////////////////////////////////////////////////////////////////////
#if 0
int main(void)
{
    init_usb_uart( 9600 ); // Initialize USB serial emulation at 9600 baud
    printf("LCD!\n"); // Show the world you are alive!

    // Configure LCD
    init_spi_lcd();

    uint8_t fbuffer[LCD_BUFF_SIZE];         // Buffer to store LCD display
    memset(fbuffer, 0x00, LCD_BUFF_SIZE);   // Initialize array

    char line1 [128];
    char line2 [128];
    char line3 [128];
    char line4 [128];
    sprintf(line1, "x1 = %f", 1.234);
    sprintf(line2, "x2 = %d", 50);
    sprintf(line3, "x3 = %f", 5.001);
    sprintf(line4, "x4 = %d", 99);

    // Write to LCD
    lcd_write_string((uint8_t*)line1, fbuffer, 0, 0);
    lcd_write_string((uint8_t*)line2, fbuffer, 20, 1);
    lcd_write_string((uint8_t*)line3, fbuffer, 40, 2);
    lcd_write_string((uint8_t*)line4, fbuffer, 60, 3);
    lcd_push_buffer(fbuffer);

    while(1)
    {}
}
#endif


///////////////////////////////////////////////////////////////////////////
//
// Exe 2.3 - FLASH memory
//
///////////////////////////////////////////////////////////////////////////
#if 0
int main(void)
{

}
#endif

///////////////////////////////////////////////////////////////////////////
//
// Exe 2.4 - ADC
//
///////////////////////////////////////////////////////////////////////////
void ADC_setup_PA()
{
    // Enable clocks for peripherals
    RCC_ADCCLKConfig(RCC_ADC12PLLCLK_Div8);             // Set ADC clk to sys_clk/8
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

    // Calibrate ADC
    ADC_VoltageRegulatorCmd(ADC1,ENABLE);   // set internal reference voltage source and wait
    for(uint32_t i = 0; i<10000;i++);       //Wait for at least 10uS before continuing...

    //ADC_Cmd(ADC1,DISABLE);
    //while(ADC_GetDisableCmdStatus(ADC1)){} // wait for disable of ADC

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
    ADC_RegularChannelConfig(ADC1, channel, 1, ADC_SampleTime_1Cycles5);
    ADC_StartConversion(ADC1); // Start ADC read
    while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == 0); // Wait for ADC read
    return ADC_GetConversionValue(ADC1); // Read the ADC value
}

#if 1
int main(void)
{
    init_usb_uart( 9600 ); // Initialize USB serial emulation at 9600 baud
    printf("ADC!\n"); // Show the world you are alive!

    // Configure LCD
    init_spi_lcd();

    uint8_t fbuffer[LCD_BUFF_SIZE];         // Buffer to store LCD display
    memset(fbuffer, 0x00, LCD_BUFF_SIZE);   // Initialize array

    // Setup ADC
    ADC_setup_PA();

    // Variables for while(1) loop
    uint16_t PA0_val = 0;
    uint16_t PA1_val = 0;
    char line1 [128];
    char line2 [128];

    while(1)
    {
        PA0_val = ADC_measure_PA(ADC_Channel_1);    // Read PA0 ADC
        PA1_val = ADC_measure_PA(ADC_Channel_2);    // Read PA1 ADC

        // Write to LCD
        sprintf(line1, "PA0 = %d", PA0_val);
        sprintf(line2, "PA1 = %d", PA1_val);
        lcd_write_string((uint8_t*)line1, fbuffer, 0, 0);
        lcd_write_string((uint8_t*)line2, fbuffer, 0, 1);
        lcd_push_buffer(fbuffer);
    }

}
#endif
