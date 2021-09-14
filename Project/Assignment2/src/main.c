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
#include "flash.h"

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

void flashTesting()
{
    uint8_t oneByteVar = 111;           //One byte
    uint16_t twoBytesVar = 22222;       //Two bytes
    uint32_t fourBytesVar = 44444444;   //Four bytes
    float floatVar = 7.77;              //Four bytes

    int32_t address = 0x800FF00;        //Next page to be overwritten
    uint32_t tempVal = 0;

    //Reading what is there
    printf("Before writing\n");
    tempVal = *(uint16_t *)(address);
    printf("1 byte data: %lu\n", tempVal);
    tempVal = *(uint16_t *)(address + 2);
    printf("2 byte data: %lu\n", tempVal);
    tempVal = *(uint32_t *)(address + 4);
    printf("4 byte data: %lu\n", tempVal);
    tempVal = *(uint32_t *)(address + 8);
    printf("float: %f\n", (float) tempVal);

    //Ok, let's overwrite this with declared values
    printf("Writing in progress...\n");
    FLASH_Unlock();
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);
    FLASH_ErasePage( address );
    FLASH_ProgramHalfWord(address, (uint16_t) oneByteVar);
    FLASH_ProgramHalfWord(address + 2, twoBytesVar);
    FLASH_ProgramWord(address + 4, fourBytesVar);
    FLASH_ProgramWord(address + 8, (uint32_t)(*(uint32_t*)&floatVar));
    FLASH_Lock();

    //Reading what is there
    printf("Before writing\n");
    tempVal = *(uint16_t *)(address);
    printf("1 byte data: %lu\n", tempVal);
    tempVal = *(uint16_t *)(address + 2);
    printf("2 byte data: %lu\n", tempVal);
    tempVal = *(uint32_t *)(address + 4);
    printf("4 byte data: %lu\n", tempVal);
    tempVal = *(uint32_t *)(address + 8);
    printf("float: %f\n", (float)(*(float*)&tempVal));
}

#if 1
int main(void)
{
    init_usb_uart( 9600 ); // Initialize USB serial emulation at 9600 baud
    /*
    int32_t address = 0x0800F800;
    uint16_t tempVal;
    printf("Before writing\n");
    for ( int i = 0 ; i < 10 ; i++ )
    {
        tempVal = *(uint16_t *)(address + i * 2); // Read Command
        printf("%d\n", tempVal);
    }

    printf("Writing in progress...\n");
    uint16_t data[10] = {0x0000, 0x000F, 0x00FF, 0x0F00, 0x0F0F, 0x0FF0, 0x0FFF, 0xF000, 0xF00F, 0xF0F0};
    FLASH_Unlock();
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);
    FLASH_ErasePage( address );
    for ( int i = 0; i < 10; i++ )
    {
        FLASH_ProgramHalfWord(address + i * 2, data[i]);
    }
    FLASH_Lock();

    printf("After writing\n");
    for ( int i = 0 ; i < 10 ; i++ )
    {
        tempVal = *(uint16_t *)(address + i * 2); // Read Command
        printf("%d\n", tempVal);

    }
    */

    flashTesting();
    /*
    tempfloat = read_float_flash(PG31_BASE,0);
    init_page_flash(PG31_BASE);
    FLASH_Unlock();
    write_float_flash(PG31_BASE,0,(float)1.0);
    FLASH_Lock();
    tempfloat = read_float_flash(PG31_BASE,0);
    ...
    tempval = read_word_flash(PG31_BASE,0);
    if(tempval!=(uint32_t)0xDEADBEEF)
    {
        init_page_flash(PG31_BASE);
        FLASH_Unlock();
        write_word_flash(PG31_BASE,0,0xDEADBEEF);
        FLASH_Lock();
    }
    tempval = read_hword_flash(PG31_BASE,0);
    */
}
#endif

///////////////////////////////////////////////////////////////////////////
//
// Exe 2.4 - ADC
//
///////////////////////////////////////////////////////////////////////////
#if 0
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
#endif

#if 0
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
