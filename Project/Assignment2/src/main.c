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
    printf("LCD strings!\n"); // Show the world you are alive!

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

#if 0
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

uint8_t adc_update_flag = 0;

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

void tim2_setup()
{
    // Timer
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);

    // System clock is 64 MHz
    TIM_TimeBaseInitTypeDef TIM_InitStructure;
    TIM_TimeBaseStructInit(&TIM_InitStructure);
    TIM_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_InitStructure.TIM_Prescaler = 64000; // 64MHz / 64e3 = 1kHz
    TIM_InitStructure.TIM_Period = 100; // 1kHz/100 = 10Hz iterrput period
    TIM_TimeBaseInit(TIM2,&TIM_InitStructure);

    // NVIC for timer
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_Init(&NVIC_InitStructure);

    TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);

    TIM_Cmd(TIM2,ENABLE);
}

void TIM2_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
    {
        adc_update_flag = 1;
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    }
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
    ADC_SelfCalibrate();

    // Setup TIM2 for ADC values timing on LCD
    tim2_setup();

    while(1)
    {
        if (adc_update_flag)
        {
            adc_update_flag = 0;
            printLCD_ADC_int(&fbuffer);
        }
    }
}
#endif

///////////////////////////////////////////////////////////////////////////
//
// Exe 2.5 - ADC self VREF calibration
//
///////////////////////////////////////////////////////////////////////////
#define VREFINT_CAL *((uint16_t*) ((uint32_t) 0x1FFFF7BA))   //calibrated at 3.3V@ 30C

volatile float VDDA = 0.0;

void VREF_calibrate()
{
    ADC_VrefintCmd(ADC1,ENABLE); // setup ref voltage to channel 18
    for(uint32_t i = 0; i<10000;i++); // Delay

    // Setup ADC to measure VREFINT_DATA
    ADC_RegularChannelConfig(ADC1, ADC_Channel_18, 1, ADC_SampleTime_61Cycles5); // 16MHz/61=270kHz -> 3.7us > 2.2us
    ADC_StartConversion(ADC1); // Start ADC read
    while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == 0); // Wait for ADC read
    uint16_t VREF_DATA =  ADC_GetConversionValue(ADC1); // Read the ADC value

    ADC_VrefintCmd(ADC1,DISABLE);

    // Calculate calibrated VDDA
    VDDA = 3.3 * (float)VREFINT_CAL/(float)VREF_DATA;
}

void printLCD_ADC_float(uint8_t* fbuffer)
{
    uint16_t PA0_val = 0;
    uint16_t PA1_val = 0;
    char line1 [128];
    char line2 [128];

    PA0_val = ADC_measure_PA(ADC_Channel_1);    // Read PA0 ADC
    PA1_val = ADC_measure_PA(ADC_Channel_2);    // Read PA1 ADC

    // Write to LCD ADC values
    sprintf(line1, "PA0 = %.3f", (float)PA0_val*VDDA/4096);
    sprintf(line2, "PA1 = %.3f", (float)PA1_val*VDDA/4096);
    lcd_write_string((uint8_t*)line1, fbuffer, 0, 0);
    lcd_write_string((uint8_t*)line2, fbuffer, 0, 1);
    lcd_push_buffer(fbuffer);
}

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
    ADC_SelfCalibrate();
    VREF_calibrate();

    // Setup TIM2 for ADC values timing on LCD
    tim2_setup();


    while(1)
    {
        if (adc_update_flag)
        {
            adc_update_flag = 0;
            printLCD_ADC_float(&fbuffer);
        }
    }
}
#endif

///////////////////////////////////////////////////////////////////////////
//
// Exe 2.6 - ADC external calibration
//
///////////////////////////////////////////////////////////////////////////
volatile uint8_t runCalibration = 0;
volatile float adc_calibration_coeff = 0.0;

void EXTI4_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line4) != RESET)
    {
        runCalibration = 1;
        EXTI_ClearITPendingBit(EXTI_Line4);
    }
}

void setupPinInterrupt()
{
    // Enable clocks
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA,ENABLE); // Enable clock for GPIO Port A

    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA,EXTI_PinSource4); // sets port A pin 4 to the IRQ

    // define and set setting for EXTI
    EXTI_InitTypeDef EXTI_InitStructure;
    EXTI_InitStructure.EXTI_Line = EXTI_Line4;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_Init(&EXTI_InitStructure);

    // setup NVIC
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_Init(&NVIC_InitStructure);
}

#define ADC_CALIBRATION_ADDRESS 0x800FF00

void ADC_readExternalCalibration()
{
    uint32_t tempVal = *(uint32_t *)(ADC_CALIBRATION_ADDRESS);
    adc_calibration_coeff = (float)(*(float*)&tempVal);

    if (adc_calibration_coeff < 0.5 || adc_calibration_coeff > 1.5)
        adc_calibration_coeff = 1.0f;
}

void ADC_runExteralCalibration()
{
    // Read ADC 16 times
    uint16_t ADC_vals[16];
    for (int i = 0; i < 16; i++)
        ADC_vals[i] = ADC_measure_PA(ADC_Channel_1);    // Read PA0 ADC

    // Compute Calibration Value
    float ADC_avg = 0.0;
    for (int i = 0; i < 16; i++)
        ADC_avg += ADC_vals[i];

    ADC_avg = ADC_avg / 16;

    adc_calibration_coeff = 3.2/(ADC_avg/4096*VDDA);

    // Store Calibration Value
    FLASH_Unlock();
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);
    FLASH_ErasePage( ADC_CALIBRATION_ADDRESS );
    FLASH_ProgramWord(ADC_CALIBRATION_ADDRESS, (uint32_t)(*(uint32_t*)&adc_calibration_coeff));
    FLASH_Lock();
}

void printLCD_ADC_float_calibrated(uint8_t* fbuffer)
{
    uint16_t PA0_val = 0;
    uint16_t PA1_val = 0;
    char line1 [128];
    char line2 [128];

    PA0_val = ADC_measure_PA(ADC_Channel_1);    // Read PA0 ADC
    PA1_val = ADC_measure_PA(ADC_Channel_2);    // Read PA1 ADC

    // Write to LCD ADC values
    sprintf(line1, "PA0 = %.3f", (float)PA0_val*VDDA*adc_calibration_coeff/4096);
    sprintf(line2, "PA1 = %.3f", (float)PA1_val*VDDA*adc_calibration_coeff/4096);
    lcd_write_string((uint8_t*)line1, fbuffer, 0, 0);
    lcd_write_string((uint8_t*)line2, fbuffer, 0, 1);
    lcd_push_buffer(fbuffer);
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
    ADC_SelfCalibrate();
    VREF_calibrate();
    ADC_readExternalCalibration();

    // Setup TIM2 for ADC values timing on LCD
    tim2_setup();

    // Setup EXTI interrupt to run calibration on joystick press
    setupPinInterrupt();

    while(1)
    {
        if (runCalibration)
        {
            runCalibration = 0;
            ADC_runExteralCalibration();
        }
        if (adc_update_flag)
        {
            adc_update_flag = 0;
            printLCD_ADC_float_calibrated(&fbuffer);
        }
    }
}
#endif
