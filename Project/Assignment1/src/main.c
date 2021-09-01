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
#include "30021_io.h" // Input/output library for this course

void initJoystickPins()
{
    GPIO_InitTypeDef GPIO_InitStructAll; // Define typedef struct for setting pins

    // Set PC0
    GPIO_StructInit(&GPIO_InitStructAll); // Initialize GPIO struct
    GPIO_InitStructAll.GPIO_Mode = GPIO_Mode_IN; // Set as input
    GPIO_InitStructAll.GPIO_PuPd = GPIO_PuPd_DOWN; // Set as pull down
    GPIO_InitStructAll.GPIO_Pin = GPIO_Pin_0; // Set so the configuration is on pin 4
    GPIO_Init(GPIOC, &GPIO_InitStructAll); // Setup of GPIO with the settings chosen

    // Set PA4
    GPIO_StructInit(&GPIO_InitStructAll); // Initialize GPIO struct
    GPIO_InitStructAll.GPIO_Mode = GPIO_Mode_IN; // Set as input
    GPIO_InitStructAll.GPIO_PuPd = GPIO_PuPd_DOWN; // Set as pull down
    GPIO_InitStructAll.GPIO_Pin = GPIO_Pin_4; // Set so the configuration is on pin 4
    GPIO_Init(GPIOA, &GPIO_InitStructAll); // Setup of GPIO with the settings chosen

    // Set PB5
    GPIO_StructInit(&GPIO_InitStructAll); // Initialize GPIO struct
    GPIO_InitStructAll.GPIO_Mode = GPIO_Mode_IN; // Set as input
    GPIO_InitStructAll.GPIO_PuPd = GPIO_PuPd_DOWN; // Set as pull down
    GPIO_InitStructAll.GPIO_Pin = GPIO_Pin_5; // Set so the configuration is on pin 4
    GPIO_Init(GPIOB, &GPIO_InitStructAll); // Setup of GPIO with the settings chosen

    // Set PC1
    GPIO_StructInit(&GPIO_InitStructAll); // Initialize GPIO struct
    GPIO_InitStructAll.GPIO_Mode = GPIO_Mode_IN; // Set as input
    GPIO_InitStructAll.GPIO_PuPd = GPIO_PuPd_DOWN; // Set as pull down
    GPIO_InitStructAll.GPIO_Pin = GPIO_Pin_1; // Set so the configuration is on pin 4
    GPIO_Init(GPIOC, &GPIO_InitStructAll); // Setup of GPIO with the settings chosen

    // Set PB0
    GPIO_StructInit(&GPIO_InitStructAll); // Initialize GPIO struct
    GPIO_InitStructAll.GPIO_Mode = GPIO_Mode_IN; // Set as input
    GPIO_InitStructAll.GPIO_PuPd = GPIO_PuPd_DOWN; // Set as pull down
    GPIO_InitStructAll.GPIO_Pin = GPIO_Pin_0; // Set so the configuration is on pin 4
    GPIO_Init(GPIOB, &GPIO_InitStructAll); // Setup of GPIO with the settings chosen
}

void printJoystick()
{
    uint16_t dataWord;
    uint8_t JoystickWord [5];

    // PC0
    dataWord = GPIO_ReadInputData(GPIOC);
    JoystickWord[0] = (dataWord & 1);
    // PC1
    JoystickWord[3] = (dataWord & 0b10) >> 1;

    // PA4
    dataWord = GPIO_ReadInputData(GPIOA);
    JoystickWord[1] = (dataWord & 0b10000) >> 4;

    // PB5
    dataWord = GPIO_ReadInputData(GPIOB);
    JoystickWord[2] = (dataWord & 0b100000) >> 5;
    // PB0
    JoystickWord[4] = (dataWord & 0b1);


    printf("PC0: %d\n", JoystickWord[0]);
    printf("PA4: %d\n", JoystickWord[1]);
    printf("PB5: %d\n", JoystickWord[2]);
    printf("PC1: %d\n", JoystickWord[3]);
    printf("PB0: %d\n\n", JoystickWord[4]);
}

int main(void)
{
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA,ENABLE); // Enable clock for GPIO Port A
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB,ENABLE); // Enable clock for GPIO Port B
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC,ENABLE); // Enable clock for GPIO Port C

    init_usb_uart( 9600 ); // Initialize USB serial at 9600 baud
    initJoystickPins();

    int a = 1;

    while(1)
    {
        printJoystick();

        for (int i = 0; i < 2000000; i++)
            a++;
    }
}
