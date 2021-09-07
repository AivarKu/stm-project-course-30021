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
#define GetBit(data,BitNum) ((data) & (1<<(BitNum)))

#if 0
///////////////////////////////////////////////////////////////////////////
//
// Exe 1.1 - Joystick
//
///////////////////////////////////////////////////////////////////////////

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

uint8_t readJoystick()
{
    uint8_t gpioWord;
    uint8_t joystickState;

    // PA4 - Up
    gpioWord = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_4);
    joystickState = gpioWord;
    // PB0 - Down
    gpioWord = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0);
    joystickState |= gpioWord << 1;
    // PC1 - Left
    gpioWord = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_1);
    joystickState |= gpioWord << 2;
    // PC0 - Right
    gpioWord = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_0);
    joystickState |= gpioWord << 3;
    // PB5 - Center
    gpioWord = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_5);
    joystickState |= gpioWord << 4;

    return joystickState;
}

void printJoystick(uint8_t joystickState)
{
    printf("Joystick state: ");

    if (GetBit(joystickState,0))
        printf("UP ");

    if (GetBit(joystickState,1))
        printf("DOWN ");

    if (GetBit(joystickState,2))
        printf("LEFT ");

    if (GetBit(joystickState,3))
        printf("RIGHT ");

    if (GetBit(joystickState,4))
        printf("CENTER ");

    printf("\n");
}

int main(void)
{
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA,ENABLE); // Enable clock for GPIO Port A
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB,ENABLE); // Enable clock for GPIO Port B
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC,ENABLE); // Enable clock for GPIO Port C

    init_usb_uart( 9600 ); // Initialize USB serial at 9600 baud
    initJoystickPins();

    int a = 1;

    uint8_t joystickState = 0;

    while(1)
    {
        joystickState = readJoystick();
        printJoystick(joystickState);

        for (int i = 0; i < 2000000; i++)
            a++;
    }
}
#endif

///////////////////////////////////////////////////////////////////////////
//
// Exe 1.2 - RGB
//
///////////////////////////////////////////////////////////////////////////
void initRGBpins()
{
    // Sets PA9 to output
    GPIO_StructInit(&GPIO_InitStructAll); // Initialize GPIO struct
    GPIO_InitStructAll.GPIO_Mode = GPIO_Mode_OUT; // Set as output
    GPIO_InitStructAll.GPIO_OType = GPIO_OType_PP; // Set as Push-Pull
    GPIO_InitStructAll.GPIO_Pin = GPIO_Pin_9; // Set so the configuration is on pin 9
    GPIO_InitStructAll.GPIO_Speed = GPIO_Speed_2MHz; // Set speed to 2 MHz
    GPIO_Init(GPIOA, &GPIO_InitStructAll); // Setup of GPIO with the settings chosen

    // Sets PC7 to output
    GPIO_StructInit(&GPIO_InitStructAll); // Initialize GPIO struct
    GPIO_InitStructAll.GPIO_Mode = GPIO_Mode_OUT; // Set as output
    GPIO_InitStructAll.GPIO_OType = GPIO_OType_PP; // Set as Push-Pull
    GPIO_InitStructAll.GPIO_Pin = GPIO_Pin_7; // Set so the configuration is on pin 9
    GPIO_InitStructAll.GPIO_Speed = GPIO_Speed_2MHz; // Set speed to 2 MHz
    GPIO_Init(GPIOC, &GPIO_InitStructAll); // Setup of GPIO with the settings chosen

    // Sets PB4 to output
    GPIO_StructInit(&GPIO_InitStructAll); // Initialize GPIO struct
    GPIO_InitStructAll.GPIO_Mode = GPIO_Mode_OUT; // Set as output
    GPIO_InitStructAll.GPIO_OType = GPIO_OType_PP; // Set as Push-Pull
    GPIO_InitStructAll.GPIO_Pin = GPIO_Pin_4; // Set so the configuration is on pin 9
    GPIO_InitStructAll.GPIO_Speed = GPIO_Speed_2MHz; // Set speed to 2 MHz
    GPIO_Init(GPIOB, &GPIO_InitStructAll); // Setup of GPIO with the settings chosen
}


int main(void)
{
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA,ENABLE); // Enable clock for GPIO Port A
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB,ENABLE); // Enable clock for GPIO Port B
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC,ENABLE); // Enable clock for GPIO Port C

    init_usb_uart( 9600 ); // Initialize USB serial at 9600 baud
    initJoystickPins();

    int a = 1;

    uint8_t joystickState = 0;

    while(1)
    {
        joystickState = readJoystick();
        printJoystick(joystickState);

        for (int i = 0; i < 2000000; i++)
            a++;
    }
}
