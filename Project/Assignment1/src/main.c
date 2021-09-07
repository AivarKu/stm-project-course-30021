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
    GPIO_InitStructAll.GPIO_Pin = GPIO_Pin_0; // Set so the configuration is on pin 0
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
    GPIO_InitStructAll.GPIO_Pin = GPIO_Pin_5; // Set so the configuration is on pin 5
    GPIO_Init(GPIOB, &GPIO_InitStructAll); // Setup of GPIO with the settings chosen

    // Set PC1
    GPIO_StructInit(&GPIO_InitStructAll); // Initialize GPIO struct
    GPIO_InitStructAll.GPIO_Mode = GPIO_Mode_IN; // Set as input
    GPIO_InitStructAll.GPIO_PuPd = GPIO_PuPd_DOWN; // Set as pull down
    GPIO_InitStructAll.GPIO_Pin = GPIO_Pin_1; // Set so the configuration is on pin 1
    GPIO_Init(GPIOC, &GPIO_InitStructAll); // Setup of GPIO with the settings chosen

    // Set PB0
    GPIO_StructInit(&GPIO_InitStructAll); // Initialize GPIO struct
    GPIO_InitStructAll.GPIO_Mode = GPIO_Mode_IN; // Set as input
    GPIO_InitStructAll.GPIO_PuPd = GPIO_PuPd_DOWN; // Set as pull down
    GPIO_InitStructAll.GPIO_Pin = GPIO_Pin_0; // Set so the configuration is on pin 0
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

#if 0
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
    GPIO_InitTypeDef GPIO_InitStructAll; // Define typedef struct for setting pins

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
    GPIO_InitStructAll.GPIO_Pin = GPIO_Pin_7; // Set so the configuration is on pin 7
    GPIO_InitStructAll.GPIO_Speed = GPIO_Speed_2MHz; // Set speed to 2 MHz
    GPIO_Init(GPIOC, &GPIO_InitStructAll); // Setup of GPIO with the settings chosen

    // Sets PB4 to output
    GPIO_StructInit(&GPIO_InitStructAll); // Initialize GPIO struct
    GPIO_InitStructAll.GPIO_Mode = GPIO_Mode_OUT; // Set as output
    GPIO_InitStructAll.GPIO_OType = GPIO_OType_PP; // Set as Push-Pull
    GPIO_InitStructAll.GPIO_Pin = GPIO_Pin_4; // Set so the configuration is on pin 4
    GPIO_InitStructAll.GPIO_Speed = GPIO_Speed_2MHz; // Set speed to 2 MHz
    GPIO_Init(GPIOB, &GPIO_InitStructAll); // Setup of GPIO with the settings chosen
}

void writeRGB(uint8_t rgb)
{
    // Set BLUE if UP
    if (GetBit(rgb, 0) && ~GetBit(rgb, 1) && ~GetBit(rgb, 2) && ~GetBit(rgb, 3))
    {
        GPIO_ResetBits(GPIOA, GPIO_Pin_9);
        GPIO_SetBits(GPIOC, GPIO_Pin_7);
        GPIO_SetBits(GPIOB, GPIO_Pin_4);
    }
    // Set GREEN if DOWN
    else if (~GetBit(rgb, 0) && GetBit(rgb, 1) && ~GetBit(rgb, 2) && ~GetBit(rgb, 3))
    {
        GPIO_SetBits(GPIOA, GPIO_Pin_9);
        GPIO_ResetBits(GPIOC, GPIO_Pin_7);
        GPIO_SetBits(GPIOB, GPIO_Pin_4);
    }
    // Set RED if LEFT
    else if (~GetBit(rgb, 0) && ~GetBit(rgb, 1) && GetBit(rgb, 2) && ~GetBit(rgb, 3))
    {
        GPIO_SetBits(GPIOA, GPIO_Pin_9);
        GPIO_SetBits(GPIOC, GPIO_Pin_7);
        GPIO_ResetBits(GPIOB, GPIO_Pin_4);
    }
    // Set WHITE if RIGHT
    else if (~GetBit(rgb, 0) && ~GetBit(rgb, 1) && ~GetBit(rgb, 2) && GetBit(rgb, 3))
    {
        GPIO_ResetBits(GPIOA, GPIO_Pin_9);
        GPIO_ResetBits(GPIOC, GPIO_Pin_7);
        GPIO_ResetBits(GPIOB, GPIO_Pin_4);
    }
    // Clear RGB if no joystick input detected
    else
    {
        GPIO_SetBits(GPIOA, GPIO_Pin_9);
        GPIO_SetBits(GPIOC, GPIO_Pin_7);
        GPIO_SetBits(GPIOB, GPIO_Pin_4);
    }
}

#if 0
int main(void)
{
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA,ENABLE); // Enable clock for GPIO Port A
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB,ENABLE); // Enable clock for GPIO Port B
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC,ENABLE); // Enable clock for GPIO Port C

    init_usb_uart( 9600 ); // Initialize USB serial at 9600 baud
    initRGBpins();
    initJoystickPins();

    int a = 1;
    uint8_t joystickState = 0;

    while(1)
    {
        joystickState = readJoystick();
        writeRGB(joystickState);
        printJoystick(joystickState);

        // Simple delay
        for (int i = 0; i < 2000000; i++)
            a++;
    }
}
#endif

///////////////////////////////////////////////////////////////////////////
//
// Exe 1.3 - Interrputs
//
///////////////////////////////////////////////////////////////////////////
void EXTI0_IRQHandler(void)
{
        if (EXTI_GetITStatus(EXTI_Line0) != RESET)
        {
            uint8_t joystickState = readJoystick();

            if (GetBit(joystickState, 1))
                printf("DOWN\n");
            else if (GetBit(joystickState, 3))
                printf("RIGHT\n");

            EXTI_ClearITPendingBit(EXTI_Line0);
        }
}

void EXTI1_IRQHandler(void)
{
        if (EXTI_GetITStatus(EXTI_Line1) != RESET)
        {
            printf("LEFT\n");

            EXTI_ClearITPendingBit(EXTI_Line1);
        }
}

void EXTI4_IRQHandler(void)
{
        if (EXTI_GetITStatus(EXTI_Line4) != RESET)
        {
            printf("UP\n");

            EXTI_ClearITPendingBit(EXTI_Line4);
        }
}

void EXTI9_5_IRQHandler(void)
{
        if (EXTI_GetITStatus(EXTI_Line5) != RESET)
        {
            printf("CENTER\n");

            EXTI_ClearITPendingBit(EXTI_Line5);
        }
}

#if 0
int main(void)
{
    // Enable clocks
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE);

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA,ENABLE); // Enable clock for GPIO Port A
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB,ENABLE); // Enable clock for GPIO Port B
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC,ENABLE); // Enable clock for GPIO Port C

    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA,EXTI_PinSource4); // sets port A pin 4 to the IRQ
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB,EXTI_PinSource5); // sets port B pin 5 to the IRQ
    //SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC,EXTI_PinSource1); // sets port C pin 1 to the IRQ
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC,EXTI_PinSource0); // sets port C pin 0 to the IRQ
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB,EXTI_PinSource0); // sets port B pin 0 to the IRQ

    // define and set setting for EXTI
    EXTI_InitTypeDef EXTI_InitStructure;

    EXTI_InitStructure.EXTI_Line = EXTI_Line0;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_Init(&EXTI_InitStructure);

    EXTI_InitStructure.EXTI_Line = EXTI_Line1;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_Init(&EXTI_InitStructure);

    EXTI_InitStructure.EXTI_Line = EXTI_Line4; // line 4 see [RM p. 215]
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_Init(&EXTI_InitStructure);

    EXTI_InitStructure.EXTI_Line = EXTI_Line5;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_Init(&EXTI_InitStructure);

    // setup NVIC
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_Init(&NVIC_InitStructure);

    init_usb_uart( 9600 ); // Initialize USB serial at 9600 baud
    initJoystickPins();

    while(1)
    {

    }
}
#endif


///////////////////////////////////////////////////////////////////////////
//
// Exe 1.4 - Timer
//
///////////////////////////////////////////////////////////////////////////
int milliseconds10 = 0;
int seconds = 0;
int minutes = 0;
int hours = 0;

#if 0
void TIM2_IRQHandler(void)
{
        if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
        {
            milliseconds10++;

            if (milliseconds10 == 100)
            {
                seconds++;
                milliseconds10 = 0;
            }
            else if (seconds == 60)
            {
                minutes++;
                seconds = 0;
            }
            else if (minutes == 60)
            {
                hours++;
                minutes = 0;
            }
            else if (hours == 24)
            {
                hours = 0;
            }

            TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
        }
}

int main()
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

    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_Init(&NVIC_InitStructure);

    TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);

    TIM_Cmd(TIM2,ENABLE);

    init_usb_uart( 9600 ); // Initialize USB serial at 9600 baud

    while(1)
    {
        printf("Time:%d:%d:%d:%d\n", hours, minutes, seconds, milliseconds10);
    }
}
#endif

///////////////////////////////////////////////////////////////////////////
//
// Exe 1.5 - Printing
//
///////////////////////////////////////////////////////////////////////////
#if 0
int main()
{
    init_usb_uart( 9600 ); // Initialize USB serial at 9600 baud

    // Printing integers
    printf("\nValue = %02ld\n", (long int)11223344);

    // Printing floats
    printf("f = %f\n", 1.5);

    // Printing strings
    uint8_t a = 10;
    char str[7];
    sprintf(str, "a = %2d", a);
    printf("%s", str);



    while(1)
    {

    }
}
#endif

///////////////////////////////////////////////////////////////////////////
//
// Exe 1.6 - Compare
//
///////////////////////////////////////////////////////////////////////////
int ICValue1 = 0;
int ICValue2 = 0;
int ICValueChanged = 0;

void TIM2_IRQHandler(void)
{
    TIM_ClearITPendingBit(TIM2, TIM_IT_CC2);
    ICValue1 = TIM_GetCapture1(TIM2); // Period
    ICValue2 = TIM_GetCapture2(TIM2); // 5Duty/Width
    ICValueChanged = 1;
    TIM_ITConfig(TIM2, TIM_IT_CC2, DISABLE);
}

int main()
{
    SystemInit();
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA,ENABLE); // Enable clock for GPIO Port B


    init_usb_uart( 9600 ); // Initialize USB serial at 9600 baud

    // Connect Alternative function AF_1 to PA0
    GPIO_InitTypeDef GPIO_InitStructAll; // Define typedef struct for setting pins

    GPIO_StructInit(&GPIO_InitStructAll);
    GPIO_InitStructAll.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructAll.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructAll.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_InitStructAll.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructAll);

    //GPIO_PinAFConfig(GPIOA, GPIO_Pin_0, GPIO_AF_1); //Sets pin y at port x to alternative function z
    GPIOA->AFR[0] |= GPIO_AF_1;

    // Input Capture Configuration ---------------------------------------------
  /*  TIM_ICInitTypeDef  TIM_ICInitStructure;
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = 0;
    TIM_ICInitStructure.TIM_ICFilter = 0x0;
    TIM_ICInit(TIM2, &TIM_ICInitStructure); */

    //TIM_PWMIConfig(TIM2, &TIM_ICInitStructure);

    TIM_TimeBaseInitTypeDef TIM_InitStructure;

    TIM_TimeBaseStructInit(&TIM_InitStructure);
    TIM_InitStructure.TIM_ClockDivision = 0;
    TIM_InitStructure.TIM_Period = 0xFFFFFFFF;
    TIM_InitStructure.TIM_Prescaler = 64;   // Will count at 1 MHz
    TIM_TimeBaseInit(TIM2,&TIM_InitStructure);


    // Internal Clock frequency is 64 MHz
    TIM2->CCMR1 = TIM_CCMR1_CC2S_1 | TIM_CCMR1_CC1S_0;
    TIM2->SMCR = TIM_SMCR_TS_2 | TIM_SMCR_TS_0 | TIM_SMCR_SMS_2;
    TIM2->CCER = TIM_CCER_CC2P | TIM_CCER_CC1E | TIM_CCER_CC2E;

    // Enable the CC2 Interrupt Request
    TIM_ITConfig(TIM2, TIM_IT_CC2, ENABLE);

    // Configure the Nested Interrupt Vector -----------------------------------
    // Enable the TIM3 global Interrupt
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    TIM_Cmd(TIM2, ENABLE);

    int a = 1;

    while(1)
    {
        // Print capture values
        if (ICValueChanged)
        {
            printf("Frequency: %f kHz, Period: %f ms, Duty: %f, %d\n", 1000/(double)ICValue1, ICValue1*1e-3, (double)ICValue2/ICValue1, ICValue1);
            ICValueChanged = 0;
            TIM_ITConfig(TIM2, TIM_IT_CC2, ENABLE);
        }

        // Simple delay
        for (int i = 0; i < 2000000; i++)
            a++;
    }
}
