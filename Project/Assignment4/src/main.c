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
#include "lcd.h"
#include "30021_io.h"

void init_spi_lsm98ds1()
{
    // Enable Clocks
    RCC->AHBENR  |= RCC_AHBENR_GPIOCEN;    // Enable Clock for GPIO Bank C
    RCC->APB1ENR |= RCC_APB1ENR_SPI3EN;    // Enable Clock for SPI3

    // Connect pins to SPI3
    GPIO_InitTypeDef GPIO_InitStructAll; //PC10 - SCLK
    GPIO_StructInit(&GPIO_InitStructAll);
    GPIO_InitStructAll.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructAll.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructAll.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_InitStructAll.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructAll);

    GPIO_StructInit(&GPIO_InitStructAll); // PC11 - MISO
    GPIO_InitStructAll.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructAll.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructAll.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructAll.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructAll);

    GPIO_StructInit(&GPIO_InitStructAll); // PC12 - MOSI
    GPIO_InitStructAll.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructAll.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructAll.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_InitStructAll.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructAll);

    GPIO_StructInit(&GPIO_InitStructAll); // PC2 - CS Mag
    GPIO_InitStructAll.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructAll.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructAll.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructAll.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructAll);

    GPIO_StructInit(&GPIO_InitStructAll); // PC3 - CS Acc/Gyro
    GPIO_InitStructAll.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructAll.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructAll.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructAll.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructAll);
    GPIO_SetBits(GPIOC, GPIO_Pin_3);

    GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_6); // Set pin PC10,PC11,PC12 to AF6
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_6);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_6);

    /*
    // Configure SPI3
    SPI3->CR1 |= SPI_CR1_SSM | SPI_CR1_BR_1|SPI_CR1_BR_0 | SPI_CR1_MSTR | SPI_CR1_CPOL;
    SPI3->CR2 |= SPI_CR2_DS_3|SPI_CR2_DS_2|SPI_CR2_DS_1|SPI_CR2_DS_0;
    SPI3->CR1 |= SPI_CR1_SPE;
    */

    // Configure SPI3
    SPI3->CR1 &= 0x3040; // Clear CR1 Register
    SPI3->CR1 |= 0x0000; // Configure direction (0x0000 - 2 Lines Full Duplex, 0x0400 - 2 Lines RX Only, 0x8000 - 1 Line RX, 0xC000 - 1 Line TX)
    SPI3->CR1 |= 0x0104; // Configure mode (0x0000 - Slave, 0x0104 - Master)
    SPI3->CR1 |= 0x0002; // Configure clock polarity (0x0000 - Low, 0x0002 - High)
    SPI3->CR1 |= 0x0001; // Configure clock phase (0x0000 - 1 Edge, 0x0001 - 2 Edge)
    SPI3->CR1 |= 0x0200; // Configure chip select (0x0000 - Hardware based, 0x0200 - Software based)
    SPI3->CR1 |= 0x0028; // Set Baud Rate Prescaler (0x0000 - 2, 0x0008 - 4, 0x0018 - 8, 0x0020 - 16, 0x0028 - 32, 0x0028 - 64, 0x0030 - 128, 0x0038 - 128)
    SPI3->CR1 |= 0x0000; // Set Bit Order (0x0000 - MSB First, 0x0080 - LSB First)
    SPI3->CR2 &= ~0x0F00; // Clear CR2 Register
    SPI3->CR2 |= 0x0F00; // Set Number of Bits (0x0300 - 4, 0x0400 - 5, 0x0500 - 6, ...);
    SPI3->I2SCFGR &= ~0x0800; // Disable I2S
    SPI3->CRCPR = 7; // Set CRC polynomial order
    SPI3->CR2 &= ~0x1000;
    SPI3->CR2 |= 0x1000; // Configure RXFIFO return at (0x0000 - Half-full (16 bits), 0x1000 - Quarter-full (8 bits))
    SPI3->CR1 |= 0x0040; // Enable SPI3

}

#define WHO_AM_I 0x0F
#define CTRL_REG1_G 0x10
#define CTRL_REG2_G 0x11
#define CTRL_REG3_G 0x12
#define CTRL_REG4_G 0x1E
#define CTRL_REG5_G 0x1F
#define CTRL_REG6_XL 0x20

#define OUT_X_L_G 0x18
#define OUT_X_H_G 0x19
#define OUT_Y_L_G 0x1A
#define OUT_Y_H_G 0x1B
#define OUT_Z_L_G 0x1C
#define OUT_Z_H_G 0x1D

#define OUT_X_L_XL 0x28
#define OUT_X_H_XL 0x29
#define OUT_Y_L_XL 0x2A
#define OUT_Y_H_XL 0x2B
#define OUT_Z_L_XL 0x2C
#define OUT_Z_H_XL 0x2D

#define FIFO_CTRL_REG 0x2E
#define FIFO_SRC_REG 0x2F
#define INT1_CFG 0x30
#define INT1_SRC 0x31
#define INT1_TSH_XH 0x32
#define INT1_TSH_XL 0x33
#define INT1_TSH_YH 0x34
#define INT1_TSH_YL 0x35
#define INT1_TSH_ZH 0x36
#define INT1_TSH_ZL 0x37
#define INT1_DURATION 0x38

uint8_t lsm9ds1_rx(uint16_t addr)
{
    GPIO_ResetBits(GPIOC, GPIO_Pin_3); // CS = 0 - Start Transmission
    while(SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_BSY) == 1) { }

    SPI3->DR = (0x80 | addr) << 8;

    while(SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_BSY) == 1) { }
    GPIO_SetBits(GPIOC, GPIO_Pin_3); // CS = 1 - End Transmission

    uint16_t fullRxData = SPI3->DR;
    uint8_t rx_data = (fullRxData & 0xFF);

    for (int i = 0; i < 10000; i++);

    return rx_data;
}

void lsm9ds1_tx(uint8_t addr, uint8_t data)
{
    GPIO_ResetBits(GPIOC, GPIO_Pin_3); // CS = 0 - Start Transmission
    while(SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_BSY) == 1) { }

    SPI3->DR = (addr<<8) | data;

    while(SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_BSY) == 1) { }
    GPIO_SetBits(GPIOC, GPIO_Pin_3); // CS = 1 - End Transmission

    for (int i = 0; i < 10000; i++);
}

void conf_lsm9ds1()
{
    uint8_t rx_data = 0;
    rx_data = lsm9ds1_rx(WHO_AM_I);
    if (rx_data != 104)
    {
        // Error handling
    }

}

void readAndDisplayAccelaration()
{
    uint8_t x_low, x_high;
    uint8_t y_low, y_high;
    uint8_t z_low, z_high;

    x_low   = lsm9ds1_rx(OUT_X_L_XL);
    x_high  = lsm9ds1_rx(OUT_X_H_XL);
    y_low   = lsm9ds1_rx(OUT_Y_L_XL);
    y_high  = lsm9ds1_rx(OUT_Y_H_XL);
    z_low   = lsm9ds1_rx(OUT_Z_L_XL);
    z_high  = lsm9ds1_rx(OUT_Z_H_XL);

    int16_t x = (signed)((x_high<<8)|x_low);
    int16_t y = (signed)((y_high<<8)|y_low);
    int16_t z = (signed)((z_high<<8)|z_low);


    char line1 [128];
    char line2 [128];
    char line3 [128];
    uint8_t fbuffer[LCD_BUFF_SIZE];         // Buffer to store LCD display
    memset(fbuffer, 0x00, LCD_BUFF_SIZE);   // Initialize array

    sprintf(line1, "x = %.2fg", (float)x*8/32765);
    sprintf(line2, "y = %.2fg", (float)y*8/32765);
    sprintf(line3, "z = %.2fg", (float)z*8/32765);
    lcd_write_string((uint8_t*)line1, fbuffer, 0, 0);
    lcd_write_string((uint8_t*)line2, fbuffer, 0, 1);
    lcd_write_string((uint8_t*)line3, fbuffer, 0, 2);
    lcd_push_buffer(fbuffer);
}

void printLCD(uint8_t data)
{
    char line1 [128];
    uint8_t fbuffer[LCD_BUFF_SIZE];         // Buffer to store LCD display
    memset(fbuffer, 0x00, LCD_BUFF_SIZE);   // Initialize array

    sprintf(line1, "0x%x", data);
    lcd_write_string((uint8_t*)line1, fbuffer, 0, 0);
    lcd_push_buffer(fbuffer);
}

int main(void)
{
    SystemInit();

    init_usb_uart(9600);
    printf("SPI\n");
    init_spi_lcd();
    init_spi_lsm98ds1();
    conf_lsm9ds1();


    while(1)
    {
        readAndDisplayAccelaration();

        for (int i = 0; i < 1000000; i++)
            ;
    }
}
