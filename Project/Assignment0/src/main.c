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
///////////// ASSIGNMENT 0.1-0.2
#if 0
#include "stm32f30x_conf.h" // STM32 config
#include "30021_io.h" // Input/output library for this course

int main(void)
{
      init_usb_uart( 9600 ); // Initialize USB serial emulation at 9600 baud
      printf("Hello World!\n"); // Show the world you are alive!
      while(1){}
}
#endif

///////////// ASSIGNMENT 0.3
#if 1
#include "stm32f30x_conf.h" // STM32 config
#include "30021_io.h" // Input/output library for this course
uint16_t power(uint16_t a, uint16_t exp) {
// calculates a^exp
    uint16_t i, r = a;
    for (i = 1; i < exp; i++)
        r *= a;
        return(r);
}

int main(void)
{
        uint16_t a;
        init_usb_uart( 9600 ); // Initialize USB serial at 9600 baud
        printf("\n\n x x^2 x^3 x^4\n");
        for (a = 0; a < 10; a++)
            printf("%8d%8d%8d%8d\n",a, power(a, 2), power(a, 3), power(a, 4));
        while(1){}
}
#endif
