// RGB_HAL.c
#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include "RGB_HAL.h"

void RGB_Init(void)
{
    TRISBbits.TRISB3 = 0; // Green
    TRISAbits.TRISA3 = 0; // White
    LATBbits.LATB3 = 1;
    LATAbits.LATA3 = 1;
}

void RGB_TurnGreen(void)
{
    LATBbits.LATB3 = 0;
    LATAbits.LATA3 = 1;
}

void RGB_TurnBlue(void)
{
    LATBbits.LATB3 = 1;
    LATAbits.LATA3 = 0;
}

void RGB_TurnCyan(void)
{
    LATBbits.LATB3 = 0;
    LATAbits.LATA3 = 0;
}

void RGB_TurnOff(void)
{
    LATBbits.LATB3 = 1;
    LATAbits.LATA3 = 1;
}