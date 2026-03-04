// Flywheel_HAL.c
#include "Flywheel_HAL.h"
#include <xc.h>
#include <sys/attribs.h>
#include <stdint.h>
#include <stdbool.h>

// PWM define
#define PBCLK_HZ            20000000UL

#define PWM_FREQ_HZ         7000UL
#define T2_PRESCALE         4UL

#define MAX_RPM_CMD         350U

#define DUTY_MIN            0.0f
#define DUTY_MAX            100.0f
#define INT_MIN             0.0f
#define INT_MAX             100.0f

static uint16_t s_pr2 = 0;

void Flywheel_Init(void)
{
    // Timer2 for PWM
    TRISBbits.TRISB8 = 0;
    
    T2CONbits.ON = 0;
    T2CONbits.TCS = 0;
    TMR2 = 0;
    s_pr2 = (uint16_t)((PBCLK_HZ / (T2_PRESCALE * PWM_FREQ_HZ)) - 1UL);
    PR2 = s_pr2;
    T2CONbits.TCKPS = 0b010;  // 1:4
    T2CONbits.ON = 1;

    // OC3 - RB10
    OC3CONbits.ON = 0;
    OC3R = 0;
    OC3RS = 0;
    OC3CONbits.OCTSEL = 0;     // Timer2
    OC3CONbits.OCM = 0b110;
    RPB10Rbits.RPB10R = 0b0101;
    OC3CONbits.ON = 1;
}

void Flywheel_SetDuty(uint8_t duty_percent)
{
    LATBbits.LATB8 = 1;
    if (duty_percent > 100) duty_percent = 100;

    uint16_t duty_counts = (uint16_t)(((uint32_t)duty_percent * (uint32_t)(s_pr2 + 1U)) / 100UL);

    OC3RS = duty_counts;
}
void Flywheel_SetDuty_Counter(uint8_t duty_percent)
{
    LATBbits.LATB8 = 0;
    if (duty_percent > 100) duty_percent = 100;
    uint16_t duty_counts = (uint16_t)(((uint32_t)duty_percent * (uint32_t)(s_pr2 + 1U)) / 100UL);
    OC3RS = duty_counts;
}