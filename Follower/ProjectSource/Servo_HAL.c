#include "Servo_HAL.h"
#include <xc.h>
#include <sys/attribs.h>
#include <stdint.h>

#define TICS_PER_MS      2500U

//#define SERVO_MIN_TICKS  1500U
//#define SERVO_MAX_TICKS  6500U

static uint16_t SERVO_MIN_TICKS[3] = {
    500U,
    1500U,
    900U
};

static uint16_t SERVO_MAX_TICKS[3] = {
    2500U,
    6500U,
    2100U
};

#define PR3_20MS         (uint16_t)(TICS_PER_MS*20U - 1U)  // 20ms

static volatile uint16_t servo_pw_ticks[3] = {
    (uint16_t)(TICS_PER_MS * 15U / 10U),
    (uint16_t)(TICS_PER_MS * 15U / 10U),
    (uint16_t)(TICS_PER_MS * 15U / 10U)
};

void Servo_Init(void)
{
    T3CONbits.ON = 0;
    T3CONbits.TCS = 0;
    T3CONbits.TCKPS = 0b011;

    TMR3 = 0;
    PR3  = PR3_20MS;

    IPC3bits.T3IP = 2;
    IPC3bits.T3IS = 0;
    IFS0CLR = _IFS0_T3IF_MASK;
    IEC0SET = _IEC0_T3IE_MASK;
    T3CONbits.ON = 1;

    // OC1 - RB15
    OC1CONbits.ON = 0;
    OC1R = 0;
    OC1RS = 0;
    OC1CONbits.OCTSEL = 1;
    OC1CONbits.OCM = 0b110;
    RPB15Rbits.RPB15R = 0b0101;
    OC1CONbits.ON = 1;

    // OC2 - RB5
    OC2CONbits.ON = 0;
    OC2R = 0;
    OC2RS = 0;
    OC2CONbits.OCTSEL = 1;
    OC2CONbits.OCM = 0b110;
    RPB5Rbits.RPB5R = 0b0101;
    OC2CONbits.ON = 1;

    // OC4 - RA2
    OC4CONbits.ON = 0;
    OC4R = 0;
    OC4RS = 0;
    OC4CONbits.OCTSEL = 1;
    OC4CONbits.OCM = 0b110;
    RPA2Rbits.RPA2R = 0b0101;
    OC4CONbits.ON = 1;
}

// Servo_Arm - OC1 - 0
// Servo_Indicator - OC2 - 1
// Servo_Trapdoor - OC4 - 2

void Servo_SetAngle(uint8_t id, uint8_t angle)
{
    if (id < 3) {
        if (angle > 180) angle = 180;

        uint16_t pw = (uint16_t)(SERVO_MIN_TICKS[id] +((uint16_t)angle * (uint16_t)(SERVO_MAX_TICKS[id] - SERVO_MIN_TICKS[id])) / 180UL);
        if (pw < SERVO_MIN_TICKS[id]) pw = SERVO_MIN_TICKS[id];
        if (pw > SERVO_MAX_TICKS[id]) pw = SERVO_MAX_TICKS[id];
        servo_pw_ticks[id] = pw;
    }
}


void Servo_SetPalseWidth(uint8_t id, uint16_t pw){
    
    if (pw < SERVO_MIN_TICKS[id]) pw = SERVO_MIN_TICKS[id];
    if (pw > SERVO_MAX_TICKS[id]) pw = SERVO_MAX_TICKS[id];
    if (id < 3) {
        servo_pw_ticks[id] = pw;
    }
    
}



void __ISR(_TIMER_3_VECTOR, IPL2SOFT) T3Handler(void)
{
    IFS0CLR = _IFS0_T3IF_MASK;

    OC1RS = servo_pw_ticks[0];
    OC2RS = servo_pw_ticks[1];
    OC4RS = servo_pw_ticks[2];
}

