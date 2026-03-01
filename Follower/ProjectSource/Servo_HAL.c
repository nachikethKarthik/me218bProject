#include "Servo_HAL.h"
#include <xc.h>
#include <sys/attribs.h>
#include <stdint.h>

#define TICS_PER_MS      2500U

//#define SERVO_MIN_TICKS  1500U
//#define SERVO_MAX_TICKS  6500U

static uint16_t SERVO_MIN_TICKS[3] = {
    1500U,
    1500U,   //1500
    1600U
};

static uint16_t SERVO_MAX_TICKS[3] = {
    4800U,
    6500U,   //6500
    5800U
};

uint16_t SERVO_TARGET[3] = {
    1500U,
    1500U,   //1500
    1600U
};

static uint16_t SERVO_CURRENT[3] = {
    1500U,
    1500U,   //1500
    1600U
};

#define PR3_20MS         (uint16_t)(TICS_PER_MS*20U - 1U)  // 20ms
#define SERVO_STEP_TICKS  7U

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
    
    if (id >= 3) return;
    if (pw < SERVO_MIN_TICKS[id]) pw = SERVO_MIN_TICKS[id];
    if (pw > SERVO_MAX_TICKS[id]) pw = SERVO_MAX_TICKS[id];
        servo_pw_ticks[id] = pw;
    
}



void __ISR(_TIMER_3_VECTOR, IPL2SOFT) T3Handler(void)
{
    IFS0CLR = _IFS0_T3IF_MASK;

    OC1RS = servo_pw_ticks[0];
    OC2RS = servo_pw_ticks[1];
    OC4RS = servo_pw_ticks[2];
}

void Servo_Angle_Step(void)
{
    for (int i = 0; i < 3; i++) {
        uint16_t cur = SERVO_CURRENT[i];
        uint16_t tgt = SERVO_TARGET[i];

        if (cur < tgt) {
            uint16_t next = cur + SERVO_STEP_TICKS;
            if (next > tgt) next = tgt;
            Servo_SetPalseWidth((uint8_t)i, next);
            SERVO_CURRENT[i] = next;
        } 
        else if (cur > tgt) {
            uint16_t next = (cur > SERVO_STEP_TICKS) ? (cur - SERVO_STEP_TICKS) : 0;
            if (next < tgt) next = tgt;
            Servo_SetPalseWidth((uint8_t)i, next);
            SERVO_CURRENT[i] = next;
        }
    }
}

void Servo_SetAngle_Step(uint8_t id, uint8_t angle){
    if (angle > 180) angle = 180;
    uint16_t pw = (uint16_t)(SERVO_MIN_TICKS[id] +((uint16_t)angle * (uint16_t)(SERVO_MAX_TICKS[id] - SERVO_MIN_TICKS[id])) / 180UL);
    SERVO_TARGET[id] = pw;
}


void Servo_SyncCurrentToOutput(void)
{
    for (int i = 0; i < 3; i++) {
        SERVO_CURRENT[i] = servo_pw_ticks[i];
    }
}