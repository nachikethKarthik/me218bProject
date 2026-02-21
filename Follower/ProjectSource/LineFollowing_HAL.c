// LineFollowing_HAL.c
#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

#include "LineFollowing_HAL.h"
#include "PIC32_AD_Lib.h"

#define CHANNEL_SET  ((1u<<0)|(1u<<1)|(1u<<4)|(1u<<5)|(1u<<11)|(1u<<12))

#define KP        10.0f
#define KD        1.0f

static uint32_t ADCResults[8];

static uint32_t FrontLeft   = 0;
static uint32_t FrontCenter = 0;
static uint32_t FrontRight  = 0;

static uint32_t BackLeft    = 0;
static uint32_t BackCenter  = 0;
static uint32_t BackRight   = 0;

// PD state (use float)
static float last_err_front = 0.0f;
static float last_err_back  = 0.0f;

bool Init_LineFollowing(void)
{
    ANSELAbits.ANSA0 = 1;
    ANSELAbits.ANSA1 = 1;
    ANSELBbits.ANSB0 = 1;
    ANSELBbits.ANSB1 = 1;
    ANSELBbits.ANSB2 = 1;
    ANSELBbits.ANSB3 = 1;

    TRISAbits.TRISA0 = 1;
    TRISAbits.TRISA1 = 1;
    TRISBbits.TRISB0 = 1;
    TRISBbits.TRISB1 = 1;
    TRISBbits.TRISB2 = 1;
    TRISBbits.TRISB3 = 1;

    return ADC_ConfigAutoScan(CHANNEL_SET);
}

void ReadIRSensors(void)
{
    ADC_MultiRead(ADCResults);

    FrontLeft   = ADCResults[0];
    FrontCenter = ADCResults[1];
    FrontRight  = ADCResults[2];

    BackLeft    = ADCResults[3];
    BackCenter  = ADCResults[4];
    BackRight   = ADCResults[5];
}

uint32_t LineFollowing_GetFrontLeft(void)   { return FrontLeft; }
uint32_t LineFollowing_GetFrontCenter(void) { return FrontCenter; }
uint32_t LineFollowing_GetFrontRight(void)  { return FrontRight; }

uint32_t LineFollowing_GetBackLeft(void)    { return BackLeft; }
uint32_t LineFollowing_GetBackCenter(void)  { return BackCenter; }
uint32_t LineFollowing_GetBackRight(void)   { return BackRight; }

void LineFollowing_ResetPD(void)
{
    last_err_front = 0.0f;
    last_err_back  = 0.0f;
}

// return normalized error in [-1, 1] (approximately)
static inline float FrontError(void)
{
    float r = (float)FrontRight;
    float l = (float)FrontLeft;
    float c = (float)FrontCenter;

    return (r - l) / (r + l + c + 1.0f);
}

static inline float BackError(void)
{
    float r = (float)BackRight;
    float l = (float)BackLeft;
    float c = (float)BackCenter;

    return (r - l) / (r + l + c + 1.0f);
}

float LineFollowing_ComputeFrontTurn(void)
{
    float e  = FrontError();
    float de = e - last_err_front;
    last_err_front = e;

    return (KP * e) + (KD * de);
}

float LineFollowing_ComputeBackTurn(void)
{
    float e  = BackError();
    float de = e - last_err_back;
    last_err_back = e;

    return (KP * e) + (KD * de);
}