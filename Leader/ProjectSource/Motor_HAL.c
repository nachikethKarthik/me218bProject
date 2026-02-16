// Motor_HAL.c
#include "Motor_HAL.h"
#include <xc.h>
#include <sys/attribs.h>
#include <stdint.h>
#include <stdbool.h>

// PWM define
#define PBCLK_HZ            20000000UL

#define PWM_FREQ_HZ         7000UL
#define T2_PRESCALE         4UL

// Control loop define
#define CTRL_TS_MS          2U
#define CTRL_TS_S           0.002f


#define T4_PRESCALE_BITS    0b011
#define PR4_2MS             4999U

#define ENC_A_EDGES_PER_REV   512UL

#define MAX_RPM_CMD         350U

#define KP                 0.08f
#define KI                 0.30f

#define DUTY_MIN            0.0f
#define DUTY_MAX            100.0f
#define INT_MIN             0.0f
#define INT_MAX             100.0f

//Pin mapping for encoder
#define ENC1A_TRIS          TRISBbits.TRISB8
#define ENC2A_TRIS          TRISBbits.TRISB9
#define ENC1A_PORT          PORTBbits.RB8
#define ENC2A_PORT          PORTBbits.RB9

static uint16_t s_pr2 = 0;

// Motor structure
typedef struct {
    volatile uint16_t cmd_rpm;
    volatile uint16_t meas_rpm;
    volatile float    integ;
    volatile uint8_t  duty;

    volatile int32_t  enc_count;   // count for the encoder
    volatile int32_t  enc_delta;   // increasement during one period
    volatile uint8_t  a_prev;
} MotorState;

static MotorState m[2] = {0};

// Enter the count crit, saving the status


/*

static inline uint32_t enter_crit(void)
{
    uint32_t s = __builtin_getstatus();
    __builtin_disable_interrupts();
    return s;
}

// Exit the count crit, recovering the status
static inline void exit_crit(uint32_t s)
{
    __builtin_setstatus(s);
    __builtin_enable_interrupts();
}

*/

// PWM
static void PWM_Init(void)
{

#ifdef ANSELAbits
    ANSELAbits.ANSA0 = 0;  // RA0 digital
#endif
#ifdef ANSELBbits
    ANSELBbits.ANSB8 = 0;
    ANSELBbits.ANSB9 = 0;
    ANSELBbits.ANSB5 = 0;
#endif

    // Timer2 for PWM
    T2CONbits.ON = 0;
    T2CONbits.TCS = 0;
    TMR2 = 0;

    s_pr2 = (uint16_t)((PBCLK_HZ / (T2_PRESCALE * PWM_FREQ_HZ)) - 1UL);
    PR2 = s_pr2;

    T2CONbits.TCKPS = 0b010;  // 1:4
    T2CONbits.ON = 1;

    // OC1 - RA0
    OC1CONbits.ON = 0;
    OC1R = 0;
    OC1RS = 0;
    OC1CONbits.OCTSEL = 0;     // Timer2
    OC1CONbits.OCM = 0b110;
    RPA0Rbits.RPA0R = 0b0101;
    OC1CONbits.ON = 1;

    // OC2 - RB5
    OC2CONbits.ON = 0;
    OC2R = 0;
    OC2RS = 0;
    OC2CONbits.OCTSEL = 0;     // Timer2
    OC2CONbits.OCM = 0b110;
    RPB5Rbits.RPB5R = 0b0101;
    OC2CONbits.ON = 1;
}

static inline void PWM_SetDutyPercent(uint8_t id, uint8_t duty_percent)
{
    if (duty_percent > 100) duty_percent = 100;

    uint16_t duty_counts =
        (uint16_t)(((uint32_t)duty_percent * (uint32_t)(s_pr2 + 1U)) / 100UL);

    if (id == 0) {
        OC1RS = duty_counts;
    } else {
        OC2RS = duty_counts;
    }
}

// CN encoder
static void EncoderCN_Init(void)
{
    // inputs
    ENC1A_TRIS = 1;
    ENC2A_TRIS = 1;

    // Record the first value
    m[0].a_prev = (uint8_t)(ENC1A_PORT ? 1 : 0);
    m[1].a_prev = (uint8_t)(ENC2A_PORT ? 1 : 0);

    CNCONBbits.ON = 1;       // enable CNB
    CNCONBbits.SIDL = 0;

    // Enable CN for RB8/RB9
    CNENBbits.CNIEB8 = 1;
    CNENBbits.CNIEB9 = 1;

    volatile uint32_t dummy = PORTB;
    (void)dummy;

    // CN interrupt priority
    IPC8bits.CNIP = 6;
    IPC8bits.CNIS = 0;

    IFS1bits.CNBIF = 0;   // clear CNB flag
    IEC1bits.CNBIE = 1;
}

// Timer4 init
static void ControlTimer4_Init(void)
{
    T4CONbits.ON = 0;
    T4CONbits.TCS = 0;
    T4CONbits.TCKPS = T4_PRESCALE_BITS; // 1:8

    TMR4 = 0;
    PR4  = PR4_2MS;

    IPC4bits.T4IP = 2;
    IPC4bits.T4IS = 0;
    IFS0CLR = _IFS0_T4IF_MASK;
    IEC0SET = _IEC0_T4IE_MASK;

    T4CONbits.ON = 1;
}
// APIs
void MotorHAL_Init(void)
{
    PWM_Init();
    PWM_SetDutyPercent(0, 0);
    PWM_SetDutyPercent(1, 0);

    EncoderCN_Init();
    ControlTimer4_Init();

    __builtin_enable_interrupts();
}

void MotorHAL_SetSpeedCmdRPM(uint8_t id, uint16_t rpm)
{
    if (id > 1) return;
    if (rpm > MAX_RPM_CMD) rpm = MAX_RPM_CMD;
    m[id].cmd_rpm = rpm;
}

uint16_t MotorHAL_GetSpeedMeasRPM(uint8_t id)
{
    if (id > 1) return 0;
    return m[id].meas_rpm;
}

int32_t MotorHAL_GetEncoderCount(uint8_t id)
{
    if (id > 1) return 0;
    //uint32_t s = enter_crit();
    __builtin_disable_interrupts();
    int32_t c = m[id].enc_count;
    __builtin_enable_interrupts();
    //exit_crit(s);
    return c;
}

uint8_t MotorHAL_GetDutyOut(uint8_t id)
{
    if (id > 1) return 0;
    return m[id].duty;
}

// CN ISR
void __ISR(_CHANGE_NOTICE_VECTOR, IPL6SOFT) CNHandler(void)
{
    volatile uint32_t dummy = PORTB;
    (void)dummy;

    IFS1bits.CNBIF = 0;

    uint8_t a1 = (uint8_t)(ENC1A_PORT ? 1 : 0);
    if ((m[0].a_prev == 0U) && (a1 == 1U)) {
        m[0].enc_count++;
        m[0].enc_delta++;
    }
    m[0].a_prev = a1;

    uint8_t a2 = (uint8_t)(ENC2A_PORT ? 1 : 0);
    if ((m[1].a_prev == 0U) && (a2 == 1U)) {
        m[1].enc_count++;
        m[1].enc_delta++;
    }
    m[1].a_prev = a2;
}

// ISR for PI control
void __ISR(_TIMER_4_VECTOR, IPL2SOFT) T4Handler(void)
{
    IFS0CLR = _IFS0_T4IF_MASK;

    for (uint8_t id = 0; id < 2; id++) {

        int32_t dc;
        //uint32_t s = enter_crit();
        __builtin_disable_interrupts();
        dc = m[id].enc_delta;
        m[id].enc_delta = 0;
        __builtin_enable_interrupts();
        //exit_crit(s);

        // rpm = (dc / edges_per_rev) / Ts * 60
        float rev_per_s = ((float)dc / (float)ENC_A_EDGES_PER_REV) / CTRL_TS_S;
        float rpm_f = rev_per_s * 60.0f;
        if (rpm_f < 0.0f) rpm_f = 0.0f;
        if (rpm_f > 65535.0f) rpm_f = 65535.0f;

        m[id].meas_rpm = (uint16_t)(rpm_f + 0.5f);

        // PI
        int32_t err = (int32_t)m[id].cmd_rpm - (int32_t)m[id].meas_rpm;

        float u = KP * (float)err + m[id].integ;

        if (u < DUTY_MIN) u = DUTY_MIN;
        if (u > DUTY_MAX) u = DUTY_MAX;

        uint8_t duty = (uint8_t)(u + 0.5f);
        m[id].duty = duty;
        PWM_SetDutyPercent(id, duty);

        // anti-windup
        bool sat_hi = (duty >= 100);
        bool sat_lo = (duty <= 0);

        if (!((sat_hi && err > 0) || (sat_lo && err < 0))) {
            m[id].integ += (KI * (float)err * CTRL_TS_S);
            if (m[id].integ < INT_MIN) m[id].integ = INT_MIN;
            if (m[id].integ > INT_MAX) m[id].integ = INT_MAX;
        }
    }
}
