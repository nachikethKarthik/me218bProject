// Motor_HAL.c
#include "Motor_HAL.h"
#include "ES_CheckEvents.h"

#include "ES_Configure.h"
#include "ES_Framework.h"
#include "ES_Events.h"
#include "ES_PostList.h"
#include "ES_ServiceHeaders.h"
#include "ES_Port.h"

#include <xc.h>
#include <sys/attribs.h>
#include <stdint.h>
#include <stdbool.h>

// PWM define
#define PBCLK_HZ            20000000UL

#define PWM_FREQ_HZ         7000UL
#define T2_PRESCALE         4UL

// Control loop define
//#define CTRL_TS_MS          2U
//#define CTRL_TS_S           0.002f
#define CTRL_TS_MS          25U
#define CTRL_TS_S           0.025f

#define T4_PRESCALE_BITS    0b011
//#define PR4_2MS             4999U
#define PR4_2MS             62499U

#define ENC_A_EDGES_PER_REV   300UL



#define KP                 2.2f //0.2 - 0.6 range
#define KI                 0.9f

#define DUTY_MIN            0.0f
#define DUTY_MAX            100.0f
#define INT_MIN             0.0f
#define INT_MAX             100.0f

//Pin mapping for encoder
#define ENC1A_TRIS          TRISBbits.TRISB9
#define ENC2A_TRIS          TRISBbits.TRISB8
#define ENC1A_PORT          PORTBbits.RB9
#define ENC2A_PORT          PORTBbits.RB8

//Pin mapping for motor direction
#define M1_DIR_TRIS         TRISAbits.TRISA4
#define M1_DIR_LAT          LATAbits.LATA4
#define M2_DIR_TRIS         TRISAbits.TRISA1
#define M2_DIR_LAT          LATAbits.LATA1

static uint16_t s_pr2 = 0;
static const float alpha = 0.7f;

// Motor structure
typedef struct {
    volatile uint16_t cmd_rpm;
    volatile uint16_t meas_rpm;
    
    volatile float    integ;
    volatile uint8_t  duty;
    volatile bool     reverse;

    volatile int32_t  enc_count;   // count for the encoder
    volatile int32_t  enc_delta;   // increasement during one period
    volatile uint8_t  a_prev;
    volatile float rpm_filt;
    
    volatile bool is_driving_fixed_dis;
    volatile uint32_t start_count;
    volatile uint32_t drive_count;
    //volatile uint8_t meas_rpm;
} MotorState;

static MotorState m[2] = {0};
static volatile bool action_done_sent = false;
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

    // Set direction pins as outputs
    M1_DIR_TRIS = 0;
    M1_DIR_LAT = 0;
    M2_DIR_TRIS = 0;     // default forward
    M2_DIR_LAT = 0;     // default forward

    
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
    
    
    m[0].rpm_filt = 0.0f;
    m[1].rpm_filt = 0.0f;
    m[0].is_driving_fixed_dis = false;
    m[1].is_driving_fixed_dis = false;
}

static inline void PWM_SetDutyPercent(uint8_t id, uint8_t duty_percent, bool reverse)
{
    if (duty_percent > 100) duty_percent = 100;
    
    uint16_t duty_counts;
    uint16_t inv_duty_counts;
    uint8_t inv_duty;
    
    if (reverse) {
        // backward
        inv_duty = 100 - duty_percent;
        inv_duty_counts = (uint16_t)(((uint32_t)inv_duty * (uint32_t)(s_pr2 + 1U)) / 100UL);
    } 
    else
    {
        // forward
        duty_counts = (uint16_t)(((uint32_t)duty_percent * (uint32_t)(s_pr2 + 1U)) / 100UL);
    }

    if (id == 0) {
        if (reverse)
        {
            M1_DIR_LAT = 1;
            OC1RS = inv_duty_counts;
        }
        else
        {
            M1_DIR_LAT = 0;
            OC1RS = duty_counts;
        }
    } else {
        if (reverse)
        {
            M2_DIR_LAT = 1;
            OC2RS = inv_duty_counts;
        }
        else
        {
            M2_DIR_LAT = 0;
            OC2RS = duty_counts;
        }
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
    PWM_SetDutyPercent(0, 0, 0); // start at rest
    PWM_SetDutyPercent(1, 0, 0);

    EncoderCN_Init();
    ControlTimer4_Init();

    __builtin_enable_interrupts();
}

void MotorHAL_SetSpeedCmdRPM(uint8_t id, uint16_t rpm, bool reverse)
{
    if (id > 1) return;
    if (rpm > MAX_RPM_CMD) rpm = MAX_RPM_CMD;
    
    // If direction changes, clear integrator to avoid a kick
    if (m[id].reverse != reverse) {
        m[id].integ = 0.0f;
    }

    m[id].cmd_rpm = rpm;
    m[id].reverse = reverse;
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

// CN ISR - Counts both rising and falling edges on channel A
void __ISR(_CHANGE_NOTICE_VECTOR, IPL6SOFT) CNHandler(void)
{
    volatile uint32_t dummy = PORTB;   // clear mismatch
    (void)dummy;

    IFS1bits.CNBIF = 0;                // clear flag

    // Motor 1 A
    uint8_t a1 = (uint8_t)(ENC1A_PORT ? 1U : 0U);
    if (a1 != m[0].a_prev) {           // counts rising + falling
        m[0].enc_count++;
        m[0].enc_delta++;
        m[0].a_prev = a1;
    }

    // Motor 2 A
    uint8_t a2 = (uint8_t)(ENC2A_PORT ? 1U : 0U);
    if (a2 != m[1].a_prev) {           // counts rising + falling
        m[1].enc_count++;
        m[1].enc_delta++;
        m[1].a_prev = a2;
    }
    
    if (m[0].is_driving_fixed_dis == true){
        if ((m[0].enc_count - m[0].start_count) >= m[0].drive_count){
            MotorHAL_SetSpeedCmdRPM(0, 0, 0);
            m[0].is_driving_fixed_dis = false;
            
            if (!action_done_sent && m[0].is_driving_fixed_dis == false && m[1].is_driving_fixed_dis == false){
                action_done_sent = true;
                ES_Event_t ThisEvent;
                ThisEvent.EventType = ES_ACTION_DONE;
                PostLeaderService(ThisEvent);
            }
        }
    }
    if (m[1].is_driving_fixed_dis == true){
        if ((m[1].enc_count - m[1].start_count) >= m[1].drive_count){
            MotorHAL_SetSpeedCmdRPM(1, 0, 0);
            m[1].is_driving_fixed_dis = false;
            
            if (!action_done_sent && m[0].is_driving_fixed_dis == false && m[1].is_driving_fixed_dis == false){
                action_done_sent = true;
                ES_Event_t ThisEvent;
                ThisEvent.EventType = ES_ACTION_DONE;
                PostLeaderService(ThisEvent);
            }
        }
    }
    
}

// ISR for PI control
void __ISR(_TIMER_4_VECTOR, IPL2SOFT) T4Handler(void)
{
    IFS0CLR = _IFS0_T4IF_MASK;

    for (uint8_t id = 0; id < 2; id++) {

        int32_t dc;
        __builtin_disable_interrupts();
        dc = m[id].enc_delta;
        m[id].enc_delta = 0;
        __builtin_enable_interrupts();

        // rpm = (dc / edges_per_rev) / Ts * 60
        float rev_per_s = ((float)dc / (float)ENC_A_EDGES_PER_REV) / CTRL_TS_S;
        float rpm_f = rev_per_s * 60.0f;
        if (rpm_f < 0.0f) rpm_f = 0.0f;
        if (rpm_f > 65535.0f) rpm_f = 65535.0f;

        m[id].rpm_filt = alpha*m[id].rpm_filt + (1.0f-alpha)*rpm_f;
        m[id].meas_rpm = (uint16_t)(m[id].rpm_filt + 0.5f);
        
        if (m[id].cmd_rpm == 0) {
            m[id].integ = 0.0f;
            m[id].duty  = 0;
            PWM_SetDutyPercent(id, 0, m[id].reverse);
            continue;
        }
        
        // PI
        int32_t err = (int32_t)m[id].cmd_rpm - (int32_t)m[id].meas_rpm;

        float u = KP * (float)err + m[id].integ;

        if (u < DUTY_MIN) u = DUTY_MIN;
        if (u > DUTY_MAX) u = DUTY_MAX;

        uint8_t duty = (uint8_t)(u + 0.5f);
        m[id].duty = duty;
        PWM_SetDutyPercent(id, duty, m[id].reverse);

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

void MotorHAL_DriveEncoderCount(uint8_t id, uint16_t EncoderCounts){
    if (id > 1) return;
    __builtin_disable_interrupts();
    action_done_sent = false;
    m[id].start_count = m[id].enc_count;
    m[id].drive_count = EncoderCounts;
    m[id].is_driving_fixed_dis = true; 
    __builtin_enable_interrupts();
}


int32_t MotorHAL_GetStartCount(uint8_t id)
{
    if (id > 1) return 0;
    //uint32_t s = enter_crit();
    __builtin_disable_interrupts();
    int32_t s = m[id].start_count;
    __builtin_enable_interrupts();
    //exit_crit(s);
    return s;
}