/****************************************************************************
 Module
   BeaconDetector.c

 Description
   Implementation of beacon detection using Input Capture (IC1) with Timer3.
   Measures the period of the incoming IR beacon square wave and posts
   ES_BEACON_DETECTED event with the measured frequency.

 Notes
   Pin Configuration (from pinMapping.pdf):
   - IC1 on RA2 (Pin 9), IC1R = 0b0000
   - Timer3 as IC timebase (with 1:8 prescaler)
   
   Beacon Frequencies to detect:
   - G: 3333 Hz (300 탎 period)  -> 750 ticks at 2.5 MHz
   - L: 2000 Hz (500 탎 period)  -> 1250 ticks
   - B: 1427 Hz (700 탎 period)  -> 1750 ticks
   - R: 909 Hz  (1100 탎 period) -> 2750 ticks

 Author
   karthi24
****************************************************************************/

#include "BeaconDetector.h"
#include <xc.h>
#include <sys/attribs.h>
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "ES_Events.h"
#include "dbprintf.h"
#include "LeaderService.h"

#define PBCLK_HZ            20000000UL   // Peripheral bus clock

// Timer3 configuration - 1:8 prescaler for good resolution with less overhead
// Timer clock = 20 MHz / 8 = 2.5 MHz (400 ns per tick)
// Overflow period = 65536 * 400 ns = 26.2 ms
#define T3_PRESCALE         8UL
#define T3_PRESCALE_BITS    0b011        // 1:8 prescaler

// Number of consistent period measurements required before posting event
#define REQUIRED_CONSISTENT_PERIODS  3

// Tolerance for period consistency check, once REQUIRED_CONSISTENT_PERIODS periods are measured, average is calculated and then 
//we check if each period is within avg +- PERIOD_TOLERANCE_PERCENT*avg
#define PERIOD_TOLERANCE_PERCENT     15

// Minimum and maximum valid periods (in timer ticks at 2.5 MHz)
// Beacon G: 3333 Hz -> 300 탎 -> 750 ticks
// Beacon R: 909 Hz  -> 1100 탎 -> 2750 ticks
#define MIN_VALID_PERIOD    600U     // ~4167 Hz max
#define MAX_VALID_PERIOD    3500U    // ~714 Hz min

// Timeout: if no edges for this many Timer3 overflows, reset detection
// Timer3 overflows every 26.2 ms at 1:8 prescaler
//Without the reset, the old stale period from previous beacons could get mixed with new periods from the new beacon, giving garbage results.

#define OVERFLOW_TIMEOUT    2

/*---------------------------- Module Variables ---------------------------*/
// s_ : static variable
static volatile bool s_armed = false;
static volatile BeaconID_t s_targetBeacon = BEACON_NONE;

static volatile uint16_t s_lastCapture = 0;
static volatile uint16_t s_periods[REQUIRED_CONSISTENT_PERIODS];
static volatile uint8_t s_periodIndex = 0;
static volatile uint8_t s_validPeriodCount = 0;

static volatile uint16_t s_lastFrequency = 0;
static volatile BeaconID_t s_lastBeacon = BEACON_NONE;

static volatile uint8_t s_overflowCount = 0;
static volatile bool s_firstEdge = true;

/*---------------------------- Module Functions ---------------------------*/
static bool PeriodsAreConsistent(void);
static uint16_t CalculateAverageFrequency(void);
static void ResetDetectionState(void);

/****************************************************************************
 Function
    PeriodsAreConsistent
****************************************************************************/
static bool PeriodsAreConsistent(void)
{
    if (s_validPeriodCount < REQUIRED_CONSISTENT_PERIODS) {
        return false;
    } // Only enter the function if you have collected all the periods
    
    // Calculate average period
    uint32_t sum = 0;
    for (uint8_t i = 0; i < REQUIRED_CONSISTENT_PERIODS; i++) {
        sum += s_periods[i];
    }
    uint16_t avgPeriod = (uint16_t)(sum / REQUIRED_CONSISTENT_PERIODS);
    
    // Check all periods within tolerance
    uint16_t tolerance = (avgPeriod * PERIOD_TOLERANCE_PERCENT) / 100;
    if (tolerance < 50) tolerance = 50; // Maximum tolerance allowed is 50 ticks
    
    for (uint8_t i = 0; i < REQUIRED_CONSISTENT_PERIODS; i++) {
        int32_t diff = (int32_t)s_periods[i] - (int32_t)avgPeriod;
        if (diff < 0) diff = -diff;
        
        if ((uint16_t)diff > tolerance) {
            return false;
        }
    }
    
    return true;
}

/****************************************************************************
 Function
    CalculateAverageFrequency
****************************************************************************/
static uint16_t CalculateAverageFrequency(void)
{
    uint32_t sum = 0;
    for (uint8_t i = 0; i < REQUIRED_CONSISTENT_PERIODS; i++) {
        sum += s_periods[i];
    }
    uint16_t avgPeriod = (uint16_t)(sum / REQUIRED_CONSISTENT_PERIODS);
    
    if (avgPeriod == 0) return 0;
    
    // freq = (PBCLK / prescaler) / period = 2,500,000 / avgPeriod
    uint32_t frequency = (PBCLK_HZ / T3_PRESCALE) / avgPeriod;
    
    if (frequency > 65535) frequency = 65535; // rail to maximum
    
    return (uint16_t)frequency;
}

/****************************************************************************
 Function
    ResetDetectionState
****************************************************************************/
static void ResetDetectionState(void)
{
    s_firstEdge = true;
    s_periodIndex = 0;
    s_validPeriodCount = 0;
    s_overflowCount = 0;
}

/****************************************************************************
 Function
    BeaconDetector_Init
****************************************************************************/
bool BeaconDetector_Init(void)
{
    // Configure RA2 as digital input
    TRISAbits.TRISA2 = 1;
    
    // Map IC1 to RA2 (Pin 9)
    IC1R = 0b0000;
    
    // ---- Configure Timer3 as timebase for IC1 with 1:8 prescaler----
    T3CONbits.ON = 0;
    T3CONbits.TCS = 0;                      // Internal peripheral clock
    T3CONbits.TCKPS = T3_PRESCALE_BITS;     // 1:8 prescaler
    TMR3 = 0;
    PR3 = 0xFFFF;                           // Free-running
    
    IFS0CLR = _IFS0_T3IF_MASK;
    
    // Timer3 overflow interrupt
    IPC3bits.T3IP = 3; // priority
    IPC3bits.T3IS = 0; //sub-priority
    
    IEC0SET = _IEC0_T3IE_MASK;
    
    
    T3CONbits.ON = 1;
    
    // ---- Configure IC1 ----
    IC1CONbits.ON = 0;
    IC1CONbits.ICTMR = 0;       // Timer3 as time base
    IC1CONbits.C32 = 0;         // 16-bit capture
    IC1CONbits.ICI = 0b00;      // Interrupt on every capture
    IC1CONbits.FEDGE = 1;       // First capture on rising edge
    IC1CONbits.ICM = 0b011;     // Capture on every rising edge
    
    // IC1 interrupt
    IPC1bits.IC1IP = 5; // priority
    IPC1bits.IC1IS = 0; // subpriority
    
    IFS0bits.IC1IF = 0;
    
    IFS0CLR = _IFS0_IC1IF_MASK;
    
    IEC0SET = _IEC0_IC1IE_MASK;
    
    IC1CONbits.ON = 1;
    
    // Initialize state
    s_armed = false;
    s_targetBeacon = BEACON_NONE;
    ResetDetectionState();
    
    DB_printf("BeaconDetector initialized (IC1 on RA2, Timer3 @ 2.5MHz)\n");
    
    return true;
}

/****************************************************************************
 Function
    BeaconDetector_ArmForTarget
****************************************************************************/
void BeaconDetector_ArmForTarget(BeaconID_t target)
{
    __builtin_disable_interrupts();
    
    s_targetBeacon = target;
    ResetDetectionState();
    
    // Clear any pending captures in the FIFO
    while (IC1CONbits.ICBNE) {
        volatile uint16_t dummy = IC1BUF;
        (void)dummy;
    }
    
    IFS0CLR = _IFS0_IC1IF_MASK;
    
    s_armed = true;
    
    __builtin_enable_interrupts();
    
    if (target == BEACON_NONE) {
        DB_printf("Beacon detector armed for ANY beacon\n");
    } else {
        DB_printf("Beacon detector armed for %u Hz\n", (uint16_t)target);
    }
}

/****************************************************************************
 Function
    BeaconDetector_Arm
****************************************************************************/
void BeaconDetector_Arm(void)
{
    BeaconDetector_ArmForTarget(BEACON_NONE);
}

/****************************************************************************
 Function
    BeaconDetector_Disarm
****************************************************************************/
void BeaconDetector_Disarm(void)
{
    s_armed = false;
}

/****************************************************************************
 Function
    BeaconDetector_IsArmed
****************************************************************************/
bool BeaconDetector_IsArmed(void)
{
    return s_armed;
}

/****************************************************************************
 Function
    BeaconDetector_GetLastFrequency
****************************************************************************/
uint16_t BeaconDetector_GetLastFrequency(void)
{
    return s_lastFrequency;
}

/****************************************************************************
 Function
    BeaconDetector_GetLastBeacon
****************************************************************************/
BeaconID_t BeaconDetector_GetLastBeacon(void)
{
    return s_lastBeacon;
}

/****************************************************************************
 Function
    BeaconDetector_IdentifyBeacon
****************************************************************************/
BeaconID_t BeaconDetector_IdentifyBeacon(uint16_t frequency)
{
    // R: 909 Hz (�10%)
    if (frequency >= 818 && frequency <= 1000) {
        return BEACON_R;
    }
    // B: 1427 Hz (�10%)
    if (frequency >= 1284 && frequency <= 1570) {
        return BEACON_B;
    }
    // L: 2000 Hz (�10%)
    if (frequency >= 1800 && frequency <= 2200) {
        return BEACON_L;
    }
    // G: 3333 Hz (�10%)
    if (frequency >= 3000 && frequency <= 3666) {
        return BEACON_G;
    }
    
    return BEACON_NONE;
}

/****************************************************************************
 Function
    IC1 ISR
****************************************************************************/
void __ISR(_INPUT_CAPTURE_1_VECTOR, IPL5SOFT) IC1Handler(void)
{
    IFS0CLR = _IFS0_IC1IF_MASK;
    while (IC1CONbits.ICBNE) {
        uint16_t currentCapture = IC1BUF; // read the buffer
        
        if (!s_armed) {
            continue; // skip all the code if not armed
        }
        
        s_overflowCount = 0; // Count of how many times the timer overflows when there is no signal. If a signal is detected, this is reset to 0
        
        if (s_firstEdge) {
            s_lastCapture = currentCapture;
            s_firstEdge = false;
            continue; // we need two edges to calculate a period, so forget this iteration of the while loop
        }
        
        // Calculate period (handles wraparound)
        uint16_t period = currentCapture - s_lastCapture;
        s_lastCapture = currentCapture;
        
        // Validate period simple min and max bound for all 4 possible periods
        if (period < MIN_VALID_PERIOD || period > MAX_VALID_PERIOD) {
            s_periodIndex = 0;
            s_validPeriodCount = 0;
            continue;
        }
        
        // Store period
        s_periods[s_periodIndex] = period;
        s_periodIndex = (s_periodIndex + 1) % REQUIRED_CONSISTENT_PERIODS;
        
        if (s_validPeriodCount < REQUIRED_CONSISTENT_PERIODS) {
            s_validPeriodCount++;
        }
        
        // Check for valid detection
        if (s_validPeriodCount >= REQUIRED_CONSISTENT_PERIODS && PeriodsAreConsistent()) {
            uint16_t frequency = CalculateAverageFrequency();
            BeaconID_t detectedBeacon = BeaconDetector_IdentifyBeacon(frequency);
            
            // Check target filter
            if (s_targetBeacon != BEACON_NONE && detectedBeacon != s_targetBeacon) {
                continue;
            }
            
            // Store results
            s_lastFrequency = frequency;
            s_lastBeacon = detectedBeacon;
            
            // Post event and disarm
            ES_Event_t ThisEvent;
            ThisEvent.EventType = ES_BEACON_DETECTED;
            ThisEvent.EventParam = frequency;
            PostLeaderService(ThisEvent);
            
            s_armed = false;
        }
    }
    

}