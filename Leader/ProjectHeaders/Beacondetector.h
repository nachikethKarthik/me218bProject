/****************************************************************************
 Module
   BeaconDetector.h

 Description
   Header file for BeaconDetector module using Input Capture for IR beacon
   frequency measurement on the PIC32MX170F256B.

 Notes
   Uses IC1 on RA2 (Pin 9) with Timer3 as timebase.
   
   Beacon Frequencies:
   - G: 3333 Hz (300 탎)  - Green dispenser
   - B: 1427 Hz (700 탎)  - Blue dispenser  
   - R: 909 Hz  (1100 탎) - Seesaw without line
   - L: 2000 Hz (500 탎)  - Seesaw without line

 Author
   karthi24
****************************************************************************/
#ifndef BEACON_DETECTOR_H
#define BEACON_DETECTOR_H

#include <stdint.h>
#include <stdbool.h>

// Beacon identifiers based on frequency
typedef enum {
    BEACON_NONE = 0,
    BEACON_R = 909,      // 909 Hz  - Seesaw without line
    BEACON_B = 1427,     // 1427 Hz - Blue dispenser
    BEACON_L = 2000,     // 2000 Hz - Seesaw without line
    BEACON_G = 3333      // 3333 Hz - Green dispenser
} BeaconID_t;

/****************************************************************************
 Function
    BeaconDetector_Init

 Parameters
    None

 Returns
    bool: true if initialization successful

 Description
    Initializes IC1 and Timer3 for beacon frequency detection.
****************************************************************************/
bool BeaconDetector_Init(void);

/****************************************************************************
 Function
    BeaconDetector_Arm

 Parameters
    None

 Returns
    None

 Description
    Arms the beacon detector to detect any beacon.
    Posts ES_BEACON_DETECTED with frequency as EventParam once detected.
    Auto-disarms after detection.
****************************************************************************/
void BeaconDetector_Arm(void);

/****************************************************************************
 Function
    BeaconDetector_ArmForTarget

 Parameters
    BeaconID_t target: The specific beacon to look for (or BEACON_NONE for any)

 Returns
    None

 Description
    Arms the beacon detector to only trigger on a specific beacon.
    Other beacons will be ignored. Auto-disarms after detection.
****************************************************************************/
void BeaconDetector_ArmForTarget(BeaconID_t target);

/****************************************************************************
 Function
    BeaconDetector_Disarm

 Parameters
    None

 Returns
    None

 Description
    Disarms the beacon detector. No events will be posted until re-armed.
****************************************************************************/
void BeaconDetector_Disarm(void);

/****************************************************************************
 Function
    BeaconDetector_IsArmed

 Parameters
    None

 Returns
    bool: true if detector is currently armed

 Description
    Returns the current armed state of the detector.
****************************************************************************/
bool BeaconDetector_IsArmed(void);

/****************************************************************************
 Function
    BeaconDetector_GetLastFrequency

 Parameters
    None

 Returns
    uint16_t: Last measured frequency in Hz (0 if none measured)

 Description
    Returns the last measured beacon frequency.
****************************************************************************/
uint16_t BeaconDetector_GetLastFrequency(void);

/****************************************************************************
 Function
    BeaconDetector_GetLastBeacon

 Parameters
    None

 Returns
    BeaconID_t: Last identified beacon

 Description
    Returns the last identified beacon type.
****************************************************************************/
BeaconID_t BeaconDetector_GetLastBeacon(void);

/****************************************************************************
 Function
    BeaconDetector_IdentifyBeacon

 Parameters
    uint16_t frequency: Measured frequency in Hz

 Returns
    BeaconID_t: The identified beacon, or BEACON_NONE if unrecognized

 Description
    Identifies which beacon corresponds to a given frequency.
    Allows +/- 10% tolerance.
****************************************************************************/
BeaconID_t BeaconDetector_IdentifyBeacon(uint16_t frequency);

#endif /* BEACON_DETECTOR_H */






