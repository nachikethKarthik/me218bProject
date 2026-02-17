/****************************************************************************
 Module
   EventCheckers.c

 Revision
   1.0.1

 Description
   This is the sample for writing event checkers along with the event
   checkers used in the basic framework test harness.

 Notes
   Note the use of static variables in sample event checker to detect
   ONLY transitions.

 History
 When           Who     What/Why
 -------------- ---     --------
 08/06/13 13:36 jec     initial version
****************************************************************************/




// this will pull in the symbolic definitions for events, which we will want
// to post in response to detecting events
#include "ES_Configure.h"
// This gets us the prototype for ES_PostAll
#include "ES_Framework.h"
// this will get us the structure definition for events, which we will need
// in order to post events in response to detecting events
#include "ES_Events.h"
// if you want to use distribution lists then you need those function
// definitions too.
#include "ES_PostList.h"
// This include will pull in all of the headers from the service modules
// providing the prototypes for all of the post functions
#include "ES_ServiceHeaders.h"
// this test harness for the framework references the serial routines that
// are defined in ES_Port.c
#include "ES_Port.h"
// include our own prototypes to insure consistency between header &
// actual functionsdefinition
#include "EventCheckers.h"

#include "dbprintf.h"
/*---------------------------- Module Variables ---------------------------*/
// Beacon detection parameters
// Beacon frequency is 1427 Hz, so period is ~700 us
#define BEACON_MIN_EDGES       5   // Minimum edges to detect beacon (1427 Hz * 0.01s * 2 edges/cycle = ~28)
// For beacon detection - track last state
static uint8_t LastBeaconState = 0;  // 0 = no beacon, 1 = beacon detected
static uint8_t LastRA2State = 0;     // For beacon detection
static uint32_t EdgeCount = 0;       // Count of edges seen
static bool BeaconArmed = false;

/****************************************************************************
 Function
   Check4Keystroke
 Parameters
   None
 Returns
   bool: true if a new key was detected & posted
 Description
   checks to see if a new key from the keyboard is detected and, if so,
   retrieves the key and posts an ES_NewKey event to TestHarnessService0
 Notes
   The functions that actually check the serial hardware for characters
   and retrieve them are assumed to be in ES_Port.c
   Since we always retrieve the keystroke when we detect it, thus clearing the
   hardware flag that indicates that a new key is ready this event checker
   will only generate events on the arrival of new characters, even though we
   do not internally keep track of the last keystroke that we retrieved.
 Author
   J. Edward Carryer, 08/06/13, 13:48
****************************************************************************/
bool Check4Keystroke(void)
{
  if (IsNewKeyReady())   // new key waiting?
  {
    ES_Event_t ThisEvent;
    ThisEvent.EventType   = ES_NEW_KEY;
    ThisEvent.EventParam  = GetNewKey();
    ES_PostAll(ThisEvent);
    return true;
  }
  return false;
}
/****************************************************************************
 Function
   Check4Beacon
 Parameters
   None
 Returns
   bool: true if beacon detection state changed and event was posted
 Description
   Monitors the digital input on RA2 (from comparator) for the 1427 Hz
   beacon signal. Counts edges and posts ES_BEACON_DETECTED
   when the number of edges counted crosses a threshold.
 
 Author
   karthi24, 02042026
****************************************************************************/
bool Check4Beacon(void)
{
    
    // If not armed, don't do anything
    if (!BeaconArmed)
    {
        return false;
    }
    
    
  bool ReturnVal = false;
  uint8_t CurrentRA2 = PORTAbits.RA2;
  
  // Count edges (rising or falling)
  if (CurrentRA2 != LastRA2State)
  {
    EdgeCount++;
  }
  LastRA2State = CurrentRA2;
  
  uint8_t CurrentBeaconState;
  
  // Determine if beacon is present based on edge count
  if (EdgeCount >= BEACON_MIN_EDGES)
  {
    CurrentBeaconState = 1;  // Beacon detected
  }
  else
  {
    CurrentBeaconState = 0;  // No beacon
  }
  
  // Check for transition: no beacon -> beacon detected
  if ((CurrentBeaconState != LastBeaconState) && (CurrentBeaconState == 1))
  {
    ES_Event_t ThisEvent;
    ThisEvent.EventType = ES_BEACON_DETECTED;
    ThisEvent.EventParam = (uint16_t)EdgeCount;
    ES_PostAll(ThisEvent);
    ReturnVal = true;
    EdgeCount = 0;
    BeaconArmed = false;  // Disarm after detection - only one event
    DB_printf("Beacon detected Event\n");
  }
  
  LastBeaconState = CurrentBeaconState; // actually this line is not necessary because the event checker is no longer armed
  
  return ReturnVal;
}
