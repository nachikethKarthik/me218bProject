#ifndef LeaderSerice_H
#define LeaderSerice_H

// Event Definitions
#include <stdint.h>
#include <stdbool.h>

#include "ES_Events.h"
#include "ES_Port.h"      /* gets bool type for returns */

// typedefs for the states
// State definitions for use with the query function
typedef enum
{
    InitPState, 
    TestState, 
    Field_Determine_State_1, 
    LineFollowingState_1, 
    Field_Determine_State_2, 
    LineFollowingState_2, 
    LineFollowingState_3, 
    LineFollowingState_4,
    LineFollowingState_5,
    LineFollowingState_6, 
    LineFollowingState_7, 
    Seesaw1_State_1,
    Seesaw1_State_2,
    Seesaw1_State_3,
    CheckPoint3State_1, 
    CheckPoint3State_2
}LeaderState_t;

// Public Function Prototypes

bool InitLeaderService(uint8_t Priority);
bool PostLeaderService(ES_Event_t ThisEvent);
ES_Event_t RunLeaderService(ES_Event_t ThisEvent);
LeaderState_t QueryLeaderService(void);

#endif

