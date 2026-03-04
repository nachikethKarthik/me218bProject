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
    Seesaw2_State_1,
    Seesaw2_State_2,
    Seesaw2_State_3,
    Seesaw2_State_4,
    Seesaw2_State_5,
    Seesaw3_State_1,
    Seesaw3_State_2,
    Seesaw3_State_3,
    Seesaw3_State_4,
    Seesaw3_State_5,
    Seesaw3_State_6,
    Seesaw3_State_7,
    Seesaw3_State_8,
    Seesaw3_State_9,
    Seesaw3_State_10,
    Seesaw3_State_11,
    Seesaw1_State_2_1,
    Seesaw1_State_2_2,
    Seesaw2_State_4_1,
    Seesaw2_State_4_2,
    Seesaw2_State_4_3,
    Stop_State,  
    Push_Seesaw1_State_1,
    Push_Seesaw1_State_2,
    Seesaw4_State_1,
    Seesaw4_State_2,
    Seesaw4_State_3,
    Seesaw4_State_4,
    Seesaw4_State_5,
    Seesaw4_State_6,
    Seesaw4_State_7,
    Seesaw5_State_1,
    Seesaw5_State_2,
    Seesaw5_State_3
}LeaderState_t;

// Public Function Prototypes

bool InitLeaderService(uint8_t Priority);
bool PostLeaderService(ES_Event_t ThisEvent);
ES_Event_t RunLeaderService(ES_Event_t ThisEvent);
LeaderState_t QueryLeaderService(void);

#endif

