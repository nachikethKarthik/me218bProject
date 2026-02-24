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
  InitPState, TestState, LineFollowingState, CheckPoint3State_1, CheckPoint3State_2
}LeaderState_t;

// Public Function Prototypes

bool InitLeaderService(uint8_t Priority);
bool PostLeaderService(ES_Event_t ThisEvent);
ES_Event_t RunLeaderService(ES_Event_t ThisEvent);
LeaderState_t QueryLeaderService(void);

#endif

