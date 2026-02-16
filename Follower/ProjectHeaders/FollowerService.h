/****************************************************************************

  Header file for template Flat Sate Machine
  based on the Gen2 Events and Services Framework

 ****************************************************************************/

#ifndef FollowerService_H
#define FollowerService_H

#include <stdint.h>
#include <stdbool.h>

#include "ES_Events.h"
#include "ES_Port.h"      /* gets bool type for returns */

// typedefs for the states
// State definitions for use with the query function
typedef enum
{
  InitPState, TestState
}FollowerState_t;

// Public Function Prototypes

bool InitFollowerService(uint8_t Priority);
bool PostFollowerService(ES_Event_t ThisEvent);
ES_Event_t RunFollowerService(ES_Event_t ThisEvent);
FollowerState_t QueryFollowerService(void);

#endif /* FSMTemplate_H */

