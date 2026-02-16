#include "../ProjectHeaders/LeaderService.h"
#include "Motor_HAL.h"
#include "SPI1_CommHAL.h"
// Hardware
#include <xc.h>
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "ES_DeferRecall.h"
#include "ES_Port.h"
#include "terminal.h"
#include "dbprintf.h"
/*----------------------------- Module Defines ----------------------------*/

/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this machine.They should be functions
   relevant to the behavior of this state machine
*/

/*---------------------------- Module Variables ---------------------------*/
// everybody needs a state variable, you may need others as well.
// type of state variable should match htat of enum in header file
static LeaderState_t CurrentState;

// with the introduction of Gen2, we need a module level Priority var as well
static uint8_t MyPriority;

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitTemplateFSM

 Parameters
     uint8_t : the priorty of this service

 Returns
     bool, false if error in initialization, true otherwise

 Description
     Saves away the priority, sets up the initial transition and does any
     other required initialization for this state machine
 Notes

 Author
     J. Edward Carryer, 10/23/11, 18:55
****************************************************************************/
bool InitLeaderService(uint8_t Priority)
{
  ES_Event_t ThisEvent;
  DB_printf("Start!\n");
  MyPriority = Priority;
  // put us into the Initial PseudoState
  CurrentState = InitPState;
  // post the initial transition event
  ThisEvent.EventType = ES_INIT;
  
  //while(SPI1Leader_Init()){  
  //}
  SPI1Leader_Init();
  
  
  
//  DB_printf("Leader: ON=%u MSTEN=%u MSSEN=%u CKP=%u CKE=%u MODE16=%u\r\n",
//          (unsigned)SPI1CONbits.ON,
//          (unsigned)SPI1CONbits.MSTEN,
//          (unsigned)SPI1CONbits.MSSEN,
//          (unsigned)SPI1CONbits.CKP,
//          (unsigned)SPI1CONbits.CKE,
//          (unsigned)SPI1CONbits.MODE16);
//
//    DB_printf("Leader: SS(RB4)=%u\r\n", (unsigned)PORTBbits.RB4);
//  
  
  //DB_printf("%d",IFSUC);
  
  if (ES_PostToService(MyPriority, ThisEvent) == true)
  {
    return true;
  }
  else
  {
    return false;
  }
}

/****************************************************************************
 Function
     PostTemplateFSM

 Parameters
     EF_Event_t ThisEvent , the event to post to the queue

 Returns
     boolean False if the Enqueue operation failed, True otherwise

 Description
     Posts an event to this state machine's queue
 Notes

 Author
     J. Edward Carryer, 10/23/11, 19:25
****************************************************************************/
bool PostLeaderService(ES_Event_t ThisEvent)
{
  return ES_PostToService(MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RunTemplateFSM

 Parameters
   ES_Event_t : the event to process

 Returns
   ES_Event_t, ES_NO_EVENT if no error ES_ERROR otherwise

 Description
   add your description here
 Notes
   uses nested switch/case to implement the machine.
 Author
   J. Edward Carryer, 01/15/12, 15:23
****************************************************************************/
ES_Event_t RunLeaderService(ES_Event_t ThisEvent)
{
  ES_Event_t ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT; // assume no errors

  switch (CurrentState)
  {
    case InitPState:        // If current state is initial Psedudo State
    {
      if (ThisEvent.EventType == ES_INIT)    // only respond to ES_Init
      {

        CurrentState = TestState;
      }
    }
    break;

    case TestState:        // If current state is state one
    {
      switch (ThisEvent.EventType)
      {
        case ES_NEW_KEY:  //If event is event one
        {   // Execute action function for state one : event one
            //DB_printf("%c\n",(char)ThisEvent.EventParam);
            char key = (char)ThisEvent.EventParam;
            //DB_printf("%c", key);
            
            if (key == 's'){
                //DB_printf("s pressed\n");
                //uint16_t resp = SPI1Leader_RequestResponse16(0x0001);
                //SPI1Leader_SendCmd16(0x0001);
                //DB_printf("%d",resp);
                uint16_t R = SPI1Leader_RequestResponse16(0x0001);
                DB_printf("%d\n",R);
            }
            
        }
        break;

        // repeat cases as required for relevant events
        default:
          ;
      }  // end switch on CurrentEvent
    }
    break;
    // repeat state pattern as required for other states
    default:
      ;
  }                                   // end switch on Current State
  return ReturnEvent;
}

/****************************************************************************
 Function
     QueryTemplateSM

 Parameters
     None

 Returns
     TemplateState_t The current state of the Template state machine

 Description
     returns the current state of the Template state machine
 Notes

 Author
     J. Edward Carryer, 10/23/11, 19:21
****************************************************************************/
LeaderState_t QueryLeaderService(void)
{
  return CurrentState;
}

/***************************************************************************
 private functions
 ***************************************************************************/

