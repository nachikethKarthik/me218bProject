#include "../ProjectHeaders/FollowerService.h"
#include "Flywheel_HAL.h"
#include "Servo_HAL.h"
#include "SPI1_CommHAL.h"
#include "LineFollowing_HAL.h"
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
static FollowerState_t CurrentState;

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
bool InitFollowerService(uint8_t Priority)
{
  DB_printf("Start Follower\n");
  ES_Event_t ThisEvent;
  
  SPI1Follower_Init();
  Servo_Init();
  //Flywheel_Init();
  Init_LineFollowing();
  //DB_printf("SS1R=%u SDI1R=%u\r\n", (unsigned)SS1R, (unsigned)SDI1R);
  
  TRISBbits.TRISB9 = 0;
  MyPriority = Priority;
  
//  DB_printf("SPI1CON=0x%08lx SPI1STAT=0x%08lx\n", SPI1CON, SPI1STAT);
//  DB_printf("MSTEN=%d SSEN=%d MODE16=%d CKP=%d CKE=%d\n",
//        SPI1CONbits.MSTEN, SPI1CONbits.SSEN, SPI1CONbits.MODE16,
//        SPI1CONbits.CKP, SPI1CONbits.CKE);
//  DB_printf("SS pin=%d\n", PORTBbits.RB4);
  
  
  
  
  // put us into the Initial PseudoState
  CurrentState = InitPState;
  // post the initial transition event
  ThisEvent.EventType = ES_INIT;
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
bool PostFollowerService(ES_Event_t ThisEvent)
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
ES_Event_t RunFollowerService(ES_Event_t ThisEvent)
{
  ES_Event_t ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT; // assume no errors

  switch (CurrentState)
  {
    case InitPState:        // If current state is initial Psedudo State
    {
      if (ThisEvent.EventType == ES_INIT)    // only respond to ES_Init
      {
        // this is where you would put any actions associated with the
        // transition from the initial pseudo-state into the actual
        // initial state

        // now put the machine into the actual initial state
        CurrentState = TestState;
      }
    }
    break;

    case TestState:        // If current state is state one
    {
      
      switch (ThisEvent.EventType)
      {
        case ES_COMMU:  //If event is event one
        {   // Execute action function for state one : event one
            SPI1Follower_LoadTx16(10);
            LATBbits.LATB9 = 1;
            DB_printf("Received\n");
            //Servo_SetAngle(1, 60);
            //Servo_SetPalseWidth(1, 6500);
            //Flywheel_SetDuty(100);
            ES_Timer_InitTimer(Follower_TIMER, 500);
        }
        break;

        case ES_TIMEOUT:
        {
            if (ThisEvent.EventParam == Follower_TIMER){
                LATBbits.LATB9 = 0;
                //Servo_SetAngle(1, 0);
                //Servo_SetPalseWidth(1, 1500);
                //ES_Timer_InitTimer(Follower_TIMER, 500);
            }
                
        }
          case ES_NEW_KEY:
            if (ThisEvent.EventParam == 'b'){
                //ReadIRSensors();
                //uint32_t bl = LineFollowing_GetBackLeft();
                //uint32_t bc = LineFollowing_GetBackCenter(); 
                //uint32_t br = LineFollowing_GetBackRight();
                
                //DB_printf("BackLeft = %d, BackCenter = %d, BackRight = %d\r\n", bl, bc, br);
                
                ReadIRSensors();
                //printf("%d %d %d %d %d %d\r\n",LineFollowing_GetFrontLeft(),LineFollowing_GetFrontCenter(),LineFollowing_GetFrontRight(),LineFollowing_GetBackLeft(),LineFollowing_GetBackCenter(),LineFollowing_GetBackRight());
                float turn = LineFollowing_ComputeFrontTurn();
                PrintTurn(turn);

            } 
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
FollowerState_t QueryFollowerService(void)
{
  return CurrentState;
}

/***************************************************************************
 private functions
 ***************************************************************************/
void PrintTurn(float turn)
{
    char sign = (turn < 0.0f) ? '-' : '+';
    float a = (turn < 0.0f) ? -turn : turn;

    uint16_t ip = (uint16_t)a;
    uint16_t fp = (uint16_t)((a - (float)ip) * 1000.0f + 0.5f);
    if (fp >= 1000) { fp -= 1000; ip += 1; }

    if (fp < 10) {
    DB_printf("turn is %c%u.00%u\r\n", sign, ip, fp);
    } else if (fp < 100) {
    DB_printf("turn is %c%u.0%u\r\n", sign, ip, fp);
    } else {
    DB_printf("turn is %c%u.%u\r\n", sign, ip, fp);
    }
}
