#include "../ProjectHeaders/FollowerService.h"
#include "Flywheel_HAL.h"
#include "Servo_HAL.h"
#include "SPI1_CommHAL.h"
#include "LineFollowing_HAL.h"
// Hardware
#include <xc.h>
#include <sys/attribs.h>
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "ES_DeferRecall.h"
#include "ES_Port.h"
#include "terminal.h"
#include "dbprintf.h"
#include "ES_Events.h"
#include "ES_PostList.h"
#include "ES_ServiceHeaders.h"

/*----------------------------- Module Defines ----------------------------*/

#define Servo_MS      100



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

static volatile uint16_t turn_latest_LineFollowing = 100;
static volatile uint16_t turn_latest_LineFollowing_B = 100;
static volatile uint16_t turn_latest_BackCenter = 0;
static volatile uint16_t turn_latest_FrontCenter = 0;
static volatile uint16_t turn_latest_FrontLeft = 0;
volatile uint16_t cmd_pending = 0;
volatile bool cmd_pending_valid = false;


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
  SPI1BUF = turn_latest_LineFollowing;
  IFS1bits.SPI1RXIF = 0;
  IEC1bits.SPI1RXIE = 1;
  IPC7bits.SPI1IP = 4; 
  IPC7bits.SPI1IS = 0;
  
  Flywheel_Init();
  Flywheel_SetDuty(100);
  
  Init_LineFollowing();

  TRISBbits.TRISB9 = 0;
  MyPriority = Priority;
  
  // Servo Init
  Servo_Init();
  Servo_SetAngle(1, 90);
  Servo_SetAngle(0, 20);
  Servo_SetAngle(2, 180);
  Servo_SetAngle_Step(1, 90);
  Servo_SetAngle_Step(0, 20);
  Servo_SetAngle_Step(2, 180);
  Servo_SyncCurrentToOutput();
  
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
        ES_Timer_InitTimer(RETURN_TIMER, 10);
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
            //SPI1Follower_LoadTx16(10);
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
            if (ThisEvent.EventParam == RETURN_TIMER){
                ReadIRSensors();
                Servo_Angle_Step();
                if (is_T_F()){
                    turn_latest_LineFollowing = 0;
                }else{
                    float turn_F = LineFollowing_ComputeFrontTurn();
                    turn_latest_LineFollowing = (uint16_t)(turn_F + 100.0f);
                }
                if (is_T_B()){
                    turn_latest_LineFollowing_B = 0;
                }else{
                    float turn_B = LineFollowing_ComputeBackTurn();
                    turn_latest_LineFollowing_B = (uint16_t)(turn_B + 100.0f);
                }
                
                
                turn_latest_BackCenter = LineFollowing_GetBackCenter();
                turn_latest_FrontCenter = LineFollowing_GetFrontCenter();
                turn_latest_FrontLeft = LineFollowing_GetFrontLeft();
                ES_Timer_InitTimer(RETURN_TIMER, 10);
            }
            
        }
        break;
        case ES_NEW_KEY:
        {
            if (ThisEvent.EventParam == 'b'){
                
                ReadIRSensors();
                printf("%d %d %d %d %d %d\r\n",LineFollowing_GetFrontLeft(),LineFollowing_GetFrontCenter(),LineFollowing_GetFrontRight(),LineFollowing_GetBackLeft(),LineFollowing_GetBackCenter(),LineFollowing_GetBackRight());
                DB_printf("BackCenter is %d\r\n", turn_latest_BackCenter);
                float turn = LineFollowing_ComputeFrontTurn();
                float LineFollowing_result = turn + 100.0f;
                //DB_printf("Turn is %d\r\n", (int)LineFollowing_result);
                //PrintTurn(turn);
            } 
            if (ThisEvent.EventParam == 'l'){
                Servo_SetAngle_Step(0, 20);
            }
            if (ThisEvent.EventParam == 'r'){
                Servo_SetAngle_Step(0, 90);
            }
            if (ThisEvent.EventParam == 'f'){
                Flywheel_SetDuty(1);
                DB_printf("Cmd received\n");
            }
            if (ThisEvent.EventParam == 's'){
                Flywheel_SetDuty(100);
                //DB_printf("Cmd received\n");
            }
            if (ThisEvent.EventParam == 'a'){
                Servo_SetAngle_Step(2, 20);
            }
            if (ThisEvent.EventParam == 'c'){
                Servo_SetAngle_Step(2, 90);
            }
        }
        break;
        case ES_FLYWHEEL_ON:
        {
            Flywheel_SetDuty(1);
            DB_printf("Flywheel\n");
        }
        break;
        case ES_FLYWHEEL_OFF:
        {
            Flywheel_SetDuty(100);
            
        }
        break;
        case ES_SERVO0_Idel:
        {
            Servo_SetAngle_Step(0, 20);
        }
        break;
        case ES_SERVO0_Rise:
        {
            Servo_SetAngle_Step(0, 90);
        }
        break;
        case ES_SERVO1_Idel:
        {
            Servo_SetAngle(1, 0);
        }
        break;
        case ES_SERVO1_Rise:
        {
            Servo_SetAngle(1, 180);
        }
        break;
        case ES_SERVO2_Idel:
        {
            Servo_SetAngle_Step(2, 180);
        }
        break;
        case ES_SERVO2_Rise:
        {
            Servo_SetAngle_Step(2, 0);
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



/*
 * Cmd list:
 * 0x0011 - return last front line following reading
 * 0x0012 - return last back line following reading
 * 0x0013 - return last back center reading
 * 0x0014 - return last front left reading
 * 0x0015 - Flywheel start
 * 0x0016 - Flywheel stop
 * 0x0021 - Servo_Arm(Servo 0) idel
 * 0x0022 - Servo_Arm(Servo 0) rise
 * 0x0023 - Servo_Indicator (Servo 1) idel
 * 0x0024 - Servo_Indicator (Servo 1) rise
 * 0x0025 - Servo_Trapdoor(Servo 2) idel
 * 0x0026 - Servo_Trapdoor(Servo 2) rise
 
 */

void __ISR(_SPI1_VECTOR, IPL4SOFT) SPI1RxHandler(void)
{
    if (IFS1bits.SPI1RXIF == 0) {
        return;
    }
    IFS1bits.SPI1RXIF = 0;

    if (SPI1STATbits.SPIROV) SPI1STATCLR = (1u<<6);

    uint16_t cmd = 0;
    while (SPI1STATbits.SPIRBF) {
        cmd = (uint16_t)SPI1BUF;
    }
    if (cmd == 0x0011){
        SPI1BUF = turn_latest_LineFollowing;
    }else if (cmd == 0x0012){
        SPI1BUF = turn_latest_LineFollowing_B;
    }else if (cmd == 0x0013){
        SPI1BUF = turn_latest_BackCenter;
    }else if (cmd == 0x0014){
        SPI1BUF = turn_latest_FrontLeft;
    }else if (cmd == 0x0015){
        ES_Event_t ThisEvent;
        ThisEvent.EventType = ES_FLYWHEEL_ON;
        PostFollowerService(ThisEvent);
    }else if (cmd == 0x0016){
        ES_Event_t ThisEvent;
        ThisEvent.EventType = ES_FLYWHEEL_OFF;
        PostFollowerService(ThisEvent);
    }else if (cmd == 0x0021){
        ES_Event_t ThisEvent;
        ThisEvent.EventType = ES_SERVO0_Idel;
        PostFollowerService(ThisEvent);
    }else if(cmd == 0x0022){
        ES_Event_t ThisEvent;
        ThisEvent.EventType = ES_SERVO0_Rise;
        PostFollowerService(ThisEvent);
    }else if(cmd == 0x0023){
        ES_Event_t ThisEvent;
        ThisEvent.EventType = ES_SERVO1_Idel;
        PostFollowerService(ThisEvent);
    }else if(cmd == 0x0024){
        ES_Event_t ThisEvent;
        ThisEvent.EventType = ES_SERVO1_Rise;
        PostFollowerService(ThisEvent);
    }else if(cmd == 0x0025){
        ES_Event_t ThisEvent;
        ThisEvent.EventType = ES_SERVO2_Idel;
        PostFollowerService(ThisEvent);
    }else if(cmd == 0x0026){
        ES_Event_t ThisEvent;
        ThisEvent.EventType = ES_SERVO2_Rise;
        PostFollowerService(ThisEvent);
    }

//    if (cmd != 0x00011) {
//        cmd_pending = cmd;
//        cmd_pending_valid = true;
//    }
}
