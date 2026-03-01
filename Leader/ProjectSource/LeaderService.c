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
#include "EventCheckers.h"
#include "Beacondetector.h"
#include "RGB_HAL.h"
/*----------------------------- Module Defines ----------------------------*/

#define LineFollowing_MS     50

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
static bool BeaconGDetected = false;
static uint16_t base_speed;

static bool FrontFollowing = false;
static bool BackFollowing = false;

static bool FrontT = false;

static bool is_Forward_Timer_on = true;
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
  DB_printf("Start Leader!\n");
  MyPriority = Priority;
  CurrentState = InitPState;
  ThisEvent.EventType = ES_INIT;
  
  SPI1Leader_Init();
  MotorHAL_Init();
  BeaconDetector_Init();
  RGB_Init();
  RGB_TurnCyan();
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
          
          
//          CurrentState = LineFollowingState_1;
//          DB_printf("Start Line 1\n");
//          MotorHAL_SetSpeedCmdRPM(0, 40, 0);
//          MotorHAL_SetSpeedCmdRPM(1, 40, 0);
//          base_speed = 40;
//          FrontFollowing = true;
//          FrontT = true;
//          ES_Timer_InitTimer(LINEFOLLOWING_TIMER, LineFollowing_MS);
          
//        MotorHAL_SetSpeedCmdRPM(0, 20, 1);
//        MotorHAL_SetSpeedCmdRPM(1, 20, 0);
//        CurrentState = Field_Determine_State_1;
//        BeaconDetector_Arm();
        
        
      }
    }
    break;

    //Encoder 1020 is about 360 degree rotation
    
    
    case TestState:        // If current state is state one
    {
        switch (ThisEvent.EventType)
        {
            case ES_NEW_KEY:    
            {  
                char key = (char)ThisEvent.EventParam;
                switch (key)
                {
                    case 'A':
                    {
                        DB_printf("B pressed, beacon detector armed (any beacon)\n");
                        BeaconDetector_Arm();
                    }
                    break;

                    case 'L':
                    {
                        DB_printf("L pressed, looking for beacon L only\n");
                        BeaconDetector_ArmForTarget(BEACON_L);
                    }
                    break;

                    case 'R':
                    {
                        DB_printf("L pressed, looking for beacon R only\n");
                        BeaconDetector_ArmForTarget(BEACON_R);
                    }
                    break;

                    case 'G':
                    {
                        DB_printf("L pressed, looking for beacon G only\n");
                        BeaconDetector_ArmForTarget(BEACON_G);
                    }
                    break;

                    case 'B':
                    {
                        DB_printf("L pressed, looking for beacon B only\n");
                        BeaconDetector_ArmForTarget(BEACON_B);
                    }
                    break;

                    case 'D':
                    {
                        DB_printf("D pressed, beacon detector disarmed\n");
                        BeaconDetector_Disarm();
                    }
                    break;

                    case 's':
                    {
                        //DB_printf("s pressed\n");
                        uint16_t R = SPI1Leader_RequestResponse16(0x0001);
                        DB_printf("%d\n",R);
                    }
                    break;

                    case 't':
                    {
                        uint16_t m0 = MotorHAL_GetSpeedMeasRPM(0);
                        uint16_t m1 = MotorHAL_GetSpeedMeasRPM(1);
                        uint8_t duty0 = MotorHAL_GetDutyOut(0);
                        uint8_t duty1 = MotorHAL_GetDutyOut(1);
                        DB_printf("M0: %u RPM, Duty0: %u%% | M1: %u RPM, Duty1: %u%% \n", m0, duty0, m1, duty1);
                    }
                    break;

                    case 'l':       
                    {
                        // left motor forward
                        DB_printf("l pressed, left motor forward\n");
                        MotorHAL_SetSpeedCmdRPM(0, 20, 0);
                        MotorHAL_DriveEncoderCount(0, 600);
                        MotorHAL_SetSpeedCmdRPM(1, 0, 0);
                    }
                    break;

                    case 'r':       
                    {
                        // right motor forward
                        DB_printf("r pressed, right motor forward\n");
                        MotorHAL_SetSpeedCmdRPM(1, 30, 0);
                        MotorHAL_SetSpeedCmdRPM(0, 0, 0);
                    }
                    break;

                    case 'b':       
                    {
                        // left motor backward
                        DB_printf("b pressed, left motor backward\n");
                        MotorHAL_SetSpeedCmdRPM(0, 80, 1l);
                        MotorHAL_SetSpeedCmdRPM(1, 0, 1);
                    }
                    break;

                    case 'n':       
                    {
                        // right motor backward
                        DB_printf("n pressed, right motor backward\n");
                        MotorHAL_SetSpeedCmdRPM(1, 30, 1);
                        MotorHAL_SetSpeedCmdRPM(0, 0, 1);
                    }
                    break;

                    case 'p':
                    {
                        DB_printf("p pressed, stop all motors\n");
                        MotorHAL_SetSpeedCmdRPM(0, 0, 0);
                        MotorHAL_SetSpeedCmdRPM(1, 0, 0);
                    }
                    break;  

                    case 'e':
                    {
                        int32_t start_count = MotorHAL_GetStartCount(0);
                        int32_t enc_count = MotorHAL_GetEncoderCount(0);
                        DB_printf("enc_count = %d, start count = %d\n", enc_count, start_count);
                    }
                    break; 
                    
                    case 'a':
                    {
                        //RGB_TurnGreen();
                        SPI1Leader_SendCmd16(0x0015);
                        DB_printf("Flywheel\n");
                    }
                    break;
                    case 'q':
                    {
                        SPI1Leader_SendCmd16(0x0023);
                    }
                    break;
                    case 'z':
                    {
                        SPI1Leader_SendCmd16(0x0024);
                    }
                    break;
                    case 'm':
                    {
                        RGB_TurnOff();
                    }
                    break;
                    case 'f':
                    {
//                        MotorHAL_DriveEncoderCount(0, 260);
//                        MotorHAL_DriveEncoderCount(1, 260);
                        MotorHAL_DriveEncoderCount(0, 1020);
                        MotorHAL_DriveEncoderCount(1, 1020);
                        MotorHAL_SetSpeedCmdRPM(0, 30, 1);
                        MotorHAL_SetSpeedCmdRPM(1, 30, 0);
                    }
                    break;
                }
            }
            break;
            case ES_BEACON_DETECTED:
            {
                uint16_t R = SPI1Leader_RequestResponse16(0x0001);
                DB_printf("%d\n",ThisEvent.EventParam);
            }
            break;  
            
            case ES_ACTION_DONE:
            {
                DB_printf("Action done!\r\n");
            }
            break;
            // repeat cases as required for relevant events
            default:
            ;
        }  // end switch on CurrentEvent
    }
    break;
    
    case Field_Determine_State_1:
    {
        switch (ThisEvent.EventType)
        {
            case ES_BEACON_DETECTED:
            {
                if (ThisEvent.EventParam >= 818 && ThisEvent.EventParam <= 1000){ // R detected
                    //SPI1Leader_SendCmd16(0x0023);
                    uint16_t R = (uint16_t)SPI1Leader_RequestResponse16(0x0023);
                    RGB_TurnBlue();
                    
                    MotorHAL_DriveEncoderCount(0, 220);
                    MotorHAL_DriveEncoderCount(1, 220);
                    MotorHAL_SetSpeedCmdRPM(0, 20, 0);
                    MotorHAL_SetSpeedCmdRPM(1, 20, 1);
                }else if (ThisEvent.EventParam >= 1800 && ThisEvent.EventParam <= 2200){
                    //SPI1Leader_SendCmd16(0x0024);
                    uint16_t R = (uint16_t)SPI1Leader_RequestResponse16(0x0024);
                    RGB_TurnGreen();
                    
                    MotorHAL_DriveEncoderCount(0, 220);
                    MotorHAL_DriveEncoderCount(1, 220);
                    MotorHAL_SetSpeedCmdRPM(0, 20, 0);
                    MotorHAL_SetSpeedCmdRPM(1, 20, 1);
                }else{
                    BeaconDetector_Arm();
                }
            }
            break;
            
            case ES_ACTION_DONE:
            {
                CurrentState = Field_Determine_State_2;
                MotorHAL_DriveEncoderCount(0, 50);
                MotorHAL_DriveEncoderCount(1, 50);
                MotorHAL_SetSpeedCmdRPM(0, 20, 0);
                MotorHAL_SetSpeedCmdRPM(1, 20, 0);
            }
            break;
        }
    }
    break;
    
    case Field_Determine_State_2:
    {
        switch (ThisEvent.EventType)
        {
            case ES_ACTION_DONE:
            {
                CurrentState = LineFollowingState_1;
                ES_Timer_InitTimer(LINEFOLLOWING_TIMER, LineFollowing_MS);
                MotorHAL_SetSpeedCmdRPM(0, 40, 0);
                MotorHAL_SetSpeedCmdRPM(1, 40, 0);
                base_speed = 40;
                FrontFollowing = true;
                FrontT = true;
            }
            break;
        }
    }
    break;
    
    
    
    case LineFollowingState_1:
    {
        
        switch (ThisEvent.EventType)
        {
            case ES_TIMEOUT:
            {
                if (ThisEvent.EventParam == LINEFOLLOWING_TIMER){
                    uint16_t R = SPI1Leader_RequestResponse16(0x0011);
                    
                    if ((R == 0) && FrontT){
                        FrontT = false;
                        FrontFollowing = false;
                        MotorHAL_SetSpeedCmdRPM(0, 30, 0);
                        MotorHAL_SetSpeedCmdRPM(1, 30, 0);
                        MotorHAL_DriveEncoderCount(0, 40);
                        MotorHAL_DriveEncoderCount(1, 40);
                    }else if (FrontFollowing){
                        MotorHAL_SetSpeedCmdRPM(0, base_speed + R - 100UL, 0);
                        MotorHAL_SetSpeedCmdRPM(1, base_speed - R + 100UL, 0);
                        //DB_printf("%d\n",R);
                        ES_Timer_InitTimer(LINEFOLLOWING_TIMER, LineFollowing_MS);
                    }
                }
            }
            break;
            
            case ES_ACTION_DONE:
            {
                DB_printf("start rotate 90\n");
                CurrentState = LineFollowingState_2;
                MotorHAL_SetSpeedCmdRPM(0, 30, 1);
                MotorHAL_SetSpeedCmdRPM(1, 30, 0);
                MotorHAL_DriveEncoderCount(0, 255);
                MotorHAL_DriveEncoderCount(1, 255);
                is_Forward_Timer_on = true;
                //CurrentState = LineFollowingState_2;
                
            }
            break;
        }
    }   
    break;
    
    case LineFollowingState_2: // After drive forward, rotate 180
    {
        switch (ThisEvent.EventType)
        {
            case ES_TIMEOUT:
            {
//                if (ThisEvent.EventParam == FORWARD_TIMER){
//                    
//                    // 2 seconds are up: stop line-follow updates before starting the 180 turn
//                    //ES_Timer_StopTimer(LINEFOLLOWING_TIMER);
//
//                    DB_printf("Go to state3!\n");
//                    is_Forward_Timer_on = true;
//                    MotorHAL_DriveEncoderCount(0, 510);
//                    MotorHAL_DriveEncoderCount(1, 510);
//                    MotorHAL_SetSpeedCmdRPM(0, 30, 1);
//                    MotorHAL_SetSpeedCmdRPM(1, 30, 0);
//                    CurrentState = LineFollowingState_3;
//                }
                if (ThisEvent.EventParam == LINEFOLLOWING_TIMER){
                    
                    uint16_t R = (uint16_t)SPI1Leader_RequestResponse16(0x0011);
                    if ((R == 0)){
                        ES_Timer_InitTimer(LINEFOLLOWING_TIMER, LineFollowing_MS);
                    }else{
                        MotorHAL_SetSpeedCmdRPM(0, base_speed + R - 100UL, 0);
                        MotorHAL_SetSpeedCmdRPM(1, base_speed - R + 100UL, 0);
                        ES_Timer_InitTimer(LINEFOLLOWING_TIMER, LineFollowing_MS);
                    }
                }
            }
            break;
            case ES_ACTION_DONE:
            {
                DB_printf("finish rotate 90, start forward\n");
                MotorHAL_SetSpeedCmdRPM(0, 40, 0);
                MotorHAL_SetSpeedCmdRPM(1, 40, 0);
                //MotorHAL_DriveEncoderCount(0, 400);
                //MotorHAL_DriveEncoderCount(1, 400);
                CurrentState = LineFollowingState_3;
                //ES_Timer_InitTimer(FORWARD_TIMER, 2000);
                ES_Timer_InitTimer(LINEFOLLOWING_TIMER, LineFollowing_MS);
            }
            break;
        }
    }
    break;
    
    case LineFollowingState_3:
    {
        switch (ThisEvent.EventType)
        {
            case ES_TIMEOUT:
            {
//                if (ThisEvent.EventParam == FORWARD_TIMER){
//                    CurrentState = LineFollowingState_4;
//                    DB_printf("Start Line 4\n");
//                    MotorHAL_SetSpeedCmdRPM(0, 0, 0);
//                    MotorHAL_SetSpeedCmdRPM(1, 0, 0);
//                }
                if (ThisEvent.EventParam == LINEFOLLOWING_TIMER){
                    
                    
                    uint16_t R = (uint16_t)SPI1Leader_RequestResponse16(0x0011);
                    if ((R == 0)){
                        //ES_Timer_InitTimer(LINEFOLLOWING_TIMER, LineFollowing_MS);
                        ES_Event_t NewEvent;
                        NewEvent.EventType = ES_T_DETECTED;
                        ES_PostAll(NewEvent);
                        
                    }else{
                        MotorHAL_SetSpeedCmdRPM(0, base_speed + R - 100UL, 0);
                        MotorHAL_SetSpeedCmdRPM(1, base_speed - R + 100UL, 0);
                        ES_Timer_InitTimer(LINEFOLLOWING_TIMER, LineFollowing_MS);
                    }
                }
//                if (ThisEvent.EventParam == ROTATION_TIMER){
//                    ES_Timer_InitTimer(FORWARD_TIMER, 5000);
//                    ES_Timer_InitTimer(LINEFOLLOWING_TIMER, LineFollowing_MS);
//                    MotorHAL_SetSpeedCmdRPM(0, 30, 0);
//                    MotorHAL_SetSpeedCmdRPM(1, 30, 0);
//                }
            }
            break;
            case ES_T_DETECTED:
            {
                ES_Timer_StopTimer(LINEFOLLOWING_TIMER);
                DB_printf("Finish forward, start rotate 180\n");
                MotorHAL_DriveEncoderCount(0, 510);
                MotorHAL_DriveEncoderCount(1, 510);
                MotorHAL_SetSpeedCmdRPM(0, 30, 1);
                MotorHAL_SetSpeedCmdRPM(1, 30, 0);
                CurrentState = LineFollowingState_4;
            }
            break;           
        }
    }
    break;
    
    case LineFollowingState_4:
    {
        switch (ThisEvent.EventType)
        {
            case ES_ACTION_DONE:
            {
                //ES_Timer_InitTimer(FORWARD_TIMER, 5000);
                DB_printf("Finish rotate 180, start forward\n");
                SPI1Leader_SendCmd16(0x0015);
                //uint16_t R = SPI1Leader_RequestResponse16(0x0015);
                ES_Timer_StartTimer(LINEFOLLOWING_TIMER);
                ES_Timer_InitTimer(LINEFOLLOWING_TIMER, LineFollowing_MS);
                MotorHAL_DriveEncoderCount(0, 550);
                MotorHAL_DriveEncoderCount(1, 550);
                MotorHAL_SetSpeedCmdRPM(0, 30, 0);
                MotorHAL_SetSpeedCmdRPM(1, 30, 0);
                base_speed = 30;
                CurrentState = LineFollowingState_5;
            }
            break;
            case ES_TIMEOUT:
            {
                if (ThisEvent.EventParam == LINEFOLLOWING_TIMER){                
                    uint16_t R = (uint16_t)SPI1Leader_RequestResponse16(0x0011);
                    if ((R == 0)){
                        ES_Timer_InitTimer(LINEFOLLOWING_TIMER, LineFollowing_MS);
                    }else{
                        MotorHAL_SetSpeedCmdRPM(0, base_speed + R - 100UL, 0);
                        MotorHAL_SetSpeedCmdRPM(1, base_speed - R + 100UL, 0);
                        ES_Timer_InitTimer(LINEFOLLOWING_TIMER, LineFollowing_MS);
                    }
                }
            }
            break;
        }
    }
    break;
    
    case LineFollowingState_5:
    {
        switch (ThisEvent.EventType)
        {
        
            case ES_ACTION_DONE:
            {
                //ES_Timer_InitTimer(FORWARD_TIMER, 5000);
                DB_printf("Done\n");
                //MotorHAL_SetSpeedCmdRPM(0, 0, 0);
                //MotorHAL_SetSpeedCmdRPM(1, 0, 0);
                MotorHAL_DriveEncoderCount(0, 100);
                MotorHAL_DriveEncoderCount(1, 100);
                MotorHAL_SetSpeedCmdRPM(0, 30, 1);
                MotorHAL_SetSpeedCmdRPM(1, 30, 1);
                CurrentState = LineFollowingState_6;
            }
            break;
            case ES_TIMEOUT:
            {
                if (ThisEvent.EventParam == LINEFOLLOWING_TIMER){                
                    uint16_t R = (uint16_t)SPI1Leader_RequestResponse16(0x0011);
                    if ((R == 0)){
                        ES_Timer_InitTimer(LINEFOLLOWING_TIMER, LineFollowing_MS);
                    }else{
                        MotorHAL_SetSpeedCmdRPM(0, base_speed + R - 100UL, 0);
                        MotorHAL_SetSpeedCmdRPM(1, base_speed - R + 100UL, 0);
                        ES_Timer_InitTimer(LINEFOLLOWING_TIMER, LineFollowing_MS);
                    }
                }
            }
            break;
        }
    }
    break;
    
    case LineFollowingState_6:
    {
        switch (ThisEvent.EventType)
        {
            case ES_ACTION_DONE:
            {
                MotorHAL_DriveEncoderCount(0, 100);
                MotorHAL_DriveEncoderCount(1, 100);
                MotorHAL_SetSpeedCmdRPM(0, 30, 0);
                MotorHAL_SetSpeedCmdRPM(1, 30, 0);
                CurrentState = LineFollowingState_7;
            }
            break;
        }
    }
    break;
    
        case LineFollowingState_7:
    {
        switch (ThisEvent.EventType)
        {
            case ES_ACTION_DONE:
            {
                MotorHAL_DriveEncoderCount(0, 100);
                MotorHAL_DriveEncoderCount(1, 100);
                MotorHAL_SetSpeedCmdRPM(0, 30, 1);
                MotorHAL_SetSpeedCmdRPM(1, 30, 1);
                CurrentState = LineFollowingState_6;
            }
            break;
        }
    }
    break;
    
    
    case CheckPoint3State_1:
    { 
        switch (ThisEvent.EventType){
            case ES_BEACON_DETECTED:
            {
                if(ThisEvent.EventParam > 818 && ThisEvent.EventParam < 1000){
                    ES_Timer_InitTimer(SIDEDECITION_TIMER, 6000);
                    BeaconDetector_ArmForTarget(BEACON_L);
                    DB_printf("Beacon R detected!\n");
                    BeaconGDetected = true;
                }
                if(BeaconGDetected == true && ThisEvent.EventParam >= 1800 && ThisEvent.EventParam <= 2200){
                    uint16_t R = SPI1Leader_RequestResponse16(0x0004);
                    CurrentState = CheckPoint3State_2;
                    DB_printf("Then Beacon G detected!\n");
                    MotorHAL_SetSpeedCmdRPM(0, 20, 0);
                    MotorHAL_SetSpeedCmdRPM(1, 20, 1);
                    BeaconDetector_ArmForTarget(BEACON_R);
                }
            }
            break;
            
            case ES_TIMEOUT:
            {
                if (ThisEvent.EventParam == SIDEDECITION_TIMER){
                    DB_printf("Timer expired!\n");
                    uint16_t R = SPI1Leader_RequestResponse16(0x0003);
                    CurrentState = CheckPoint3State_2;
                    MotorHAL_SetSpeedCmdRPM(0, 20, 0);
                    MotorHAL_SetSpeedCmdRPM(1, 20, 1);
                    BeaconDetector_ArmForTarget(BEACON_R);
                }
            }
         
        }
    
        
    } 
    break;
    
    case CheckPoint3State_2:
    {
        switch (ThisEvent.EventType)
        {
            case ES_BEACON_DETECTED:
            {
                if(ThisEvent.EventParam > 818 && ThisEvent.EventParam < 1000){
                    MotorHAL_DriveEncoderCount(0, 600);
                    MotorHAL_DriveEncoderCount(1, 600);
                    MotorHAL_SetSpeedCmdRPM(0, 20, 0);
                    MotorHAL_SetSpeedCmdRPM(1, 20, 0); 
                }
            }
            break;
        }
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

