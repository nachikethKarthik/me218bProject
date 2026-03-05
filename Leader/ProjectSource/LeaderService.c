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

#define ROTATE_L_R_RPM  60  // //used to be 30

#define BASE_SPEED_TO_DISPENSER  60  // used to be 30
#define BASE_SPEED_TO_BUCKET1    30   // 

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

static uint8_t suck_count;
static uint8_t seesaw_num;

static uint16_t Game_count = 0;

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
  //DB_printf("Start Leader!\n");
  MyPriority = Priority;
  CurrentState = InitPState;
  ThisEvent.EventType = ES_INIT;
  
  //Start Switch
  TRISBbits.TRISB10 = 1;
  
  seesaw_num = 1;
  
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

  if (ThisEvent.EventType == ES_TIMEOUT){
      if(ThisEvent.EventParam == GAME_TIMER){
          ES_Timer_InitTimer(GAME_TIMER, 1000);
          Game_count += 1;
          //DB_printf("%d",Game_count);
          if(Game_count >= 138){
                ES_Event_t ThisEvent;
                ThisEvent.EventType = ES_GAME_END;
                ES_PostAll(ThisEvent);
                CurrentState = Stop_State;
          }
      }
  }
  
  
  switch (CurrentState)
  {
    case InitPState:        // If current state is initial Psedudo State
    {
        switch (ThisEvent.EventType)
        {
            case ES_INIT:    // only respond to ES_Init
            {
            }
            break;
            
            case ES_GAME_START:    // only respond to ES_Init
            {

                ES_Timer_InitTimer(GAME_TIMER, 1000);
                
                
              //CurrentState = TestState;
//              MotorHAL_SetSpeedCmdRPM(0, 40, 0);
//              MotorHAL_SetSpeedCmdRPM(1, 40, 0);
//              MotorHAL_DriveEncoderCount(0, 400);
//              MotorHAL_DriveEncoderCount(1, 400);
              
//                SPI1Leader_SendCmd16(0x0015);
//                DB_printf("Game Start!\n");

      //          CurrentState = LineFollowingState_1;
      //          DB_printf("Start Line 1\n");
      //          MotorHAL_SetSpeedCmdRPM(0, 40, 0);
      //          MotorHAL_SetSpeedCmdRPM(1, 40, 0);
      //          base_speed = 40;
      //          FrontFollowing = true;
      //          FrontT = true;
      //          ES_Timer_InitTimer(LINEFOLLOWING_TIMER, LineFollowing_MS);

              MotorHAL_SetSpeedCmdRPM(0, 30, 0);
              MotorHAL_SetSpeedCmdRPM(1, 30, 1);
              CurrentState = Field_Determine_State_1;
              BeaconDetector_Arm();


//              MotorHAL_DriveEncoderCount(0, 100);
//              MotorHAL_DriveEncoderCount(1, 100);
//              MotorHAL_SetSpeedCmdRPM(0, 20, 1);
//              MotorHAL_SetSpeedCmdRPM(1, 20, 1);
//              base_speed = 20;
//              CurrentState = Seesaw1_State_1;
//              ES_Timer_InitTimer(LINEFOLLOWING_TIMER, LineFollowing_MS);


            }
            break;
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
                        RGB_TurnGreen();
                        
                    }
                    break;
                    case 'q':
                    {
                        RGB_TurnBlue();
                    }
                    break;
                    case 'z':
                    {
                        uint16_t E = (uint16_t)SPI1Leader_RequestResponse16(0x0022);
                        uint16_t D = (uint16_t)SPI1Leader_RequestResponse16(0x0027);
                    }
                    break;
                    case 'm':
                    {
                        uint16_t E = (uint16_t)SPI1Leader_RequestResponse16(0x0025);
                        uint16_t D = (uint16_t)SPI1Leader_RequestResponse16(0x0021);
                    }
                    break;
                    case 'f':
                    {
//                        MotorHAL_DriveEncoderCount(0, 260);
//                        MotorHAL_DriveEncoderCount(1, 260);
                        MotorHAL_DriveEncoderCount(0, 510);
                        MotorHAL_DriveEncoderCount(1, 510);
                        MotorHAL_SetSpeedCmdRPM(0, 40, 0);
                        MotorHAL_SetSpeedCmdRPM(1, 40, 0);
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
                // After beacon detected, react to the beacon, and rotate to the line
                if (ThisEvent.EventParam >= 818 && ThisEvent.EventParam <= 1000){ // R detected
                    //SPI1Leader_SendCmd16(0x0023);
                    uint16_t R = (uint16_t)SPI1Leader_RequestResponse16(0x0023);
                    RGB_TurnBlue();
                    
                    MotorHAL_DriveEncoderCount(0, 220);
                    MotorHAL_DriveEncoderCount(1, 220);
                    MotorHAL_SetSpeedCmdRPM(0, 20, 1);
                    MotorHAL_SetSpeedCmdRPM(1, 20, 0);
                }else if (ThisEvent.EventParam >= 1800 && ThisEvent.EventParam <= 2200){
                    //SPI1Leader_SendCmd16(0x0024);
                    uint16_t R = (uint16_t)SPI1Leader_RequestResponse16(0x0024);
                    RGB_TurnGreen();
                    
                    MotorHAL_DriveEncoderCount(0, 220);
                    MotorHAL_DriveEncoderCount(1, 220);
                    MotorHAL_SetSpeedCmdRPM(0, 20, 1);
                    MotorHAL_SetSpeedCmdRPM(1, 20, 0);
                }else{
                    BeaconDetector_Arm();
                }
            }
            break;
            
            case ES_ACTION_DONE:
            {
                // Move forward a little to cross the T
                CurrentState = Field_Determine_State_2;
                MotorHAL_DriveEncoderCount(0, 70);
                MotorHAL_DriveEncoderCount(1, 70);
                MotorHAL_SetSpeedCmdRPM(0, 40, 0);
                MotorHAL_SetSpeedCmdRPM(1, 40, 0);
            }
            break;
        }
    }
    break;
    
    case Field_Determine_State_2: // MAKING CHANGES HERE
    {
        switch (ThisEvent.EventType)
        {
            case ES_ACTION_DONE:
            {
                // // Start linefollowing and drive forward (to dispenser), stop at the T right in front of the dispenser
                CurrentState = LineFollowingState_1;
                ES_Timer_InitTimer(LINEFOLLOWING_TIMER, LineFollowing_MS);
                base_speed = BASE_SPEED_TO_DISPENSER; // used to be 40
                MotorHAL_SetSpeedCmdRPM(0, BASE_SPEED_TO_DISPENSER, 0); // used to be 40
                MotorHAL_SetSpeedCmdRPM(1, BASE_SPEED_TO_DISPENSER, 0); // used to be 40

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
                    // When detect T, drive forward a little
                    if ((R == 0) && FrontT){
                        FrontT = false;
                        FrontFollowing = false;
                        MotorHAL_SetSpeedCmdRPM(0, 30, 0);
                        MotorHAL_SetSpeedCmdRPM(1, 30, 0);
                        MotorHAL_DriveEncoderCount(0, 40);
                        MotorHAL_DriveEncoderCount(1, 40);
                    }else if (R == 1){
                        MotorHAL_SetSpeedCmdRPM(1, base_speed, 0);
                        MotorHAL_SetSpeedCmdRPM(0, base_speed, 0);
                        ES_Timer_InitTimer(LINEFOLLOWING_TIMER, LineFollowing_MS);
                    }else if (FrontFollowing){
                        MotorHAL_SetSpeedCmdRPM(1, base_speed + R - 100UL, 0);
                        MotorHAL_SetSpeedCmdRPM(0, base_speed - R + 100UL, 0);
                        //DB_printf("%d\n",R);
                        ES_Timer_InitTimer(LINEFOLLOWING_TIMER, LineFollowing_MS);
                    }
                }
            }
            break;
            
            case ES_ACTION_DONE:
            {
                // rotate 90
                DB_printf("start rotate 90\n");
                CurrentState = LineFollowingState_2;
                MotorHAL_SetSpeedCmdRPM(1, ROTATE_L_R_RPM, 1);
                MotorHAL_SetSpeedCmdRPM(0, ROTATE_L_R_RPM, 0);
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
                if (ThisEvent.EventParam == LINEFOLLOWING_TIMER){
                    
                    uint16_t R = (uint16_t)SPI1Leader_RequestResponse16(0x0011);
                    if ((R == 0)){
                        ES_Timer_InitTimer(LINEFOLLOWING_TIMER, LineFollowing_MS);
                    }else if (R == 1){
                        MotorHAL_SetSpeedCmdRPM(1, base_speed, 0);
                        MotorHAL_SetSpeedCmdRPM(0, base_speed, 0);
                        ES_Timer_InitTimer(LINEFOLLOWING_TIMER, LineFollowing_MS);
                    }else{
                        MotorHAL_SetSpeedCmdRPM(1, base_speed + R - 100UL, 0);
                        MotorHAL_SetSpeedCmdRPM(0, base_speed - R + 100UL, 0);
                        ES_Timer_InitTimer(LINEFOLLOWING_TIMER, LineFollowing_MS);
                    }
                }
            }
            break;
            case ES_ACTION_DONE:
            {
                // After rotate 90, drive forward to the T before the bucket
                
                DB_printf("finish rotate 90, start forward\n");
                base_speed = 50;
                MotorHAL_SetSpeedCmdRPM(1, base_speed, 0);
                MotorHAL_SetSpeedCmdRPM(0, base_speed, 0);
                //MotorHAL_DriveEncoderCount(0, 400);
                //MotorHAL_DriveEncoderCount(1, 400);
                CurrentState = LineFollowingState_3;
                uint16_t R = (uint16_t)SPI1Leader_RequestResponse16(0x0022);
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
                if (ThisEvent.EventParam == LINEFOLLOWING_TIMER){
                    
                    // After detect T, rotate 180
                    uint16_t R = (uint16_t)SPI1Leader_RequestResponse16(0x0011);
                    if ((R == 0)){
                        //ES_Timer_InitTimer(LINEFOLLOWING_TIMER, LineFollowing_MS);
                        ES_Event_t NewEvent;
                        NewEvent.EventType = ES_T_DETECTED;
                        ES_PostAll(NewEvent);
                    }else if (R == 1){
                        //branch detected
                        MotorHAL_SetSpeedCmdRPM(1, base_speed, 0);
                        MotorHAL_SetSpeedCmdRPM(0, base_speed, 0);
                        ES_Timer_InitTimer(LINEFOLLOWING_TIMER, LineFollowing_MS);

                        
                    }else{
                        MotorHAL_SetSpeedCmdRPM(1, base_speed + R - 100UL, 0);
                        MotorHAL_SetSpeedCmdRPM(0, base_speed - R + 100UL, 0);
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
                //DB_printf("Finish forward, start rotate 180\n");
                MotorHAL_DriveEncoderCount(0, 510);
                MotorHAL_DriveEncoderCount(1, 510);
                MotorHAL_SetSpeedCmdRPM(1, ROTATE_L_R_RPM, 1);
                MotorHAL_SetSpeedCmdRPM(0, ROTATE_L_R_RPM, 0);
                
                //CurrentState = LineFollowingState_4;  // Change here will go back to fill the first seesaw
                
                //This is for the new method
                CurrentState = Push_Seesaw1_State_1;
                uint16_t R = (uint16_t)SPI1Leader_RequestResponse16(0x0022);
            }
            break;           
        }
    }
    break;
    
    
            case Push_Seesaw1_State_1:
            {
                switch (ThisEvent.EventType)
                {
                    case ES_ACTION_DONE:
                    {
                       MotorHAL_SetSpeedCmdRPM(1, 30, 1);
                       MotorHAL_SetSpeedCmdRPM(0, 30, 1);
                       MotorHAL_DriveEncoderCount(0, 110);
                       MotorHAL_DriveEncoderCount(1, 110);
                       CurrentState = Push_Seesaw1_State_2;
                    }
                    break;
                }
            }
            break;

            case Push_Seesaw1_State_2:
            {
                switch (ThisEvent.EventType)
                {
                    case ES_ACTION_DONE:
                    {
                      uint16_t R = (uint16_t)SPI1Leader_RequestResponse16(0x0021);
                      MotorHAL_SetSpeedCmdRPM(1, 30, 0);
                      MotorHAL_SetSpeedCmdRPM(0, 30, 0);
                      MotorHAL_DriveEncoderCount(0, 110);
                      MotorHAL_DriveEncoderCount(1, 110);
                      CurrentState = Seesaw1_State_2_2;
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
                // Turn on flywheel and drive forward
                //DB_printf("Finish rotate 180, start forward\n");
                uint16_t B = (uint16_t)SPI1Leader_RequestResponse16(0x0029);
                SPI1Leader_SendCmd16(0x0015);
                
                //uint16_t R = (uint16_t)SPI1Leader_RequestResponse16(0x0011);
                
                //uint16_t R = SPI1Leader_RequestResponse16(0x0015);
                ES_Timer_StartTimer(LINEFOLLOWING_TIMER);
                ES_Timer_InitTimer(LINEFOLLOWING_TIMER, LineFollowing_MS);
                MotorHAL_DriveEncoderCount(0, 600); //550
                MotorHAL_DriveEncoderCount(1, 600);
                MotorHAL_SetSpeedCmdRPM(1, 30, 0);
                MotorHAL_SetSpeedCmdRPM(0, 30, 0);
                base_speed = 30;
                CurrentState = LineFollowingState_5;
                ES_Timer_InitTimer(SUCK_TIMER, 6300);
            }
            break;
            case ES_TIMEOUT:
            {
                if (ThisEvent.EventParam == LINEFOLLOWING_TIMER){                
                    uint16_t R = (uint16_t)SPI1Leader_RequestResponse16(0x0011);
                    if ((R == 0)){
                        ES_Timer_InitTimer(LINEFOLLOWING_TIMER, LineFollowing_MS);
                    }else if (R == 1){
                        MotorHAL_SetSpeedCmdRPM(1, base_speed, 0);
                        MotorHAL_SetSpeedCmdRPM(0, base_speed, 0);
                        ES_Timer_InitTimer(LINEFOLLOWING_TIMER, LineFollowing_MS);
                    }else{
                        MotorHAL_SetSpeedCmdRPM(1, base_speed + R - 100UL, 0);
                        MotorHAL_SetSpeedCmdRPM(0, base_speed - R + 100UL, 0);
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
                MotorHAL_DriveEncoderCount(0, 100);
                MotorHAL_DriveEncoderCount(1, 100);
                MotorHAL_SetSpeedCmdRPM(1, 30, 1);
                MotorHAL_SetSpeedCmdRPM(0, 30, 1);
                CurrentState = LineFollowingState_6;
                
                suck_count = 1;
            }
            break;
            case ES_TIMEOUT:
            {
                if (ThisEvent.EventParam == LINEFOLLOWING_TIMER){                
                    uint16_t R = (uint16_t)SPI1Leader_RequestResponse16(0x0011);
                    if ((R == 0)){
                        ES_Timer_InitTimer(LINEFOLLOWING_TIMER, LineFollowing_MS);
                    }else if (R == 1){
                        MotorHAL_SetSpeedCmdRPM(1, base_speed, 0);
                        MotorHAL_SetSpeedCmdRPM(0, base_speed, 0);
                        ES_Timer_InitTimer(LINEFOLLOWING_TIMER, LineFollowing_MS);
                    }else{
                        MotorHAL_SetSpeedCmdRPM(1, base_speed + R - 100UL, 0);
                        MotorHAL_SetSpeedCmdRPM(0, base_speed - R + 100UL, 0);
                        ES_Timer_InitTimer(LINEFOLLOWING_TIMER, LineFollowing_MS);
                    }
                }
                if (ThisEvent.EventParam == SUCK_TIMER){
                    //ES_Timer_InitTimer(FORWARD_TIMER, 5000);
                    DB_printf("Done\n");
                    //MotorHAL_SetSpeedCmdRPM(0, 0, 0);
                    //MotorHAL_SetSpeedCmdRPM(1, 0, 0);
                    
                    //uint16_t R = (uint16_t)SPI1Leader_RequestResponse16(0x0016);
                    MotorHAL_DriveEncoderCount(0, 100);
                    MotorHAL_DriveEncoderCount(1, 100);
                    MotorHAL_SetSpeedCmdRPM(1, 30, 1);
                    MotorHAL_SetSpeedCmdRPM(0, 30, 1);
                    CurrentState = LineFollowingState_6;
                
                    suck_count = 1;
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
                uint16_t R = (uint16_t)SPI1Leader_RequestResponse16(0x0015);
                MotorHAL_DriveEncoderCount(0, 150); //100 before
                MotorHAL_DriveEncoderCount(1, 150);
                MotorHAL_SetSpeedCmdRPM(1, 40, 0);
                MotorHAL_SetSpeedCmdRPM(0, 40, 0);
                CurrentState = LineFollowingState_7;
                ES_Timer_InitTimer(SUCK_TIMER, 3000);
                suck_count += 1;
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
                MotorHAL_SetSpeedCmdRPM(1, 40, 1);
                MotorHAL_SetSpeedCmdRPM(0, 40, 1);
                
                if (suck_count < 3){
                    CurrentState = LineFollowingState_6;
                }else{
                    if (seesaw_num == 1){
                        CurrentState = Seesaw1_State_1;
                        base_speed = BASE_SPEED_TO_BUCKET1;
                        MotorHAL_SetSpeedCmdRPM(0, base_speed, 1);      
                        MotorHAL_SetSpeedCmdRPM(1, base_speed, 1);
                        ES_Timer_InitTimer(LINEFOLLOWING_TIMER, LineFollowing_MS);
                        uint16_t R = (uint16_t)SPI1Leader_RequestResponse16(0x0016);
                    }else if (seesaw_num == 2){
                        CurrentState = Seesaw2_State_1;
                        uint16_t B = (uint16_t)SPI1Leader_RequestResponse16(0x0029);
                        ES_Timer_InitTimer(LINEFOLLOWING_TIMER, LineFollowing_MS);
                        uint16_t R = (uint16_t)SPI1Leader_RequestResponse16(0x0016);
                    }else if (seesaw_num == 3){
                        CurrentState = Seesaw3_State_2;
                        MotorHAL_DriveEncoderCount(0, 300);
                        MotorHAL_DriveEncoderCount(1, 300); 
                        MotorHAL_SetSpeedCmdRPM(1, 30, 1);
                        MotorHAL_SetSpeedCmdRPM(0, 30, 1);
                        uint16_t R = (uint16_t)SPI1Leader_RequestResponse16(0x0016);
                    }else if (seesaw_num == 4){
                        CurrentState = Seesaw4_State_1;
                        MotorHAL_DriveEncoderCount(0, 300);
                        MotorHAL_DriveEncoderCount(1, 300); 
                        MotorHAL_SetSpeedCmdRPM(1, 30, 1);
                        MotorHAL_SetSpeedCmdRPM(0, 30, 1);
                        uint16_t R = (uint16_t)SPI1Leader_RequestResponse16(0x0016);
                    }
                    
                }
                
            }
            break;
            
            case ES_TIMEOUT:
            {
                
                if (ThisEvent.EventParam == SUCK_TIMER){
                    
                    
                    
                    MotorHAL_DriveEncoderCount(0, 100);
                    MotorHAL_DriveEncoderCount(1, 100);
                    MotorHAL_SetSpeedCmdRPM(1, 40, 1);
                    MotorHAL_SetSpeedCmdRPM(0, 40, 1);

                    if (suck_count < 3){
                        CurrentState = LineFollowingState_6;
                    }else{
                        if (seesaw_num == 1){
                            CurrentState = Seesaw1_State_1;
                            base_speed = BASE_SPEED_TO_BUCKET1;
                            MotorHAL_SetSpeedCmdRPM(0, base_speed, 1);    
                            MotorHAL_SetSpeedCmdRPM(1, base_speed, 1);
                            ES_Timer_InitTimer(LINEFOLLOWING_TIMER, LineFollowing_MS);
                            uint16_t R = (uint16_t)SPI1Leader_RequestResponse16(0x0016);
                        }else if (seesaw_num == 2){
                            CurrentState = Seesaw2_State_1;
                            uint16_t B = (uint16_t)SPI1Leader_RequestResponse16(0x0029);
                            uint16_t R = (uint16_t)SPI1Leader_RequestResponse16(0x0016);
                            ES_Timer_InitTimer(LINEFOLLOWING_TIMER, LineFollowing_MS);
                        }else if (seesaw_num == 3){
                            CurrentState = Seesaw3_State_2;
                            MotorHAL_DriveEncoderCount(0, 300);
                            MotorHAL_DriveEncoderCount(1, 300); 
                            MotorHAL_SetSpeedCmdRPM(1, 30, 1);
                            MotorHAL_SetSpeedCmdRPM(0, 30, 1);
                            uint16_t R = (uint16_t)SPI1Leader_RequestResponse16(0x0016);
                        }

                    }
                }
            }
            break;
            
        }
    }
    break;
    
    case Seesaw1_State_1:
    {
        switch (ThisEvent.EventType)
        {
            case ES_ACTION_DONE:
            {
                // Drive toward T, and rise the arm
                
                base_speed = BASE_SPEED_TO_BUCKET1;
                MotorHAL_SetSpeedCmdRPM(1, base_speed, 1);
                MotorHAL_SetSpeedCmdRPM(0, base_speed, 1);
                
                uint16_t A = (uint16_t)SPI1Leader_RequestResponse16(0x0022);
                uint16_t B = (uint16_t)SPI1Leader_RequestResponse16(0x0027);
                uint16_t C = (uint16_t)SPI1Leader_RequestResponse16(0x0016);
            }
            break;
            
            case ES_TIMEOUT:
            {
                if (ThisEvent.EventParam == LINEFOLLOWING_TIMER){                
                    uint16_t R = (uint16_t)SPI1Leader_RequestResponse16(0x0012);
                    DB_printf("%d\n",R);
                    if ((R == 0)){
                        CurrentState = Seesaw1_State_2;
                        MotorHAL_DriveEncoderCount(0, 110);
                        MotorHAL_DriveEncoderCount(1, 110);
                    }else if(R == 1){
                        MotorHAL_SetSpeedCmdRPM(1, base_speed, 1);
                        MotorHAL_SetSpeedCmdRPM(0, base_speed, 1);
                        ES_Timer_InitTimer(LINEFOLLOWING_TIMER, LineFollowing_MS);
                    }else{
                        MotorHAL_SetSpeedCmdRPM(1, base_speed + R - 100UL, 1);
                        MotorHAL_SetSpeedCmdRPM(0, base_speed - R + 100UL, 1);
                        ES_Timer_InitTimer(LINEFOLLOWING_TIMER, LineFollowing_MS);
                    }
                }
            }
            break;
            
        }
    }
    break;
    
    case Seesaw1_State_2:
    {
        switch (ThisEvent.EventType)
        {
            case ES_ACTION_DONE:
            {
                // Drop the coal into bucket
                uint16_t B = (uint16_t)SPI1Leader_RequestResponse16(0x0026);
                ES_Timer_InitTimer(TRAPDOOR_TIMER, 5000);
            }
            break;
            
            case ES_TIMEOUT:
            {
                
                if(ThisEvent.EventParam == TRAPDOOR_TIMER){
                    uint16_t E = (uint16_t)SPI1Leader_RequestResponse16(0x0025);
                    ES_Timer_InitTimer(ARM_TIMER, 5000);
                    CurrentState = Seesaw1_State_2_1;
                }
                
//                if (ThisEvent.EventParam == ARM_TIMER){
//                    //Drive back to the dispenser
//                    MotorHAL_DriveEncoderCount(0, 600); //550
//                    MotorHAL_DriveEncoderCount(1, 600);
//                    MotorHAL_SetSpeedCmdRPM(1, 30, 0);
//                    MotorHAL_SetSpeedCmdRPM(0, 30, 0);
//                    
//                    SPI1Leader_SendCmd16(0x0015);  //Turn on Flywheel
//                    ES_Timer_InitTimer(SUCK_TIMER, 7500);
//                    //Return the bucket
//                    uint16_t E = (uint16_t)SPI1Leader_RequestResponse16(0x0025);
//                    uint16_t D = (uint16_t)SPI1Leader_RequestResponse16(0x0021);
//                   
//                    CurrentState = Seesaw1_State_3;
//                    ES_Timer_InitTimer(LINEFOLLOWING_TIMER, LineFollowing_MS);
//                }
                
            }
            break;
            
        }
    }
    break;
    
    case Seesaw1_State_2_1:
    {
        switch (ThisEvent.EventType)
        {
            case ES_TIMEOUT:
            {
                if(ThisEvent.EventParam == ARM_TIMER){
                    MotorHAL_DriveEncoderCount(0, 255); //550
                    MotorHAL_DriveEncoderCount(1, 255);
                    MotorHAL_SetSpeedCmdRPM(1, ROTATE_L_R_RPM, 0); // used to be 30
                    MotorHAL_SetSpeedCmdRPM(0, ROTATE_L_R_RPM, 1); // used to be 30
                }
                if(ThisEvent.EventParam == FLYWHEEL_TIMER){
                    MotorHAL_DriveEncoderCount(0, 255); //550
                    MotorHAL_DriveEncoderCount(1, 255);
                    MotorHAL_SetSpeedCmdRPM(1, ROTATE_L_R_RPM, 1); // used to be 30
                    MotorHAL_SetSpeedCmdRPM(0, ROTATE_L_R_RPM, 0); // used to be 30
                    CurrentState = Seesaw1_State_2_2;
                }
            }
            break;
            case ES_ACTION_DONE:
            {
                uint16_t E = (uint16_t)SPI1Leader_RequestResponse16(0x0028);
                ES_Timer_InitTimer(FLYWHEEL_TIMER, 2000);
            }
            break;
        }
    }
    break;      
    
    case Seesaw1_State_2_2:
    {
        switch (ThisEvent.EventType)
        {
            case ES_ACTION_DONE:
            {
                MotorHAL_DriveEncoderCount(0, 600); //550
                MotorHAL_DriveEncoderCount(1, 600);
                MotorHAL_SetSpeedCmdRPM(1, 30, 0);
                MotorHAL_SetSpeedCmdRPM(0, 30, 0);

                SPI1Leader_SendCmd16(0x0015);  //Turn on Flywheel
                ES_Timer_InitTimer(SUCK_TIMER, 7500);
                //Return the bucket
                uint16_t E = (uint16_t)SPI1Leader_RequestResponse16(0x0025);
                uint16_t D = (uint16_t)SPI1Leader_RequestResponse16(0x0021);

                CurrentState = Seesaw1_State_3;
                ES_Timer_InitTimer(LINEFOLLOWING_TIMER, LineFollowing_MS);
            }
            break;
            
        }
    }
    break;    
    
    case Seesaw1_State_3:
    {
        switch (ThisEvent.EventType)
        {
            case ES_ACTION_DONE:
            {
                //Drive backward to finish first suck
                uint16_t R = (uint16_t)SPI1Leader_RequestResponse16(0x0016);
                MotorHAL_DriveEncoderCount(0, 100);
                MotorHAL_DriveEncoderCount(1, 100);
                MotorHAL_SetSpeedCmdRPM(1, 30, 1);
                MotorHAL_SetSpeedCmdRPM(0, 30, 1);
                CurrentState = LineFollowingState_6;
                seesaw_num = 2;
                suck_count = 1;
                
            }
            break;
            case ES_TIMEOUT:
            {
                if(ThisEvent.EventParam == SUCK_TIMER){
                    //Drive backward to finish first suck
                    MotorHAL_DriveEncoderCount(0, 100);
                    MotorHAL_DriveEncoderCount(1, 100);
                    MotorHAL_SetSpeedCmdRPM(1, 30, 1);
                    MotorHAL_SetSpeedCmdRPM(0, 30, 1);
                    CurrentState = LineFollowingState_6;
                    seesaw_num = 2;
                    suck_count = 1;
                }
                // Line following
                if (ThisEvent.EventParam == LINEFOLLOWING_TIMER){
                    
                    uint16_t R = (uint16_t)SPI1Leader_RequestResponse16(0x0011);
                    if ((R == 0)){
                        ES_Timer_InitTimer(LINEFOLLOWING_TIMER, LineFollowing_MS);
                    }else if (R == 1){
                        ES_Timer_InitTimer(LINEFOLLOWING_TIMER, LineFollowing_MS);
                    }else{
                        MotorHAL_SetSpeedCmdRPM(1, base_speed + R - 100UL, 0);
                        MotorHAL_SetSpeedCmdRPM(0, base_speed - R + 100UL, 0);
                        ES_Timer_InitTimer(LINEFOLLOWING_TIMER, LineFollowing_MS);
                    }
                }
                
                
            }
            break;
        }
    }
    break;
    case Seesaw2_State_1:
    {
        switch (ThisEvent.EventType)
        {
            case ES_ACTION_DONE:
            {
                //Drive toward the T before the first bucket
                
                MotorHAL_SetSpeedCmdRPM(1, 30, 1);
                MotorHAL_SetSpeedCmdRPM(0, 30, 1);
                
                uint16_t A = (uint16_t)SPI1Leader_RequestResponse16(0x0022);
                uint16_t B = (uint16_t)SPI1Leader_RequestResponse16(0x0027);
                uint16_t C = (uint16_t)SPI1Leader_RequestResponse16(0x0016);
                base_speed = 30;
            }
            break;
            
            case ES_TIMEOUT:
            {
                if (ThisEvent.EventParam == LINEFOLLOWING_TIMER){                
                    uint16_t R = (uint16_t)SPI1Leader_RequestResponse16(0x0012);
                    if ((R == 0)){
                        
                        //After detect the T, rotate 45
                        CurrentState = Seesaw2_State_2;
                        MotorHAL_SetSpeedCmdRPM(0, 20, 0);
                        MotorHAL_SetSpeedCmdRPM(1, 20, 1);
                        MotorHAL_DriveEncoderCount(0, 160);
                        MotorHAL_DriveEncoderCount(1, 160);
                        
                    }else{
                        MotorHAL_SetSpeedCmdRPM(1, base_speed + R - 100UL, 1);
                        MotorHAL_SetSpeedCmdRPM(0, base_speed - R + 100UL, 1);
                        ES_Timer_InitTimer(LINEFOLLOWING_TIMER, LineFollowing_MS);
                    }
                }
            }
            break;
        }
    }
    break;
    case Seesaw2_State_2:
    {
        switch (ThisEvent.EventType)
        {
            case ES_ACTION_DONE:
                {
                    // After rotate, drive forward to the second seesaw
                    MotorHAL_SetSpeedCmdRPM(0, 40, 1);
                    MotorHAL_SetSpeedCmdRPM(1, 40, 1);
                    MotorHAL_DriveEncoderCount(0, 300);
                    MotorHAL_DriveEncoderCount(1, 300);
                    CurrentState = Seesaw2_State_3;
                }
                break;
        }
    }
    break;
    
    case Seesaw2_State_3:
    {
        switch (ThisEvent.EventType)
        {
            case ES_ACTION_DONE:
                {
                    // Deposit the balls into seesaw 2
                    uint16_t B = (uint16_t)SPI1Leader_RequestResponse16(0x0026);
                    ES_Timer_InitTimer(TRAPDOOR_TIMER, 5000);
                }
            break;
            case ES_TIMEOUT:
            {
                if (ThisEvent.EventParam == TRAPDOOR_TIMER){
                    uint16_t E = (uint16_t)SPI1Leader_RequestResponse16(0x0025);
                    MotorHAL_SetSpeedCmdRPM(0, 20, 0);
                    MotorHAL_SetSpeedCmdRPM(1, 20, 0);
                    MotorHAL_DriveEncoderCount(0, 300);
                    MotorHAL_DriveEncoderCount(1, 300);
                    
                    CurrentState = Seesaw2_State_4;
                }
            }
            break;
        }
    }
    break;
    
    case Seesaw2_State_4:
    {
        switch (ThisEvent.EventType)
        {
            case ES_ACTION_DONE:
                {
                    // Drive forward a fixed amount (away from seesaw 2, back to the black line). Forward = flywheel side
                    MotorHAL_SetSpeedCmdRPM(0, 20, 1);
                    MotorHAL_SetSpeedCmdRPM(1, 20, 0);
                    MotorHAL_DriveEncoderCount(0, 160);
                    MotorHAL_DriveEncoderCount(1, 160);
                    CurrentState = Seesaw2_State_4_1;
                }
                break;
        }
    }
    break;
    
    case Seesaw2_State_4_1:
    {
        switch (ThisEvent.EventType)
        {
            case ES_ACTION_DONE:
                {
                    MotorHAL_DriveEncoderCount(0, 255);
                    MotorHAL_DriveEncoderCount(1, 255);
                    MotorHAL_SetSpeedCmdRPM(1, 30, 0);
                    MotorHAL_SetSpeedCmdRPM(0, 30, 1);
                    CurrentState = Seesaw2_State_4_2;
                }
                break;
        }
    }
    break;
    
    case Seesaw2_State_4_2:
    {
        switch (ThisEvent.EventType)
        {
            case ES_TIMEOUT:
            { 
                if(ThisEvent.EventParam == FLYWHEEL_TIMER){
                    MotorHAL_DriveEncoderCount(0, 255); //550
                    MotorHAL_DriveEncoderCount(1, 255);
                    MotorHAL_SetSpeedCmdRPM(1, ROTATE_L_R_RPM, 1);
                    MotorHAL_SetSpeedCmdRPM(0, ROTATE_L_R_RPM, 0);
                    CurrentState = Seesaw2_State_5;
                    //CurrentState = Stop_State;
                }
            }
            break;
            case ES_ACTION_DONE:
            {
                uint16_t E = (uint16_t)SPI1Leader_RequestResponse16(0x0028);
                ES_Timer_InitTimer(FLYWHEEL_TIMER, 2000);
            }
            break;
        }
    }
    break;

    
    case Seesaw2_State_5:
    {
        switch (ThisEvent.EventType)
        {
            case ES_ACTION_DONE:
                {
                    //Drive back to the dispenser
                    MotorHAL_DriveEncoderCount(0, 650); //550
                    MotorHAL_DriveEncoderCount(1, 650);
                    MotorHAL_SetSpeedCmdRPM(1, 30, 0);
                    MotorHAL_SetSpeedCmdRPM(0, 30, 0);
                    
                    //Turn on Flywheel
                    SPI1Leader_SendCmd16(0x0015);  
                    ES_Timer_InitTimer(SUCK_TIMER, 7500);
                    
                    //Return the bucket to lowered position
                    uint16_t E = (uint16_t)SPI1Leader_RequestResponse16(0x0025);
                    uint16_t D = (uint16_t)SPI1Leader_RequestResponse16(0x0021);
                   
                    CurrentState = Seesaw3_State_1;
                    ES_Timer_InitTimer(LINEFOLLOWING_TIMER, LineFollowing_MS);
                }
                break;
        }
    }
    break;
    
    
    case Seesaw3_State_1:
    {
        switch (ThisEvent.EventType)
        {
            case ES_ACTION_DONE:
            {
                //Drive backward to finish first suck for seesaw 3
                
                MotorHAL_DriveEncoderCount(0, 100);
                MotorHAL_DriveEncoderCount(1, 100);
                MotorHAL_SetSpeedCmdRPM(1, 30, 1);
                MotorHAL_SetSpeedCmdRPM(0, 30, 1);
                CurrentState = LineFollowingState_6;
                seesaw_num = 3;
                suck_count = 1;     
            }
            break;
            case ES_TIMEOUT:
            {
                if(ThisEvent.EventParam == SUCK_TIMER){
                    //Drive backward to finish first suck
                    MotorHAL_DriveEncoderCount(0, 100);
                    MotorHAL_DriveEncoderCount(1, 100);
                    MotorHAL_SetSpeedCmdRPM(1, 30, 1);
                    MotorHAL_SetSpeedCmdRPM(0, 30, 1);
                    CurrentState = LineFollowingState_6;
                    seesaw_num = 3;
                    suck_count = 1;
                }
                // Line following to the dispenser
                if (ThisEvent.EventParam == LINEFOLLOWING_TIMER){
                    
                    uint16_t R = (uint16_t)SPI1Leader_RequestResponse16(0x0011);
                    if ((R == 0)){
                        ES_Timer_InitTimer(LINEFOLLOWING_TIMER, LineFollowing_MS);
                    }else if (R == 1){
                        MotorHAL_SetSpeedCmdRPM(1, base_speed, 0);
                        MotorHAL_SetSpeedCmdRPM(0, base_speed, 0);
                        ES_Timer_InitTimer(LINEFOLLOWING_TIMER, LineFollowing_MS);
                    }else{
                        MotorHAL_SetSpeedCmdRPM(1, base_speed + R - 100UL, 0);
                        MotorHAL_SetSpeedCmdRPM(0, base_speed - R + 100UL, 0);
                        ES_Timer_InitTimer(LINEFOLLOWING_TIMER, LineFollowing_MS);
                    }
                }
            }
            break;
        }
    }
    break;
    
    case Seesaw3_State_2:
    {
        switch (ThisEvent.EventType)
        {
            case ES_ACTION_DONE:
            {
                // Rotate 90 degrees
                MotorHAL_SetSpeedCmdRPM(1, 30, 1);
                MotorHAL_SetSpeedCmdRPM(0, 30, 0);
                MotorHAL_DriveEncoderCount(0, 250); //255 before
                MotorHAL_DriveEncoderCount(1, 250);
                CurrentState = Seesaw3_State_3;
            }
            break;
        }
    }
    break;
    
    case Seesaw3_State_3:
    {
        switch (ThisEvent.EventType)
        {
            case ES_ACTION_DONE:
            {
                MotorHAL_SetSpeedCmdRPM(1, 30, 1);
                MotorHAL_SetSpeedCmdRPM(0, 30, 1);
                
                //Rise the arm and stop the flywheel
                uint16_t A = (uint16_t)SPI1Leader_RequestResponse16(0x0022);
                uint16_t B = (uint16_t)SPI1Leader_RequestResponse16(0x0027);
                uint16_t C = (uint16_t)SPI1Leader_RequestResponse16(0x0016);
                
                ES_Timer_InitTimer(LINEFOLLOWING_TIMER, LineFollowing_MS);
            }
            break;
            case ES_TIMEOUT:
            {
                if (ThisEvent.EventParam == LINEFOLLOWING_TIMER){
                    // Line following until middle T
                    uint16_t R = (uint16_t)SPI1Leader_RequestResponse16(0x0012); 
                    if ((R == 0)){
                        
                        MotorHAL_SetSpeedCmdRPM(0, 30, 1); // used to be 20
                        MotorHAL_SetSpeedCmdRPM(1, 30, 1);
                        MotorHAL_DriveEncoderCount(0, 50);
                        MotorHAL_DriveEncoderCount(1, 50);
                        CurrentState = Seesaw3_State_4;
                        
//                        MotorHAL_SetSpeedCmdRPM(1, 30, 1);
//                        MotorHAL_SetSpeedCmdRPM(0, 30, 0);
//                        MotorHAL_DriveEncoderCount(0, 230);
//                        MotorHAL_DriveEncoderCount(1, 230);
//                        
//                        CurrentState = Seesaw3_State_5;
//                        ES_Timer_InitTimer(LINEFOLLOWING_TIMER, LineFollowing_MS);
                    }else{
                        ES_Timer_InitTimer(LINEFOLLOWING_TIMER, LineFollowing_MS);
                    }
                }
            }
            break;
        }
    }
    break;
    case Seesaw3_State_4:
    {
        switch (ThisEvent.EventType)
        {
            case ES_ACTION_DONE:
            {
                //DB_printf("[S4] short forward done, start 90-turn\n");
                MotorHAL_SetSpeedCmdRPM(1, ROTATE_L_R_RPM, 0);
                MotorHAL_SetSpeedCmdRPM(0, ROTATE_L_R_RPM, 1);
                MotorHAL_DriveEncoderCount(0, 255);
                MotorHAL_DriveEncoderCount(1, 255);
                CurrentState = Seesaw3_State_5;
                //ES_Timer_StartTimer(LINEFOLLOWING_TIMER);
                //ES_Timer_InitTimer(LINEFOLLOWING_TIMER, LineFollowing_MS);
            }
            break;
        }
    }
    break;
    
    
    case Seesaw3_State_5:
    {
        switch (ThisEvent.EventType)
        {
            case ES_ACTION_DONE:
            {
                MotorHAL_SetSpeedCmdRPM(1, 30, 1);
                MotorHAL_SetSpeedCmdRPM(0, 30, 1);
                ES_Timer_InitTimer(LINEFOLLOWING_TIMER, LineFollowing_MS);
            }
            break;
            case ES_TIMEOUT:
            {
                if (ThisEvent.EventParam == LINEFOLLOWING_TIMER){                
                    uint16_t R = (uint16_t)SPI1Leader_RequestResponse16(0x0012);
                    if ((R == 0)){
                        ES_Timer_StopTimer(LINEFOLLOWING_TIMER);
                        CurrentState = Seesaw3_State_6;
                        MotorHAL_DriveEncoderCount(0, 110);
                        MotorHAL_DriveEncoderCount(1, 110);
                    }else if(R == 1){
                        MotorHAL_SetSpeedCmdRPM(1, base_speed, 1);
                        MotorHAL_SetSpeedCmdRPM(0, base_speed, 1);
                        ES_Timer_InitTimer(LINEFOLLOWING_TIMER, LineFollowing_MS);
                    }else{
                        MotorHAL_SetSpeedCmdRPM(1, base_speed + R - 100UL, 1);
                        MotorHAL_SetSpeedCmdRPM(0, base_speed - R + 100UL, 1);
                        ES_Timer_InitTimer(LINEFOLLOWING_TIMER, LineFollowing_MS);
                    }
                }
            }
            break;
        }
    }
    break;
    
    
    case Seesaw3_State_6:
    {
        switch (ThisEvent.EventType)
        {
            case ES_ACTION_DONE:
                {
                    uint16_t B = (uint16_t)SPI1Leader_RequestResponse16(0x0026);
                    ES_Timer_InitTimer(TRAPDOOR_TIMER, 5000);
                }
                break;
            case ES_TIMEOUT:
            {
                if (ThisEvent.EventParam == TRAPDOOR_TIMER){
                    uint16_t E = (uint16_t)SPI1Leader_RequestResponse16(0x0025);
                    MotorHAL_DriveEncoderCount(0, 210);
                    MotorHAL_DriveEncoderCount(1, 210);
                    MotorHAL_SetSpeedCmdRPM(1, 40, 0);
                    MotorHAL_SetSpeedCmdRPM(0, 40, 0);
                    CurrentState = Seesaw3_State_7;
                    //CurrentState = Seesaw3_State_6;
                }
            }
            break;
        }
    }
    break;
    
    
    
    case Seesaw3_State_7:
    {
        switch (ThisEvent.EventType)
        {
            case ES_ACTION_DONE:
                {
                    MotorHAL_SetSpeedCmdRPM(1, ROTATE_L_R_RPM, 1);
                    MotorHAL_SetSpeedCmdRPM(0, ROTATE_L_R_RPM, 0);
                    MotorHAL_DriveEncoderCount(0, 255);
                    MotorHAL_DriveEncoderCount(1, 255);
                    CurrentState = Seesaw3_State_8;
                }
            break;
        }
    }
    break;
    
    
    case Seesaw3_State_8:
    {
        switch (ThisEvent.EventType)
        {
            case ES_ACTION_DONE:
            {
                MotorHAL_SetSpeedCmdRPM(1, 40, 0);
                MotorHAL_SetSpeedCmdRPM(0, 40, 0);
                ES_Timer_InitTimer(LINEFOLLOWING_TIMER, LineFollowing_MS);
            }
            break;
            case ES_TIMEOUT:
            {
                if (ThisEvent.EventParam == LINEFOLLOWING_TIMER){
                    uint16_t R = SPI1Leader_RequestResponse16(0x0011);
                    // When detect T, drive forward a little
                    if ((R == 0)){
                        
                        MotorHAL_SetSpeedCmdRPM(0, 30, 0);
                        MotorHAL_SetSpeedCmdRPM(1, 30, 0);
                        MotorHAL_DriveEncoderCount(0, 40);
                        MotorHAL_DriveEncoderCount(1, 40);
                        CurrentState = Seesaw3_State_9;
                    }else{
                        ES_Timer_InitTimer(LINEFOLLOWING_TIMER, LineFollowing_MS);
                    }
                }
            }
            break;
        }
    }
    break;
    
    case Seesaw3_State_9:
    {
        switch (ThisEvent.EventType)
        {
            case ES_ACTION_DONE:
            {
                CurrentState = Seesaw3_State_10;
                MotorHAL_SetSpeedCmdRPM(1, ROTATE_L_R_RPM, 0);
                MotorHAL_SetSpeedCmdRPM(0, ROTATE_L_R_RPM, 1);
                MotorHAL_DriveEncoderCount(0, 255);
                MotorHAL_DriveEncoderCount(1, 255);
                
            }
            break;
        }
    }
    break;
    
    case Seesaw3_State_10:
    {
        switch (ThisEvent.EventType)
        {
            case ES_ACTION_DONE:
            {
                MotorHAL_DriveEncoderCount(0, 600);
                MotorHAL_DriveEncoderCount(1, 600);
                MotorHAL_SetSpeedCmdRPM(1, 30, 0);
                MotorHAL_SetSpeedCmdRPM(0, 30, 0);

                SPI1Leader_SendCmd16(0x0015);  //Turn on Flywheel
                ES_Timer_InitTimer(SUCK_TIMER, 7500);
                //Return the bucket
                uint16_t E = (uint16_t)SPI1Leader_RequestResponse16(0x0025);
                uint16_t D = (uint16_t)SPI1Leader_RequestResponse16(0x0021);

                CurrentState = Seesaw3_State_11;
                ES_Timer_InitTimer(LINEFOLLOWING_TIMER, LineFollowing_MS);
            }
            break;
        }
    }
    break;
    
    case Seesaw3_State_11:
    {
        switch (ThisEvent.EventType)
        {
            case ES_ACTION_DONE:
            {
                //Drive backward to finish first suck
                uint16_t R = (uint16_t)SPI1Leader_RequestResponse16(0x0016);
                MotorHAL_DriveEncoderCount(0, 100);
                MotorHAL_DriveEncoderCount(1, 100);
                MotorHAL_SetSpeedCmdRPM(1, 30, 1);
                MotorHAL_SetSpeedCmdRPM(0, 30, 1);
                CurrentState = LineFollowingState_6;
                seesaw_num = 4;
                suck_count = 1;
                
            }
            break;
            case ES_TIMEOUT:
            {
                if(ThisEvent.EventParam == SUCK_TIMER){
                    //Drive backward to finish first suck
                    MotorHAL_DriveEncoderCount(0, 100);
                    MotorHAL_DriveEncoderCount(1, 100);
                    MotorHAL_SetSpeedCmdRPM(1, 30, 1);
                    MotorHAL_SetSpeedCmdRPM(0, 30, 1);
                    CurrentState = LineFollowingState_6;
                    seesaw_num = 4;
                    suck_count = 1;
                }
                // Line following
                if (ThisEvent.EventParam == LINEFOLLOWING_TIMER){
                    
                    uint16_t R = (uint16_t)SPI1Leader_RequestResponse16(0x0011);
                    if ((R == 0)){
                        ES_Timer_InitTimer(LINEFOLLOWING_TIMER, LineFollowing_MS);
                    }else if (R == 1){
                        ES_Timer_InitTimer(LINEFOLLOWING_TIMER, LineFollowing_MS);
                    }else{
                        MotorHAL_SetSpeedCmdRPM(1, base_speed + R - 100UL, 0);
                        MotorHAL_SetSpeedCmdRPM(0, base_speed - R + 100UL, 0);
                        ES_Timer_InitTimer(LINEFOLLOWING_TIMER, LineFollowing_MS);
                    }
                }
                
                
            }
            break;
        }
    }
    break;
    
    case Seesaw4_State_1:
    {
        switch (ThisEvent.EventType)
        {
            case ES_ACTION_DONE:
            {
                // Rotate 90 degrees
                MotorHAL_SetSpeedCmdRPM(1, 30, 1);
                MotorHAL_SetSpeedCmdRPM(0, 30, 0);
                MotorHAL_DriveEncoderCount(0, 250); //255 before
                MotorHAL_DriveEncoderCount(1, 250);
                CurrentState = Seesaw4_State_2;
            }
            break;
        }
    }
    break;
    
    case Seesaw4_State_2:
    {
        switch (ThisEvent.EventType)
        {
            case ES_ACTION_DONE:
            {
                MotorHAL_SetSpeedCmdRPM(1, 30, 1);
                MotorHAL_SetSpeedCmdRPM(0, 30, 1);
                
                //Rise the arm and stop the flywheel
                uint16_t A = (uint16_t)SPI1Leader_RequestResponse16(0x0022);
                uint16_t B = (uint16_t)SPI1Leader_RequestResponse16(0x0027);
                uint16_t C = (uint16_t)SPI1Leader_RequestResponse16(0x0016);
                
                ES_Timer_InitTimer(LINEFOLLOWING_TIMER, LineFollowing_MS);
            }
            break;
            case ES_TIMEOUT:
            {
                if (ThisEvent.EventParam == LINEFOLLOWING_TIMER){
                    // Line following until middle T
                    uint16_t R = (uint16_t)SPI1Leader_RequestResponse16(0x0012); 
                    if ((R == 0)){
                        
                        MotorHAL_SetSpeedCmdRPM(0, 30, 1); // used to be 20
                        MotorHAL_SetSpeedCmdRPM(1, 30, 1);
                        MotorHAL_DriveEncoderCount(0, 100);
                        MotorHAL_DriveEncoderCount(1, 100);
                        CurrentState = Seesaw4_State_3;
                        ES_Timer_InitTimer(LINEFOLLOWING_TIMER, LineFollowing_MS);
                    }else{
                        ES_Timer_InitTimer(LINEFOLLOWING_TIMER, LineFollowing_MS);
                    }
                }
            }
            break;
        }
    }
    break;
    
    
    case Seesaw4_State_3:
    {
        switch (ThisEvent.EventType)
        {
            case ES_TIMEOUT:
            {
                if (ThisEvent.EventParam == LINEFOLLOWING_TIMER){
                    // Line following until middle T
                    uint16_t R = (uint16_t)SPI1Leader_RequestResponse16(0x0012); 
                    if ((R == 0)){
                        MotorHAL_SetSpeedCmdRPM(0, 30, 1); // used to be 20
                        MotorHAL_SetSpeedCmdRPM(1, 30, 1);
                        MotorHAL_DriveEncoderCount(0, 50);
                        MotorHAL_DriveEncoderCount(1, 50);
                        CurrentState = Seesaw4_State_4;
                        ES_Timer_InitTimer(LINEFOLLOWING_TIMER, LineFollowing_MS);
                    }else{
                        ES_Timer_InitTimer(LINEFOLLOWING_TIMER, LineFollowing_MS);
                    }
                }
            }
            break;
        }
    }
    break;
    
    case Seesaw4_State_4:
    {
        switch (ThisEvent.EventType)
        {
            case ES_ACTION_DONE:
            {
                MotorHAL_SetSpeedCmdRPM(1, 30, 1);
                MotorHAL_SetSpeedCmdRPM(0, 30, 0);
                MotorHAL_DriveEncoderCount(0, 245); //255 before
                MotorHAL_DriveEncoderCount(1, 245);
                CurrentState = Seesaw4_State_5;
            }
            break;
            case ES_TIMEOUT:
            {
                
            }
            break;
        }
    }
    break;
    
    
    case Seesaw4_State_5:
    {
        switch (ThisEvent.EventType)
        {
            case ES_ACTION_DONE:
            {
                MotorHAL_SetSpeedCmdRPM(1, 30, 1);
                MotorHAL_SetSpeedCmdRPM(0, 30, 1);
                ES_Timer_InitTimer(LINEFOLLOWING_TIMER, LineFollowing_MS);
            }
            break;
            case ES_TIMEOUT:
            {
                if (ThisEvent.EventParam == LINEFOLLOWING_TIMER){                
                    uint16_t R = (uint16_t)SPI1Leader_RequestResponse16(0x0012);
                    if ((R == 0)){
                        ES_Timer_StopTimer(LINEFOLLOWING_TIMER);
                        CurrentState = Seesaw4_State_6;
                        MotorHAL_DriveEncoderCount(0, 110);
                        MotorHAL_DriveEncoderCount(1, 110);
                    }else if(R == 1){
                        MotorHAL_SetSpeedCmdRPM(1, base_speed, 1);
                        MotorHAL_SetSpeedCmdRPM(0, base_speed, 1);
                        ES_Timer_InitTimer(LINEFOLLOWING_TIMER, LineFollowing_MS);
                    }else{
                        MotorHAL_SetSpeedCmdRPM(1, base_speed + R - 100UL, 1);
                        MotorHAL_SetSpeedCmdRPM(0, base_speed - R + 100UL, 1);
                        ES_Timer_InitTimer(LINEFOLLOWING_TIMER, LineFollowing_MS);
                    }
                }
            }
            break;
        }
    }
    break;
    
    case Seesaw4_State_6:
    {
        switch (ThisEvent.EventType)
        {
            case ES_ACTION_DONE:
                {
                    uint16_t B = (uint16_t)SPI1Leader_RequestResponse16(0x0026);
                    ES_Timer_InitTimer(TRAPDOOR_TIMER, 5000);
                }
                break;
            case ES_TIMEOUT:
            {
                if (ThisEvent.EventParam == TRAPDOOR_TIMER){
                    //uint16_t E = (uint16_t)SPI1Leader_RequestResponse16(0x0025);
                    CurrentState = Seesaw5_State_1;
                    MotorHAL_SetSpeedCmdRPM(1, 30, 1);
                    MotorHAL_SetSpeedCmdRPM(0, 30, 0);
                    MotorHAL_DriveEncoderCount(0, 145); //255 before
                    MotorHAL_DriveEncoderCount(1, 145);
                }
            }
            break;
        }
    }
    break;
    
    
    case Seesaw5_State_1:
    {
        switch (ThisEvent.EventType)
        {
            case ES_ACTION_DONE:
                {
                    CurrentState = Seesaw5_State_2;
                    MotorHAL_SetSpeedCmdRPM(1, 30, 1);
                    MotorHAL_SetSpeedCmdRPM(0, 30, 1);
                    MotorHAL_DriveEncoderCount(0, 145); //255 before
                    MotorHAL_DriveEncoderCount(1, 145);
                }
                break;
           
        }
    }
    break;
    
    case Seesaw5_State_2:
    {
        switch (ThisEvent.EventType)
        {
            case ES_ACTION_DONE:
                {
                    uint16_t B = (uint16_t)SPI1Leader_RequestResponse16(0x0026);
                }
                break;
       
        }
    }
    break;
    
    
    
    
    case Stop_State:
    {
        switch (ThisEvent.EventType)
        {
            case ES_TIMEOUT:
            {
                if (ThisEvent.EventParam == GAME_TIMER){
                    RGB_TurnOff();
                    MotorHAL_SetSpeedCmdRPM(1, 0, 0);
                    MotorHAL_SetSpeedCmdRPM(0, 0, 0);
                    uint16_t B = (uint16_t)SPI1Leader_RequestResponse16(0x0016);
                }
            }
        }
    }
    
    
    
    
    
    
    
    
    
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

