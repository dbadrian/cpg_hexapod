#include <math.h>     //standard c library

#include <Servo.h>    // servo arduino library
#include <NewPing.h>  // faster ultrasounds library

#include "kuramoto.h"

#define PLOT_ON 
//#define DEBUG
#define SENSORY
#define MOTORS_ON

#define SONAR_NUM      3 // Number or sensors.
#define MAX_DISTANCE  50 // Max distance in cm.
#define PING_INTERVAL 30// Milliseconds between pings.

#define S_LEFT_DEFAULT 74
#define S_MIDDLE_DEFAULT 90
#define S_RIGHT_DEFAULT 74


#define  SM_CYCLE_TIME_MS   30

#define  SM_INPUT_STOP   0x0D
#define  SM_INPUT_FWD_LOWSPEED   0x00
#define  SM_INPUT_FWD_HISPEED    0x02

#define  SM_INPUT_CCWR   0x05
#define  SM_INPUT_CWR    0x01

#define  SM_MASK_STOP  0x0D
#define  SM_MASK_FWD_LOWSPEED   0x03
#define  SM_MASK_FWD_HISPEED   0x03
#define  SM_MASK_CCWR 0x0D
#define  SM_MASK_CWR  0x05

#define D_F_MIN_IN_CM 15
#define D_B_MIN_IN_CM 15
#define D_L_MIN_IN_CM 15
#define D_R_MIN_IN_CM 15

/*------------------------------------------------------------------------------
* Hardware Variables
*-----------------------------------------------------------------------------*/
//////////////////////////////////////
// Servos
//////////////////////////////////////
Servo servos[3]; 
int servo_offsets[3] = {S_MIDDLE_DEFAULT, S_RIGHT_DEFAULT, S_LEFT_DEFAULT};


//////////////////////////////////////
// Ultrasound Sensors
//////////////////////////////////////
int UltraSoundPinMap[3][2] = {
  //T, E
  {40, 41},  // Back
  {32, 33}, // Left
  {36, 37} // Right

  // {44, 45}//FRONT
};

enum USensors{ UltraSoundBack =0,
               UltraSoundLeft =1, 
               UltraSoundRight=2, 
               UltraSoundFront=3 };

// temporary variable
int ping_duration = 0;

unsigned long pingTimer[SONAR_NUM] = {0}; // When each pings.
//Initialize cm array, such that if a sensor its missing its basically set to
//infinity...
unsigned int cm[SONAR_NUM] = {500,500,500}; ///{500};//// Stores ping distances.
uint8_t currentSensor = 0; // Which sensor is active.

NewPing sonar[SONAR_NUM] =
  {
    NewPing(UltraSoundPinMap[0][0], UltraSoundPinMap[0][1], MAX_DISTANCE),
    NewPing(UltraSoundPinMap[1][0], UltraSoundPinMap[1][1], MAX_DISTANCE),
    NewPing(UltraSoundPinMap[2][0], UltraSoundPinMap[2][1], MAX_DISTANCE)
  };

unsigned long StartTime = 0;
unsigned long ElapsedTime = 0;
unsigned long ElapsedTime_ms_u32 = 0;

float alpha = 0.01; // factor to tune


/*------------------------------------------------------------------------------
* Subroutine Declarations
*-----------------------------------------------------------------------------*/
void plotData(bool plot_motors, bool plot_ultrasound);

void echoCheck() { // If ping echo, set distance to array.
  if (sonar[currentSensor].check_timer())
  {
    int measurement = sonar[currentSensor].ping_result / US_ROUNDTRIP_CM;
    if (measurement > 2)
    {
//      cm[currentSensor] = alpha * measurement + (1-alpha) * cm[currentSensor];
      cm[currentSensor] = measurement;
    }
  }
}

/*------------------------------------------------------------------------------
* Hardware Initialization
*-----------------------------------------------------------------------------*/
void setup() {
  #if defined(DEBUG) || defined(PLOT_ON)
    Serial.begin(500000);
  #endif

  #ifdef MOTORS_ON
  // Setting Up Servos
  servos[0].attach(9);
  servos[1].attach(10);
  servos[2].attach(11);
  servos[0].write(S_MIDDLE_DEFAULT); // TODO Set correct default angles
  servos[1].write(S_RIGHT_DEFAULT);
  servos[2].write(S_LEFT_DEFAULT);
  #endif

  #ifdef SENSORY
  pingTimer[0] = millis() + 75; // First ping start in ms.
  for (uint8_t i = 1; i < SONAR_NUM; i++)
    pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;
  #endif
}

/*------------------------------------------------------------------------------
* Main Loop
*-----------------------------------------------------------------------------*/
void loop() {
  StartTime = millis();
  
  ////////////////////////////////////////////
  ////   Behavior    /////////////////////////
  ////////////////////////////////////////////
  // Only call updateBehavior if measurements have been updated
  for (uint8_t i = 0; i < SONAR_NUM; i++) {
    if (millis() >= pingTimer[i]) {
      pingTimer[i] += PING_INTERVAL * SONAR_NUM;
      if (i == 0 && currentSensor == SONAR_NUM - 1)
        stateMachine();
        

      sonar[currentSensor].timer_stop();
      currentSensor = i;
      cm[currentSensor] = 500;
      sonar[currentSensor].ping_timer(echoCheck);
    }
  }

  ////////////////////////////////////////////
  ////   Update OSCILLATORS   ////////////////
  ////////////////////////////////////////////
  updateOscillators();

  ////////////////////////////////////////////
  ////   Motor Update   //////////////////////
  ////////////////////////////////////////////
  // int delay = Tms-ElapsedTime;
  #ifdef MOTORS_ON
  updateMotorPositions();
  #endif

  #ifdef DEBUG
  ElapsedTime_ms_u32 = (millis() - StartTime);
  Serial.println(ElapsedTime_ms_u32);  
  #endif
  
  ////////////////////////////////////////////
  ////   Plotting   //////////////////////////
  ////////////////////////////////////////////
  #ifdef PLOT_ON
  plotData(true, false, false);
  #endif

}

/*------------------------------------------------------------------------------
* Subroutines
*-----------------------------------------------------------------------------*/
void stateMachine()
{
  unsigned int sm_u32 = 0;

  // Rest Behavior
  switch_gait(gait_type::FWD);
  change_speed(W_default);
  
//  if(cm[UltraSoundFront] < D_F_MIN_IN_CM)
//   {
//       sm_u32 = sm_u32 | 0x1; 
//   }
  if(cm[UltraSoundBack] < D_B_MIN_IN_CM)
  {
    sm_u32 = sm_u32 | 0x2;     
  }
  if(cm[UltraSoundRight] < D_R_MIN_IN_CM)
  {
    sm_u32 = sm_u32 | 0x1; 
    sm_u32 = sm_u32 | 0x4;   
  }   
  if(cm[UltraSoundLeft] < D_L_MIN_IN_CM)
  {
    sm_u32 = sm_u32 | 0x1; 
    sm_u32 = sm_u32 | 0x8;     
  }

  if( (sm_u32 & SM_MASK_FWD_HISPEED) == SM_INPUT_FWD_HISPEED )
  {   
     switch_gait(gait_type::FWD);
     change_speed(W_max);
     #ifdef DEBUG
     Serial.print("FWD HS \n"); 
     #endif
  }
  else if ((sm_u32 & SM_MASK_FWD_LOWSPEED) == SM_INPUT_FWD_LOWSPEED)
  {
     switch_gait(gait_type::FWD);
     change_speed(W_default);
     #ifdef DEBUG
     Serial.print("FWD LS \n");  
     #endif
  }
  else if ((sm_u32 & SM_MASK_STOP) == SM_INPUT_STOP)
  {
     switch_gait(gait_type::BWD);
     change_speed(W_max);
     #ifdef DEBUG
     Serial.print("STOP \n");    
     #endif
  }
  else if ((sm_u32 & SM_MASK_CWR) == SM_INPUT_CWR)
  {
     switch_gait(gait_type::CWR);
     change_speed(W_default);
     #ifdef DEBUG
     Serial.print("CWR \n");    
     #endif
  }
  else if ((sm_u32 & SM_MASK_CCWR) == SM_INPUT_CCWR)
  {
     switch_gait(gait_type::CCWR);
     change_speed(W_default);
     #ifdef DEBUG
     Serial.print("CCWR \n");    
     #endif
  }
       
}/* end of stateMachine */

void updateMotorPositions()
{
  for(int i=0; i<N; i++)
  {
    // Shift by 90 to account for how Arduino env addresses its servos
    servos[i].write(output[i] + servo_offsets[i]);
  }
}

#ifdef PLOT_ON
void plotData(bool plot_motors, bool plot_ultrasound, bool loop_time)
{
  if(plot_ultrasound)
  {
    for (uint8_t i = 0; i < SONAR_NUM; i++) {
      Serial.print(cm[i]);
      Serial.print(" ");
    }
    Serial.println();
  }
  
  if(plot_motors)
  {
    Serial.print(output[0]);
    Serial.print(" ");
    Serial.print(output[1]);
    Serial.print(" ");
    Serial.println(output[2]);
  }

  if(loop_time)
  {
    Serial.println(ElapsedTime);
  }
}
#endif
