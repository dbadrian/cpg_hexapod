#include <math.h>     //standard c library

#include <Servo.h>    // servo arduino library

/*------------------------------------------------------------------------------
* Hardware Variables
*-----------------------------------------------------------------------------*/
// Servos
Servo servos[3]; 

/*------------------------------------------------------------------------------
* Hardware Initialization
*-----------------------------------------------------------------------------*/
void setup() {

  #ifdef PLOT_ON
    // Serial Communication
    Serial.begin(500000);
  #endif

  // Setting Up Servos
  servos[0].attach(9);
  servos[1].attach(10);
  servos[2].attach(11);
  servos[0].write(74); // 74 is a good default
  servos[1].write(90); //82 -- 98
  servos[2].write(70); // 70
}

/*------------------------------------------------------------------------------
* Main Loop
*-----------------------------------------------------------------------------*/
void loop() {

}
