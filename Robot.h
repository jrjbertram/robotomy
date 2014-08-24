
#ifndef ROBOT_H
#define ROBOT_H
#define LIBRARY_VERSION	1.0.0

#include "MotorControl.h"


class Robot
{
  // Create a convenience class to contain the "autonomous" robot logic/algorithms.  Separating this out to keep the main *.ino file 
  // manageable.  The main file will deal with the mechanics of getting input and providing this class with the glue logic, but this
  // class will deal with managing the sensors, motion planning, and motor control at a high level.
  
  public:
  
  // Define a state machine for our robot
  typedef enum   {
    ROBOT_IDLE,  // nothing is activated, no inputs or outputs processed
    ROBOT_DIAG,  // diagnostic mode, inputs are processed and reported, output is manually driven from outside this class
    ROBOT_AUTO,  // fully autonomous mode -- look out!
  } RobotMode;

  Robot( MotorControl & lft, MotorControl & rht);
  
  int setMode( RobotMode mode ) { _mode = mode; }
  
  int tick_occurred( Stream & stream );
  int reset();

  private:
  RobotMode _mode;
  
  MotorControl _lft;
  MotorControl _rht;
    
  int autonomous_reset();
  int autonomous_tick_ocurred( Stream & stream );
  
};



#endif

