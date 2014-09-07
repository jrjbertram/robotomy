
#ifndef ROBOT_H
#define ROBOT_H
#define LIBRARY_VERSION	1.0.0

#include "MotorControl.h"

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_9DOF.h>

#include "IRSensor.h"


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

    Robot(
      MotorControl * lft,
      MotorControl * rht,
      Adafruit_LSM303_Accel_Unified * accel,
      Adafruit_LSM303_Mag_Unified   * mag,
      Adafruit_L3GD20_Unified       * gyro,
      Adafruit_9DOF                 * dof,
      IRSensor                      * ir
    );

    void setStream( Stream * stream ) {
      _stream = stream;
      
      if( _lft ) _lft->setStream( _stream );
      if( _rht ) _rht->setStream( _stream );
    }

    int setMode( RobotMode mode ) {
      _mode = mode;
    }

    int tick_occurred();
    int reset();
    
    void getStatusString( String & result  );

  private:
    RobotMode _mode;

    MotorControl * _lft;
    MotorControl * _rht;

    Adafruit_LSM303_Accel_Unified * _accel;
    Adafruit_LSM303_Mag_Unified   * _mag;
    Adafruit_L3GD20_Unified       * _gyro;
    Adafruit_9DOF                 * _dof;
    
    IRSensor                      * _ir;
    
    Stream * _stream;

    int autonomous_tick_ocurred();
    int autonomous_reset();


};



#endif

