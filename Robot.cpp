
#include "Robot.h"

#define PRINT( msg )    if( _stream ) { _stream->print  ( msg ); }
#define PRINTLN( msg )  if( _stream ) { _stream->println( msg ); }

Robot::Robot( 
    MotorControl * lft, 
    MotorControl * rht,
    Adafruit_LSM303_Accel_Unified * accel,
    Adafruit_LSM303_Mag_Unified   * mag,
    Adafruit_L3GD20_Unified       * gyro,
    Adafruit_9DOF                 * dof,
    IRSensor                      * ir
  )
  :
  _mode( ROBOT_IDLE ),
  _lft( lft ),
  _rht( rht ),
  _accel( accel ),
  _mag( mag ),
  _gyro( gyro ),
  _dof( dof ),
  _ir( ir ),
  _stream( NULL )
{
  // Note, intentionally using pointers here to refer to classes that have already been instantiated.  I don't actually
  // want to create copies of the classes, since they refer to actual devices out there on my robot.  If I don't use
  // pointers to the classes, then I'll be invoking copy constructors.  If copy constructors are invoked, then I'll 
  // separate instances of what are truly a single global resources on the robot.  If there are separate instances, then
  // they can get out of sync with each other.. it creates a lot of undesirable behaviors between this Robot class and
  // the main file.  Sometimes, C++ features work against you... this is the way to work around them.
}

int Robot::reset()
{
  // Reset the motor controllers
  if( _lft ) _lft->reset();
  if( _rht ) _rht->reset();
  
  // Reset our internal mode
  _mode = ROBOT_IDLE;
  
  // Reset our autonomous operation logic
  this->autonomous_reset();
}

// This should get called each "loop" funciton iteration.  Depending on what mode we're in, different operations will be performed
int Robot::tick_occurred()
{
  
  if( _mode == ROBOT_DIAG )
  {
    PRINTLN( "managing motors" );
    if( _lft ) _lft->manage_motor();
    if( _rht)  _rht->manage_motor();
  }
  else if( _mode == ROBOT_AUTO )
  {
    PRINT( "autononomous" );
    this->autonomous_tick_ocurred();
  }
}

// All autonomous action happens here.  Note that I'm going to be cheating here.. I'm going to be creating some global types, variables, etc in this
// file rather than putting them in the class.  One reason is there will only ever be one "instance" of a Robot class, so I don't need to worry about
// multiple instances trashing the global variables.  The other reason is I don't want to have to flip back and forth to the class definition and 
// put the variables there.  There really isn't any benefit to this that I can see right now.  I can leave it as a code cleanup activity once I"m through
// some of my prototyping.

typedef enum
{
  RESET,
  INIT,
  
} PlanningState;

PlanningState planState = RESET;

int Robot::autonomous_reset()
{
  planState = RESET;
}

void Robot::getStatusString( String & msg )
{
  if( _mode == ROBOT_IDLE )
  {
    msg = String( "Mode=IDLE:" ); 
  }
  else if( _mode == ROBOT_DIAG )
  {
    msg = String( "Mode=DIAG:" );
  }
  else if( _mode == ROBOT_AUTO )
  {
    msg = String( "Mode=AUTO:" );
  }
  else
  {
    msg = String( "Mode=UNKNOWN:" );
  }
  
  // Let's spit out a live view of the state of our sensors
  sensors_event_t accel_event;
  sensors_event_t mag_event;
  sensors_vec_t   orientation;

  /* Read the accelerometer and magnetometer */
  _accel->getEvent(&accel_event);
  _mag->getEvent(&mag_event);

  /* Use the new fusionGetOrientation function to merge accel/mag data */  
  if (_dof->fusionGetOrientation(&accel_event, &mag_event, &orientation))
  {
    /* 'orientation' should have valid .roll and .pitch fields */
    msg += F("Roll=");
    msg += orientation.roll;
    msg += F(":Pitch=");
    msg += orientation.pitch;
    msg += F(":Heading=");
    msg += orientation.heading;
    msg += F(":");
  }   
  
  msg += F("IR=");
  msg += _ir->getDistance();
  
}

int Robot::autonomous_tick_ocurred()
{

}

