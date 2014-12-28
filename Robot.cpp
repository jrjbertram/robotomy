
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

int Robot::refresh_sensors()
{
  sensors_event_t accel_event;
  sensors_event_t mag_event;

  /* Read the accelerometer and magnetometer */
  _accel->getEvent(&accel_event);
  _mag->getEvent(&mag_event);

  /* Use the new fusionGetOrientation function to merge accel/mag data */  
  if (_dof->fusionGetOrientation(&accel_event, &mag_event, &_orientation))
  {
  }
}

// This should get called each "loop" funciton iteration.  Depending on what mode we're in, different operations will be performed
int Robot::tick_occurred()
{
  refresh_sensors();
  
  if( _mode == ROBOT_DIAG )
  {
    //PRINTLN( "managing motors" );
    if( _lft ) _lft->manage_motor();
    if( _rht)  _rht->manage_motor();
  }
  else if( _mode == ROBOT_AUTO )
  {
    //PRINT( "autononomous" );
    this->autonomous_tick_ocurred();
    if( _lft ) _lft->manage_motor();
    if( _rht)  _rht->manage_motor();    
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
  WANDER,
  HIT,
  
} PlanningState;

PlanningState planState = RESET;

int Robot::autonomous_reset()
{
  planState = RESET;
}

void Robot::getStatusString( String & msg )
{
  // Generate a string that represents the state of our robot at this instant in time
  // Note that due to limitations in the size of a network packet, I've had to
  // abbreviate the field names down to something that will fit within 100 chars.
  // Anything larger will currently crash the network device I'm using.
  
  // Note that any changes in these strings will require changes in the python GUI as well.
  if( _mode == ROBOT_IDLE )
  {
    msg = String( "Md=I" ); 
  }
  else if( _mode == ROBOT_DIAG )
  {
    msg = String( "Md=D" );
  }
  else if( _mode == ROBOT_AUTO )
  {
    msg = String( "Md=A" );
  }
  else
  {
    msg = String( "Md=U" );
  }

  switch( planState )
  {
    case RESET:
      msg += ":Pl=R";
      break;
    case INIT:
      msg += ":Pl=I";
      break;
    case WANDER:
      msg += ":Pl=W";
      break;
    case HIT:
      msg += ":Pl=H";
      break;
    default:
      msg += ":Pl=U";
      break;
  }

  // Let's spit out a live view of the state of our sensors
  msg += F(":R=");
  msg += _orientation.roll;
  msg += F(":P=");
  msg += _orientation.pitch;
  msg += F(":Y=");
  msg += _orientation.heading;
   
  
  msg += F(":IR=");
  msg += _ir->getDistance();
  
  msg += F(":Lf=");
  msg += _lft->velocity();
  msg += F(":Rt=");
  msg += _rht->velocity();
  msg += F(":Ld=");
  //msg += _lft->desired_position() - _lft->position();
  msg += _lft->desired_position();
  msg += F(":Rd=");
  //msg += _rht->desired_position() - _rht->position();
  msg += _rht->desired_position();
  
}


int Robot::autonomous_tick_ocurred()
{
  if( planState == RESET )
  {
    planState = INIT;
  }
  else if( planState == INIT )
  {
    if( _orientation.heading > -30.0 && _orientation.heading < 30.0 )
    {
      planState = WANDER; // what's after init?
    }
    else
    {
      // Start going left until we hit a heading of approx 0 
      _lft->set_desired_position( _lft->position() -400 );
      _rht->set_desired_position( _rht->position() +400 );
    }
  }
  else if( planState == WANDER )
  {
//    Serial.print( "lft: " );
//    Serial.print( _lft->position() );
//    Serial.print( " (" );
//    Serial.print( _lft->desired_position() );
//    Serial.print( " )" );
//    Serial.print( ", rht: " );
//    Serial.print( _rht->position() );
//    Serial.print( " (" );
//    Serial.print( _rht->desired_position() );
//    Serial.print( " )" );
//    Serial.println( "" );
    if( _ir->getDistance() < 20 )
    {
      planState = HIT;
    }
    else  
    {
      // Start forward 
      _lft->set_desired_position( _lft->position() +500 );
      _rht->set_desired_position( _rht->position() +500 );

//    Serial.print( "    set to: " );
//    Serial.print( "lft: " );
//    Serial.print( _lft->position() );
//    Serial.print( " (" );
//    Serial.print( _lft->desired_position() );
//    Serial.print( " )" );
//    Serial.print( ", rht: " );
//    Serial.print( _rht->position() );
//    Serial.print( " (" );
//    Serial.print( _rht->desired_position() );
//    Serial.print( " )" );
//    Serial.println( "" );
    }
  }
  else if( planState == HIT )
  {
    if( _ir->getDistance() > 20 )
    {
      planState = WANDER;
    }
    else
    {
      // Start going right
      _lft->set_desired_position( _lft->position() +300 );
      _rht->set_desired_position( _rht->position() -300 );
    }
  }
}

