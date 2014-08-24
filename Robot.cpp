
#include "Robot.h"

Robot::Robot( MotorControl & lft, MotorControl & rht )
  :
  _mode( ROBOT_IDLE ),
  _lft( lft ),
  _rht( rht )
{

}


int Robot::reset()
{
  _lft.reset();
  _rht.reset();
  _mode = ROBOT_IDLE;
}

// This should get called each "loop" funciton iteration.  Depending on what mode we're in, let's made some calls on what we want to do.
int Robot::tick_occurred( Stream & stream )
{
  if( _mode == ROBOT_DIAG )
  {
    _lft.manage_motor();
    _rht.manage_motor();
  }
  else if( _mode = ROBOT_AUTO )
  {
    this->autonomous_tick_ocurred( stream );
  }
}

// All autonomous action happens here.  Note that I'm going to be cheating here.. I'm going to be creating some global types, variables, etc in this
// file rather than putting them in the class.  One reason is there will only ever be one "instance" of a Robot class, so I don't need to worry about
// multiple instances trashing the global variables.  The other reason is I don't want to have to flip back and forth to the class definition and 
// put the variables there.  There really isn't any benefit to this that I can see right now.  I can leave it as a code cleanup activity once I"m through
// some of my prototyping.





int Robot::autonomous_tick_ocurred( Stream & stream )
{
  
}

