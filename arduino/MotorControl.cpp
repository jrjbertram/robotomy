
#include <Encoder.h>
#include <PID_v1.h>

#include "due_pwm.h"
#include "MotorControl.h"


//#define PRINT( msg )    if( _stream ) { _stream->print  ( msg ); }
//#define PRINTLN( msg )  if( _stream ) { _stream->println( msg ); }

#define PRINT( msg )    
#define PRINTLN( msg )  

MotorControl::MotorControl(
      char * motor_name,
      int enable_pin,
      long pwm_resolution,
      int direction_A_pin,
      int direction_B_pin,
      int quadrature_A_pin,
      int quadrature_B_pin,
      double pid_kp,
      double pid_ki,
      double pid_kd,
      int invert
      )
      : name( motor_name ),
        pinEn( enable_pin ),
        pwmRes( pwm_resolution ),
        pinDirA( direction_A_pin ),
        pinDirB( direction_B_pin ),
        pinQuadA( quadrature_A_pin ),
        pinQuadB( quadrature_B_pin ),
        Kp( pid_kp ),
        Ki( pid_ki ),
        Kd( pid_kd ),
        encoder( quadrature_A_pin, quadrature_B_pin ),
        pid( &pidInput, &pidOutput, &pidSetpoint, pid_kp, pid_ki, pid_kd, DIRECT ),
        _stream( NULL )
{
  // Set the hbridge control pins as outputs
  pinMode( pinEn,  OUTPUT);
  pinMode( pinDirA, OUTPUT);
  pinMode( pinDirB, OUTPUT);
    
  currPos = 0;
  desiredPos = 0;
  currentDirection = 0;
  
  lastPos = 0;
  lastVel = 0;
  
  pidInput = 0;
  pidSetpoint = 0;
  pidOutput = 0;
  
  if( invert == 1 )
  {
    inverted = -1;
  }
  else
  {
    inverted = 1;
  }

  // Turn the PID on
  pid.SetOutputLimits( -pwmRes, pwmRes );
  pid.SetMode(AUTOMATIC);  
}

MotorControl::MotorControl(const MotorControl& other) :
        name( other.name ),
        pinEn( other.pinEn ),
        pwmRes( other.pwmRes ),
        pinDirA( other.pinDirA ),
        pinDirB( other.pinDirB ),
        pinQuadA( other.pinQuadA ),
        pinQuadB( other.pinQuadB ),
        Kp( other.Kp ),
        Ki( other.Ki ),
        Kd( other.Kd ),
        encoder( other.encoder ),
        pid( other.pid ),
        _stream( other._stream )
{
}

void MotorControl::operator=(const MotorControl& other)
{
  this->name = other.name;
  this->pinEn = other.pinEn;
  this->pwmRes = other.pwmRes;
  this->pinDirA = other.pinDirA;
  this->pinDirB = other.pinDirB;
  this->pinQuadA = other.pinQuadA;
  this->Kp = other.Kp;
  this->Ki = other.Ki;
  this->Kd = other.Kd;
  this->encoder = other.encoder;
  this->pid = other.pid;
  this->_stream = other._stream;
}
    

void MotorControl::set_desired_position( long newPosition )
{
  desiredPos = (newPosition * inverted);
  PRINT( name );
  PRINT( "new position set to " );
  PRINTLN( desiredPos );
}

long MotorControl::desired_position()
{
  return desiredPos * inverted;
}

long MotorControl::position()
{
  return lastPos * inverted;
}

void MotorControl::reset()
{
  desiredPos = currPos = 0;
}

void MotorControl::set_kp( double new_kp )
{
  Kp = new_kp;
  pid.SetTunings( Kp, Ki, Kd );
  PRINT( "Kp=" );
  PRINT( Kp );
  PRINT( ", Ki=" );
  PRINT( Ki );
  PRINT( ", Kd=" );
  PRINTLN( Kd );
}

void MotorControl::set_ki( double new_ki )
{
  Ki = new_ki;
  pid.SetTunings( Kp, Ki, Kd );
  PRINT( "Kp=" );
  PRINT( Kp );
  PRINT( ", Ki=" );
  PRINT( Ki );
  PRINT( ", Kd=" );
  PRINTLN( Kd );
}

void MotorControl::set_kd( double new_kd )
{
  Kd = new_kd;
  pid.SetTunings( Kp, Ki, Kd );
  PRINT( "Kp=" );
  PRINT( Kp );
  PRINT( ", Ki=" );
  PRINT( Ki );
  PRINT( ", Kd=" );
  PRINTLN( Kd );
}
    
void MotorControl::stop()
{
  // set duty cycle to 0 (off)
  pwm_write_duty( pinEn, 0); 
}

void MotorControl::forward()
{
  stop();
  
  digitalWrite( pinDirA, HIGH);
  digitalWrite( pinDirB, LOW);
}

void MotorControl::reverse()
{
  stop();
  
  digitalWrite( pinDirA, LOW );
  digitalWrite( pinDirB, HIGH );
}

void MotorControl::go( int speed )
{
  pwm_write_duty( pinEn, speed);
}

void MotorControl::update_position()
{
  currPos = encoder.read(); 
  PRINT( "Reading encoder position" );
  PRINTLN( currPos );
}

void MotorControl::set_motor_speed( int velocity )
{
  if( velocity > 0 && currentDirection != 1 )
  {
    forward();
    currentDirection = 1;
  }
  else if ( velocity < 0 && currentDirection != -1 )
  {
    reverse();
    currentDirection = -1;
  }
  else if ( velocity == 0 && currentDirection != 0 )
  {
    // note that velocity of zero will stop us below
    currentDirection = 0;
  }
  
  go( abs(velocity) );
}

int MotorControl::calculate_new_velocity()
{
  int velocity; 

//  long diff = desiredPos - currPos;
//  int absdiff = abs( desiredPos - currPos );
//  
//  if( abs( desiredPos - currPos ) < 100 )
//  {
//    // Its not worth worrying about.
//    // Note that I put this test in here to eliminate oscillations of the motor below a hand-determined threshold
//    
//    velocity = 0;
//  }
//  else
//  { 
    // Convert our integer values into doubles
    pidInput = currPos;
    pidSetpoint = desiredPos;
    
    
    // Perform the PID calculation
    pid.Compute();

    // Convert the doulbe output back to an integer
    velocity = (int)pidOutput;
//  }
  
  PRINT( "New velocity: " );
  PRINTLN( velocity );
  
  return velocity;
}



// Expected to be called each 'loop' iteration
void MotorControl::manage_motor()
{
  int velocity;

  PRINTLN( "Updating position" );
  update_position();
  
  PRINTLN( "Calculating velocity" );
  velocity = calculate_new_velocity();

  
  PRINT( "lastpos(" );
  PRINT( lastPos );
  PRINT( ") != currPos(" );
  PRINT( currPos );
  PRINT( ") || lastDes(" );
  PRINT( lastDes );
  PRINT( ") != desiredPos (" );
  PRINT( desiredPos );
  PRINT( ") || lastVel(" );
  PRINT( lastVel );
  PRINT( ") != velocity( " );
  PRINT( velocity );
  PRINTLN( ")" );

  if( lastPos != currPos || 
      lastDes != desiredPos || 
      lastVel != velocity )
  {
    PRINT( name );
    PRINT( ": c=" );
    PRINT( currPos );
    PRINT( ", d=" );
    PRINT( desiredPos );
    PRINT( ", v=" );
    PRINT( velocity );
    PRINT( " " );
    PRINTLN( "" );
    
    lastPos = currPos;
    lastDes = desiredPos;
    lastVel = velocity;
  }

  set_motor_speed( velocity );
}
