
#include <Encoder.h>
#include <PID_v1.h>

#include "MotorControl.h"

MotorControl::MotorControl(
      char * motor_name,
      int enable_pin,
      int pwm_resolution,
      int direction_A_pin,
      int direction_B_pin,
      int quadrature_A_pin,
      int quadrature_B_pin,
      double pid_kp,
      double pid_ki,
      double pid_kd
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
        pid( &pidInput, &pidOutput, &pidSetpoint, pid_kp, pid_ki, pid_kd, DIRECT )
{
  // Set the hbridge control pins as outputs
  pinMode( pinEn,  OUTPUT);
  pinMode( pinDirA, OUTPUT);
  pinMode( pinDirB, OUTPUT);
    
  currPos = 0;
  desiredPos = 0;
  currentDirection = 0;
  
  lastPos = -1;
  lastVel = -1;
  
  pidInput = 0;
  pidSetpoint = 0;
  pidOutput = 0;

  // Turn the PID on
  pid.SetOutputLimits( -pwmRes, pwmRes );
  pid.SetMode(AUTOMATIC);  
}

void MotorControl::set_desired_position( long newPosition )
{
  desiredPos = newPosition;
}

void MotorControl::stop()
{
  // analogWrite on a digital pin causes the signal to be PWM'd.
  analogWrite( pinEn, 0); 
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

void MotorControl::go( int velocity )
{
  analogWrite( pinEn, velocity );
}

void MotorControl::update_position()
{
  currPos = encoder.read(); 
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
  
  
  return velocity;
}



// Expected to be called each 'loop' iteration
void MotorControl::manage_motor()
{
  int velocity;

  
  update_position();
  
  velocity = calculate_new_velocity();

  
  if( lastPos != currPos || 
      lastDes != desiredPos || 
      lastVel != velocity )
  {
    Serial.print( name );
    Serial.print( ": c=" );
    Serial.print( currPos );
    Serial.print( ", d=" );
    Serial.print( desiredPos );
    Serial.print( ", v=" );
    Serial.print( velocity );
    Serial.print( " " );
    Serial.println( "" );
    
    lastPos = currPos;
    lastDes = desiredPos;
    lastVel = velocity;
  }

  set_motor_speed( velocity );
}
