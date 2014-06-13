
#include <Encoder.h>
#include <PID_v1.h>

#include "MotorControl.h"

// Did some quick tuning of PID again after switching over to 7.2V battery.
// The PID is actually not really working very well... these motors don't
// activate on anything below 60 or so PWM (60 out of 255)... so that's probably
// around 1.6V or so.  Also, below that PWM value the motors are making an audible
// squeal.  The PWM frequency of the Arduinos needs to be changed to something 
// above 20KHz so that its not audible to humans.  I looked at that briefly, but
// I'll need to change around some of the pins to make it work correctly.

// All of this is a wash anyway because I will eventually need to move towards 
// managing the rate of change between the two motors rather than the absolute
// position in order to make it go in a straight line.

double lKp = .5;
double lKi = .05;
double lKd = .05;

double rKp = .5;
double rKi = .05;
double rKd = .05;

// Pin mappings - these will eventually be the real map

int lftEn = 6;
int lftA  = 8; // reversing these to get left motor to go correct dir
int lftB  = 7;

int rhtEn = 10;
int rhtA  = 12;
int rhtB  = 11;


int lftQa = 2;
int lftQb = 4;

int rhtQa = 3;
int rhtQb = 5;


MotorControl lft( "lft", lftEn, 255, lftA, lftB, lftQa, lftQb, lKp, lKi, lKd );  
MotorControl rht( "rht", rhtEn, 255, rhtA, rhtB, rhtQa, rhtQb, rKp, rKi, rKd );  


void setup()
{
  Serial.begin(115200);
  Serial.println( "dc motor with encoder test" );

  //setPwmFrequency( 6, 1 );
  //setPwmFrequency( 10, 1 );
  
}

void loop()
{
  if( Serial.available() )
  {
    long newPosition = Serial.parseInt();
  
    Serial.print( "New Postition: " );
    Serial.println( newPosition );
    
    lft.set_desired_position(newPosition);
    rht.set_desired_position(newPosition);

  } 
  
  lft.manage_motor();
  rht.manage_motor();
  
    
}





