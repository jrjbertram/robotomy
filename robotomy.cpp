
#include <Encoder.h>
#include <PID_v1.h>

#include "MotorControl.h"


double Kp = 0.2;
double Ki = 0;
double Kd = 0;

// Pin mappings - these will eventually be the real map

int lftEn = 10;
int lftA = 12;  // I have these backwards I think
int lftB = 11;

int rhtEn = 6;
int rhtA  = 8;
int rhtB  = 7;

int lftQa = 3;
int lftQb = 5;

int rhtQa = 2;
int rhtQb = 4;


MotorControl lft( lftEn, 255, lftA, lftB, lftQa, lftQb, Kp, Ki, Kd );  
MotorControl rht( rhtEn, 255, rhtA, rhtB, rhtQa, rhtQb, Kp, Ki, Kd );  


void setup()
{
  Serial.begin(115200);
  Serial.println( "dc motor with encoder test" );
  
}

void loop()
{
  if( Serial.available() )
  {
    long newPosition = Serial.parseInt();
  
    Serial.print( "New Postition: " );
    Serial.println( newPosition );
    
    lft.set_desired_position(newPosition);

  } 
  
  lft.manage_motor();
  rht.manage_motor();
  
    
}




