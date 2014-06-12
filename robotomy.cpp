

#include <Encoder.h>
#include <PID_v1.h>

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
//PID myPID(&Input, &Output, &Setpoint, 0.05, 0, 0, DIRECT);
PID myPID(&Input, &Output, &Setpoint, .2, 0, 0, DIRECT);   // this one works pretty well it looks like




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

// Compatibility with current code

int en  = rhtEn; 
int inA = rhtA;
int inB = rhtB;

int quadA = rhtQa;
int quadB = rhtQb;

Encoder leftEncoder = Encoder( quadA, quadB );
long positionLeft = -999;
long desiredPosition = -999;

void setup()
{
  Serial.begin(115200);
  Serial.println( "dc motor with encoder test" );
  
  pinMode( en, OUTPUT);
  pinMode( inA, OUTPUT);
  pinMode( inB, OUTPUT);

  desiredPosition = positionLeft = leftEncoder.read();
  Input = positionLeft;
  Setpoint = desiredPosition;
  
    //turn the PID on
  myPID.SetOutputLimits( -255, 255 );
  myPID.SetMode(AUTOMATIC);

}

void stop()
{
  analogWrite(en, 0);
}

void forward()
{
  stop();
  
  digitalWrite(inA, HIGH);
  digitalWrite(inB, LOW);
}

void reverse()
{
  stop();
  
  digitalWrite( inA, LOW );
  digitalWrite( inB, HIGH );
}

void go( int velocity )
{
  analogWrite( en, velocity );
}

long prompt_for_new_position()
{
  if( Serial.available() )
  {
    long newPosition = Serial.parseInt();
  
    Serial.print( "Current position:  " );
    Serial.println( positionLeft );
  
    Serial.print( "New Postition: " );
    Serial.println( newPosition );
    
    return newPosition;
  } else {
    return desiredPosition; 
  }
}


void get_current_position()
{
  long newLeft;
  
  newLeft = leftEncoder.read();
  
  if( newLeft != positionLeft )
  {
    //Serial.print( "Current Position: " );
    //Serial.println( newLeft );
    positionLeft = newLeft;
  }
  
}

int directionLeft = 0;

void set_motor_speed( int velocity )
{
  if( velocity > 0 && directionLeft != 1 )
  {
    Serial.println( "forward" );
    forward();
    directionLeft = 1;
  }
  else if ( velocity < 0 && directionLeft != -1 )
  {
    Serial.println( "reverse" );
    reverse();
    directionLeft = -1;
  }
  else if ( velocity == 0 && directionLeft != 0 )
  {
    Serial.println( "stop" );
    stop();
    directionLeft = 0;
  }
  
  go( abs(velocity) );
}


int calculate_new_velocity( long desiredPosition, long positionLeft )
{
  int velocity; 

  if( abs( desiredPosition - positionLeft ) < 250 )
  {
    // Its not worth worrying about
  
    velocity = 0;
  }
  else
  {  
    Input = positionLeft;
    Setpoint = desiredPosition;
    myPID.Compute();
    velocity = (int)Output;
  }
  
  
  return velocity;
}

void loop()
{
  desiredPosition = prompt_for_new_position();
  
  int velocity = calculate_new_velocity( desiredPosition, positionLeft );
 
  if( velocity != 0 )
  {
    Serial.print( "x=" );
    Serial.print( positionLeft );
    Serial.print( " x'=" );
    Serial.print( desiredPosition );
    Serial.print( " , v=" );
    Serial.println( velocity );
  }
  
  set_motor_speed( velocity );

  
  get_current_position();
    
}




