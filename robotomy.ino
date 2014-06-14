
#include <Encoder.h>
#include <PID_v1.h>

#include "due_pwm.h"  // library to allow custom PWM frequencies
#include "MotorControl.h"

// PID tuning constants.  Currently using same values for both left and right, but
// have it set up so I can easily use individual values if needed.
#define KP  2.5
#define KI  0.0
#define KD  0.0

double lKp = KP;
double lKi = KI;
double lKd = KD;

double rKp = KP;
double rKi = KI;
double rKd = KD;

// Pin mappings - these will eventually be the real map

// Notes:
//   - Pins chosen assuming Arduino Due pin capabilities.  Won't work on other Aruduino variants such as Mega or Uno.
//
//   - On Due, pins 6, 7, 8, and 9 can be used for special high-frequency PWM signals.  Normally the frequency is 500Hz I believe,
//     but when driving motors, at low duty cycles this creates an audible squeal.  Upping the PWM frequency above 20KHz puts it
//     outside the range of human hearing, so while the squeal still happens, you just can't hear it.  Note that I've read in various
//     forums that not all motor drivers support high PWMs like this.  I built my own based on an L298N, which does.. I haven't noticed
//     any ill effects so far.
//
//   - My motor driver design uses 3 inputs.. an "enable" signal, and an "A" and "B" input.  Truth table for the signals:
//          En  A  B  |  Result
//         =====================
//           0  x  x  | Off
//           1  1  0  | Forward
//           1  0  1  | Reverse
//           1  1  1  | Active Braking (which I don't use)
//
//   - I purchased motors with quadrature encoders.  Each encoder generates an "A" and "B" signal.  I'm using the Encoder library to
//     decode the encoder signals.  The Encoder library requires that at least the "A" signal be on a pin that can cause an interrupt, 
//     and recommends that both the A and B pins are both capable of interrupts.
//   
//   - On Uno, I had previously needed to use pins 2 and 3 for Encoder input as they were the only two pins that supported interrupts.
//     On Due, I believe many more pins (all of them maybe?) can be configured to generate an interrupt, so I may be able to move 
//     those around.  I should get better resolution now, too, since both the A and B pins will generate interrupts now.


int lftEn = 6;
int lftA  = 8; 
int lftB  = 7;

int rhtEn = 9; // needs to move to 7, 8, or 9 so I can PWM it.
int rhtA  = 12;
int rhtB  = 11;


int lftQa = 2;
int lftQb = 4;

int rhtQa = 3;
int rhtQb = 10;


#define PWM_BITS        12     // On Due, you can change the resolution from 8 bits (0-255) to 12 bits (0-4095).
#define PWM_RESOLUTION  4095   // 
#define PWM_FREQUENCY   20000  // In Hz.  Frequency of the overall PWM period... not the duty cycle
#define PWM_CLOCK       1      // 1="clock A", 2="clock B".  Note I'm choosing to use the same PWM clock for both PWM outputs.

MotorControl lft( "lft", lftEn, PWM_RESOLUTION, lftA, lftB, lftQa, lftQb, lKp, lKi, lKd, 0 );  
MotorControl rht( "rht", rhtEn, PWM_RESOLUTION, rhtA, rhtB, rhtQa, rhtQb, rKp, rKi, rKd, 1 );  


void setup()
{
  Serial.begin(115200);
  Serial.println( "dc motor with encoder test" );
  
  // On the Due, set the PWM resolution to 12 bits.  Won't compile for other board types.  Also requires
  // arduino 1.5.6-r2 (BETA) or later.
  //analogWriteResolution( 12 );
  pwm_set_resolution( PWM_BITS );
  
  pwm_setup( lftEn, PWM_FREQUENCY, 1 );
  pwm_setup( rhtEn, PWM_FREQUENCY, 1 ); 
  
  //setPwmFrequency( 5, DIVISOR_62500_HZ );  // this also sets it for pin 6
  //setPwmFrequency( 6, ... );             // so this is unnecessary
  
}

void loop()
{
  if( Serial.available() )
  {
    char cmd = Serial.read();
    
    switch( cmd )
    {
      case 'L':
      case 'R':
      case 'B':
      {
        long newPosition = Serial.parseInt();

        Serial.print( cmd );
        Serial.print( " new position: " );
        Serial.println( newPosition );
    
        if( cmd == 'L' || cmd == 'B' )
        {
          lft.set_desired_position(newPosition);
        }
        
        if( cmd == 'R' || cmd == 'B' )
        {
          rht.set_desired_position(newPosition);
        }
      }
      break;
      case 'K':
      {
        double kp = lft.get_kp();
        double ki = lft.get_ki();
        double kd = lft.get_kd();
        
        Serial.print( "Kp=" );
        Serial.print( kp );
        Serial.print( ", Ki=" );
        Serial.print( ki );
        Serial.print( ", Kd=" );
        Serial.println( kd );
      }
      break;      case 'P':
      {
        double k = Serial.parseFloat();
        lft.set_kp( k );
        rht.set_kp( k );
      }
      break;
      case 'I':
      {
        double k = Serial.parseFloat();
        lft.set_ki( k );
        rht.set_ki( k );
      }
      break;
      case 'D':
      {
        double k = Serial.parseFloat();
        lft.set_kd( k );
        rht.set_kd( k );
      }
      break;
      case 'X':
      {
        lft.reset();
        rht.reset();
      }
      break;
      case '?':
      default:
      {
        Serial.println( "Commands:" );
        Serial.println( "" );
        Serial.println( "  Position: " );
        Serial.println( "    L<int>  - Left Motor Only" );
        Serial.println( "    R<int>  - Right Motor Only" );
        Serial.println( "    B<int>  - Both Motors" );
        Serial.println( "" );
        Serial.println( "  PID Constants: " );
        Serial.println( "    K         - Print current values" );
        Serial.println( "    P<float>  - Kp, proportional" );
        Serial.println( "    I<float>  - Ki, integral" );
        Serial.println( "    D<float>  - Kd, derivative" );
        Serial.println( "" );
        Serial.println( "  Misc: " );
        Serial.println( "    X         - Reset" );
        Serial.println( "" );
      }
      break;
    }
      
  } 
  
  lft.manage_motor();
  rht.manage_motor();
  
    
}





