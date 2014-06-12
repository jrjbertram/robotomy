
#ifndef MOTOR_CONTROL_h
#define MOTOR_CONTROL_h
#define LIBRARY_VERSION	1.0.0

#include <Encoder.h>
#include <PID_v1.h>

class MotorControl
{


  public:
  
    MotorControl( 
      int enable_pin,
      int pwm_resolution,  // normally 0-255 for arduino
      int direction_A_pin,
      int direction_B_pin,
      int quadrature_A_pin,
      int quadrature_B_pin,
      int pid_kp,
      int pid_ki,
      int pid_kd
      );

    void stop();
    
    void set_desired_position( long newPosition );
    
    void manage_motor(); // call this each 'loop' iteration

  private:
  
    // Assumes that we are controlling a DC motor that is driven with an h-bridge with 3 inputs:
    int pinEn;    // enable pin (can be PWM controlled)
    int pwmRes;   // resolution of PWM signal (normally 0-255 on Arduino, but can depend on the pin)
    int pinDirA;  // direction input A
    int pinDirB;  // direction input B
    
    // With the enable pin set to 0, we are free to change the direction.  The way this type of hbridge works
    // to go "forward", you set pinA to a 1, and pinB to a 0.  To go "backward", you set pinB to a 1 and pin A to a 0.
    // Some hbridge designs also have an active braking, but I'm not going to support that in this code.
    //
    // Once the direction is set, the motor is activated by setting the enable pin to 1.  To control the speed the
    // enable pin can be driven with a PWM signal.
    
    // Assumes that the motors have a quadrature encoder attached to them.  For decent resolution, pinQuadA should be
    // an interrupt pin.  For best resolution, both pins should be interrupt pins.
    int pinQuadA;
    int pinQuadB;
    
    // Use the Encoder library to track the motors current position.  That sure makes life easy!
    Encoder encoder; 

    // Motor control is currently position based.  By setting a new desired position, the MotorControl class will
    // automatically set the motor speed to reach the new position.   
    long currPos;
    long desiredPos;
    
    // Use the PID library to generate smooth motion of our motors
    PID pid;
    
    // The PID library requires its inputs and outputs to be doubles
    double pidInput;
    double pidSetpoint;
    double pidOutput;
    
    // The PID library also requies 3 constants to configure the PID algorithm's response.
    double Kp;  // proporitional constant
    double Ki;  // integral constant
    double Kd;  // derivitive constant

    // State tracking variables
    int currentDirection;

    // Internal low-level motor API
    void forward();
    void reverse();
    void go( int spped );  // value of 0 - 'pwmRes', always a positive value.
    void update_position();
    
    // Internal higher-level motor API
    void set_motor_speed( int velocity );  // value of -pwmRes through pwmRes.  Speed + direction = velocity
    int calculate_new_velocity();
    
 
};
#endif
