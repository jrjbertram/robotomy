
#ifndef IR_SENSOR_h
#define IR_SENSOR_h

class IRSensor  {  
  public:
    IRSensor( int pin ) : _pin( pin ) {};

    double getDistance()
    {
        
       // the Sharp IR sensors output voltage will range from
       // 3V when the obj is 4" / 10 cm away and will be 0.4V
       // when the obj is 32" / 80 cm away.
       
       // We are using a 3.3V reference signal.  The ADCs on the
       // Due are 12-bit ADCs, but are configured by default to be
       // 10-bit ADCs to be compatible with prior Arduinos.  For
       // now we're using the default, so our voltagle will range
       // from 0 - 1024.
       
       // To calculate the equation to convert voltage to distance,
       // we need to solve the following equations:
       //
       //    4" = m * 3.0V + b
       //   32" = m * 0.4V + b
       //
       //   Solving the first equation for m:
       //     m = ( 4" - b ) / 3V
       //     m = 4/3  - b/3
       //     m = 1.333333 - .333333 b
       //   
       //   Substituting into second equation:
       //   32 = ( 1.33333 - .33333 b ) * 0.4 + b
       //   32 = .53333333 - .13333 b + b
       //   32 = .53333333 + .86667 b
       //   31.46666666  = .86666667 b
       //   36.3 = b
    
       //   Solving for m using b:
       //
       //     m = 1.333333 - .333333 * 36.3
       //     m = 1.333333 - 12.09999999
       //     m = -10.76666667
       //
       //   Formula is thus:
       //     y = -10.76666667 * x + 36.3
       //
       //   Or:
       //
       //     dist = -10.76666667 * voltage + 36.3   
       
       int sensor_value = analogRead( _pin );
       
       double reference = 3.3; 
       double resolution = 1024.0;
       
       double normal_value = sensor_value / resolution;
       double voltage = normal_value * reference;
       
       double dist = -10.76666667 * voltage + 36.3;
       
       return dist;
    }
    
  private:
    int _pin;
};
  

#endif

