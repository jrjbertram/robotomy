

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_9DOF.h>

/* Assign a unique ID to the sensors */
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_L3GD20_Unified       gyro  = Adafruit_L3GD20_Unified(20);
Adafruit_9DOF                 dof   = Adafruit_9DOF();



// Motor control
#include <Encoder.h>
#include <PID_v1.h>
#include "due_pwm.h"  // library to allow custom PWM frequencies
#include "MotorControl.h"

// Wifi shield
#include <ccspi.h>
#include <Adafruit_CC3000.h>
#include <Adafruit_CC3000_Server.h>
#include <SPI.h>  // neede for some reason here too

// Elapsed time measurement
#include "elapsedMillis.h"

// Helper classes to simplify serial + telnet input/output.
#include "StreamUtils.h"


// Pin mappings - these will eventually be the real map

// Notes:
//   - Pins chosen assuming Arduino Due pin capabilities.  Won't work on other Aruduino variants such as Mega or Uno.
//     The Arduino Due is an ARM-based Arduino with many additional pins, but more importantly, all of the digital pins
//     can be interrupts, and the PWM capabilities are much better with the Due.
//
//   - Also assumes that you have purchased an Adafruit CC3000 wifi shield for your Arduino.  I confirmed around 7/1/14 that
//     the Adafruit shield is compatible with the Due (someone else had already made the necessary ports.)
//
//   - When choosing our pins, we are giving priority to the Adafruit CC3000 shield because it is physcially connected to certain 
//     pins.  My custom motor controller circuits are wired up to pins that are available after the CC3000 shield's requirements
//     are satisfied.
//
//   - The Adafruit CC3000 requires the following pins:
//         * IRQ     - pin 3    - interrupt request pin
//         * VBAT    - pin 5    - wireless enable pin?   -- says can be any pin
//         * CS      - pin 10   - chip select            -- says can be any pin
//         * SPI SCK - pin 13                            -- I believe these are hard wired
//         * SPI MISO - pin 12                           -- I believe these are hard wired
//         * SPI MOSI - pin 11                           -- I believe these are hard wired

#define ADAFRUIT_CC3000_IRQ   3  // MUST be an interrupt pin!
#define ADAFRUIT_CC3000_VBAT  5  // Can be any pin
#define ADAFRUIT_CC3000_CS    10 // Can be any pin
// Use hardware SPI for the remaining pins
// On an UNO, SCK = 13, MISO = 12, and MOSI = 11

//   - On Due, pins 6, 7, 8, and 9 can be used for special high-frequency PWM signals.  Normally the frequency is 500Hz I believe,
//     but when driving motors, at low duty cycles this creates an audible squeal.  Upping the PWM frequency above 20KHz puts it
//     outside the range of human hearing, so while the squeal still happens, you just can't hear it.  Note that I've read in various
//     forums that not all motor drivers support high PWMs like this.  I built my own based on an L298N, which does.. I haven't noticed
//     any ill effects so far.  These pins need to be hooked up to our "enabled" pins for the h-bridges

int lftEn = 6;
int rhtEn = 7; 

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
//   - Since on the Due, pins 22-53 are digtal inputs that can support interrupts and they can't possibly interfere with a shield I might want
//     to use, let's allocate the rest of our motor control signals to these pins reserving the remaining "standard" shield-compatible pins.

// h-bridge direction pins
int lftA  = 26; 
int lftB  = 27;

int rhtA  = 28;
int rhtB  = 29;

// encoder pins
int lftQa = 22;
int lftQb = 23;

int rhtQa = 24;
int rhtQb = 25;

// On my analog pins, I have a Sharp IR distance sensor.  The analog value will be 3V when an object is 4" (10 cm) away, and 0.4V when an object
// is 32" (80 cm) away.
int ir1 = A0;
int sonar1 = A1;



Adafruit_CC3000 cc3000 = Adafruit_CC3000(ADAFRUIT_CC3000_CS, ADAFRUIT_CC3000_IRQ, ADAFRUIT_CC3000_VBAT,
                                         SPI_CLOCK_DIVIDER); // you can change this clock speed
#include "wifi_credentials.h"  // defines WLAN_SSID and WLAN_PASS strings

// Security can be WLAN_SEC_UNSEC, WLAN_SEC_WEP, WLAN_SEC_WPA or WLAN_SEC_WPA2
#define WLAN_SECURITY   WLAN_SEC_WPA2

#define LISTEN_PORT           7    // What TCP port to listen on for connections.  The echo protocol uses port 7.

Adafruit_CC3000_Server echoServer(LISTEN_PORT);


#define PWM_BITS        12     // On Due, you can change the resolution from 8 bits (0-255) to 12 bits (0-4095).
#define PWM_RESOLUTION  4095   // 
#define PWM_FREQUENCY   20000  // In Hz.  Frequency of the overall PWM period... not the duty cycle
#define PWM_CLOCK       1      // 1="clock A", 2="clock B".  Note I'm choosing to use the same PWM clock for both PWM outputs.



// PID tuning constants.  Currently using same values for both left and right, but
// have it set up so I can easily use individual values if needed.
//   - NOTE:  One problem I'm having here is that the values I need are highly dependent on the 
//            voltage level of my power supply.  When I switch from my 5V power supply to my 7.2V
//            battery, I (obviously) get different performance out of my motors.  This messes with
//            my PID tunings though.  Need a better approach here I think.. maybe standard PID is
//            not going to work in this case.  May need a more "involved" approach, especially as I
//            move towards controlling the velocity of the motors rather than the position.
#define KP  1.5
#define KI  0.0
#define KD  0.0

double lKp = KP;
double lKi = KI;
double lKd = KD;

double rKp = KP;
double rKi = KI;
double rKd = KD;


MotorControl lft( "lft", lftEn, PWM_RESOLUTION, lftA, lftB, lftQa, lftQb, lKp, lKi, lKd, 0 );  
MotorControl rht( "rht", rhtEn, PWM_RESOLUTION, rhtA, rhtB, rhtQa, rhtQb, rKp, rKi, rKd, 1 );  


#include "Robot.h"

Robot robot = Robot( lft, rht, accel, mag, gyro, dof );

#include "UdpStream.h"

UdpStream netConsole = UdpStream( 31000 );


void setup()
{
  Serial.begin(115200);
  Serial.println( "dc motor with encoder test" );
  
    /* Initialise the sensors */
  if(!accel.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
    while(1);
  }
  if(!mag.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }
  if(!gyro.begin())
  {
    /* There was a problem detecting the L3GD20 ... check your connections */
    Serial.print("Ooops, no L3GD20 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  
  // On the Due, set the PWM resolution to 12 bits.  Won't compile for other board types.  Also requires
  // arduino 1.5.6-r2 (BETA) or later.
  pwm_set_resolution( PWM_BITS );
  
  pwm_setup( lftEn, PWM_FREQUENCY, 1 );
  pwm_setup( rhtEn, PWM_FREQUENCY, 1 ); 
  
  displayDriverMode();
  //Serial.print("Free RAM: "); Serial.println(getFreeRam(), DEC);

  Serial.println(F("\nInitializing wifi..."));
  if (!cc3000.begin())
  {
    Serial.println(F("Couldn't begin()! Check your wiring?"));
    while(1);
  }
  Serial.println(F("\nInitialized..."));
  
  //uint16_t firmware = checkFirmwareVersion();
  //if (firmware < 0x113) {
  //  Serial.println(F("Wrong firmware version!"));
  //  for(;;);
  //} 
  
  //displayMACAddress();
 
 
  Serial.print(F("\nAttempting to connect to ")); Serial.println(WLAN_SSID);
  if (!cc3000.connectToAP(WLAN_SSID, WLAN_PASS, WLAN_SECURITY)) {
    Serial.println(F("Failed!"));
    while(1);
  }
   
  Serial.println(F("Connected!"));
  
  Serial.println(F("Request DHCP"));
  while (!cc3000.checkDHCP())
  {
    delay(100); // ToDo: Insert a DHCP timeout!
  }  

  /* Display the IP address DNS, Gateway, etc. */  
  while (! displayConnectionDetails()) {
    delay(1000);
  }

  // Start listening for connections
  echoServer.begin();
  
  // Start up our net conole
  int status = netConsole.begin();
  
  Serial.print( "Netconsole status " );
  Serial.println( status );
  
  int ip = 0xC0A80164;
  Serial.print(F("\n\rPinging ")); cc3000.printIPdotsRev(ip); Serial.print("...");  
  int replies = cc3000.ping(ip, 5);
  Serial.print(replies); Serial.println(F(" replies"));
  
  Serial.println(F("Listening for connections..."));
  
  
  
}


elapsedMillis elapsed;



void getValidStreams( Stream & stream )
{
  // I'm not sure why its done this way, but Adafruit has in their sample code for the CC3000
  // that at the beginning of each loop call, you determine whether there is a client by
  // calling "availalble()" method.
  
  // Because Adafruit didn't implement their class as a Stream, I created a wrapper class (ClientStream)
  // that provides a Stream API.  I then created teh MuxStream class that takes input from any of the
  // available Streams, and then echoes all output out both Streams.  (Sort of a bi-directional "mux".)
  
  // This was done so I would be able to implement my command parser once, and have it be available on any
  // stream.
  
  // Note for the future, the echoServer used here is a TCP-based connection (echo port 7)..  I would like
  // to eventually make this UDP based so I don't have to reconnect with a telnet client on my PC each
  // time I reboot the arduino.  Making this into a UDP mechanism that works off broadcast packets would
  // allow me to create a "sniffer" app in Python to monitor the state of the Arduino, and to implement
  // a "Stream" that makes any broadcast packets from the Python app available as input data to the Arduino.
  // It is the way it is right now because I wanted something quick to test with.

  
  // Try to get a client which is connected.
  Adafruit_CC3000_ClientRef client = echoServer.available();
  
  if (client) {
    // We have a connection from an eithernet client active, let's make sure
    // we mux our serial output with the ethernet channel.
  
    // Create a stream from the client connection
    ClientStream clientStream = ClientStream( &client );

    // Mux the stream with the Serial port
    stream = MuxStream( &clientStream, &Serial );    
  }
  else
  {
    // Just one stream
    stream = MuxStream( &Serial );
  }

}


void loop()
{
    
  char cmd = '\0';
  MuxStream stream;  
  /* Get a new sensor event */
  sensors_event_t event;

  
  // Get out input/output channel(s)
  getValidStreams( stream );
  

  // Get commands from either client or serial port
  
  if( stream.available() )
  {
    char cmd = stream.read();
    
    switch( cmd )
    {
      case 'L':
      case 'R':
      case 'B':
      {
        long newPosition = stream.parseInt();
        
        robot.setMode( Robot::ROBOT_DIAG );

        stream.print( cmd );
        stream.print( " new position: " );
        stream.println( newPosition );
    
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
        
        robot.setMode( Robot::ROBOT_DIAG );
                
        stream.print( "Kp=" );
        stream.print( kp );
        stream.print( ", Ki=" );
        stream.print( ki );
        stream.print( ", Kd=" );
        stream.println( kd );
      }
      break;      
      case 'P':
      {
        double k = stream.parseFloat();
        
        robot.setMode( Robot::ROBOT_DIAG );
        
        lft.set_kp( k );
        rht.set_kp( k );
      }
      break;
      case 'I':
      {
        double k = stream.parseFloat();
        
        robot.setMode( Robot::ROBOT_DIAG );
        
        lft.set_ki( k );
        rht.set_ki( k );
      }
      break;
      case 'D':
      {
        double k = stream.parseFloat();
        
        robot.setMode( Robot::ROBOT_DIAG );
        
        lft.set_kd( k );
        rht.set_kd( k );
      }
      break;
      case 'X':
      {
        robot.reset();

      }
      break;
      case 'S':
      {
        robot.setMode( Robot::ROBOT_DIAG );
        displayConnectionDetails();
      }
      break;   
      case 'A':
      {
        stream.println( "Going autonomous.  Type X to reset." );
        robot.setMode( Robot::ROBOT_AUTO );
      }
      break;       
      case '?':
      default:
      {
        stream.println( "Commands:" );
        stream.println( "" );
        stream.println( "  Position: " );
        stream.println( "    L<int>  - Left Motor Only" );
        stream.println( "    R<int>  - Right Motor Only" );
        stream.println( "    B<int>  - Both Motors" );
        stream.println( "" );
        stream.println( "  PID Constants: " );
        stream.println( "    K         - Print current values" );
        stream.println( "    P<float>  - Kp, proportional" );
        stream.println( "    I<float>  - Ki, integral" );
        stream.println( "    D<float>  - Kd, derivative" );
        stream.println( "" );
        stream.println( "  Misc: " );
        stream.println( "    A         - Autonomous" );
        stream.println( "    X         - Reset" );
        stream.println( "    S         - Status" );
        stream.println( "" );
      }
      break;
    }
      
  } 
  
  robot.tick_occurred( stream );

  
  if( elapsed > 5000 )
  {
    // Need to convert these to a distance measurement at some point.  Right now just using raw analog input value.
    // Also eventually need these to gate any motor movement.  Should they cross below some threshold, I want them
    // to trigger a "turn around and find new path" behavior.  i.e., we want to change our main loop here to be
    //  (a) read sensors
    //  (b) create motion "plan"
    //  (c) execute any movement
    //
    // At this point, I'm pleased that the sensors are working and there are no obvious wiring problems.
    
    double irDist = getIrDist( analogRead( ir1 ) );
    


    
    stream.print( "IR=" );
    stream.print( irDist );
    stream.print( " in, " );
    
    // Disabling sonar for now.  The sensor that I have has a beam pattern that goes out at a 45 degree angle for
    // maybe 90 cm or so.  My robot is only about 8 inches high, so the sonar always seems to pick up the floor.
    // I need to mount it on a pole I think and use it as a long-view distance sensor, possibly on a servo.
    //double sonarDist = getSonarDist( analogRead( sonar1 ) );    stream.print( "  Sonar=" );
    //stream.print( sonarDist );
    //stream.print( " in, " );
    
    sensors_event_t accel_event;
    sensors_event_t mag_event;
    sensors_vec_t   orientation;
  
    /* Read the accelerometer and magnetometer */
    accel.getEvent(&accel_event);
    mag.getEvent(&mag_event);
  
    /* Use the new fusionGetOrientation function to merge accel/mag data */  
    if (dof.fusionGetOrientation(&accel_event, &mag_event, &orientation))
    {
      /* 'orientation' should have valid .roll and .pitch fields */
      Serial.print(F("Orientation: "));
      Serial.print(orientation.roll);
      Serial.print(F(" "));
      Serial.print(orientation.pitch);
      Serial.print(F(" "));
      Serial.print(orientation.heading);
      Serial.println(F(""));
    }      
    
    int status = netConsole.print( "testing" );
    Serial.print( "netConsole print status: " );
    Serial.println( status );
    
    
    elapsed = 0;

  }
    
}

// I use this to simulate various voltage values when testing my analog sensor "algorithms".  If you pass in the voltage
// it will return what analogRead() would have returned.  This lets you quickly validate that your interpretation of
// the voltage values is correct over a range of voltage values.
int simulate( double voltage )
{
  double reference = 3.3;
  double ratio = voltage / reference;
  double resolution = 1024.0;
  int adc_value = ratio * resolution;
  
  return adc_value;
}

double getIrDist( int sensor_value )
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

   double reference = 3.3; 
   double resolution = 1024.0;
   
   double normal_value = sensor_value / resolution;
   double voltage = normal_value * reference;
   
   Serial.print( "IR raw: " );
   Serial.print( sensor_value );
   Serial.print( ", " );
   
   Serial.print( "voltage: " );
   Serial.print( voltage );
   Serial.print( ", " );
   
   double dist = -10.76666667 * voltage + 36.3;
   
   Serial.print( "distance: " );
   Serial.print( dist );
   Serial.println( "" );   
   
   return dist;
}


double getSonarDist( int sensor_value )
{
    
   // the Maxbotix Sonar sensor analog interface provides Vcc/512 
   // volts-per-inch.  So a voltage of 0 is 0 inches and a voltage
   // of Vcc (3.3V) is 254 inches.
   
   // We are using a 3.3V reference signal.  The ADCs on the
   // Due are 12-bit ADCs, but are configured by default to be
   // 10-bit ADCs to be compatible with prior Arduinos.  For
   // now we're using the default, so our voltagle will range
   // from 0 - 1024.
   
   // Thus, to calculate distances, the formula is:
   //     dist = ( sensor_value / 1024 ) * 254"
  
   double reference = 3.3; 
   double resolution = 1024.0;
   
   double normal_value = sensor_value / resolution;
   double voltage = normal_value * reference;
   
   //Serial.print( "Sonar raw: " );
   //Serial.print( sensor_value );
   //Serial.print( ", " );
   
   //Serial.print( "voltage: " );
   //Serial.print( voltage );
   //Serial.print( ", " );
   
   double dist = normal_value * 254.0;

   //Serial.print( "distance: " );
   //Serial.print( dist );
   //Serial.println( "" );   
      
   return dist;
}


/**************************************************************************/
/*!
    @brief  Displays the driver mode (tiny of normal), and the buffer
            size if tiny mode is not being used

    @note   The buffer size and driver mode are defined in cc3000_common.h
*/
/**************************************************************************/
void displayDriverMode(void)
{
  #ifdef CC3000_TINY_DRIVER
    Serial.println(F("CC3000 is configure in 'Tiny' mode"));
  #else
    Serial.print(F("RX Buffer : "));
    Serial.print(CC3000_RX_BUFFER_SIZE);
    Serial.println(F(" bytes"));
    Serial.print(F("TX Buffer : "));
    Serial.print(CC3000_TX_BUFFER_SIZE);
    Serial.println(F(" bytes"));
  #endif
}

/**************************************************************************/
/*!
    @brief  Tries to read the CC3000's internal firmware patch ID
*/
/**************************************************************************/
uint16_t checkFirmwareVersion(void)
{
  uint8_t major, minor;
  uint16_t version;
  
#ifndef CC3000_TINY_DRIVER  
  if(!cc3000.getFirmwareVersion(&major, &minor))
  {
    Serial.println(F("Unable to retrieve the firmware version!\r\n"));
    version = 0;
  }
  else
  {
    Serial.print(F("Firmware V. : "));
    Serial.print(major); Serial.print(F(".")); Serial.println(minor);
    version = major; version <<= 8; version |= minor;
  }
#endif
  return version;
}

/**************************************************************************/
/*!
    @brief  Tries to read the 6-byte MAC address of the CC3000 module
*/
/**************************************************************************/
void displayMACAddress(void)
{
  uint8_t macAddress[6];
  
  if(!cc3000.getMacAddress(macAddress))
  {
    Serial.println(F("Unable to retrieve MAC Address!\r\n"));
  }
  else
  {
    Serial.print(F("MAC Address : "));
    cc3000.printHex((byte*)&macAddress, 6);
  }
}


bool displayConnectionDetails(void)
{
  uint32_t ipAddress, netmask, gateway, dhcpserv, dnsserv;
  
  if(!cc3000.getIPAddress(&ipAddress, &netmask, &gateway, &dhcpserv, &dnsserv))
  {
    Serial.println(F("Unable to retrieve the IP Address!\r\n"));
    return false;
  }
  else
  {
    Serial.print(F("\nIP Addr: ")); cc3000.printIPdotsRev(ipAddress);
    Serial.print(F("\nNetmask: ")); cc3000.printIPdotsRev(netmask);
    Serial.print(F("\nGateway: ")); cc3000.printIPdotsRev(gateway);
    Serial.print(F("\nDHCPsrv: ")); cc3000.printIPdotsRev(dhcpserv);
    Serial.print(F("\nDNSserv: ")); cc3000.printIPdotsRev(dnsserv);
    Serial.println();
    return true;
  }
}



