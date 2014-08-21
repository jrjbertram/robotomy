


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
#define KP  2.5
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


void setup()
{
  Serial.begin(115200);
  Serial.println( "dc motor with encoder test" );
  
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
  
  uint16_t firmware = checkFirmwareVersion();
  if (firmware < 0x113) {
    Serial.println(F("Wrong firmware version!"));
    for(;;);
  } 
  
  displayMACAddress();
 
 
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
  
  Serial.println(F("Listening for connections..."));
  
}


elapsedMillis elapsed;

void loop()
{
    
  char cmd = '\0';
  MuxStream stream;

  // Get commands from either client or serial port
  
  // Try to get a client which is connected.
  Adafruit_CC3000_ClientRef client = echoServer.available();
  if (client) {
    ClientStream clientStream = ClientStream( &client );

    stream = MuxStream( &clientStream );    
  }
  else
  {
    stream = MuxStream( &Serial );
  }

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
        lft.set_kp( k );
        rht.set_kp( k );
      }
      break;
      case 'I':
      {
        double k = stream.parseFloat();
        lft.set_ki( k );
        rht.set_ki( k );
      }
      break;
      case 'D':
      {
        double k = stream.parseFloat();
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
      case 'S':
      {
        displayConnectionDetails();
      }
      break;      case '?':
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
        stream.println( "    X         - Reset" );
        stream.println( "    S         - Status" );
        stream.println( "" );
      }
      break;
    }
      
  } 
  
  lft.manage_motor();
  rht.manage_motor();
  
  if( elapsed > 1000 )
  {
    // Need to convert these to a distance measurement at some point.  Right now just using raw analog input value.
    // Also eventually need these to gate any motor movement.  Should they cross below some threshold, I want them
    // to trigger a "turn around and find new path" behavior.  i.e., we want to change our main loop here to be
    //  (a) read sensors
    //  (b) create motion "plan"
    //  (c) execute any movement
    //
    // At this point, I'm pleased that the sensors are working and there are no obvious wiring problems.
    
    int ir1In = analogRead( ir1 );
    int sonar1In = analogRead( sonar1 );
    
    stream.print( "IR=" );
    stream.print( ir1In );
    
    stream.print( "  Sonar=" );
    stream.println( sonar1In );
    elapsed = 0;
  }
    
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
