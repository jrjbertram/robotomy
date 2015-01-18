robotomy
========

Repo for my home-brewed robot.

architecture
============

My robot uses an Arduino Due to perform fine-grained control functions like motor control and management, object detection, position estimation, and other low-level sensor management functionality.  These functions are best suited for the C/C++-only microcontroller environment provided by the Arduino.  

It also uses a Raspberry Pi to perform high-level functions such as motion planning, ethernet interfacing.  These functions are best suited for the full Linux environment available on the Raspberry Pi.

The Raspberry Pi and the Arduino Due are connected via serial port.  The raspi sends commands over the serial link, and the arduino sends status updates back to the raspi.

At this point, I'm putting the code up on github in case it helps others in some way.

interface description
=====================

Interface between the arduino and raspberry pi is serial port at 115200.  Serial3 port on arduino connected to ttyAMA0 on raspi.

Arduino periodically sends status messages over serial in following format.  This format is very simple to parse in python using "split" function, and will allow me great flexibility in adding new status params as needed.
Param1=value1:Param2=value2:Param3=value3

Status parameter descriptions:
-----------------------

Controller state params:
* Mode:
    * Auto - Fully autonomous, actively following current command from raspi.
    * Diag - In diagnostic mode... currently not actually used.
    * Idle - Arduino is idle, reporting status but not actively driving around. 

*Plan:
    * Reset - No plan currently loaded, not actively doing anything.
    * Init - Legacy: Plan in "init" mode, will seek to magnetic heading of zero, then switches to Wander.
    * Wander - Legacy: Drives around until sensors indiciate an obstacle collision.
    * Hit - Legacy: Rotates around until obstacle collision indication clears.  Once clear goes back to wander.

Orientation params:
* Roll - degrees, 0 is level (I think).. currently unused.
* Pitch - degrees, 0 is level (I think).. currently unused.
* Yaw - degrees, 0 is magnetic north (I think)

Obstacle sensors:
* IR - distance to obstacle in inches (I think)

Motors:
* Left - Velocity of left motor (unit unknown)
* Right - Velocity of right motor (unit unknown)
* LeftDesitred - Desired encoder position of left motor
* RightDesired - Desired encoder position of right motor


Command set from raspi to arduino:
----------------------------------

The command set from the raspi to the arduino is a different format than the status commands, and is instead formatted to be as easy as possible for the C++ arduino code to parse the command set.  Consequently, each command itself will be one letter, and all parameters will be separated by spaces.  This will be easiest to parse using the arduino codebases' "parseInt" "parseFloat", etc functions.

Commands will be separated by newline characters.  The number of arguments and type of arguments will be dependent on the command.  Due to the limited support code available on the arduino, at this time error checking on the commands will be minimal.

Commands:

* R  - rotate to new magnetic heading
    * param 1 - heading in degrees, float

* F - go forward until obstacle collision detected
    * no params

* I - go idle

New status from arduino to raspi:
---------------------------------

* Cmd - last command received from raspi, or "Idle" if no command received

* Sts - current status of the last command, one of:
    ** Exec - currently executing last command
    ** Done - last command completed successfully
    ** Error - last command could not be completed due to an error / exception condition

* Err - reason for last error
    ** Fault - a fault condition occurred (currently unused)
    ** Ob - an obstacle was detected that prevents previous command from being executed
    ** Fail - no fault was detected, but attempts to move failed.  may be due to low motor battery, undetectable obstruction, etc. 

* Head - Current magnetic heading

* Pos - Best estimate at absolute position based off available sensors and algorithms measured from power on position, listed as x and y coordinate in inches where x and y are separated by a comma

Current sensor state.. will change over time as sensors are added.

* IR1 - Infrared #1 distance sensor, distance in inches
* IR2 - Infrared #2 distance sensor, distance in inches






lessons learned
===============

Originally, I had started with a Raspberry Pi only and tried performing all of my IO with only the raspi.  However, the raspi has a pretty limited number of pins, which forced me to try to drive my stepper motors via an I2C-based PWM breakout board (which in turn was hooked up to an H-bridge to actually drive the motors).  I2C was a bottleneck though... the PWM breakout board was more intented for applications like controlling banks of LEDs where the I2C bottleneck is less of an issue.  This drove me to look for something that had a higher number of discretes which would give better low-level performance so that I did not have to use I2C, and I settled on an Arduino Due.

The Arduino Due worked great for interfacing with my H-bridge circuitry and my motor's quadrature encoders, as it has many pins.  (The Arduino Uno has more pins than a Raspberry Pi, but a limited number of interrupt pins.  I used a Due as it has more interrupt pins.)

For a long time, I wasted a few months trying to get an Adafruit CC3000 ethernet shield working well with the Arduino.  I got basic functionality working, but found that the CC3000 is so limited in what it supports that it ended up causing more problems than it solved.  I'd highly recommend not purchasing that card... the primary problems were that you are limited to very small packet sizes (~100 bytes), and if you attempt to send packets too often such that no space is left within the CC3000's buffers, I found that it would just hang.  

Even once I worked around all the limitations, I found that it really wasn't an effective way to develop once you "untether" the robot, as every time I wanted to update my Arduino environment, I had to hook back up my USB cable anyway.  The Arduino IDE environment is also a little cumbersome once you reach more than 5 or 10 files in your project...  so long story short, adding in a Raspberry Pi with a usb-based wifi dongle provides a lot of benefits once the Arduino development is stable.  This will let me develop the remaining high-level functionality by ssh'ing into my raspi and updating the robot logic (e.g., python scripts) without a long compile and programming cycle. 

I'd like to eventually use the raspi to program the arduino, providing me with true "tetherless" operation:
* https://github.com/synthetos/PiOCD/wiki/Using-a-Raspberry-Pi-as-a-JTAG-Dongle

cleanup activities
==================

* need to make a breakout board with connectors for all the new wiring
* wait for ethernet shield to arrive on Tues?

future direction
================

* the other thing to figure out is the motion planning math for this 
  robot.
    - Have it modeled in HTML

* with the Due and its arm core, I may have enough horsepower to run the 
  motion planning from the Due, and leave the rpi for vision processing.
    - Probably true.

* may also get a pixy.. not sure it'll really help though beyond the rpi cam.

* This would be pretty useful
* https://github.com/synthetos/PiOCD/wiki/Using-a-Raspberry-Pi-as-a-JTAG-Dongle

Motion Planning
===============

Notes on motion.  I have a two wheeled robot.. want to start being able to
specify speeds of motors so that I can specify a trajetory.

If the robot is turning in a circle where 'r' is the distance from the
center of the turn to the inner wheel, and 'b' is the lenght between the
two wheels, and theta is the angle that you turn, the the distance
that each wheel travels is:

d-inner = r * theta
d-outer = ( r + b ) * theta = r * theta + b * theta

So the difference between the two, d-outer - d-inner, is b* theta.

    (d-outer - d-inner ) = b * theta

Solving for theta, you get:

    theta = ( d-outer - d-inner ) / b

Once theta is known, r can be calulated as follows:

     r = d-inner / theta

So over any time, t, if I measure the distance the wheels traveled,
I will get two measurements... either left or right will be shorter
and will thus be d-inner.

b will always be fixed for my robot, so I will always be able to 
calculate all of the other variables.

Some examples:

  In time "t", the left motor traveled 3 inches and the right motor
  travelled 3.5 inches.  Let's assume that the distance between wheels
  is 8 inches.

  In this example, d-inner is 3 inches, and d-outer is 3.5 inches, and
  b is 8 inches.

  To calculate theta (the angle), we perform the following calculation:

    theta = ( d-outer - d-inner ) / b
          = ( 3.5 inches - 3 inches ) / 8 inches
          = .5 / 8
          = .0625 radians  ( 3.58 degrees )

    also, we can calculate the center of the radius of the circle the
    robot is moving along.  it will always be aligned along the axis
    between the two motors, and will either be left or right.

    r = d-inner / theta
      = 3 inches / .0625 radians
      = 48 inches


    We can also determine our new x, y and gamma (orientation angle) as
    
       x-new = x-old + r * cos( theta )
       y-new = y-old + y * sin( theta )

       gamma-new  =  gamma-old + ( 90 degrees - theta )
                  =  gamma-old + ( pi / 2 - theta )



     This should give us a pretty simple way to keep track of where we are.
    
     Now how about planning ahead?  What I'd like to do here is calculate
     my desired velocities of the motors given a new desired position.

     I think the way I need to do this is given the new position and 
     orientation that is desired, what circle would I have to follow 
     to get there?

      Nope that won't work as there will be plenty of paths that can't be
      met by a circle.  BUT, I think that any path can be met by following
      two circles who meet at a tangent?
                - JRB update ... not so sure about that.  Switched over to
                  bezier curves.

    The bezier curves I'm using are smooth, but they have sharp angles. 
    I'm okay with that for now I think.

Robot GUI
===============

Have created a simple python-based gui that displays the real-time status of the robot.  This is useful for monitoring robot state while it is untethered from the host PC.

Dependencies.  Was going to use PyQt, but Qt and PyQt are not well integrated
with Ubuntu's package managers from what I'm reading.  So.. not worth the effort.

Instead, looking at gui2py, which is built on wxPython and wxWidgets.  Followed
these instructions to get wxPython and wxWidgets installed on Ubuntu 14.10.  Wasn't
"turnkey" but did get the job done.
http://codelite.org/LiteEditor/WxWidgets30Binaries#toc2

Followed these directions to get gui2py installed:
https://code.google.com/p/gui2py/wiki/InstallationGuide

Verified installation via following line:
$> python -c "import gui; print gui.__version__"
0.9.4

Overall, its been working great for what I need.  Will probably stick with it for a while.

