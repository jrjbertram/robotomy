robotomy
========

Repo for my home-brewed robot.

architecture
============

My robot uses an Arduino Due to perform fine-grained control functions like motor control and management, object detection, position estimation, and other low-level sensor management functionality.  These functions are best suited for the C/C++-only microcontroller environment provided by the Arduino.  

It also uses a Raspberry Pi to perform high-level functions such as motion planning, ethernet interfacing.  These functions are best suited for the full Linux environment available on the Raspberry Pi.

The Raspberry Pi and the Arduino Due are connected via serial port.  The raspi sends commands over the serial link, and the arduino sends status updates back to the raspi.

At this point, I'm putting the code up on github in case it helps others in some way.

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

