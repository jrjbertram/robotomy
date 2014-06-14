robotomy
========

Repo for my home-brewed robot.

design
======

Robot was originally based on rasperry pi, but I found that it wasn't quite as responsive and deterministic as I wanted.
Also, originally, I was using a pair of stepper motors, but the particular motors I used were very noisy and didn't quite give me the smooth motion I was looking for.  Part of the issue was also that I was using an I2C-based device to turn on and off the signals to my h-bridge, so the I2C accesses were probably to slow, and the timing of which coils were activated when within the stepper motor were probably giving me the poor response.

From there, I decided to move the motor control to an arduino, and got better results with the stepper motors.  I at least got smooth, fast motion with the arduino, but the stepper motors were still too loud.

I switched over to some metal gearmotors with encoders, and rewrote the arduino code to drive these motors.  Driving the motors was easy, and it wasn't very hard to get the Arduino Encoder library integrated in.  From there I integrated in the PID library, and spent a little time tuning the PID parameters so that performance was reasonable.

At this point, I'm putting the code up on github in case it helps others in some way.

current status
==============

Converted to Arduino Due.  The code has some Due-specific PWM library included, so will no longer work with an Uno as is.

Next steps...?  

* I also need to make a breakout board with connectors for all the new wiring
* package up the motor controller circuit into an enclosure?
* maybe solder up a new controller board with just two h-bridges instead of
  the dual h-bridge one I have now.. its taking up too much space now.
* hook up to raspberry pi for wireless access?
   - I guess I will want to do this now and run the arduino from the
     pi so that the pi figures out the motion plan.
* wait for ethernet shield to arrive on Tues?

* the other thing to figure out is the motion planning math for this 
  robot.

* sensors also arrive on Tues.. couple of infrared, a sonar, and a 9-dof 
  orientation sensor

* with the Due and its arm core, I may have enough horsepower to run the 
  motion planning from the Due, and leave the rpi for vision processing.

* may also get a pixy.. not sure it'll really help though beyond the rpi cam.
