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

\     
