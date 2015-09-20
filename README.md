# ME2011_Robot_Project
Arduino code for an autonomous object sensing robot arm in addition to some documentation/schematics.

Video for robot in action can be seen here: https://youtu.be/Hm5hVab4eCg?t=13s

Detailed Description of Robot: This robot uses a gear motor to move the arm between a raised and lowered state.  In the code that drivers the motor there is a specific velocity and acceleration profile for the forward and reverse directions to optimize motion and minimize the effect of repeated impacts.  Normally-open limit switches are on either side of the robot arm and turn off the motor when pressed.  When in a downward state, the robot arm swivels using a servo motor attached to a turntable bearing.  On the end of the arm, an infrared sensor takes voltage readings and stores the raw data in an array in the Arduino Code.  The raw data goes through a rolling average filter algorithm to eliminate any glitches in the readings.  Afterwards, a pair of functions locates the two edges of the object (a cup with paper clips in it) based on if the rolling average exceeds a threshold value of the height of the object under scrutiny.  The servo then moves to the average of the two angles that detect the rise of the object's edge.  At this point, an electromagnet turns on and lifts paper clips out of the cup and the motor moves to a raised state and drops the paper clips in a self-contained compartment.  

![](https://s3.amazonaws.com/githubreadmepictures/Horizontal_Robot_Pictures.png)
