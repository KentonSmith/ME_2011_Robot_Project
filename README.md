# ME2011_Robot_Project
Arduino code for an autonomous object sensing robot arm in addition to some documentation/schematics.

Video for robot in action can be seen here: https://youtu.be/Hm5hVab4eCg?t=13s

Detailed Description of Robot: This robot uses a gear motor to move the arm between a raised and lowered state.  In the code that drivers the motor there is a specific velocity and acceleration profile for the forward and reverse directions to optimize motion and minimize the effect of repeated impacts.  Normally-open limit switches are on either side of the robot arm and turn off the motor when pressed.  When in a downward state, the robot arm swivels using a servo motor attached to a turntable bearing.  On the end of the arm, an infrared sensor takes voltage readings and stores the raw data in an array in the Arduino Code.  The raw data goes through a rolling average filter algorithm to eliminate any glitches in the readings.  Afterwards, a pair of functions locates the two edges of the object (a cup with paper clips in it) based on if the rolling average exceeds a threshold value of the height of the object under scrutiny.  The servo then moves to the average of the two angles that detect the rise of the object's edge.  At this point, an electromagnet turns on and lifts paper clips out of the cup and the motor moves to a raised state and drops the paper clips in a self-contained compartment.  

![](https://s3.amazonaws.com/githubreadmepictures/Horizontal_Robot_Pictures.png?X-Amz-Date=20150920T001733Z&X-Amz-Expires=300&X-Amz-Algorithm=AWS4-HMAC-SHA256&X-Amz-Signature=8ad8214b1f40709fbf2ec04d5285744b76c3f980069838886bead88023ada8ae&X-Amz-Credential=ASIAJKNSPRHX4E6L2TXQ/20150920/us-east-1/s3/aws4_request&X-Amz-SignedHeaders=Host&x-amz-security-token=AQoDYXdzEJn//////////wEakAK9hv3l7vptn1yCoODVGgDR%2BQEXqAxz70nDKQlOyPcq%2B7bb4HVSNfz14U/1I7p0uxwVIPiY9SJIYFnay2mSslRViiuHujonA6nAa2zlbV/xh1u2yuKfqgbzv2RCluGCynRu6DlbttsOr5aG0BzK2Acz3feJpXpzLhWzbBJbiw3EM84Jkt7RpgCJUqYM8T/4r1CEP2PXQZMhbLuipiBiZ4wvFRi8hp3Sld6B5HFUwB%2BYMitkFw684UuP19VLBfXWzpVFoQSNs6P5Jjb%2ByvYJICRenClIenHb1e6QmT5AVEnOm11uNA4%2Bo%2Bgz/8OExVxQJEohHK/fJcQN9%2BHNivg3kJAjU5CXrUklWDvDqNUVP8aciCC47vevBQ%3D%3D)
