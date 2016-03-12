# KUKA iiwa Closed Form IK
***Closed form solution for KUKA LWR iiwa manipulator.***

This is a Visual Basic sln project that solves the IK problem for the KUKA iiwa robot R820 in a closed form. It depends only on Eigen library. To add the Eigen follow [this link](http://microsolutions.info/2012/11/eigen-linear-algebra-library-with-visual-c-2010.html).

The program simply asks a user for the desired configuration (Cartesian position and orientation in quaternion format) and outputs the required joint angles to move the robot to the desired configuration. The codes pays attention for the joint limits and the out of reachability configurations. It solves the IK in a closed form by neglecting the value of the forth joint J4. The mathematics behinds this work can found in the book:

 *"Robotics - Modelling, Planning and Control", by Siciliano et al. (2010) pp. 96*



#Acknowledgement:
This code is motivated by these repos:
 - [IDSCETHZurich](https://github.com/IDSCETHZurich/re_trajectory-generator.git).


