# iiwa_ik
Closed form solution for KUKA LWR iiwa manipulator.

This is a Visual Basic sln project that solves the IK problem for the KUKA iiwa robot in a closed form. It only depends on Eigen library. 

The program asks a user for the desired configuration (cartesian position and orientation quaternion) and outputs the required joint angles to move the robot to the desired configuration. The codes pays attention for the joint limits and the out of reacability configurations. 

The motivational repo behinds this work is:
https://github.com/CentroEPiaggio/kuka-lwr
