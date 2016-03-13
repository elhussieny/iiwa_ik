/*
* main.cpp
*
*  Created on: Mar 9, 2016
*      Author: haitham
*/
#include <Eigen/Dense>
#include <math.h>
#include <stdio.h>
#include <iostream>
using namespace std;
using namespace Eigen;
#define PI 3.1415926

double kukaJoints[7];
// DH Parameters for the KUKA. Refer to Descriptions.png
double D1 = 0.36;
double D3 = 0.42;
double D5 = 0.40;
double D7 = 0.126;
double JOINT_LIMITS[] = { 170.0*PI / 180.0,
120.0*PI / 180.0,
170.0*PI / 180.0,
120.0*PI / 180.0,
170.0*PI / 180.0,
120.0*PI / 180.0,
175.0*PI / 180.0 };
/*----------------------------------------------------------------------------*/
bool inverseKuka(double poseDsr[])
{
	//    // Returning value
	bool validIK = true;
	// Initialize variables
	//    	// Auxiliary variables
	double mod_pW, mod_pWxy, c2, s2, c3, s3;
	//    		// Getting transformation matrix
	Eigen::Quaterniond R = Eigen::Quaterniond(poseDsr[3], poseDsr[4], poseDsr[5], poseDsr[6]);
	//    			//	The -ve sign for the desired pos. is due to the difference between V-REP and code
	Eigen::Vector3d p(-poseDsr[0], -poseDsr[1], poseDsr[2] - D1);
	//    			// Inverse kinematics calculations. Closed form solution.
	//    			// hand position
	Eigen::Vector3d pW = p - D7 * R.toRotationMatrix().col(2);
	// Calculate wrist position
	kukaJoints[0] = atan2(pW[1], pW[0]);
	mod_pW = pow(pW.norm(), 2); // Pwx^2+Pwy^2+Pwz^2
	//
	c3 = (mod_pW - D3*D3 - D5*D5) / (2 * D3*D5);
	// If c3>1, there is no solution for IKT
	if (c3>1){
		validIK = false;
		printf("ikSolver: Attempt to access to a point out of the workspace. Zero array will be returned.\n");
		for (int j = 0; j<7; j++)kukaJoints[j] = 0;
		return validIK;
	}
	//
	s3 = -sqrt(1 - c3*c3);
	kukaJoints[3] = atan2(s3, c3) + PI / 2;
	//
	//    					// We do not use the extra dof for getting the inverse kinematics
	kukaJoints[2] = 0.0;
	//
	mod_pWxy = sqrt(pW[0] * pW[0] + pW[1] * pW[1]);
	s2 = ((D3 + D5*c3)*pW[2] - D5*s3*mod_pWxy) / mod_pW;
	c2 = ((D3 + D5*c3)*mod_pWxy + D5*s3*pW[2]) / mod_pW;
	kukaJoints[1] = atan2(s2, c2);
	//
	//    					// Calculate orientation (angles of the wrist joints)
	Eigen::Matrix3d T01; T01 << cos(kukaJoints[0]), 0.0, sin(kukaJoints[0]), sin(kukaJoints[0]), 0.0, -cos(kukaJoints[0]), 0.0, 1.0, 0.0;
	Eigen::Matrix3d T12; T12 << cos(kukaJoints[1]), -sin(kukaJoints[1]), 0.0, sin(kukaJoints[1]), cos(kukaJoints[1]), 0.0, 0.0, 0.0, 1.0;
	Eigen::Matrix3d T23; T23 << cos(kukaJoints[3]), 0.0, sin(kukaJoints[3]), sin(kukaJoints[3]), 0.0, -cos(kukaJoints[3]), 0.0, 1.0, 0.0;
	//
	Eigen::Matrix3d pose03 = T01*T12*T23;
	Eigen::Matrix3d  pose36 = pose03.inverse() * (R.toRotationMatrix());
	//
	kukaJoints[4] = atan2(pose36(1, 2), pose36(0, 2));
	kukaJoints[5] = atan2(sqrt(pose36(0, 2)*pose36(0, 2) + pose36(1, 2)*pose36(1, 2)), pose36(2, 2));
	kukaJoints[6] = atan2(pose36(2, 1), -pose36(2, 0));
	//
	//    					//Adjust to robot from IK coordinates (keeping joint coord. within the interval [-pi,pi])
	kukaJoints[1] < -PI / 2 ? kukaJoints[1] += 3 * PI / 2 : kukaJoints[1] -= PI / 2;
	kukaJoints[3] < -PI / 2 ? kukaJoints[3] += 3 * PI / 2 : kukaJoints[3] -= PI / 2;
	kukaJoints[6] <     0 ? kukaJoints[6] += PI : kukaJoints[6] -= PI;
	//
	kukaJoints[3] = -kukaJoints[3]; //Correcting for the RobotRotation
	//
	for (int i = 0; i < 7; i++){
		if (abs(kukaJoints[i]) > JOINT_LIMITS[i]){
			validIK = false;
			kukaJoints[i] > 0 ? kukaJoints[i] = JOINT_LIMITS[i] : kukaJoints[i] = -JOINT_LIMITS[i];
			printf("Warning!!! IK gives values out of bounds for joint %d \n", i);
		}
		//
		printf("Joint [%d]: %.3f \n", i, kukaJoints[i] * 180 / PI);
	}
	//
	//
	return validIK;
	//
}
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/


int main(int argc, char** argv)
{
	double desiredPose[7];
	printf("KUKA IK Solver \n");
	printf("------------------\n");
	printf("Enter the desired configuration [x,y,z,qx,qy,qz,qw]\n");
	char againYN = 'y'; int index = 0;
	while (againYN == 'y')
	{
		while (index<7)
		{
			printf("Desired Pose [%d] \n", index);
			cin >> desiredPose[index];
			index++;
		}
		printf("-------------------------------\n");
		inverseKuka(desiredPose);

		printf("Try Again ? (y/N) \n");
		cin >> againYN;
		index = 0;
	}
	printf("Good Bye!\n");

	return 0;
}
