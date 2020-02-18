#ifndef statetracker_H
#define statetracker_H

// Standard C++ includes
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <cstddef>
#include <iostream>
#include <cmath>
#include <vector>
#include <string>
#include <fstream>

// ROS includes
#include <ros/ros.h>
#include "nav_msgs/Odometry.h"

using namespace std;

typedef struct
{
	float  x, y, z;
}Cartesian;

typedef struct
{
	float  qx, qy, qz, qw;
}Quaternion;

typedef struct
{
	float  roll, pitch, yaw;
}Euler;

typedef struct
{
	Quaternion quaternion;
	Euler euler; 
}Orientation;

class statetracker
{
public:
	Cartesian pose;
	Cartesian velocity;
	Orientation orientation;
	Cartesian angularrate;
	Euler e;
	
	statetracker();
	~statetracker();
	void callback(const nav_msgs::Odometry::ConstPtr& msg);
	Euler quaternion2euler(double q0, double q1, double q2, double q3);
};

#endif
