#ifndef BTsimulator_H
#define BTsimulator_H

// Standard C++ includes
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <cstddef>
#include <iostream>
#include <cmath>	/* log10 */
#include <vector>
#include <string>
#include <fstream>
#include <random>

// ROS includes
#include <ros/ros.h>
#include "nav_msgs/Odometry.h"

// Own includes
#include "auxiliary.h"
#include "statetracker.h"

#define PI 3.141592653589793

using namespace std;

// Set up the Bluetooth message that will be sent
typedef struct
{
   float  vx, vy, psi, h, RSSI, dist, x, y;
}BTmessage;

class BTsimulator
{
	ros::NodeHandle n;
	ros::Subscriber trackersub;
	ros::Subscriber trackedsub;

	string ID_tracker;
	string ID_tracked;

	statetracker trackerTracker;
	statetracker trackedTracker;

	BTmessage message;

public:

	BTsimulator(const string &tracker, const string &tracked);
	~BTsimulator();
	BTmessage getBTmessage();

protected:
	// This function has to remain secret to anything else
	int distanceToRSSI(const float &dist); 

};

#endif
