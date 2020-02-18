#ifndef gazebotracker_H
#define gazebotracker_H

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
#include "gazebo_msgs/ModelStates.h"

using namespace std;

class gazebotracker
{
public:
	vector<string> list;
	gazebotracker();
	~gazebotracker();
	void callback(const gazebo_msgs::ModelStates::ConstPtr& msg);
};

#endif
