#ifndef relativetracker_H
#define relativetracker_H

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

// Own includes
#include "BTsimulator.h"

using namespace std;

class relativetracker
{
	
	string ID_tracked;
	string ID_tracker;
	BTmessage btmsg;

	BTsimulator *btsim;

public:
	float distace;
	float bearing;

	relativetracker(const string &ID_tracker, const string &ID_tracked);
	~relativetracker();

	void update(void);
	BTmessage read(void);
	BTmessage get(void);
	int getID(void);

};

#endif
