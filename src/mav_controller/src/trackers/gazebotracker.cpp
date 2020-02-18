#include "gazebotracker.h"

using namespace std;
	
gazebotracker::gazebotracker(){}

gazebotracker::~gazebotracker(){}

void gazebotracker::callback(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
    	list = msg->name;
}