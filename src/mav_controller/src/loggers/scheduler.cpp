/* 
This is the main for the node controlling a single MAV 
All control functionalities are defined here.
It is designed to work with gazebo models flying around.
	
	**************** Input ROS topics ****************
		
	/gazebo/model_states:
		To get a string of all gazebo objects
		
	/<name>/ground_truth/states:
		To get the own state, where <name> is the argument as defined in the launch file.
		
	/<other>/ground_truth/states:
		To get relative states (and simulate measurements). These are found automatically based on the output from /gazebo/model_states, and need not be specified.

	**************** Output ROS topics ****************	
	/<name>/cmd_vel:
		Outputs a desired velocity to be followed (in the Earth inertial reference frame)

*/

// Standard C++ includes
#include <sstream>
#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <iomanip>
#include <string>
#include <map>
#include <random>

// ROS trackers
#include "ros/ros.h"  // Sets up all standard ros headers
#include "statetracker.h" // Tracks the /ground_truth/state vector

using namespace std;

int main(int argc, char **argv)
{
	// Initialize ROS node
	ros::init(argc, argv, "scheduler_node");
	ros::NodeHandle n;
	ros::Rate loop_rate(10);

	float mavsize; // Get arena corners
	if (!ros::param::get("~msize",mavsize))
		ROS_ERROR("Could not get desired MAV size!");

	int mavs; // Get name of own rotor from launch file
	if (!ros::param::get("~nmavs", mavs))
		ROS_ERROR("Could not discern behavior!");

	statetracker uav1Tracker;
	statetracker uav2Tracker;
	statetracker uav3Tracker;
	ros::Subscriber name_sub_1;
	ros::Subscriber name_sub_2;
	ros::Subscriber name_sub_3;


	if (mavs > 0){
		name_sub_1 = n.subscribe("/uav1/ground_truth/state", 
					1000, &statetracker::callback, &uav1Tracker); 	// Check for other mavs
	}
	
	if (mavs > 1){
		name_sub_2 = n.subscribe("/uav2/ground_truth/state",
					1000, &statetracker::callback, &uav2Tracker); 	// Check for other mavs
	}

	if (mavs > 2){
		name_sub_3 = n.subscribe("/uav3/ground_truth/state",
					1000, &statetracker::callback, &uav3Tracker); 	// Check for other mavs
	}

	float d12, d13, d23;
	bool flag;
	float posx;
	while(ros::ok())
	{
		flag = false;
		posx = uav1Tracker.pose.x;

		if (mavs > 1)
		{
			d12 = sqrt(pow(uav1Tracker.pose.x - uav2Tracker.pose.x,2)+
					 pow(uav1Tracker.pose.y - uav2Tracker.pose.y,2)+
	  				 pow(uav1Tracker.pose.z - uav2Tracker.pose.z,2));
			
			if (d12 < mavsize)
				flag = true;
		
		}

		if (mavs > 2)
		{
			d13 = sqrt(pow(uav1Tracker.pose.x - uav3Tracker.pose.x,2)+ 
					 pow(uav1Tracker.pose.y - uav3Tracker.pose.y,2)+ 
					 pow(uav1Tracker.pose.z - uav3Tracker.pose.z,2));

			d23 = sqrt(pow(uav2Tracker.pose.x - uav3Tracker.pose.x,2)+
					 pow(uav2Tracker.pose.y - uav3Tracker.pose.y,2)+
					 pow(uav2Tracker.pose.z - uav3Tracker.pose.z,2));
		
			if ((d13 < mavsize) || (d23 < mavsize))
				flag = true;
		
		}

		if (flag == true)
		{
			cout << "collision " << mavs << " " << posx << endl;
			// system("rosservice call /gazebo/set_model_state '{model_state: { model_name: uav1, pose: { position: { x: 1, y: 0 ,z: 0 }, orientation: {x: 0, y: 0, z: 0, w: 1 } }, twist: { linear: {x: 0.0 , y: 0 ,z: 0 } , angular: { x: 0.0 , y: 0 , z: 0.0 } } , reference_frame: world } }' && rosservice call /gazebo/set_model_state '{model_state: { model_name: uav2, pose: { position: { x: -1, y: 0 ,z: 0 }, orientation: {x: 0, y: 0, z: 0, w: 1 } }, twist: { linear: {x: 0.0 , y: 0 ,z: 0 } , angular: { x: 0.0 , y: 0 , z: 0.0 } } , reference_frame: world } }' && rosservice call /gazebo/set_model_state '{model_state: { model_name: uav3, pose: { position: { x: 0, y: 0 ,z: 0 }, orientation: {x: 0, y: 0, z: 0, w: 1 } }, twist: { linear: {x: 0.0 , y: 0 ,z: 0 } , angular: { x: 0.0 , y: 0 , z: 0.0 } } , reference_frame: world } }'");
			// system("rosservice call /gazebo/reset_simulation") ;
		}

		ros::spinOnce(); // Keep ROS running at the rate defined at the top of main(ROS)
		loop_rate.sleep(); // Sleep till next iteration (ROS)
	}

	return 0;
}