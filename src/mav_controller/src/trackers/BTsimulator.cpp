#include "BTsimulator.h"

using namespace std;

BTsimulator::BTsimulator(const string &tracker, const string &tracked)
{

	// Store the IDs for the tracker and the tracked
	ID_tracker = tracker;
	ID_tracked = tracked;

	// Set up the topic names for the tracker and the tracked
	string trackertopic = "/" + ID_tracker + "/ground_truth/state";
	string trackedtopic = "/" + ID_tracked + "/ground_truth/state";

	// Set up the callback function for the tracker
	trackersub = n.subscribe(trackertopic,
		1000,
		&statetracker::callback,
		&trackerTracker);

	// Set up the callback function for the tracked object
	trackedsub = n.subscribe(trackedtopic,
		1000,
		&statetracker::callback,
		&trackedTracker);

}

BTsimulator::~BTsimulator(){}

BTmessage BTsimulator::getBTmessage()
{
	BTmessage msg;

	float trackedPsi = trackedTracker.orientation.euler.yaw;
	float vx = trackedTracker.velocity.x;
	float vy = trackedTracker.velocity.y;
				
	// Get the distance (needed for RSSI simulation)
	float d = magnitude( 3,
		trackerTracker.pose.x-trackedTracker.pose.x,
		trackerTracker.pose.y-trackedTracker.pose.y,
		trackerTracker.pose.z-trackedTracker.pose.z
		);
	
	msg.RSSI = distanceToRSSI(d);
	
	msg.vx = vx;
	msg.vy = vy;

	msg.h = trackedTracker.pose.z;
	msg.psi = trackedPsi;

	msg.dist = d;	// Just for simulation purposes so we know if there is a collision :)
	msg.x = trackedTracker.pose.x; // for simulation purposes to simulate the lobes
	msg.y = trackedTracker.pose.y; // for simulation purposes to simulate the lobes
	return msg;

}

int BTsimulator::distanceToRSSI(const float &dist){
	// TODO: Randomize Pn and gamma and add extra (weird) noise model

	int Pn = -63.0;
	float gamma = 2.0;

	return int( Pn - 10 * gamma * log10(dist) );

}