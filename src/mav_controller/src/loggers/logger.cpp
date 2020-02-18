#include "logger.h"

/* Some useful namespaces */
using namespace std;
string logstring(const float &timer, const statetracker &ownTracker, const int &state_index)
{
    stringstream ss;
    ss  << timer << " " 
        << ownTracker.pose.y << " " << ownTracker.pose.x << " " << ownTracker.pose.z 
        << " " << state_index;
    return ss.str();
}