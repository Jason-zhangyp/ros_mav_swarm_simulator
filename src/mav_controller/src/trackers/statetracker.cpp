#include "statetracker.h"

using namespace std;
	
statetracker::statetracker(){}

statetracker::~statetracker(){}

void statetracker::callback(const nav_msgs::Odometry::ConstPtr& msg)
{

	pose.x = msg->pose.pose.position.x;
	pose.y = msg->pose.pose.position.y;
	pose.z = msg->pose.pose.position.z;

	orientation.quaternion.qx = msg->pose.pose.orientation.x;
	orientation.quaternion.qy = msg->pose.pose.orientation.y;
	orientation.quaternion.qz = msg->pose.pose.orientation.z;
	orientation.quaternion.qw = msg->pose.pose.orientation.w;

	e = quaternion2euler(
		orientation.quaternion.qx,
		orientation.quaternion.qy,
		orientation.quaternion.qz,
		orientation.quaternion.qw
		);

	orientation.euler.roll  = e.roll;
	orientation.euler.pitch = e.pitch;
	orientation.euler.yaw   = e.yaw;

	velocity.x = msg->twist.twist.linear.x;
	velocity.y = msg->twist.twist.linear.y;
	velocity.z = msg->twist.twist.linear.z;

	angularrate.x = msg->twist.twist.angular.x;
	angularrate.y = msg->twist.twist.angular.y;
	angularrate.z = msg->twist.twist.angular.z;

}

Euler statetracker::quaternion2euler(double qx, double qy, double qz, double qw)
{
	Euler e;

	double ysqr = qy * qy;

	// roll (x-axis rotation)
	double t0 = +2.0f * (qw * qx + qy * qz);
	double t1 = +1.0f - 2.0f * (qx * qx + ysqr);
	e.roll = std::atan2(t0, t1);

	// pitch (y-axis rotation)
	double t2 = +2.0f * (qw * qy - qz * qx);
	t2 = t2 > 1.0f ? 1.0f : t2;
	t2 = t2 < -1.0f ? -1.0f : t2;
	e.pitch = std::asin(t2);

	// yaw (z-axis rotation)
	double t3 = +2.0f * (qw * qz + qx *qy);
	double t4 = +1.0f - 2.0f * (ysqr + qz * qz);  
	e.yaw = std::atan2(t3, t4);

	return e;
}