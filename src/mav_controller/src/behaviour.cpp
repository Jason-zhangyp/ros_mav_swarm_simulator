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
#include <hector_uav_msgs/EnableMotors.h>
#include "trackers/gazebotracker.h" // Tracks the list of gazebo objects under /gazebo/model_states
#include "trackers/statetracker.h" // Tracks the /ground_truth/state vector
#include "trackers/relativetracker.h" //Simulates a relative sensor (BT in this case)
#include "loggers/logger.h"

// C includes -- for integration with Paparazzi
extern "C" {

  // Standard C includes
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

  // Own C includes
#include "functions/PID.h" // Function(s) for a basic PID controller
#include "functions/coordinateconversions.h"
#include "functions/randomgenerator.h" // Functions for random generations of number 
#include "functions/discreteekf.h"
#include "functions/collisioncone.h"
#include "functions/fmatrix.h"
#include "functions/shape.h"
#include "functions/arrayfunctions.h"
}

#include "trackers/relativetracker.h" //Simulates a relative sensor (BT in this case)

#define NUAVS_MAX 10     // Maximum (expected) number of MAVs in simulation
#define H_DES     1.0    // Desired level height
#define KP_V      1.0    // Vertical PID controller proportional gain
#define KP_H      0.5    // Horizontal PID controller proportional gain
#define STARTTIME 30.0   // Start time of experiment (in simulation time, seconds)
#define NTAR      2      // Number of targets in gazebo world (excl. MAVs, this includes ground and own)

#define RANGE_LIM 1.6    // Maximum range of sensor
#define _kr       1.0    // Repulsion gain
#define _ka       5.0    // Attraction gain
#define _ddes     1.0    // Desired equilibrium distance in lattice
#define d_safe    0.5    // Safety distance
#define _v_adj    2.0    // Adjustment velocity for lattice motions
#define ros_rate  10
// State action map txt file to load
// #define SA_MAP "/home/mario/phdwork/swarmulator/conf/state_action_matrices/state_action_matrix_triangle4.txt"
#define SA_MAP "/home/mario/phdwork/swarmulator/conf/state_action_matrices/state_action_matrix_triangle9.txt"

using namespace std;

float timelim = 1.5 * ros_rate;
int moving_timer = 0;
int nagents;
int selected_action_idx;
vector<relativetracker> btrcv;  // Set up a vector of relative position filters
float posx, posy, posz;
vector<float> blink;
std::map<int, vector<int>> state_action_matrix;
vector<int> closest(NUAVS_MAX);
bool doinganaction = false;
ros::ServiceClient motor_enable_service_;

template <typename Iter, typename RandomGenerator>
inline static Iter select_randomly(Iter start, Iter end, RandomGenerator &g)
{
  uniform_int_distribution<> dis(0, std::distance(start, end) - 1);
  advance(start, dis(g));
  return start;
}

template <typename Iter>
inline static Iter select_randomly(Iter start, Iter end)
{
  static random_device rd;
  static mt19937 gen(rd());
  return select_randomly(start, end, gen);
}

void collisionchecker(const float &dist_planar)
{
  float mavsize = 0.3;
  if ( dist_planar <= mavsize ) { 
      system("pkill -f ros");
  }
}

float deg2rad(float deg)
{
  return M_PI / 180.0 * deg;
}

int bool2int(vector<bool> t)
{
  int n = 0; //initialize
  for (int i = 0; i < 8; i++) {
    n += (int)t[i] * (int)pow(2, 7 - i);
  }
  return n;
}

float wrapToPi_f(float ang)
{
  if (ang > M_PI) {
    while (ang > M_PI) {
      ang = ang - 2 * M_PI;
    }
  } else if (ang < -M_PI) {
    while (ang < -M_PI) {
      ang = ang + 2 * M_PI;
    }
  }
  return ang;
}

float wrapTo2Pi_f(float ang)
{
  if (ang > 2 * M_PI) {
    while (ang > 2 * M_PI) {ang = ang - 2 * M_PI;}
  } else if (ang < 0.0) {
    while (ang < 0.0) {
      ang = ang + 2 * M_PI;
    }
  }
  return ang;
}

bool see_if_moving(uint8_t state_ID)
{
  if ( sqrt(pow(btrcv[state_ID].get().vx,2.0) + pow(btrcv[state_ID].get().vy,2.0)) > 0.1) {
    return true;
  } else {
    return false;
  }
}

bool check_motion(const vector<int> &state_ID)
{
  bool canImove = true;
  for (uint8_t i = 0; i < state_ID.size(); i++) {
    if (see_if_moving(state_ID[i])) {
      canImove = false; // Somebody nearby is already moving
    }
  }
  return canImove;
}

float request_distance(uint8_t ID_tracked)
{
  return sqrt(pow(btrcv[ID_tracked].get().y - posx, 2.0) + pow(btrcv[ID_tracked].get().x - posy, 2.0));// + rand_normal(0.0,0.1);
}

float request_bearing(uint8_t ID_tracked)
{
  return atan2(btrcv[ID_tracked].get().x - posy, btrcv[ID_tracked].get().y - posx);// rand_normal(0.0,0.1);
}

float f_repulsion(float u)
{
  return -_kr / u;
}

float f_attraction(const float &u, const float &b_eq)
{
  float w;
  // To do -- move general attraction equation to controller
  if (!(abs(b_eq - M_PI / 4.0) < 0.1 || abs(b_eq - (3 * M_PI / 4.0)) < 0.1)) {
    w = log((_ddes / _kr - 1) / exp(-_ka * _ddes)) / _ka;
  } else {
    w = log((sqrt(pow(_ddes, 2.0) + pow(_ddes, 2.0)) / _kr - 1) / exp(-_ka * sqrt(pow(_ddes, 2.0) + pow(_ddes, 2.0)))) / _ka;
  }

  return 1 / (1 + exp(-_ka * (u - w)));
}

float get_attraction_velocity(const float &u, const float &b_eq)
{
  return f_attraction(u, b_eq) + f_repulsion(u);
}

void attractionmotion(const float &v_r, const float &v_b, float &v_x, float &v_y)
{
  v_x += v_r * cos(v_b);
  v_y += v_r * sin(v_b);
}

void latticemotion(const float &v_r, const float &v_adj, const float &v_b, const float &bdes, float &v_x, float &v_y)
{
  attractionmotion(v_r + v_adj, v_b, v_x, v_y);
  v_x += -v_adj * cos(bdes * 2 - v_b);
  v_y += -v_adj * sin(bdes * 2 - v_b);
}

void actionmotion(const int &selected_action, float &v_x, float &v_y)
{
  float actionspace_x[8] = {1.0, sqrt(1.0), 0,   -sqrt(1.0), -1.0,  -sqrt(1.0), 0,      sqrt(1.0)};
  float actionspace_y[8] = {0,   sqrt(1.0), 1.0,  sqrt(1.0),  0,    -sqrt(1.0), -1.0,  -sqrt(1.0)};
  v_x = _v_adj * actionspace_x[selected_action];
  v_y = _v_adj * actionspace_y[selected_action];
}

// TODO: Make smarter
float get_preferred_bearing(const vector<float> &bdes, const float v_b)
{
  // Define in bv all equilibrium angles at which the agents can organize themselves
  vector<float> bv;
  for (int i = 0; i < 5; i++) {
    for (int j = 0; j < (int)bdes.size(); j++) {
      bv.push_back(bdes[j]);
    }
  }

  // Find what the desired angle is in bdes
  for (int i = 0; i < (int)bv.size(); i++) {
    if (i < (int)bdes.size() * 1) {
      bv[i] = abs(bv[i] - 2 * M_PI - v_b);
    } else if (i < (int)bdes.size() * 2) {
      bv[i] = abs(bv[i] - M_PI - v_b);
    } else if (i < (int)bdes.size() * 3) {
      bv[i] = abs(bv[i] - v_b);
    } else if (i < (int)bdes.size() * 4) {
      bv[i] = abs(bv[i] + M_PI - v_b);
    } else if (i < (int)bdes.size() * 5) {
      bv[i] = abs(bv[i] + 2 * M_PI - v_b);
    }
  }

  int minindex = 0;
  for (int i = 1; i < (int)bv.size(); i++) {
    if (bv[i] < bv[minindex]) {
      minindex = i;
    }
  }

  // Reduce the index for the angle of interest from bdes
  while (minindex >= (int)bdes.size()) {
    minindex -= (int)bdes.size();
  }

  // Returned the desired equilibrium bearing
  return bdes[minindex];
}

void get_lattice_motion(const int &state_ID, float &v_x, float &v_y)
{
  float v_b, b_eq, v_r;
  vector<float> beta_des;
  beta_des.push_back(0.0);
  beta_des.push_back(M_PI / 4.0);
  beta_des.push_back(M_PI / 2.0);
  beta_des.push_back(3.0 * M_PI / 4.0);

  v_b = wrapToPi_f(request_bearing(state_ID));
  b_eq = get_preferred_bearing(beta_des, v_b);
  v_r = get_attraction_velocity(request_distance(state_ID), b_eq);
  latticemotion(v_r, _v_adj, v_b, b_eq, v_x, v_y);
}

void get_lattice_motion_all(const vector<int> &state_ID, float &v_x, float &v_y)
{
  if (!state_ID.empty()) {
    if (request_distance(state_ID[0]) > 0.9) {
      for (size_t i = 0; i < state_ID.size(); i++) {
        get_lattice_motion(state_ID[i], v_x, v_y);
      }
      v_x = v_x / (float)state_ID.size();
      v_y = v_y / (float)state_ID.size();
    } else {
      get_lattice_motion(state_ID[0], v_x, v_y);
    }
  }
}

void set_state_action_matrix(string filename)
{
  state_action_matrix.clear();
  ifstream state_action_matrix_file(filename);

  if (state_action_matrix_file.is_open()) {
    // Collect the data inside the a stream, do this line by line
    while (!state_action_matrix_file.eof()) {
      string line;
      getline(state_action_matrix_file, line);
      stringstream stream_line(line);
      int buff[10] = {};
      int columns = 0;
      int state_index;
      bool index_checked = false;
      while (!stream_line.eof()) {
        if (!index_checked) {
          stream_line >> state_index;
          index_checked = true;
        } else {
          stream_line >> buff[columns];
          columns++;
        }
      }
      if (columns > 0) {
        vector<int> actions_index(begin(buff), begin(buff) + columns);
        state_action_matrix.insert(pair<int, vector<int>>(state_index, actions_index));
      }
    }
    state_action_matrix_file.close();
  }
}

// TODO: Make smarter
bool fill_template(vector<bool> &q, const float &b_i, const float &u, const float &dmax, const float &angle_err)
{
  // Determine link (cycle through all options)
  if (u < dmax) { // If in range of sensor
    for (int j = 0; j < (int)blink.size(); j++) { // For all angle options
      if (abs(b_i - blink[j]) < deg2rad(angle_err) && !q[j]) {   // If in the right angle and not already taken by another agent
        if (j == (int)blink.size() - 1) { // last element is back to 0
          j = 0;
        }
        q[j] = true;
        return true;
      }
    }
  }
  return false;
}

void assess_situation(vector<bool> &q)
{
  q.clear();
  q.assign(8, false);
  // Fill the template with respect to the agent in question
  for (uint8_t i = 0; i < nagents; i++) {
    fill_template(q, // Vector to fill
                  wrapTo2Pi_f(request_bearing(closest[i])), // Bearing
                  request_distance(closest[i]), // Distance
                  RANGE_LIM, 22.499); // Log ID (for simulation purposes only, depending on assumptions)
  }
}

bool enableMotors(bool enable)
{
  if (!motor_enable_service_.waitForExistence(ros::Duration(10.0))) {
    ROS_WARN("Motor enable service not found");
    return false;
  }

  hector_uav_msgs::EnableMotors srv;
  srv.request.enable = enable;
  return motor_enable_service_.call(srv);
}

int main(int argc, char **argv)
{
  // Initialize ROS node
  ros::init(argc, argv, "behaviour");
  ros::NodeHandle n;
  ros::Rate loop_rate(ros_rate);

  blink.push_back(0);
  blink.push_back(M_PI / 4.0);
  blink.push_back(M_PI / 2.0);
  blink.push_back(3 * M_PI / 4.0);
  blink.push_back(M_PI);
  blink.push_back(deg2rad(180 + 45));
  blink.push_back(deg2rad(180 + 90));
  blink.push_back(deg2rad(180 + 135));
  blink.push_back(2 * M_PI);

  // Get name of own rotor from launch file
  int ID;
  if (!ros::param::get("~id", ID)) {
    ROS_ERROR("Could not get ID!");
  }
  ostringstream ss;
  string name;
  ss << ID;
  name = "uav" + ss.str();
  vector<int> state_ID;

  // Set up a message object that outputs the data
  geometry_msgs::Twist cmddata;
  string cmdtopic = "/" + name + "/cmd_vel";
  ros::Publisher cmd_pub = n.advertise<geometry_msgs::Twist>(cmdtopic, 1000);

  // Set up the listeners
  gazebotracker gazeboTracker;
  statetracker  ownTracker;
  string ownstate_topic        = "/" + name + "/ground_truth/state";
  ros::Subscriber name_sub     = n.subscribe("/gazebo/model_states", 1000, &gazebotracker::callback, &gazeboTracker);   // Check for other mavs
  ros::Subscriber ownstate_sub = n.subscribe(ownstate_topic, 1000, &statetracker::callback, &ownTracker);   // Get own state

  // Relative tracker
  btrcv.reserve(NUAVS_MAX - 1); // Reserve spots for the expected amount of MAVs

  // State Action Matrix
  set_state_action_matrix(SA_MAP);


  /************** Random number generator ***************/
  random_device rd; // Initialize
  mt19937 e2(rd()); // Start with random seed
  randomgen_init(); // Initialize random seed

  /************** Simulation Temp Stuff *********************/
  float t_elps;
  string seen = "temp";

  /************** Set up the waypoint generator **************/
  /*
  The functions below to do with random number generation over a uniform distribution are C compatible.
  There are also C++ functions used later to simulate Gaussian measurement noise, but that is ok because those will not need to be exported to Paparazzi.
  */

  /***** Init *****/
  int ntargets = NTAR;
  int nf = 0;

  // Periodic variables
  float errint_x = 0;
  float errint_y = 0;
  float errint_z = 0;
  struct indexed_array dm[NUAVS_MAX];

  int IDarray[NUAVS_MAX - 1], index;
  array_make_zeros_int(NUAVS_MAX - 1, IDarray);

  // Initialize moving_timer with random variable
  if (moving_timer == 0) {
    moving_timer = rand() % (int)timelim;
  }

  /******************** Log file ******************/
  string filename = name + "_log.txt";
  ofstream logfile;
  logfile.open (filename.c_str());
  logfile.precision(4);
  bool flag = false;
  motor_enable_service_ = n.serviceClient<hector_uav_msgs::EnableMotors>("enable_motors");
  while (!enableMotors(true)) {}

  while (ros::ok()) {
    t_elps = ros::Time::now().toSec() - STARTTIME;

    if (t_elps > 0) {
      // Collect own data (ROS specific)
      posx   = ownTracker.pose.y; // ENU TO NEU
      posy   = ownTracker.pose.x;
      posz   = ownTracker.pose.z;
      
      // Re-initialize variables
      float vx_des = 0.0;
      float vy_des = 0.0;

      /* Scan for other MAVs */
      if (gazeboTracker.list.size() > ntargets) {
        /* If we are here then we have more than one MAV around
        What we want to do now is to check that the ID of this object is not the same as ours or the other objects in the environment
        */
        for (int t = 0; t < gazeboTracker.list.size(); t++) {
          if ((gazeboTracker.list.at(t).compare("ground_plane"))
              && (gazeboTracker.list.at(t).compare(name))
              && ((ntargets - NTAR) < NUAVS_MAX - 1)) {
            if (t == 0) { // Spawn a relative tracker if new MAV is confirmed
              seen = gazeboTracker.list.at(t);
            }

            if (gazeboTracker.list.at(t).compare(seen)) { // Register the new object
              btrcv.push_back(relativetracker(name, gazeboTracker.list.at(t)));
              ntargets++; // Increase the number of objects we are already aware of
            }
          }
        }
      }

      // If a new target is found, and there is no filter for it, create a new filter
      if (nf < (NUAVS_MAX - 1)) {
        for (int i = 0; i < ntargets - NTAR; i++) {
          index = -1;
          int ac_id = btrcv[i].getID();
          // Check if we already registered an ekf for this ID. If not, then start an EKF filter.
          if (!array_find_int(NUAVS_MAX - 1, IDarray, ac_id, &index)) {
            IDarray[nf] = ac_id; // Add the ID to the list
            nf++;
          }
        }
      }
      nagents = nf;

      for (int i = 0 ; i < nf; i++) {
        btrcv[i].update(); // Update the fake Bluetooth message
        dm[i].values = sqrt(pow(btrcv[i].get().y - posx, 2.0) + pow(btrcv[i].get().x - posy, 2.0)) ;
        dm[i].index  = i;
      }

      array_sortmintomax_index(nf, dm);
      state_ID.clear();
      for (int i = 0 ; i < nf; i++) {
        if (dm[i].values < RANGE_LIM) {
          state_ID.push_back(dm[i].index);
        }
        closest[i] = dm[i].index;
      }

      float twait_1 = timelim * 2;
      float twait_2 = twait_1 * 2;

      vector<bool> state(8, 0);
      assess_situation(state);
      int state_index = bool2int(state);
      bool canImove = check_motion(state_ID);
      if (!canImove) {
        selected_action_idx = -2;  // Reset actions
        moving_timer = twait_1; // Reset moving timer
      }

      // Try to find an action that suits the state, if available (otherwise you are in Sdes or Sblocked)
      // If you are already busy with an action, then don't change the action
      std::map<int, vector<int>>::iterator state_action_row;
      state_action_row = state_action_matrix.find(state_index);

      if (!doinganaction && state_action_row != state_action_matrix.end() && request_distance(state_ID[0]) > d_safe) {
        selected_action_idx = *select_randomly(state_action_row->second.begin(), state_action_row->second.end());
      } else if (!doinganaction) {
        selected_action_idx = -2;
      }

      // High level behavior
      doinganaction = false;
      if (canImove && ownTracker.pose.z > 0.6) {
        if (selected_action_idx > -1 && moving_timer < timelim && request_distance(state_ID[0]) > 0.5) {
          actionmotion(selected_action_idx, vx_des, vy_des);
          doinganaction = true;
        } else {
          get_lattice_motion_all(state_ID, vx_des, vy_des);
        }
        if (moving_timer > twait_2) {
          moving_timer = 1;
        } else {
          moving_timer++;
        }
      }

      // PID Controllers
      cmddata.linear.x = PID(KP_H, 2.0, 0.0, vy_des - ownTracker.velocity.x, 0.0, &errint_x, 0.0);
      cmddata.linear.y = PID(KP_H, 2.0, 0.0, vx_des - ownTracker.velocity.y, 0.0, &errint_y, 0.0);
      cmddata.linear.z = PID(KP_V, 2.0, 0.0,  H_DES - ownTracker.pose.z,     0.0, &errint_z, 0.0);
      cmd_pub.publish(cmddata); // Publish the desired velocity on a ROS topic

      if (ownTracker.pose.z > 0.9 && !flag){
        cout << ID << " has taken off" << endl;;
        flag = true;
      }

      logfile << logstring(t_elps, ownTracker,state_index) << endl;

      if (t_elps >= 5000) {
        system("pkill -f ros");
      }
    }

    ros::spinOnce();    // Keep ROS running at the pre-defined rate
    loop_rate.sleep();  // Sleep till next iteration (ROS)
  }

  return 0;
}
