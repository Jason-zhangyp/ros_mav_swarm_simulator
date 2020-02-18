#ifndef LOGGER_H
#define LOGGER_H

#include <sstream>
#include <fstream>
#include <string>
#include <sstream>
#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <cstdarg>
#include <cstring>
#include <cstddef>
#include <iostream>
#include <vector>
#include "../trackers/statetracker.h" // Tracks the /ground_truth/state vector

using namespace std;

string logstring(const float &timer, const statetracker &ownTracker, const int &state_index);

#endif /*MATRIXREADER_H*/
