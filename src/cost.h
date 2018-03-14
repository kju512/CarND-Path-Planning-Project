#ifndef COST_H
#define COST_H
#include "vehicle.h"
#include <functional>
#include <iterator>
#include <map>
#include <vector>
#include <math.h>
#include "helper.h"

using namespace std;

double final_velocity_cost(const Trajectory& trajectory,const vector<double>& target_s,const vector<double>& target_d,const double& T, map<int, Vehicle>& otherVehicles);

double avg_velocity_cost(const Trajectory& trajectory,const vector<double>& target_s,const vector<double>& target_d,const double& T, map<int, Vehicle>& otherVehicles);

double calculate_FSMcost(const Trajectory& trajectory,const vector<double>& target_s,const vector<double>& target_d,const double& T, map<int, Vehicle>& otherVehicles);

#endif
