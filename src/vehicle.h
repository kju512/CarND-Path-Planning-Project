#ifndef VEHICLE_H
#define VEHICLE_H
#include <iostream>
#include <random>
#include <vector>
#include <map>
#include <string>
#include "constants.h"
#include "helper.h"
//#include "cost.h"

using namespace std;

class Trajectory
{
    public:
        vector<double> s_coeffs;
        vector<double> d_coeffs;
        double T;
        //string FSMstate;
};

class Vehicle
{
public:

  map<string, int> lane_direction = {{"LCL", -1}, {"LCR", 1}};


  int preferred_buffer = 6; // impacts "keep lane" behavior.

  int lane;//which lane the vehicle is in

  vector<double> s;

  vector<double> d;

  vector<double> s_coeffs; //Frenet coeffients for calculate s

  vector<double> d_coeffs;  //Frenet coeffients for calculate d

  //double T;

  double target_speed;

  int lanes_available;

  double max_acceleration;


  string FSMstate;

  /**
  * Constructor
  */
  Vehicle();
  Vehicle(vector<double> s, vector<double> d, string FSMstate="CS");

  /**
  * Destructor
  */
  virtual ~Vehicle();

  Trajectory choose_next_state(map<int, Vehicle> otherVehicles);

  vector<string> successor_states();

  Trajectory generate_trajectoryByFSMstate(string FSMstate, map<int, Vehicle> otherVehicles);

  Vehicle get_target(map<int, Vehicle> otherVehicles, int lane);

  Trajectory constant_speed_trajectory();

  Trajectory keep_lane_trajectory(map<int, Vehicle> otherVehicles);

  Trajectory lane_change_trajectory(string FSMstate, map<int, Vehicle> otherVehicles);

  vector<vector<double>> state_at(double t);

  bool get_vehicle_behind(map<int, Vehicle> otherVehicles, int lane, Vehicle & rVehicle);

  bool get_vehicle_ahead(map<int, Vehicle> otherVehicles, int lane, Vehicle & rVehicle);

  void generate_executiondata(Trajectory traj,vector<double>& s,vector<double>& d,int pt_num);

};

Trajectory trajGenerate(const vector<double>& start_s,
               const vector<double>& start_d,
               const Vehicle& target,
               const vector<double>& delta_s,
               const vector<double>& delta_d,
               const double& T);

#endif
