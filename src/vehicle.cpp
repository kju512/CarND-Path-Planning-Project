#include <algorithm>
#include <iostream>
#include <cmath>
#include <map>
#include <string>
#include <iterator>
#include "constants.h"
#include "vehicle.h"
#include "cost.h"


using namespace std;



//generate a traj from a goal(vehicle)
Trajectory trajGenerate(const vector<double>& start_s,
               const vector<double>& start_d,
               const Vehicle& target,
               const vector<double>& delta_s,
               const vector<double>& delta_d,
               const double& T)
{
    vector<double> goal_s,goal_d;
    for(int i=0;i<3;i++)
    {
        goal_s.push_back(target.s[i]-delta_s[i]);
        goal_d.push_back(target.d[i]-delta_d[i]);
    }

    Trajectory traj;
    traj.s_coeffs = JMT(start_s, goal_s, T);
    traj.d_coeffs = JMT(start_d, goal_d, T);
    traj.T = T;

    return traj;
}



/**
 * Initializes Vehicle
 */

Vehicle::Vehicle(){}

Vehicle::Vehicle(vector<double> s,vector<double> d, string FSMstate) {
    this->lane = (int)(d[0]/4);
    this->s = s;
    this->d = d;
    this->s_coeffs.push_back(s[0]);
    this->s_coeffs.push_back(s[1]);
    this->s_coeffs.push_back(s[2]/2.0);
    this->d_coeffs.push_back(d[0]);
    this->d_coeffs.push_back(d[1]);
    this->d_coeffs.push_back(d[2]/2.0);
    this->FSMstate = FSMstate;
    this->max_acceleration = MAX_ACCEL;
    this->target_speed = SPEED_LIMIT;
    this->preferred_buffer = BUFFER_DISTANCE;

}

Vehicle::~Vehicle() {}


Trajectory Vehicle::choose_next_state(map<int, Vehicle> otherVehicles) {
    /*
    Here you can implement the transition_function code from the Behavior Planning Pseudocode
    classroom concept. Your goal will be to return the best (lowest cost) trajectory corresponding
    to the next state.

    INPUT: A predictions map. This is a map of vehicle id keys with predicted
        vehicle trajectories as values. Trajectories are a vector of Vehicle objects representing
        the vehicle at the current timestep and one timestep in the future.
    OUTPUT: The the best (lowest cost) trajectory corresponding to the next ego vehicle state.

    */
    vector<string> FSMstates = successor_states();
    double cost;
    vector<double> costs;
    vector<string> final_states;
    vector<Trajectory> final_trajectories;
    vector<string> final_state;

    for (vector<string>::iterator it = FSMstates.begin(); it != FSMstates.end(); ++it) {
        Trajectory trajectory = generate_trajectoryByFSMstate(*it, otherVehicles);
        if (trajectory.s_coeffs.size() != 0) {
            cost = calculate_FSMcost(trajectory,{this->s[0]+GAP_SAFE,SPEED_LIMIT,0},{lane*4.0+2.0, 0, 0},GOAL_TIME,otherVehicles);
            costs.push_back(cost);
            final_trajectories.push_back(trajectory);
            final_state.push_back(*it);
            //debug
            auto f_s =to_equation(differentiate(trajectory.s_coeffs));
            double final_v = f_s(trajectory.T);
            auto f_s1 =to_equation(trajectory.s_coeffs);
            double dist =f_s1(trajectory.T)-f_s1(0);
            double avg_v = f_s1(trajectory.T)/trajectory.T;
            std::cout<<"STATE: "<<*it<<" final_v is "<<final_v<<";\t"<<" distance is " <<dist<<";\t"<<"avg_v is "<<avg_v<<";\t"<<"its cost is "<<cost<<"\n"<<std::endl;

        }
    }

    vector<double>::iterator best_cost = min_element(begin(costs), end(costs));
    int best_idx = distance(begin(costs), best_cost);
    std::cout<<"chose the Best state is "<<final_state[best_idx]<<"\n"<<std::endl;
    return final_trajectories[best_idx];
}

vector<string> Vehicle::successor_states() {
    /*
    Provides the possible next states given the current state for the FSM
    discussed in the course, with the exception that lane changes happen
    instantaneously, so LCL and LCR can only transition back to KL.
    */
    vector<string> states;
    states.push_back("KL");
    this->lane =(int)(d[0]/4);
    if (lane != LANES_AVAILABLE - 1) {
        states.push_back("LCR");
    }
    if (lane != 0) {
        states.push_back("LCL");
    }

    //If state is "LCL" or "LCR", then just return "KL"

    return states;
}

Trajectory Vehicle::generate_trajectoryByFSMstate(string FSMstate, map<int, Vehicle> otherVehicles) {
    /*
    Given a possible next state, generate the appropriate trajectory to realize the next state.
    */
    Trajectory trajectory;
    if (FSMstate.compare("CS") == 0) {
        trajectory = constant_speed_trajectory();
    } else if (FSMstate.compare("KL") == 0) {
        trajectory = keep_lane_trajectory(otherVehicles);
    } else if (FSMstate.compare("LCL") == 0 || FSMstate.compare("LCR") == 0) {
        trajectory = lane_change_trajectory(FSMstate, otherVehicles);
    }
    return trajectory;
}

Vehicle Vehicle::get_target(map<int, Vehicle> otherVehicles, int lane) {
    /*
    Gets t time later's kinematics needed to be (position, velocity, acceleration)
    for a given lane. Tries to choose the maximum velocity and acceleration,
    given other vehicle positions and accel/velocity constraints.
    */
    Vehicle vehicle_ahead;
    //Vehicle vehicle_behind;
    Vehicle target;

    if (get_vehicle_ahead(otherVehicles, lane, vehicle_ahead)) {

            double gap = vehicle_ahead.s[0]-this->s[0];
            if (gap>GAP_SAFE)
            {

                double target_v = SPEED_LIMIT-0.5;
                double target_distance = (this->s[1]+target_v)*GOAL_TIME/2;
                double distance_max = this->s[1]*GOAL_TIME+(MAX_ACCEL-0.5)*GOAL_TIME*GOAL_TIME/2;
                double v_max = this->s[1]+(MAX_ACCEL-0.5)*GOAL_TIME;
                if(target_distance>distance_max)
                {
                    target_distance=distance_max;
                    target_v=2*target_distance/GOAL_TIME-this->s[1];
                }
                if(target_v>v_max)
                {
                    target_v=v_max;
                    target_distance = (this->s[1]+target_v)*GOAL_TIME/2;
                }
                vector<double> s = {this->s[0]+target_distance, target_v, 0};
                vector<double> d = {lane*4.0+2.0, 0, 0};
                //vector<double> s = {this->s[0]+GAP_SAFE, SPEED_LIMIT-0.5, 0};
                //vector<double> d = {lane*4.0+2.0, 0, 0};
                target = Vehicle(s, d); //must travel at the speed of traffic, regardless of preferred buffer
            }
            else
            {
                double target_v = vehicle_ahead.s[1]-0.5;
                double target_distance = (this->s[1]+target_v)*GOAL_TIME/2;
                if(target_v>SPEED_LIMIT-0.5)
                {
                     target_v = SPEED_LIMIT-0.5;
                     target_distance = (this->s[1]+target_v)*GOAL_TIME/2;
                }
                if(target_v<this->s[1])
                {
                    double distance_min = this->s[1]*GOAL_TIME-(MAX_ACCEL-0.5)*GOAL_TIME*GOAL_TIME/2;
                    double v_min = this->s[1]-(MAX_ACCEL-0.5)*GOAL_TIME;
                    if(target_distance<distance_min)
                    {
                        target_distance=distance_min;
                        target_v = this->s[1]-(MAX_ACCEL-0.5)*GOAL_TIME;
                    }
                    if(target_distance>gap)
                    {
                        target_distance = gap;
                        target_v=2*target_distance/GOAL_TIME-this->s[1];
                    }
                }
                vector<double> s = {this->s[0]+target_distance, target_v, 0};
                vector<double> d = {lane*4.0+2.0, 0, 0};

                target = Vehicle(s, d);
            }
    }
    else
    {
        double target_v = SPEED_LIMIT-0.5;
        double target_distance = (this->s[1]+target_v)*GOAL_TIME/2;
        double distance_max = this->s[1]*GOAL_TIME+(MAX_ACCEL-0.5)*GOAL_TIME*GOAL_TIME/2;
        double v_max = this->s[1]+(MAX_ACCEL-0.5)*GOAL_TIME;
        if(target_distance>distance_max)
        {
            target_distance=distance_max;
            target_v=2*target_distance/GOAL_TIME-this->s[1];
        }
        if(target_v>v_max)
        {
            target_v=v_max;
            target_distance = (this->s[1]+target_v)*GOAL_TIME/2;
        }

        vector<double> s = {this->s[0]+target_distance, target_v, 0};
        //vector<double> s = {this->s[0]+GAP_SAFE, SPEED_LIMIT-0.5, 0};
        vector<double> d = {lane*4.0+2.0, 0, 0};

        target = Vehicle(s, d);
    }

    return target;
}

Trajectory Vehicle::constant_speed_trajectory() {
    /*
    Generate a constant speed trajectory.
    */
    Trajectory trajectory;
    trajectory.s_coeffs.emplace_back(this->s_coeffs[0]);
    trajectory.s_coeffs.emplace_back(this->s_coeffs[1]);
    trajectory.d_coeffs.emplace_back(this->d_coeffs[0]);
    trajectory.T = GOAL_TIME;
    return trajectory;
}

Trajectory Vehicle::keep_lane_trajectory(map<int, Vehicle> otherVehicles) {
    /*
    Generate a keep lane trajectory.
    */
    //t = getProperTrajectoryTime(otherVehicles,this->lane);
    int target_lane = 0;
    if(this->lane < 0)
       target_lane = 0;
    else if(this->lane > 2)
       target_lane = 2;
    else
       target_lane =this->lane;
    Vehicle target = get_target(otherVehicles, target_lane);
    vector<double> delta_s = {0,0,0};
    vector<double> delta_d = {0,0,0};

    Trajectory trajectory = trajGenerate(this->s, this->d, target,delta_s,delta_d, GOAL_TIME);

    return trajectory;
}


Trajectory Vehicle::lane_change_trajectory(string FSMstate, map<int, Vehicle> otherVehicles)
{
    /*
    Generate a lane change trajectory.
    */
    Trajectory trajectory;
    int new_lane = this->lane + lane_direction[FSMstate];
    Vehicle next_lane_vehicle;
    double t;
    //Check if a lane change is possible (check if another vehicle occupies that spot).
    for (map<int, Vehicle>::iterator it = otherVehicles.begin(); it != otherVehicles.end(); ++it) {
        next_lane_vehicle = it->second;
        if (fabs(this->s[0] - next_lane_vehicle.s[0]) < this->preferred_buffer && next_lane_vehicle.lane == new_lane) {
            //If lane change is not possible, return empty trajectory.
            return trajectory;
        }
    }

    Vehicle vehicle_ahead;
    Vehicle vehicle_behind;
    double dist1,dist2;

    if (get_vehicle_ahead(otherVehicles, new_lane, vehicle_ahead))
    {
        dist1 = min(GAP_SAFE,vehicle_ahead.s[0]- this->s[0]);
    }
    else
        dist1 = GAP_SAFE;

    if (get_vehicle_behind(otherVehicles, new_lane, vehicle_behind))
    {
        dist2 = min(GAP_SAFE,this->s[0]-vehicle_behind.s[0]);
    }
    else
        dist2 = GAP_SAFE;

    if(dist1>=GAP_SAFE&&dist2>=GAP_SAFE/5)
    {
    //Keep speed of current lane so as not to collide with car behind.
        Vehicle target = get_target(otherVehicles, new_lane);
        //Choose kinematics with lowest velocity.
        vector<double> delta_s = {0,0,0};
        vector<double> delta_d = {0,0,0};

        trajectory = trajGenerate(this->s, this->d, target,delta_s,delta_d, GOAL_TIME);
        return trajectory;
    }
    else
        return trajectory;
}

vector<vector<double>> Vehicle::state_at(double t) {

    vector<double> newstate_s,newstate_d;
    auto f1_s = to_equation(s_coeffs);
    auto f1_d = to_equation(d_coeffs);
    newstate_s.push_back(f1_s(t));
    newstate_d.push_back(f1_d(t));
    auto f2_s = to_equation(differentiate(s_coeffs));
    auto f2_d = to_equation(differentiate(d_coeffs));
    newstate_s.push_back(f2_s(t));
    newstate_d.push_back(f2_d(t));
    auto f3_s = to_equation(differentiate(differentiate(s_coeffs)));
    auto f3_d = to_equation(differentiate(differentiate(d_coeffs)));
    newstate_s.push_back(f3_s(t));
    newstate_d.push_back(f3_d(t));

    return {newstate_s,newstate_d};
}


bool Vehicle::get_vehicle_behind(map<int, Vehicle> otherVehicles, int lane, Vehicle & rVehicle) {
    /*
    Returns a true if a vehicle is found behind the current vehicle, false otherwise. The passed reference
    rVehicle is updated if a vehicle is found.
    */
    double max_s = -1;
    bool found_vehicle = false;
    Vehicle temp_vehicle;
    for (map<int, Vehicle>::iterator it = otherVehicles.begin(); it != otherVehicles.end(); ++it) {
        temp_vehicle = it->second;
        if (temp_vehicle.lane == lane && temp_vehicle.s[0] < this->s[0] && temp_vehicle.s[0] > max_s) {
            max_s = temp_vehicle.s[0];
            rVehicle = temp_vehicle;
            found_vehicle = true;
        }
    }
    return found_vehicle;
}

bool Vehicle::get_vehicle_ahead(map<int, Vehicle> otherVehicles, int lane, Vehicle & rVehicle) {
    /*
    Returns a true if a vehicle is found ahead of the current vehicle, false otherwise. The passed reference
    rVehicle is updated if a vehicle is found.
    */
    double min_s = 100000;
    bool found_vehicle = false;
    Vehicle temp_vehicle;
    for (map<int, Vehicle>::iterator it = otherVehicles.begin(); it != otherVehicles.end(); ++it) {
        temp_vehicle = it->second;
        if (temp_vehicle.lane == lane && temp_vehicle.s[0] > this->s[0] && temp_vehicle.s[0] < min_s) {
            min_s = temp_vehicle.s[0];
            rVehicle = temp_vehicle;
            found_vehicle = true;
        }
    }
    return found_vehicle;
}

void Vehicle::generate_executiondata(Trajectory traj,vector<double>& s,vector<double>& d,int pt_num) {
    /*
    Sets state and kinematics for ego vehicle using the last state of the trajectory.
    */
    this->s_coeffs = traj.s_coeffs;
    this->d_coeffs = traj.d_coeffs;
    for(int i=1;i<=pt_num;i++)
    {
        vector<vector<double>> state = this->state_at(i*0.02);
        s.push_back(state[0][0]);
        d.push_back(state[1][0]);
    }
}


