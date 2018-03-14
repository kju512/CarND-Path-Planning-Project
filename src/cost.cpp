#include <functional>
#include <algorithm>
#include <iterator>
#include <map>
#include <math.h>
#include "constants.h"
#include "helper.h"
#include "cost.h"
#include "vehicle.h"

using namespace std;


double final_velocity_cost(const Trajectory & trajectory,
                         const vector<double>& target_s,
                         const vector<double>& target_d,
                         const double& T,
                         map<int, Vehicle>& otherVehicles)
{
    /*
    Cost increases based on distance of intended lane (for planning a lane change) and final lane of trajectory.
    Cost of being out of goal lane also becomes larger as vehicle approaches goal distance.
    */
    auto f_s =to_equation(differentiate(trajectory.s_coeffs));
    double final_v = f_s(trajectory.T);
    return fabs((SPEED_LIMIT - final_v) / SPEED_LIMIT);
}

double avg_velocity_cost(const Trajectory & trajectory,
                         const vector<double>& target_s,
                         const vector<double>& target_d,
                         const double& T,
                         map<int, Vehicle>& otherVehicles)
{
    /*
    Cost becomes higher for trajectories with intended lane and final lane that have traffic slower than vehicle's target speed.
    */
    auto f_s =to_equation(trajectory.s_coeffs);
    double avg_v = f_s(trajectory.T)/trajectory.T;
    return fabs((SPEED_LIMIT - avg_v) / SPEED_LIMIT);
}




double calculate_FSMcost(const Trajectory & trajectory,
                         const vector<double>& target_s,
                         const vector<double>& target_d,
                         const double& T,
                         map<int, Vehicle> & otherVehicles) {
    /*
    Sum weighted cost functions to get total cost for trajectory.
    */
    double cost = 0.0;

    //Add additional cost functions here.
    vector< function<double(const Trajectory &,const vector<double>&,const vector<double>&,const double&, map<int, Vehicle> &)>> cf_list;
    cf_list.push_back(final_velocity_cost);
    cf_list.push_back(avg_velocity_cost);

    vector<double> weight_list;
    weight_list.push_back(FINAL_VELOCITY_W);
    weight_list.push_back(AVG_VELOCITY_W);


    for (int i = 0; i < cf_list.size(); i++) {
        double new_cost = weight_list[i]*cf_list[i](trajectory,target_s,target_d,T, otherVehicles);
        cost += new_cost;
    }

    return cost;

}
