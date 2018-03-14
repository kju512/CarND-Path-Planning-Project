#include "helper.h"
#include <functional>
#include <iterator>
#include <map>
#include <math.h>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

/*
double logistic(double x)
{
   return 1.0/(1.0+exp(-x));
}
*/
function<double(double)> to_equation(vector<double> coeffs)
{
    auto f = [coeffs](double t)->double{
        double total = 0.0;
        for(int i=0; i<coeffs.size();i++)
            total += coeffs[i]*pow(t,i);
        return total;
    };
    return f;
}

vector<double> differentiate(vector<double> coeffs)
{
    vector<double> new_cos;
    for(int i=1; i<coeffs.size();i++)
        new_cos.push_back(i*coeffs[i]);
    return new_cos;
}

vector<function<double(double)>> get_f_and_N_derivatives(vector<double> coeffs,int n)
{
    vector<function<double(double)>> functions;
    functions.push_back(to_equation(coeffs));
    for(int i=0;i<n;i++)
    {
       coeffs = differentiate(coeffs);
       functions.push_back(to_equation(coeffs));
    }
    return functions;
}





vector<double> JMT(vector<double> start, vector<double> end, double T)
{
    /*
    Calculate the Jerk Minimizing Trajectory that connects the initial state
    to the final state in time T.

    INPUTS

    start - the vehicles start location given as a length three array
        corresponding to initial values of [s, s_dot, s_double_dot]

    end   - the desired end state for vehicle. Like "start" this is a
        length three array.

    T     - The duration, in seconds, over which this maneuver should occur.

    OUTPUT
    an array of length 6, each value corresponding to a coefficent in the polynomial
    s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5

    EXAMPLE

    > JMT( [0, 10, 0], [10, 10, 0], 1)
    [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
    */
    vector<double> result;
    result.push_back(start[0]);
    result.push_back(start[1]);
    result.push_back(start[2]/2);

    MatrixXd A(3,3);
    A << T*T*T, T*T*T*T, T*T*T*T*T,
             3*T*T, 4*T*T*T, 5*T*T*T*T,
             2*3*T, 3*4*T*T, 4*5*T*T*T;
    VectorXd b(3);
    VectorXd x(3);
    b << end[0]-(start[0]+start[1]*T+start[2]*T*T/2),
       end[1]-(start[1]+start[2]*T),
       end[2]-start[2];
    x = A.colPivHouseholderQr().solve(b);
    result.push_back(x[0]);
    result.push_back(x[1]);
    result.push_back(x[2]);
    return result;
}


coordinatesConverter::coordinatesConverter(const double& distance, const vector<double>& maps_s, const vector<double>& maps_x,const vector<double>& maps_y,const vector<double>& maps_dx, const vector<double>& maps_dy)
{
    this->distance = distance;
    this->maps_s=maps_s;
    this->maps_x=maps_x;
    this->maps_y=maps_y;
    this->maps_dx=maps_dx;
    this->maps_dy=maps_dy;

    this->maps_s.emplace_back(distance);
    this->maps_x.emplace_back(maps_x[0]);
    this->maps_y.emplace_back(maps_y[0]);
    this->maps_dx.emplace_back(maps_dx[0]);
    this->maps_dy.emplace_back(maps_dy[0]);


    this->spl_x.set_points(this->maps_s,this->maps_x);
    this->spl_y.set_points(this->maps_s,this->maps_y);
    this->spl_dx.set_points(this->maps_s,this->maps_dx);
    this->spl_dy.set_points(this->maps_s,this->maps_dy);

}
vector<double> coordinatesConverter::getXY(const double& s,const double& d) const
{
    double new_s = fmod(s,this->distance);
    double x_center = spl_x(new_s);
    double y_center = spl_y(new_s);
    double dx = spl_dx(new_s);
    double dy = spl_dy(new_s);
    double x = x_center+dx*d;
    double y = y_center+dy*d;
    return {x,y};
}


/**
* transform velocity x and velocity y to  s velocity and s velocity
*/
/**
vector<double> getS_dot_D_dot(const double& vx,const double& vy,const double &s ,const double & d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
    int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading1 = atan2(vy,vx);
    double heading2 = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
    double speed =sqrt(vx*vx+vy*vy);
	// the x,y,s along the segment
	double s_dot = speed* cos(heading1-heading2);
	double d_dot = speed* sin(heading1-heading2);

	return {s_dot,d_dot};
}
*/
