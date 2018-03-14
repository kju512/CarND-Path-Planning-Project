#ifndef HELPER_H
#define HELPER_H
#include <functional>
#include <iterator>
#include <map>
#include <vector>
#include <math.h>
#include "constants.h"
#include "spline.h"

using namespace std;


class coordinatesConverter
{
    public:
           double distance;
           vector<double> maps_s;
           vector<double> maps_x;
           vector<double> maps_y;
           vector<double> maps_dx;
           vector<double> maps_dy;

           coordinatesConverter(const double& distance,const vector<double>& maps_s, const vector<double>& maps_x,const vector<double>& maps_y,const vector<double>& maps_dx, const vector<double>& maps_dy);

           vector<double> getXY(const double& s,const double& d) const;
    private:
           tk::spline spl_x;
           tk::spline spl_y;
           tk::spline spl_dx;
           tk::spline spl_dy;
};

//double logistic(double x);

function<double(double)> to_equation(vector<double> coeffs);

vector<double> differentiate(vector<double> coeffs);

vector<function<double(double)>> get_f_and_N_derivatives(vector<double> coeffs,int n);

vector<double> JMT(vector<double> start, vector<double> end, double T);

//vector<double> getS_dot_D_dot(const double& vx,const double& vy,const double &s ,const double & d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);




#endif
