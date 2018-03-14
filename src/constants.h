#ifndef CONSTANTS_H
#define CONSTANTS_H
#include <iterator>
#include <map>
#include <vector>
#include <math.h>

using namespace std;


const double MAX_JERK = 3; // m/s/s/s
const double MAX_ACCEL= 3; // m/s/s
const double EXPECTED_JERK_IN_ONE_SEC = 10; // m/s/s
const double EXPECTED_ACC_IN_ONE_SEC = 10; // m/s

const double SPEED_LIMIT = 21.0;//m/s
const double VEHICLE_RADIUS = 1.5; // model vehicle as circle to simplify collision detection
const int LANES_AVAILABLE = 3;

//const double GOAL_DISTANCE=44.44;
const double GOAL_TIME=2.0;
const double BUFFER_DISTANCE=5.0;
const double GAP_SAFE =50;
const int DELAY_PTS=15;

const double FINAL_VELOCITY_W = 10.0;
const double AVG_VELOCITY_W = 10.0;

#endif
