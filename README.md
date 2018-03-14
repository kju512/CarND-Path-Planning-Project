# CarND-Path-Planning-Project
This is a Self-Driving Car Engineer Nanodegree Program completed by Michael chen.This program implements a Path-Plannner for self-driving in highway.

## Build and Run

1. clone this repository to your local machine.
2. implement below order step by step.  
>  mkdir build && cd build  
>  cmake .. && make  
>  ./path_planning   

3. open the Term 3 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases).and Select "Project 1:Path Planning".


## Model discription
This project's goal is to build a path planner for highway self driving.For completing this project,I use some techniques learned from the course.Totally,My model is composed by the following steps:
* 1)In documents vehicle.cpp and vehicle.h,we define a class"Vehicle" to depict a vehicle's state,this class can instance the car we drive and other vehicles on the same direction of the road.And define a class "Trajectory" to depict a trajectory which is represented by two quintic polynomial equations.
In this "Vehicle" class,we define the following various to discript the car.
  * a double type vector "s"-------denote Frenet coordinate s,it velocity and acceleration at the beginning
  * a double type  vector "d"-------denote Frenet coordinate d,it velocity and acceleration at the beginning. 
  * a double type vector "s_coeffs"------Frenet coordinate s of its trajectory's equation's coefficients,the equation is a quintic polynomial equation about time t. 
  * a double type vector "d_coeffs"------Frenet coordinate d of its trajectory's equation's coefficients,the equation is a quintic polynomial equation about time t.
In this "Trajectory" class,we use a double type vector "s_coeffs" and a double type vector "d_coeffs",and time t to depict the trajectory.
* 2)In documents helper.cpp and helper.h ,we define a class named "coordinatesConverter" and several functions "to_equation, differentiate, get_f_and_N_derivatives, JMT".The class "coordinatesConverter" can convert a Frenet coordinates to a XY coordinates.it uses spline to make the line's coordinates smooth.The Function "to_equation" can return a polynomial equation function from its coefficient.The Function "differentiate" can return a polynomial equation's differentiate coefficients from its coefficients. The Function "get_f_and_N_derivatives" can 
return a polynomial equation and its differentiate equations. The Function "JMT" can return a tracjectory's coefficients from its start point,end point and during time t.

* 3)In documents cost.cpp and cost.h ,we define two cost function "final_velocity_cost,avg_velocity_cost", and a function "calculate_FSMcost" for calculating a trajectory's total cost. In this scheme,we only consider a trajectory's final velocity and average velocity as factors to choose the best trajectory.
* 4)In documents constants.h,we define some parameters used in our model,such as availabe lanes number,speed limit,goal time and buffer distance etc.
* 5)In documents main.cpp,we design a pipeline as follows:
  * first, instantiate my car and other vehicles from the data received from the simulator.
  * second,use the information of my car and other vehicles to choose the best trajectory of next state from current state.
  * third, use the best trajectory to generate the execution data for the simulator,and send them to the simulator.  
* Several Other problems I met:  
1)About the latency between the simulator running and the path planner returning a path, this time is mainly determined by the path planner's running time.After some trial,I used 15 points for my solution.  
2)About the finite state mechine, at first I use "KL,LCR,LCL,PLCR,PLCL" From the course,but at last I found the only use "KL,LCR,LCL" three state ,it's enough,and I can't understand what action we need do in "PLCR" and "PLCL".  
3)About Frenet coordinate, Using spline to fit the waypoints is a good idea.  
4)About other vehicle's action,when a car in the adjacent lane change lane to my car's current lane,the distance between this two vehicle is very close. The action time is very short, a collision may happen and it is difficult to avoid.  



## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```

