#pragma once

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Core"
#include <eigen3/Eigen/Geometry>

#include "iostream"
#include "fstream"
#include <vector>
#include "math.h"
#include "cmath"
#include <ros/ros.h>

using namespace Eigen;
using namespace std;

class PID
{
public:
    PID(double timeStep);

private:
    ros::NodeHandle nh;
    // Controller Gains
    Matrix3d kp_;
    Matrix3d ki_;
    Matrix3d kd_;
    Matrix3d kcom_;
    Matrix3d kzmp_;
    Vector3d xi_error_;

    double dt_;

    double prevoiusError_; // Controller Previous Error
    double intI_;          // Controller Integrator

    // double getOutput(double deiredValue, double currentValue);
    /*
    bool dcmController(trajectory_planner::DCMController::Request &req,
                       trajectory_planner::DCMController::Response &res);
    bool comController(trajectory_planner::COMController::Request &req,
                       trajectory_planner::COMController::Response &res);
                       */
};
