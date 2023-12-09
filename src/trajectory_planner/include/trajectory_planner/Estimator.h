#pragma once

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Core"
#include <eigen3/Eigen/Geometry>
#include "iostream"

using namespace Eigen;
using namespace std;

class Estimator
{
public:
    Estimator() {}
    void atitudeEulerEstimator(Vector3d &base_atitude, Vector3d gyro);
    void poseVelEstimator(Vector3d &base_vel, Vector3d &base_pos, Vector3d acc);
    inline void setDt(double dt) { dt_ = dt; }
    void gaussianPredict(Vector3d &posterior, double &P, double Q, Vector3d delta = Vector3d::Zero());
    void gaussianUpdate(Vector3d &prior, double &P, Vector3d z, double R);

private:
    double dt_;
};