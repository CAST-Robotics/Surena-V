#pragma once

#include <iostream>
#include <eigen3/Eigen/Eigen>
#include <math.h>

using namespace std;
using namespace Eigen;

class Controller
{
public:
    template <typename T>
    T saturate(T high, T low, T value)
    {
        if (value <= high && value >= low)
            return value;
        else if (value > high)
            return high;
        else
            return low;
    }
    Controller(Matrix3d K_p_ = Matrix3d::Zero(3, 3), Matrix3d K_i_ = Matrix3d::Zero(3, 3), Matrix3d K_zmp_ = Matrix3d::Zero(3, 3), Matrix3d K_com_ = Matrix3d::Zero(3, 3));
    Vector3d dcmController(Vector3d xiRef, Vector3d xiDotRef, Vector3d xiReal, double deltaZVRP);
    Vector3d comController(Vector3d xCOMRef, Vector3d xDotCOMRef, Vector3d xCOMReal, Vector3d rZMPRef, Vector3d rZMPReal);
    void setK_p_(Matrix3d K_p);
    void setK_i_(Matrix3d K_i);
    void setK_zmp_(Matrix3d K_zmp);
    void setK_com_(Matrix3d K_com);
    void setDt(double dt);
    void setBaseHeight(double h);
    void setBaseIdle(double h);
    void setBaseLowHeight(double h);
    void setInitCoM(Vector3d init_com);
    double footLenController(double fz_d, double fz, double kp, double kd, double kr);
    Vector3d footOrientController(Vector3d tau_d, Vector3d tau, double k_p, double k_d, double k_r, bool is_right);
    Vector3d footDampingController(Vector3d zmp, Vector3d f_measured, Vector3d tau_measured, Matrix3d cop_gain, bool is_right);
    Vector3d bumpFootOrientController(int *const bump_mesured, Vector3d mean_bump_d, double k_p, double k_d, double k_r, bool is_right);
    Vector3d earlyContactController(int *const bump_measured, double desired_mean_bump, double K_p, double K_r, bool is_right);
    Vector3d ZMPAdmitanceComtroller_(Vector3d com_d, Vector3d com, Vector3d zmp_m, Vector3d zmp_qp, Matrix3d kp, Matrix3d kc);

private:
    Matrix3d K_p_;
    Matrix3d K_i_;
    Matrix3d K_zmp_;
    Matrix3d K_com_;
    Vector3d xiErrorInt;
    Vector3d CoM_;
    double dt_;
    double deltaZ_;

    Vector3d thetaAnkleR_;
    Vector3d thetaAnkleL_;

    Vector3d uOrientR_;
    Vector3d uOrientL_;
    Vector3d uBumpOrientR_;
    Vector3d uBumpOrientL_;
    Vector3d prevTau_[2];
    Vector3d prevTau_d_[2];
    double prevForceError_;
    Vector3d prevBumpMean_[2];
    Vector3d prevBump_d_[2];
    double prevEarlyContactMeanBump_[2];
    Vector3d prevAnkleTraj_[2];
    Vector3d preDeltaU_;
    bool firstContact_;
    Vector3d desiredContactPos_;
    Vector3d deltaUBumpR_;
    Vector3d deltaUBumpL_;
    double desired_mean_bump;
    double baseHeight_;
    double baseIdle_;
    double baseLowHeight_;

    Vector3d CoMAdmitance_;
    Matrix3d ACoM_;
};