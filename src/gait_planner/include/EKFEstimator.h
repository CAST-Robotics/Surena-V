#pragma once

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cmath>

using namespace std;
using namespace Eigen;

class EKFEstimator
{
public:
    EKFEstimator();
    ~EKFEstimator();
    void setSensorData(Vector3d acc, Vector3d gyro);
    void setMeasuredData(Vector3d r_kynematic, Vector3d l_kynematic);

    void setState2prior();
    void setState2posterior();

    void predict();
    void predictState();
    void updateLc(Matrix3d rot);
    void updateQc();
    void updateFc(Matrix3d rot);
    void predictCov();

    void update();
    void updateRk();
    void updateHk();

    void runFilter(Vector3d acc, Vector3d gyro, Vector3d rfmeasured, Vector3d lfmeasured);

    Matrix3d skewSym(const Vector3d &vec);

private:
    Vector3d GLeftFootPos_;
    Vector3d GRightFootPos_;
    Vector3d BLeftFootPos_;
    Vector3d BRightFootPos_;
    Vector3d GBasePos_;
    Vector3d GBaseVel_;
    Quaterniond GBaseQuat_;
    Matrix3d GBaseRot_;

    Vector3d BRFootMeasured_;
    Vector3d BLFootMeasured_;

    Vector3d BAcc_;
    Vector3d BGyro_;

    Vector3d BAccBias_;
    Vector3d BGyroBias_;

    VectorXd xPrior_;
    VectorXd xPosterior_;
    VectorXd z_;
    VectorXd y_;
    VectorXd deltaX_;

    MatrixXd P_;

    MatrixXd Lc_;
    MatrixXd Qc_;
    MatrixXd Fc_;
    MatrixXd Fk_;
    MatrixXd Qk_;

    MatrixXd Rk_;
    MatrixXd Hk_;
    MatrixXd Sk_;
    MatrixXd Kk_;

    double wf_;
    double ww_;
    double wp_r_;
    double wp_l_;
    double wbf_;
    double wbw_;
    double np_r_;
    double np_l_;

    double initStateVar_;

    vector<double> Config_;
    vector<double> ConfigDot_;

    Vector3d Gravity_;

    int statesDim_;
    int statesErrDim_;
    int processNoiseDim_;
    int rvqStatesDim_;

    int measurmentDim_;
    double dt_;
};