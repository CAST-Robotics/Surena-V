#pragma once

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cmath>
#include <random>

#include "utils.h"

using namespace std;
using namespace Eigen;

class LieEKF {
    public:
        LieEKF();
        ~LieEKF();
        void initializeStates(Matrix3d R, Vector3d base_vel, Vector3d base_pos, Vector3d lf_pos, 
                              Vector3d rf_pos, Vector3d gyro_bias, Vector3d acc_bias);
        void concatStates(Matrix3d R, Vector3d base_vel, Vector3d base_pos, Vector3d lf_pos, 
                          Vector3d rf_pos, Vector3d gyro_bias, Vector3d acc_bias, MatrixXd &output, VectorXd &output_theta);
        void initializeCovariance(double quat_std, double vel_std, double pos_std,
                                  double contact_std, double gyro_std, double acc_std);
        inline void setDt(double dt){dt_ = dt;}
        void setSensorData(Vector3d gyro, Vector3d acc);
        void setMeasuredData(Vector3d l_kynematic, Vector3d r_kynematic, Matrix3d l_kynematic_rot, Matrix3d r_kynematic_rot);
        void setNoiseStd(double gyro_noise, double acc_noise, double gyro_bias_noise, 
                         double acc_bias_noise, double contact_noise, double measurement_noise);

        void seprateStates(const MatrixXd &x, const VectorXd &theta);

        void predict();
        void updateQd();
        
        void updateState(VectorXd delta, const MatrixXd &prev_x, const VectorXd &prev_theta, MatrixXd &x, VectorXd &theta);
        void update();

        void runFilter(Vector3d gyro, Vector3d acc, Vector3d lfpmeasured, Vector3d rfpmeasured, Matrix3d lfrot, Matrix3d rfrot, int* contact, bool update_enaled);

        Vector3d getGBasePose();
        Vector3d getGBaseVel();
        Quaterniond getGBaseQuat();
    
    private:
        Vector3d GLeftFootPos_;
        Vector3d GRightFootPos_;
        Vector3d BLeftFootPos_;
        Vector3d BRightFootPos_;
        Vector3d GBasePos_;
        Vector3d GBaseVel_;
        Matrix3d GBaseRot_;

        Vector3d BRFootMeasured_;
        Vector3d BLFootMeasured_;
        Matrix3d BRFootRotMeasured_;
        Matrix3d BLFootRotMeasured_;
        int contact_[2];

        Vector3d BAcc_;
        Vector3d BGyro_;

        Vector3d BAccBias_;
        Vector3d BGyroBias_;

        MatrixXd x_;
        MatrixXd xPrev_;
        MatrixXd xPred_;
        VectorXd theta_;
        VectorXd thetaPrev_;
        VectorXd thetaPred_;

        VectorXd Y_;
        VectorXd b_;
        MatrixXd PI_;
        MatrixXd BigX_;
        VectorXd deltaX_;


        MatrixXd P_;
        MatrixXd Ppred_;

        MatrixXd Qd_;
        MatrixXd Phi_;

        MatrixXd Hd_;
        MatrixXd Rd_;
        MatrixXd Sd_;
        MatrixXd Kd_;
        
        double gyroNoiseStd_;
        double accNoiseStd_;
        double gyroBiasNoiseStd_;
        double accBiasNoiseStd_;
        double contactNoiseStd_;

        double measurementNoiseStd_;

        //initial states standard deviations
        double quatStd_;
        double velStd_;
        double posStd_;
        double contactStd_;
        double gyroBiasStd_;
        double accBiasStd_;

        Vector3d Gravity_;

        int statesDim_;
        int thetaDim_;
        int statesErrDim_;

        double dt_;
        bool updateEnabled_;

};
