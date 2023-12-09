#include "Controller.h"

Controller::Controller(Matrix3d K_p, Matrix3d K_i, Matrix3d K_zmp, Matrix3d K_com)
{
    setK_p_(K_p);
    setK_i_(K_i);
    setK_zmp_(K_zmp);
    setK_com_(K_com);

    xiErrorInt << 0.0, 0.0, 0.0;
    deltaZ_ = 0.0;
    uOrientR_ << 0, 0, 0;
    uOrientL_ << 0, 0, 0;
    uBumpOrientR_ << 0, 0, 0;
    uBumpOrientL_ << 0, 0, 0;
    thetaAnkleR_ << 0, 0, 0;
    thetaAnkleL_ << 0, 0, 0;
    // Index 0 is for right foot
    for (int i; i < 2; i++)
    {
        prevTau_[i] << 0, 0, 0;
        prevTau_d_[i] << 0, 0, 0;
        prevEarlyContactMeanBump_[i] = 0;
        prevAnkleTraj_[i] << 0, 0, 0;
    }
    preDeltaU_ << 0.0, 0.0, 0.0;
    firstContact_ = false;
    desiredContactPos_ << 0.0, 0.0, 0.0;
    deltaUBumpR_ << 0.0, 0.0, 0.0;
    deltaUBumpL_ << 0.0, 0.0, 0.0;
    desired_mean_bump = 0;
    prevForceError_ = 0;
    baseHeight_ = 0.71; // Surena 5 = 0.71
    baseIdle_ = 0.71;   // Surena 5 = 0.71

    // ZMP Admitance Controller Gain
    ACoM_ = Matrix3d::Zero(3, 3);
    CoMAdmitance_ = Vector3d::Zero(3);
}

Vector3d Controller::dcmController(Vector3d xiRef, Vector3d xiDotRef, Vector3d xiReal, double deltaZVRP)
{
    Vector3d xiError = xiRef - xiReal;
    xiErrorInt += xiError * dt_;

    Vector3d rRefZMP;
    rRefZMP = xiRef - xiDotRef / sqrt(9.81 / deltaZVRP) - (K_p_)*xiError + (K_i_)*xiErrorInt;
    return rRefZMP;
}

Vector3d Controller::comController(Vector3d xCOMRef, Vector3d xDotCOMRef, Vector3d xCOMReal, Vector3d rZMPRef, Vector3d rZMPReal)
{
    Vector3d xDotStar;
    xDotStar = xDotCOMRef - K_zmp_ * (rZMPRef - rZMPReal) + K_com_ * (xCOMRef - xCOMReal);
    CoM_ += xDotStar * dt_;
    return CoM_;
}

Vector3d Controller::ZMPAdmitanceComtroller_(Vector3d com_d, Vector3d com, Vector3d zmp_m, Vector3d zmp_qp, Matrix3d kp, Matrix3d kc)
{
    Vector3d u = (-kp * (zmp_qp - zmp_m) + kc * (com_d - com));
    CoMAdmitance_ += dt_ * u;
    CoMAdmitance_(0) = saturate<double>(0.09, -0.07, CoMAdmitance_(0));
    CoMAdmitance_(1) = saturate<double>(0.05, -0.05, CoMAdmitance_(1));
    return CoMAdmitance_;
}

double Controller::footLenController(double delta_fz_d, double delta_fz, double kp, double kd, double kr)
{
    double diff_e;
    double error = delta_fz_d - delta_fz;
    diff_e = 1 / dt_ * (error - prevForceError_);

    prevForceError_ = error;

    double deltaZ_dot = kp * (error) + kd * diff_e - kr * deltaZ_;
    deltaZ_ += deltaZ_dot * dt_;
    return deltaZ_;
}

Vector3d Controller::footOrientController(Vector3d tau_d, Vector3d tau, double k_p, double k_d, double k_r, bool is_right)
{
    Vector3d u_dot, diff, diff_d;

    int index = 0;
    if (!is_right)
        index = 1;

    diff_d = 1 / dt_ * (tau_d - prevTau_d_[index]);
    diff = 1 / dt_ * (tau - prevTau_[index]);

    prevTau_[index] = tau;
    prevTau_d_[index] = tau_d;

    if (is_right)
    {
        u_dot = k_p * (tau_d - tau) + k_d * (diff_d - diff) - k_r * (uOrientR_);
        uOrientR_ += u_dot * dt_;
        uOrientR_(0) = saturate<double>(0.2, -0.2, uOrientR_(0));
        uOrientR_(1) = saturate<double>(0.5, -0.7, uOrientR_(1));
        uOrientR_(2) = 0;
        return uOrientR_;
    }
    else
    {
        u_dot = k_p * (tau_d - tau) + k_d * (diff_d - diff) - k_r * (uOrientL_);
        uOrientL_ += u_dot * dt_;
        uOrientL_(0) = saturate<double>(0.2, -0.2, uOrientL_(0));
        uOrientL_(1) = saturate<double>(0.5, -0.7, uOrientL_(1));
        uOrientL_(2) = 0;
        return uOrientL_;
    }
}

Vector3d Controller::bumpFootOrientController(int *const bump_mesured, Vector3d mean_bump_d, double k_p, double k_d, double k_r, bool is_right)
{
    Vector3d bump_mean(0.0, 0.0, 0.0);
    Vector3d u_dot, diff, diff_d;
    ;

    bump_mean(0) = bump_mesured[0] - bump_mesured[1] - bump_mesured[2] + bump_mesured[3];
    bump_mean(1) = -bump_mesured[0] - bump_mesured[1] + bump_mesured[2] + bump_mesured[3];

    int index = 0;
    if (!is_right)
        index = 1;

    for (int i = 0; i < 4; i++)
    {
        if (bump_mesured[i] > 100)
            bump_mean = prevBumpMean_[index];
    }

    diff_d = 1 / dt_ * (mean_bump_d - prevBump_d_[index]);
    diff = 1 / dt_ * (bump_mean - prevBumpMean_[index]);

    prevBumpMean_[index] = bump_mean;
    prevBump_d_[index] = mean_bump_d;

    if (is_right)
    {
        u_dot = k_p * (mean_bump_d - bump_mean) + k_d * (diff_d - diff) - k_r * (uBumpOrientR_);
        uBumpOrientR_ += u_dot * dt_;
        uBumpOrientR_(2) = 0;
        return uBumpOrientR_;
    }
    else
    {
        u_dot = k_p * (mean_bump_d - bump_mean) + k_d * (diff_d - diff) - k_r * (uBumpOrientL_);
        uBumpOrientL_ += u_dot * dt_;
        uBumpOrientL_(2) = 0;
        return uBumpOrientL_;
    }
}

Vector3d Controller::footDampingController(Vector3d zmp, Vector3d f_measured, Vector3d tau_measured, Matrix3d cop_gain, bool is_right)
{
    Vector3d theta_dot = cop_gain * (zmp.cross(f_measured) - tau_measured);
    if (is_right)
    {
        thetaAnkleR_ += theta_dot * dt_;
        return thetaAnkleR_;
    }
    else
    {
        thetaAnkleL_ += theta_dot * dt_;
        return thetaAnkleL_;
    }
}

// Vector3d Controller::earlyContactController(int* const bump_measured, Vector3d designedTraj){
//     Vector3d delta_u(0.0, 0.0, 0.0);
//     //MatrixXd S(1, 3);
//     double mean_bump = (bump_measured[0] + bump_measured[1] + bump_measured[2] + bump_measured[3])/4.0;
//     //cout << bump_measured[0] << ", " << bump_measured[1] << ", " << bump_measured[2] << ", " << bump_measured[3] << ", " << mean_bump << ", ";
//     if(mean_bump < -50 && mean_bump > -70){
//         if(!firstContact_){
//             desiredContactPos_ = designedTraj - Vector3d(0.0, 0.0, 0.0);
//             firstContact_ = true;
//         }
//         delta_u = 0.9391 * preDeltaU_ + 0.0609*(desiredContactPos_ - designedTraj);
//         delta_u(0) = 0.0;
//         delta_u(1) = 0.0;
//     }
//     prevEarlyContactMeanBump_ = mean_bump;
//     preDeltaU_ = delta_u;
//     return delta_u;
//     //&& ((prevEarlyContactMeanBump_ - mean_bump) > 0)
// }

Vector3d Controller::earlyContactController(int *const bump_measured, double desired_mean_bump, double K_p, double K_r, bool is_right)
{
    Vector3d delta_u_dot(0.0, 0.0, 0.0);
    // double K_p = 0.0;
    // double K_r = 0.0;
    // MatrixXd S(1, 3);
    double mean_bump = (bump_measured[0] + bump_measured[1] + bump_measured[2] + bump_measured[3]) / 4.0;

    if (abs(mean_bump) > 70)
    {
        mean_bump = prevEarlyContactMeanBump_[!is_right];
    }

    // if(mean_bump > -57 && mean_bump < -20 && ankle_traj(2) < prevAnkleTraj_[!is_right](2)){
    //     K_p = 0.008;
    //     K_r = 1;
    // }else{
    //     K_p = 0.008;
    //     K_r = 1;
    // }
    // if(mean_bump - prevEarlyContactMeanBump_ < -1 && iter < 1400)
    //     desired_mean_bump = -50;
    // else if(mean_bump - prevEarlyContactMeanBump_ > 1)
    //     desired_mean_bump = 0;
    // if(iter > 1400){
    //     desired_mean_bump = min(-50.0 + 5.0f / 60.0f * (double(iter) - 1400.0f), 0.0);
    // }
    // cout << bump_measured[0] << ", " << bump_measured[1] << ", " << bump_measured[2] << ", " << bump_measured[3] << ", " << mean_bump << ", ";
    // if(mean_bump < -49 && mean_bump > -70 && (!firstContact_))
    //     firstContact_ = true;
    firstContact_ = true;
    if (firstContact_)
    {
        // if(!firstContact_){
        //     desiredContactPos_ = designedTraj - Vector3d(0.0, 0.0, 0.0);
        //     firstContact_ = true;
        // }
        // Vector3d delta_u_dot = K_p * (desiredContactPos_ - designedTraj) + K_r * deltaU_;
        // deltaU_ += delta_u_dot * dt_;
        if (is_right)
        {
            delta_u_dot(2) = K_p * (desired_mean_bump - mean_bump) - K_r * deltaUBumpR_(2);
            deltaUBumpR_ += delta_u_dot * dt_;
            prevEarlyContactMeanBump_[0] = mean_bump;
            // prevAnkleTraj_[0] = ankle_traj;
            // deltaUBumpR_(2) = saturate<double>(baseHeight_ - baseLowHeight_, baseHeight_-baseIdle_ ,deltaUBumpR_(2));
            return deltaUBumpR_;
        }
        else
        {
            delta_u_dot(2) = K_p * (desired_mean_bump - mean_bump) - K_r * deltaUBumpL_(2);
            deltaUBumpL_ += delta_u_dot * dt_;
            prevEarlyContactMeanBump_[1] = mean_bump;
            // prevAnkleTraj_[1] = ankle_traj;
            // deltaUBumpL_(2) = saturate<double>(baseHeight_ - baseLowHeight_, baseHeight_ - baseIdle_ ,deltaUBumpL_(2));
            return deltaUBumpL_;
        }
    }
}

void Controller::setDt(double dt)
{
    this->dt_ = dt;
}

void Controller::setInitCoM(Vector3d init_com)
{
    this->CoM_ = init_com;
}

void Controller::setK_p_(Matrix3d K_p)
{
    this->K_p_ = K_p;
}
void Controller::setK_i_(Matrix3d K_i)
{
    this->K_i_ = K_i;
}
void Controller::setK_zmp_(Matrix3d K_zmp)
{
    this->K_zmp_ = K_zmp;
}
void Controller::setK_com_(Matrix3d K_com)
{
    this->K_com_ = K_com;
}

void Controller::setBaseHeight(double h)
{
    this->baseHeight_ = h;
}

void Controller::setBaseIdle(double h)
{
    this->baseIdle_ = h;
}

void Controller::setBaseLowHeight(double h)
{
    this->baseLowHeight_ = h;
}