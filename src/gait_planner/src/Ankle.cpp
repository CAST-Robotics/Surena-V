#include "Ankle.h"

Ankle::Ankle(double step_time, double ds_time, double height, double alpha,
             short int num_step, double dt, double theta, double slope)
{
    this->tStep_ = step_time;
    this->tDS_ = ds_time;
    this->alpha_ = alpha;
    this->stepCount_ = num_step;
    this->height_ = height;
    this->dt_ = dt;
    this->theta_ = theta;
    this->yawSign_ = 1;
    this->slope_ = slope;
    this->length_ = int(((stepCount_ + 2) * tStep_) / dt_);
    this->stepSize_ = int(tStep_ / dt_);
    this->dsSize_ = int(tDS_ / dt_);
    this->initDSSize_ = int((1 - alpha_) * dsSize_);
    this->finalDSSize_ = dsSize_ - initDSSize_;
    this->ssSize_ = stepSize_ - dsSize_;
    this->stateIndicator_ = 0;
    this->currentStepNum_ = 0;
}

void Ankle::updateFoot(const vector<Vector3d>& foot_pose, int sign)
{
    /*
        begining and end steps (Not included in DCM foot planner)
        should be given too.
    */
    footStepPos_ = foot_pose;
    if (foot_pose[0](1) > 0)
        leftFirst_ = true; // First Swing foot is left foot
    else
        leftFirst_ = false; // First Swing foot is right foot
    this->yawSign_ = sign;
}

void Ankle::changeFootStep(int foot_step_idx, const Vector3d& new_step)
{
    if (foot_step_idx > currentStepNum_ && foot_step_idx <= footStepPos_.size())
    {
        if (foot_step_idx == footStepPos_.size())
        {
            footStepPos_.push_back(new_step);
            this->stepCount_ += 1;
            this->length_ = int(1 / dt_ * tStep_ * (stepCount_ + 2));
        }
        else
        {
            footStepPos_[foot_step_idx] = new_step;
        }
    }
    else
    {
        throw std::out_of_range("Invalid Foot Step Index");
    }
}

void Ankle::updateOnlineFoot(const vector<Vector3d>& foot_pose, int sign, const vector<Vector3d>& foot_euler)
{
    footStepPos_ = foot_pose;
    
    if (foot_euler.empty())
    {
        int size = foot_pose.size();
        footStepEuler_ = vector<Vector3d>(size, Vector3d::Zero());
        for (int step = 0; step < size; ++step) {
            if (step <= 1) {
                footStepEuler_[step](2) = 0;
            } else if (step == size - 1) {
                footStepEuler_[step](2) = (step - 2) * theta_ * sign;
            } else {
                footStepEuler_[step](2) = (step - 1) * theta_ * sign;
            }
        }
    }
    else
        footStepEuler_ = foot_euler;
    
    if (foot_pose[0](1) > 0)
        leftFirst_ = true; // First Swing foot is left foot
    else
        leftFirst_ = false; // First Swing foot is right foot

    if (stepCount_ % 2 == 0)
        leftLast_ = !leftFirst_;
    else
        leftLast_ = leftFirst_;
    
    this->yawSign_ = sign;

    generateCoeffs();
}

void Ankle::updateCoeffs()
{
    generateCoeffs(currentStepNum_);
}

void Ankle::generateCoeffs(int step_idx)
{
    int coef_len = footStepPos_.size() - 2;
    coefs_.resize(coef_len);
    euler_coefs_.resize(coef_len);

    for (int i = coef_len - 1; i >= step_idx; --i) {
        coefs_[i] = ankle5Poly(footStepPos_[i], footStepPos_[i + 2], height_, tStep_ - tDS_, footStepPos_[i + 2](2));
        euler_coefs_[i] = cubicInterpolate<Vector3d>(footStepEuler_[i], footStepEuler_[i + 2], Vector3d(0, 0, 0), Vector3d(0, 0, 0), tStep_ - tDS_);
    }
}

const vector<Vector3d>& Ankle::getTrajectoryL()
{
    return lFoot_;
}

const vector<Vector3d>& Ankle::getTrajectoryR()
{
    return rFoot_;
}

const vector<Matrix3d>& Ankle::getRotTrajectoryR()
{
    return rFootRot_;
}

const vector<Matrix3d>& Ankle::getRotTrajectoryL()
{
    return lFootRot_;
}

const vector<int>& Ankle::getRobotState()
{
    return stateIndicatorArray_;
}

void Ankle::generateTrajectory()
{

    length_ = int(((stepCount_ + 2) * tStep_) / dt_);
    lFoot_.resize(length_);
    rFoot_.resize(length_);
    rFootRot_.resize(length_);
    lFootRot_.resize(length_);
    stateIndicatorArray_.resize(length_);

    if (leftFirst_)
        updateTrajectory(true);
    else
        updateTrajectory(false);
}

void Ankle::updateTrajectory(bool left_first)
{
    int index = 0;

    for (int i = 0; i < (tStep_) / dt_; i++)
    {
        double time = dt_ * i;
        if (footStepPos_[0](1) > footStepPos_[1](1))
        {
            lFoot_[index] = footStepPos_[0];
            rFoot_[index] = footStepPos_[1];
            lFootRot_[index] = AngleAxisd(0, Vector3d::UnitZ());
            rFootRot_[index] = AngleAxisd(0, Vector3d::UnitZ());
        }
        else
        {
            lFoot_[index] = footStepPos_[1];
            rFoot_[index] = footStepPos_[0];
            lFootRot_[index] = AngleAxisd(0, Vector3d::UnitZ());
            rFootRot_[index] = AngleAxisd(0, Vector3d::UnitZ());
        }
        stateIndicatorArray_[index] = 1;
        index++;
    }
    int step_size = int(tStep_ / dt_);
    int ds_size = int(tDS_ / dt_);
    int init_ds_size = int((1 - alpha_) * ds_size);
    int final_ds_size = ds_size - init_ds_size;
    int ss_size = step_size - ds_size;
    if (left_first)
    {
        for (int step = 1; step < stepCount_ + 1; step++)
        {
            double theta_ini = (step - 2) * theta_ * yawSign_;
            double theta_end = (step)*theta_ * yawSign_;
            double slope_ini = slope_;
            double slope_end = slope_;
            if (step == 1)
                theta_ini = 0;
            if (step == 1 || step == 2)
                slope_ini = 0;
            if (step == stepCount_)
                theta_end = (step - 1) * theta_ * yawSign_;
            if (step % 2 == 0)
            { // Left is support, Right swings
                for (int i = 0; i < init_ds_size; i++)
                {
                    lFoot_[index] = footStepPos_[step];
                    lFootRot_[index] = lFootRot_[index - 1];
                    rFoot_[index] = footStepPos_[step - 1];
                    rFootRot_[index] = rFootRot_[index - 1];
                    stateIndicatorArray_[index] = 1;
                    index++;
                }
                vector<Vector3d> coefs = ankle5Poly(footStepPos_[step - 1], footStepPos_[step + 1], height_, tStep_ - tDS_, footStepPos_[step + 1](2));
                vector<double> theta_coefs = cubicInterpolate<double>(theta_ini, theta_end, 0, 0, tStep_ - tDS_);
                vector<double> slope_coefs = cubicInterpolate<double>(slope_ini, slope_end, 0, 0, tStep_ - tDS_);
                for (int i = 0; i < ss_size; i++)
                {
                    double time = dt_ * i;
                    lFoot_[index] = footStepPos_[step];
                    lFootRot_[index] = lFootRot_[index - 1];
                    rFoot_[index] = coefs[0] + coefs[1] * time + coefs[2] * pow(time, 2) + coefs[3] * pow(time, 3) + coefs[4] * pow(time, 4) + coefs[5] * pow(time, 5);
                    Matrix3d rot_z;
                    rot_z = AngleAxisd(theta_coefs[0] + theta_coefs[1] * time + theta_coefs[2] * pow(time, 2) + theta_coefs[3] * pow(time, 3), Vector3d::UnitZ());
                    Matrix3d rot_y;
                    rot_y = AngleAxisd(slope_coefs[0] + slope_coefs[1] * time + slope_coefs[2] * pow(time, 2) + slope_coefs[3] * pow(time, 3), Vector3d::UnitY());
                    rFootRot_[index] = rot_z * rot_y;
                    stateIndicatorArray_[index] = 3;
                    index++;
                }
                for (int i = 0; i < final_ds_size; i++)
                {
                    lFoot_[index] = footStepPos_[step];
                    lFootRot_[index] = lFootRot_[index - 1];
                    rFoot_[index] = footStepPos_[step + 1];
                    rFootRot_[index] = rFootRot_[index - 1];
                    stateIndicatorArray_[index] = 1;
                    index++;
                }
            }
            else
            { // Right is support, Left swings
                for (int i = 0; i < init_ds_size; i++)
                {
                    lFoot_[index] = footStepPos_[step - 1];
                    lFootRot_[index] = lFootRot_[index - 1];
                    rFoot_[index] = footStepPos_[step];
                    rFootRot_[index] = rFootRot_[index - 1];
                    stateIndicatorArray_[index] = 1;
                    index++;
                }
                vector<Vector3d> coefs = ankle5Poly(footStepPos_[step - 1], footStepPos_[step + 1], height_, tStep_ - tDS_, footStepPos_[step + 1](2));
                vector<double> theta_coefs = cubicInterpolate<double>(theta_ini, theta_end, 0, 0, tStep_ - tDS_);
                vector<double> slope_coefs = cubicInterpolate<double>(slope_ini, slope_end, 0, 0, tStep_ - tDS_);
                for (int i = 0; i < ss_size; i++)
                {
                    double time = dt_ * i;
                    rFoot_[index] = footStepPos_[step];
                    rFootRot_[index] = rFootRot_[index - 1];
                    lFoot_[index] = coefs[0] + coefs[1] * time + coefs[2] * pow(time, 2) + coefs[3] * pow(time, 3) + coefs[4] * pow(time, 4) + coefs[5] * pow(time, 5);
                    Matrix3d rot_z;
                    rot_z = AngleAxisd(theta_coefs[0] + theta_coefs[1] * time + theta_coefs[2] * pow(time, 2) + theta_coefs[3] * pow(time, 3), Vector3d::UnitZ());
                    Matrix3d rot_y;
                    rot_y = AngleAxisd(slope_coefs[0] + slope_coefs[1] * time + slope_coefs[2] * pow(time, 2) + slope_coefs[3] * pow(time, 3), Vector3d::UnitY());
                    lFootRot_[index] = rot_z * rot_y;
                    stateIndicatorArray_[index] = 2;
                    index++;
                }
                for (int i = 0; i < final_ds_size; i++)
                {
                    lFoot_[index] = footStepPos_[step + 1];
                    lFootRot_[index] = lFootRot_[index - 1];
                    rFoot_[index] = footStepPos_[step];
                    rFootRot_[index] = rFootRot_[index - 1];
                    stateIndicatorArray_[index] = 1;
                    index++;
                }
            }
        }
    }

    else
    { // Right Foot Swings first
        for (int step = 1; step < stepCount_ + 1; step++)
        {

            double theta_ini = (step - 2) * theta_ * yawSign_;
            double theta_end = (step)*theta_ * yawSign_;
            double slope_ini = slope_;
            double slope_end = slope_;
            if (step == 1)
                theta_ini = 0;
            if (step == 1 || step == 2)
                slope_ini = 0;
            if (step == stepCount_)
                theta_end = (step - 1) * theta_ * yawSign_;

            if (step % 2 != 0)
            { // Left is support, Right swings
                for (int i = 0; i < init_ds_size; i++)
                {
                    lFoot_[index] = footStepPos_[step];
                    lFootRot_[index] = lFootRot_[index - 1];
                    rFoot_[index] = footStepPos_[step - 1];
                    rFootRot_[index] = rFootRot_[index - 1];
                    stateIndicatorArray_[index] = 1;
                    index++;
                }
                vector<Vector3d> coefs = ankle5Poly(footStepPos_[step - 1], footStepPos_[step + 1], height_, tStep_ - tDS_, footStepPos_[step + 1](2));
                vector<double> theta_coefs = cubicInterpolate<double>(theta_ini, theta_end, 0, 0, tStep_ - tDS_);
                vector<double> slope_coefs = cubicInterpolate<double>(slope_ini, slope_end, 0, 0, tStep_ - tDS_);
                for (int i = 0; i < ss_size; i++)
                {
                    double time = dt_ * i;
                    lFoot_[index] = footStepPos_[step];
                    lFootRot_[index] = lFootRot_[index - 1];
                    rFoot_[index] = coefs[0] + coefs[1] * time + coefs[2] * pow(time, 2) + coefs[3] * pow(time, 3) + coefs[4] * pow(time, 4) + coefs[5] * pow(time, 5);
                    Matrix3d rot_z;
                    rot_z = AngleAxisd(theta_coefs[0] + theta_coefs[1] * time + theta_coefs[2] * pow(time, 2) + theta_coefs[3] * pow(time, 3), Vector3d::UnitZ());
                    Matrix3d rot_y;
                    rot_y = AngleAxisd(slope_coefs[0] + slope_coefs[1] * time + slope_coefs[2] * pow(time, 2) + slope_coefs[3] * pow(time, 3), Vector3d::UnitY());
                    rFootRot_[index] = rot_z * rot_y;
                    stateIndicatorArray_[index] = 3;
                    index++;
                }
                for (int i = 0; i < final_ds_size; i++)
                {
                    lFoot_[index] = footStepPos_[step];
                    lFootRot_[index] = lFootRot_[index - 1];
                    rFoot_[index] = footStepPos_[step + 1];
                    rFootRot_[index] = rFootRot_[index - 1];
                    stateIndicatorArray_[index] = 1;
                    index++;
                }
            }
            else
            { // Right is support, Left swings
                for (int i = 0; i < init_ds_size; i++)
                {
                    lFoot_[index] = footStepPos_[step - 1];
                    lFootRot_[index] = lFootRot_[index - 1];
                    rFoot_[index] = footStepPos_[step];
                    rFootRot_[index] = rFootRot_[index - 1];
                    stateIndicatorArray_[index] = 1;
                    index++;
                }
                vector<Vector3d> coefs = ankle5Poly(footStepPos_[step - 1], footStepPos_[step + 1], height_, tStep_ - tDS_, footStepPos_[step + 1](2));
                vector<double> theta_coefs = cubicInterpolate<double>(theta_ini, theta_end, 0, 0, tStep_ - tDS_);
                vector<double> slope_coefs = cubicInterpolate<double>(slope_ini, slope_end, 0, 0, tStep_ - tDS_);
                for (int i = 0; i < ss_size; i++)
                {
                    double time = dt_ * i;
                    rFoot_[index] = footStepPos_[step];
                    rFootRot_[index] = rFootRot_[index - 1];
                    lFoot_[index] = coefs[0] + coefs[1] * time + coefs[2] * pow(time, 2) + coefs[3] * pow(time, 3) + coefs[4] * pow(time, 4) + coefs[5] * pow(time, 5);
                    Matrix3d rot_z;
                    rot_z = AngleAxisd(theta_coefs[0] + theta_coefs[1] * time + theta_coefs[2] * pow(time, 2) + theta_coefs[3] * pow(time, 3), Vector3d::UnitZ());
                    Matrix3d rot_y;
                    rot_y = AngleAxisd(slope_coefs[0] + slope_coefs[1] * time + slope_coefs[2] * pow(time, 2) + slope_coefs[3] * pow(time, 3), Vector3d::UnitY());
                    lFootRot_[index] = rot_z * rot_y;
                    stateIndicatorArray_[index] = 2;
                    index++;
                }
                for (int i = 0; i < final_ds_size; i++)
                {
                    lFoot_[index] = footStepPos_[step + 1];
                    lFootRot_[index] = lFootRot_[index - 1];
                    rFoot_[index] = footStepPos_[step];
                    rFootRot_[index] = rFootRot_[index - 1];
                    stateIndicatorArray_[index] = 1;
                    index++;
                }
            }
        }
    }

    Vector3d temp_left = lFoot_[index - 1];
    Vector3d temp_right = rFoot_[index - 1];
    for (int i = 0; i < (tStep_) / dt_; i++)
    {
        double time = dt_ * i;
        lFoot_[index] = temp_left;
        lFootRot_[index] = lFootRot_[index - 1];
        rFoot_[index] = temp_right;
        rFootRot_[index] = rFootRot_[index - 1];
        stateIndicatorArray_[index] = 1;
        index++;
    }
}

void Ankle::getOnlineTrajectory(int index, Vector3d& left_foot_pos, Matrix3d& left_foot_rot,
                                Vector3d& right_foot_pos, Matrix3d& right_foot_rot)
{
    currentStepNum_ = index / stepSize_;
    int step_index = index % stepSize_;
    int state_indicator = 0;

    if (currentStepNum_ == 0)
    {
        handleFirstStep(left_foot_pos, left_foot_rot, right_foot_pos, right_foot_rot);
        state_indicator = 1;
    }
    else if (currentStepNum_ == stepCount_ + 1)
    {
        handleLastStep(currentStepNum_, left_foot_pos, left_foot_rot, right_foot_pos, right_foot_rot);
        state_indicator = 1;
    }
    else
    {
        handleOtherSteps(currentStepNum_, step_index, left_foot_pos, left_foot_rot, right_foot_pos, right_foot_rot, state_indicator);
    }
    stateIndicator_ = state_indicator;
}

void Ankle::handleFirstStep(Vector3d& left_foot_pos, Matrix3d& left_foot_rot, Vector3d& right_foot_pos, Matrix3d& right_foot_rot)
{
    if (leftFirst_)
    {
        assignFootPosAndRot(0, 1, left_foot_pos, left_foot_rot, right_foot_pos, right_foot_rot);
    }
    else
    {
        assignFootPosAndRot(1, 0, left_foot_pos, left_foot_rot, right_foot_pos, right_foot_rot);
    }
}

void Ankle::handleLastStep(int step, Vector3d& left_foot_pos, Matrix3d& left_foot_rot, Vector3d& right_foot_pos, Matrix3d& right_foot_rot)
{
    if (leftLast_)
    {
        assignFootPosAndRot(step, step - 1, left_foot_pos, left_foot_rot, right_foot_pos, right_foot_rot);
    }
    else
    {
        assignFootPosAndRot(step - 1, step, left_foot_pos, left_foot_rot, right_foot_pos, right_foot_rot);
    }
}

void Ankle::handleOtherSteps(int step, int step_index, Vector3d& left_foot_pos, Matrix3d& left_foot_rot, Vector3d& right_foot_pos, Matrix3d& right_foot_rot, int& state_indicator)
{
    if (leftFirst_)
    {
        if (step % 2 == 0) // Left is support, Right swings
        {
            handleSupportSwing(step, step_index, left_foot_pos, left_foot_rot, right_foot_pos, right_foot_rot, state_indicator, true);
        }
        else // Right is support, Left swings
        {
            handleSupportSwing(step, step_index, left_foot_pos, left_foot_rot, right_foot_pos, right_foot_rot, state_indicator, false);
        }
    }
    else
    {
        if (step % 2 == 0) // Right is support, Left swings
        {
            handleSupportSwing(step, step_index, left_foot_pos, left_foot_rot, right_foot_pos, right_foot_rot, state_indicator, false);
        }
        else // Left is support, Right swings
        {
            handleSupportSwing(step, step_index, left_foot_pos, left_foot_rot, right_foot_pos, right_foot_rot, state_indicator, true);
        }
    }
}

void Ankle::handleSupportSwing(int step, int step_index, Vector3d& left_foot_pos, Matrix3d& left_foot_rot, Vector3d& right_foot_pos, Matrix3d& right_foot_rot, int& state_indicator, bool leftSupport)
{
    if(step_index < initDSSize_) // Initial DS Phase
    {
        if(leftSupport)
            assignFootPosAndRot(step, step - 1, left_foot_pos, left_foot_rot, right_foot_pos, right_foot_rot);
        else
            assignFootPosAndRot(step - 1, step, left_foot_pos, left_foot_rot, right_foot_pos, right_foot_rot);
        state_indicator = 1;
    }
    else if(step_index >= initDSSize_ && step_index < initDSSize_ + ssSize_) // SS Phase
    {
            handleSSPhase(step, step_index, left_foot_pos, left_foot_rot, right_foot_pos, right_foot_rot, leftSupport);
            state_indicator = leftSupport ? 3 : 2;
    }
    else // Final DS Phase
    {
        if(leftSupport)
            assignFootPosAndRot(step, step + 1, left_foot_pos, left_foot_rot, right_foot_pos, right_foot_rot);
        else
            assignFootPosAndRot(step + 1, step, left_foot_pos, left_foot_rot, right_foot_pos, right_foot_rot);
        state_indicator = 1;
    }
}

void Ankle::handleSSPhase(int step, int step_index, Vector3d& left_foot_pos, Matrix3d& left_foot_rot, Vector3d& right_foot_pos, Matrix3d& right_foot_rot, bool leftSupport)
{
    double time = dt_ * (step_index - (initDSSize_));
    if(leftSupport)
    {
        left_foot_pos = footStepPos_[step];
        left_foot_rot = Euler2Rot(footStepEuler_[step]);
        right_foot_pos = coefs_[step - 1][0] + coefs_[step - 1][1] * time + coefs_[step - 1][2] * pow(time, 2) + coefs_[step - 1][3] * pow(time, 3) + coefs_[step - 1][4] * pow(time, 4) + coefs_[step - 1][5] * pow(time, 5);
        Vector3d euler_angle = euler_coefs_[step - 1][0] + euler_coefs_[step - 1][1] * time + euler_coefs_[step - 1][2] * pow(time, 2) + euler_coefs_[step - 1][3] * pow(time, 3);
        right_foot_rot = Euler2Rot(euler_angle);
    }
    else
    {
        right_foot_pos = footStepPos_[step];
        right_foot_rot = Euler2Rot(footStepEuler_[step]);
        left_foot_pos = coefs_[step - 1][0] + coefs_[step - 1][1] * time + coefs_[step - 1][2] * pow(time, 2) + coefs_[step - 1][3] * pow(time, 3) + coefs_[step - 1][4] * pow(time, 4) + coefs_[step - 1][5] * pow(time, 5);
        Vector3d euler_angle = euler_coefs_[step - 1][0] + euler_coefs_[step - 1][1] * time + euler_coefs_[step - 1][2] * pow(time, 2) + euler_coefs_[step - 1][3] * pow(time, 3);
        left_foot_rot = Euler2Rot(euler_angle);
    }
}

void Ankle::assignFootPosAndRot(int leftIndex, int rightIndex, Vector3d& left_foot_pos, Matrix3d& left_foot_rot, Vector3d& right_foot_pos, Matrix3d& right_foot_rot)
{
    left_foot_pos = footStepPos_[leftIndex];
    left_foot_rot = Euler2Rot(footStepEuler_[leftIndex]);
    right_foot_pos = footStepPos_[rightIndex];
    right_foot_rot = Euler2Rot(footStepEuler_[rightIndex]);
}

Ankle::~Ankle()
{
    
}