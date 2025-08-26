#include "DCM.h"

DCMPlanner::DCMPlanner(double deltaZ, double stepTime, double doubleSupportTime, double dt, int stepCount, double alpha, double theta)
{
    this->deltaZ_ = deltaZ;
    this->tStep_ = stepTime;
    this->tDS_ = doubleSupportTime;
    this->theta_ = theta;
    if (alpha > 0.0 && alpha < 1.0)
        this->alpha_ = alpha;
    else
        throw "Invalid Value for alpha";
    this->dt_ = dt;
    this->stepCount_ = stepCount;
    this->yawSign_ = 1;
    this->length_ = int(1 / dt_ * tStep_ * stepCount_);
    this->CoMIntegral_ = Vector3d::Zero();
    this->CoMInit_ = Vector3d(0.0, 0.0, deltaZ_); // initial COM when robot start to walk
    this->currentStepNum_ = 0;
    this->currentZMP_ = Vector3d(0, 0, 0);
    this->prevXi_ = Vector3d(0, 0, 0);
}

void DCMPlanner::setFoot(const vector<Vector3d>& rF, int sign)
{
    rF_ = rF;
    this->yawSign_ = sign;
}

void DCMPlanner::setOnlineFoot(const vector<Vector3d>& rF, int sign)
{
    rF_ = rF;
    this->yawSign_ = sign;
    this->updateVRP();
    this->updateXiEoS();
    this->updateOnlineDS(Vector3d(0, 0, 0));
}

void DCMPlanner::changeVRP(int foot_step_idx, const Vector3d& newVRP)
{
    if (foot_step_idx > currentStepNum_ && foot_step_idx <= rVRP_.size())
    {
        if (foot_step_idx == rVRP_.size())
        {
            rVRP_.push_back(newVRP);
            this->stepCount_ += 1;
            this->length_ = int(1 / dt_ * tStep_ * stepCount_);
        }
        else
        {
            rVRP_[foot_step_idx] = newVRP;
        }
    }
    else
    {
        throw std::out_of_range("Invalid Foot Step Index");
    }
}

void DCMPlanner::updateXiPoints()
{
    this->updateXiEoS(currentStepNum_);
    this->updateOnlineDS(Vector3d(0, 0, 0), currentStepNum_);
}

void DCMPlanner::updateXiEoS(int init_step)
{
    xiEOS_.resize(stepCount_);
    for (int i = stepCount_ - 1; i >= init_step; i--)
    {
        if (i == stepCount_ - 1)
            xiEOS_[i] = rVRP_[i];
        else
            xiEOS_[i] = rVRP_[i + 1] + exp(-sqrt(K_G / deltaZ_) * tStep_) * (xiEOS_[i + 1] - rVRP_[i + 1]);
    }
}

void DCMPlanner::updateOnlineDS(Vector3d xi_0, int init_step)
{
    /*
        This function updates Double support start and end positions
    */
    xiDSI_.resize(stepCount_);
    xiDSE_.resize(stepCount_);
    DSXiCoef_.resize(stepCount_);
    Vector3d xi_dot_i, xi_dot_e;

    for (int index = stepCount_ - 1; index >= init_step; index--)
    {
        if (index == 0)
        {
            xi_0 = rVRP_[0] + exp(sqrt(K_G / deltaZ_) * (fmod(0, tStep_) - tStep_)) * (xiEOS_[0] - rVRP_[0]);
            xiDSI_[index] = xi_0;
            xiDSE_[index] = rVRP_[index] + exp(sqrt(K_G / deltaZ_) * tDS_ * (1 - alpha_)) * (xi_0 - rVRP_[index]);
            xi_dot_i = sqrt(K_G / deltaZ_) * (xiDSI_[index] - xi_0);
            xi_dot_e = sqrt(K_G / deltaZ_) * (xiDSE_[index] - rVRP_[0]);
        }
        else
        {
            xiDSI_[index] = rVRP_[index - 1] + exp(-sqrt(K_G / deltaZ_) * tDS_ * alpha_) * (xiEOS_[index - 1] - rVRP_[index - 1]);
            xiDSE_[index] = rVRP_[index] + exp(sqrt(K_G / deltaZ_) * tDS_ * (1 - alpha_)) * (xiEOS_[index - 1] - rVRP_[index]);
            xi_dot_i = sqrt(K_G / deltaZ_) * (xiDSI_[index] - rVRP_[index - 1]);
            xi_dot_e = sqrt(K_G / deltaZ_) * (xiDSE_[index] - rVRP_[index]);
        }
        vector<Vector3d> coefs = this->minJerkInterpolate(xiDSI_[index], xiDSE_[index], xi_dot_i, xi_dot_e, tDS_);
        DSXiCoef_[index] = coefs;
    }
}

const vector<Vector3d>& DCMPlanner::getXiDot()
{
    return xiDot_;
}

const vector<Vector3d>& DCMPlanner::get_CoMDot()
{
    return CoMDot_;
}

const vector<Vector3d>& DCMPlanner::getXiTrajectory()
{
    /*
        This function returns an array of positions with intervals of dt_
        It is best to call it at end of each step with new foot positions
    */
    this->updateVRP();
    this->updateXiEoS();
    this->updateSS();
    this->updateXiDSPositions();
    return xi_;
}

void DCMPlanner::updateVRP()
{
    // Updates Virtual Repelant Points !! should be called after setFoot() !!
    rVRP_.resize(stepCount_);
    Vector3d deltaZ(0.0, 0.0, deltaZ_);
    for (int i = 0; i < this->stepCount_; i++)
    {
        rVRP_[i] = rF_[i] + deltaZ;
    }
}

void DCMPlanner::updateSS()
{
    // Generates DCM trajectory without Double Support Phase
    xi_.resize(length_);
    xiDot_.resize(length_);
    int stepNum;
    double time;
    for (int i = 0; i < length_; i++)
    {
        time = dt_ * i;
        stepNum = floor(time / tStep_);
        xi_[i] = rVRP_[stepNum] + exp(sqrt(K_G / deltaZ_) * (fmod(time, tStep_) - tStep_)) * (xiEOS_[stepNum] - rVRP_[stepNum]);
        xiDot_[i] = sqrt(K_G / deltaZ_) * (xi_[i] - rVRP_[stepNum]);
    }
}

Vector3d DCMPlanner::computeCoM(int iter)
{
    // Generates DCM trajectory without Double Support Phase
    double time;
    time = dt_ * iter;
    currentStepNum_ = floor(time / tStep_);
    currentXi_ = rVRP_[currentStepNum_] + exp(sqrt(K_G / deltaZ_) * (fmod(time, tStep_) - tStep_)) * (xiEOS_[currentStepNum_] - rVRP_[currentStepNum_]);
    currentXiDot_ = sqrt(K_G / deltaZ_) * (currentXi_ - rVRP_[currentStepNum_]);
    
    // Handle double support
    if (currentStepNum_ == 0)
    {
        if(iter < (1 / dt_) * tDS_ * (1 - alpha_))
        {
            currentXi_ = DSXiCoef_[currentStepNum_][0] + DSXiCoef_[currentStepNum_][1] * time + DSXiCoef_[currentStepNum_][2] * pow(time, 2) + DSXiCoef_[currentStepNum_][3] * pow(time, 3);
            currentXiDot_ = DSXiCoef_[currentStepNum_][1] + 2 * DSXiCoef_[currentStepNum_][2] * time + 3 * DSXiCoef_[currentStepNum_][3] * pow(time, 2);
        }
    }
    else
    {
        if(iter < ((tStep_ * currentStepNum_) / dt_) + (tDS_ / dt_) * (1 - alpha_))
        {
            time = fmod(time, tStep_ * currentStepNum_ - tDS_ * alpha_);
            currentXi_ = DSXiCoef_[currentStepNum_][0] + DSXiCoef_[currentStepNum_][1] * time + DSXiCoef_[currentStepNum_][2] * pow(time, 2) + DSXiCoef_[currentStepNum_][3] * pow(time, 3);
            currentXiDot_ = DSXiCoef_[currentStepNum_][1] + 2 * DSXiCoef_[currentStepNum_][2] * time + 3 * DSXiCoef_[currentStepNum_][3] * pow(time, 2);
        }       
    }

    if(currentStepNum_ < (stepCount_ - 1) && iter > ((tStep_ * (currentStepNum_ + 1)) / dt_) - (tDS_ * alpha_ / dt_))
    {
        time = fmod(time, tStep_ * (currentStepNum_+1) - tDS_ * alpha_);
        currentXi_ = DSXiCoef_[currentStepNum_ + 1][0] + DSXiCoef_[currentStepNum_ + 1][1] * time + DSXiCoef_[currentStepNum_ + 1][2] * pow(time, 2) + DSXiCoef_[currentStepNum_ + 1][3] * pow(time, 3);
        currentXiDot_ = DSXiCoef_[currentStepNum_ + 1][1] + 2 * DSXiCoef_[currentStepNum_ + 1][2] * time + 3 * DSXiCoef_[currentStepNum_ + 1][3] * pow(time, 2);
    }

    // Compute CoM trajectory
    if (iter > 0)
        this->CoMIntegral_ += sqrt(K_G / deltaZ_) * ((prevXi_ * exp((iter - 1) * dt_ * sqrt(K_G / deltaZ_))) + (currentXi_ * exp((iter) * dt_ * sqrt(K_G / deltaZ_)))) * 0.5 * dt_;
    
    Vector3d com = (this->CoMIntegral_ + CoMInit_) * exp(-iter * dt_ * sqrt(K_G / deltaZ_));
    com(2) = deltaZ_;
    currentZMP_ = currentXi_ - currentXiDot_ * sqrt(deltaZ_ / K_G);
    prevXi_ = currentXi_;
    prevXiDot_ = currentXiDot_;
    return com;
}

void DCMPlanner::updateXiDSPositions()
{
    /*
        This Function rounds single support trajectory bends
        for double support phase.
        ! Double support starts and ends should be updated !
    */
    this->updateDS();
    
    Vector3d xi_dot_i, xi_dot_e;
    for (int step = 0; step < stepCount_; step++)
    {
        if (step == 0)
        {
            xi_dot_i = sqrt(K_G / deltaZ_) * (xiDSI_[step] - xi_[0]);
            xi_dot_e = sqrt(K_G / deltaZ_) * (xiDSE_[step] - rVRP_[0]);
            vector<Vector3d> coefs = this->minJerkInterpolate(xiDSI_[step], xiDSE_[step], xi_dot_i, xi_dot_e, tDS_);
            for (int i = 0; i < (1 / dt_) * tDS_ * (1 - alpha_); ++i)
            {
                double time = dt_ * i;
                xi_[i] = coefs[0] + coefs[1] * time + coefs[2] * pow(time, 2) + coefs[3] * pow(time, 3);
                xiDot_[i] = coefs[1] + 2 * coefs[2] * time + 3 * coefs[3] * pow(time, 2);
            }
        }
        else
        {
            xi_dot_i = sqrt(K_G / deltaZ_) * (xiDSI_[step] - rVRP_[step - 1]);
            xi_dot_e = sqrt(K_G / deltaZ_) * (xiDSE_[step] - rVRP_[step]);
            vector<Vector3d> coefs = this->minJerkInterpolate(xiDSI_[step], xiDSE_[step], xi_dot_i, xi_dot_e, tDS_);
            for (int i = (tStep_ * step) / dt_ - (tDS_ * alpha_ / dt_) + 1; i < ((tStep_ * step) / dt_) + (tDS_ / dt_) * (1 - alpha_); ++i)
            {
                double time = fmod(i * dt_, tStep_ * step - tDS_ * alpha_);
                xi_[i] = coefs[0] + coefs[1] * time + coefs[2] * pow(time, 2) + coefs[3] * pow(time, 3);
                xiDot_[i] = coefs[1] + 2 * coefs[2] * time + 3 * coefs[3] * pow(time, 2);
            }
        }
    }
}

void DCMPlanner::updateDS()
{
    /*
        This function updates Double support start and end positions
    */
    xiDSI_.resize(stepCount_);
    xiDSE_.resize(stepCount_);

    for (int index = 0; index < stepCount_; index++)
    {
        if (index == 0)
        {
            xiDSI_[index] = xi_[0];
            xiDSE_[index] = rVRP_[index] + exp(sqrt(K_G / deltaZ_) * tDS_ * (1 - alpha_)) * (xi_[0] - rVRP_[index]);
        }
        else
        {
            xiDSI_[index] = rVRP_[index - 1] + exp(-sqrt(K_G / deltaZ_) * tDS_ * alpha_) * (xiEOS_[index - 1] - rVRP_[index - 1]);
            xiDSE_[index] = rVRP_[index] + exp(sqrt(K_G / deltaZ_) * tDS_ * (1 - alpha_)) * (xiEOS_[index - 1] - rVRP_[index]);
        }
    }
}

const vector<Vector3d>& DCMPlanner::getCoM()
{
    COM_.resize(length_);
    CoMDot_.resize(length_);
    // COM trajectory based on DCM
    for (int i = 0; i < length_; i++)
    {
        if (i > 0)
            this->CoMIntegral_ += sqrt(K_G / deltaZ_) * ((xi_[i - 1] * exp((i - 1) * dt_ * sqrt(K_G / deltaZ_))) + (xi_[i] * exp((i) * dt_ * sqrt(K_G / deltaZ_)))) * 0.5 * dt_;
        COM_[i] = (this->CoMIntegral_ + CoMInit_) * exp(-i * dt_ * sqrt(K_G / deltaZ_));
        COM_[i](2) = xi_[i](2);
        CoMDot_[i] = -sqrt(K_G / deltaZ_) * (COM_[i] - xi_[i]);
    }
    return COM_;
}

vector<Vector3d> DCMPlanner::minJerkInterpolate(Vector3d theta_ini, Vector3d theta_f,
                                                Vector3d theta_dot_ini, Vector3d theta_dot_f, double tf)
{
    /*
        Returns Cubic Polynomial with the Given Boundary Conditions
        https://www.tu-chemnitz.de/informatik//KI/edu/robotik/ws2016/lecture-tg%201.pdf
    */
    vector<Vector3d> coefs(4); // a0, a1, a2, a3
    coefs[0] = theta_ini;
    coefs[1] = theta_dot_ini;
    coefs[2] = 3 / pow(tf, 2) * (theta_f - theta_ini) - 1 / tf * (2 * theta_dot_ini + theta_dot_f);
    coefs[3] = -2 / pow(tf, 3) * (theta_f - theta_ini) + 1 / pow(tf, 2) * (theta_dot_ini + theta_dot_f);
    return coefs;
}

const vector<Vector3d>& DCMPlanner::getZMP()
{
    /*
        This function returns desired ZMP. getXiTrajectory should be called first.
    */
    ZMP_.resize(length_);
    for (int i = 0; i < length_; i++)
        ZMP_[i] = xi_[i] - xiDot_[i] * sqrt(deltaZ_ / K_G);
    return ZMP_;
}

const vector<Matrix3d>& DCMPlanner::yawRotGen()
{
    yawRotation_.resize(length_);
    double ini_theta;
    double end_theta;

    for (int j = 0; j < tStep_ / dt_; j++)
        yawRotation_[j] = AngleAxisd(0, Vector3d::UnitZ());

    for (int i = 1; i < stepCount_ - 1; i++)
    {
        ini_theta = this->yawSign_ * ((i - 1) * theta_ + (i - 2) * theta_) / 2; //
        end_theta = this->yawSign_ * ((i - 1) * theta_ + i * theta_) / 2;       //

        if (i == 1)
            ini_theta = 0;
        if (i == stepCount_ - 2)
            end_theta = this->yawSign_ * (stepCount_ - 3) * theta_; //
        vector<double> coef = MinJerk::cubicInterpolate<double>(ini_theta, end_theta, 0, 0, tStep_);
        for (int j = 0; j < tStep_ / dt_; j++)
        {
            double theta_traj = coef[0] + coef[1] * j * dt_ + coef[2] * pow(j * dt_, 2) + coef[3] * pow(j * dt_, 3);
            yawRotation_[int((i)*tStep_ / dt_ + j)] = AngleAxisd(theta_traj, Vector3d::UnitZ());
        }
    }

    for (int j = 0; j < tStep_ / dt_; j++)
    {

        yawRotation_[int((stepCount_ - 1) * tStep_ / dt_ + j)] = yawRotation_[int((stepCount_ - 1) * tStep_ / dt_ + j) - 1];
    }
    return yawRotation_;
}

void DCMPlanner::calculateRotCoeffs() {
    rotCoeffs_.resize(stepCount_);
    double ini_theta, end_theta;

    for (int i = 0; i < stepCount_; i++) {
        ini_theta = this->yawSign_ * ((i - 1) * theta_ + (i - 2) * theta_) / 2;
        end_theta = this->yawSign_ * ((i - 1) * theta_ + i * theta_) / 2;

        if (i == 0) {
            ini_theta = 0;
            end_theta = 0;
        }
        if (i == 1) {
            ini_theta = 0;
        }
        if (i == stepCount_ - 2) {
            end_theta = this->yawSign_ * (i - 1) * theta_;
        }
        if (i == stepCount_ - 1) {
            ini_theta = this->yawSign_ * (i - 2) * theta_;
            end_theta = this->yawSign_ * (i - 2) * theta_;
        }
        
        rotCoeffs_[i] = MinJerk::cubicInterpolate<double>(ini_theta, end_theta, 0, 0, tStep_);
    }
}

Matrix3d DCMPlanner::getOnlineRot(int iter) {
    int i = iter / (tStep_ / dt_);
    Matrix3d rot;
    int j = iter % int(tStep_ / dt_);
    double theta_traj = rotCoeffs_[i][0] + rotCoeffs_[i][1] * j * dt_ + rotCoeffs_[i][2] * pow(j * dt_, 2) + rotCoeffs_[i][3] * pow(j * dt_, 3);
    rot = AngleAxisd(theta_traj, Vector3d::UnitZ());
    return rot;
}

DCMPlanner::~DCMPlanner()
{
    
}