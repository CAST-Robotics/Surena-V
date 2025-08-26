#include "LieEKF.h"

LieEKF::LieEKF(){
    Gravity_ = Vector3d(0, 0, -9.81);
    
    BGyro_ = Vector3d::Zero();
    BAcc_ = Vector3d::Zero();

    BLFootMeasured_ = Vector3d::Zero();
    BRFootMeasured_ = Vector3d::Zero();
    BLFootRotMeasured_ = Matrix3d::Identity();
    BRFootRotMeasured_ = Matrix3d::Identity();

    statesDim_ = 3 + 1 + 1 + 1 + 1; // rot, vel, pos, left-contact, right-contact
    thetaDim_ = BGyroBias_.size() + BAccBias_.size();

    statesErrDim_ = 3 + GBaseVel_.size() + GBasePos_.size() + GLeftFootPos_.size() + 
                    GRightFootPos_.size() + BGyroBias_.size() + BAccBias_.size();

    x_ = MatrixXd::Zero(statesDim_, statesDim_);
    xPrev_ = MatrixXd::Zero(statesDim_, statesDim_);
    xPred_ = MatrixXd::Zero(statesDim_, statesDim_);
    theta_ = VectorXd::Zero(thetaDim_);
    thetaPrev_ = VectorXd::Zero(thetaDim_);
    thetaPred_ = VectorXd::Zero(thetaDim_);

    // default initialize states
    Matrix3d R = Matrix3d::Identity();
    Vector3d base_vel = Vector3d::Zero();
    Vector3d base_pos = Vector3d(0, 0, 0.71);
    Vector3d lf_pos = Vector3d(0, 0.1, 0);
    Vector3d rf_pos = Vector3d(0, -0.1, 0);
    Vector3d gyro_bias = Vector3d::Zero();
    Vector3d acc_bias = Vector3d::Zero();

    this->initializeStates(R, base_vel, base_pos, lf_pos, rf_pos, gyro_bias, acc_bias);
    this->concatStates(R, base_vel, base_pos, lf_pos, rf_pos, gyro_bias, acc_bias, xPrev_, thetaPrev_);

    this->setDt(0.002);

    P_ = MatrixXd::Identity(statesErrDim_, statesErrDim_);
    Ppred_ = MatrixXd::Identity(statesErrDim_, statesErrDim_);
    Phi_ = MatrixXd::Identity(statesErrDim_, statesErrDim_);
    // default initial covaricance matrix
    this->initializeCovariance(0.1, 0.15, 0.1, 0.1, 0.2, 0.2);

    // default process and measurement noise
    this->setNoiseStd(0.05, 0.08, 0.001, 0.001, 0.1, 0.05);

    updateEnabled_ = true;
}

LieEKF::~LieEKF(){}

Vector3d LieEKF::getGBasePose(){
    return GBasePos_;
}

Vector3d LieEKF::getGBaseVel(){
    return GBaseVel_;
}

Quaterniond LieEKF::getGBaseQuat(){
    Quaterniond q(GBaseRot_);
    return q;
}

void LieEKF::initializeStates(Matrix3d R, Vector3d base_vel, Vector3d base_pos, Vector3d lf_pos, 
                              Vector3d rf_pos, Vector3d gyro_bias, Vector3d acc_bias){
    GBaseRot_ = R;
    GBaseVel_ = base_vel;
    GBasePos_ = base_pos;
    GLeftFootPos_ = lf_pos;
    GRightFootPos_ = rf_pos;
    BGyroBias_ = gyro_bias;
    BAccBias_ = acc_bias;
}

void LieEKF::concatStates(Matrix3d R, Vector3d base_vel, Vector3d base_pos, Vector3d lf_pos, 
                          Vector3d rf_pos, Vector3d gyro_bias, Vector3d acc_bias, MatrixXd &output, VectorXd &output_theta){
    int size = output.rows();
    output = MatrixXd::Identity(size, size);
    output.block(0, 0, 3, 3) = R;
    output.block(0, 3, 3, 1) = base_vel;
    output.block(0, 4, 3, 1) = base_pos;
    output.block(0, 5, 3, 1) = lf_pos;
    output.block(0, 6, 3, 1) = rf_pos;

    output_theta.segment(0, 3) = acc_bias;
    output_theta.segment(3, 3) = gyro_bias;
}

void LieEKF::initializeCovariance(double quat_std, double vel_std, double pos_std,
                                  double contact_std, double gyro_std, double acc_std){
    quatStd_ = quat_std;
    velStd_ = vel_std;
    posStd_ = pos_std;
    contactStd_ = contact_std;
    gyroBiasStd_ = gyro_std;
    accBiasStd_ = acc_std;

    P_ = MatrixXd::Zero(statesErrDim_, statesErrDim_);

    P_.block(0, 0, 3, 3) = pow(quatStd_, 2) * MatrixXd::Identity(3, 3);
    P_.block(3, 3, 3, 3) = pow(velStd_, 2) * MatrixXd::Identity(3, 3);
    P_.block(6, 6, 3, 3) = pow(posStd_, 2) * MatrixXd::Identity(3, 3);
    P_.block(9, 9, 3, 3) = pow(contactStd_, 2) * MatrixXd::Identity(3, 3);
    P_.block(12, 12, 3, 3) = pow(contactStd_, 2) * MatrixXd::Identity(3, 3);
    P_.block(15, 15, 3, 3) = pow(gyroBiasStd_, 2) * MatrixXd::Identity(3, 3);
    P_.block(18, 18, 3, 3) = pow(accBiasStd_, 2) * MatrixXd::Identity(3, 3);
}

void LieEKF::setNoiseStd(double gyro_noise, double acc_noise, double gyro_bias_noise, 
                         double acc_bias_noise, double contact_noise, double measurement_noise){
    gyroNoiseStd_ = gyro_noise;
    accNoiseStd_ = acc_noise;
    gyroBiasNoiseStd_ = gyro_bias_noise;
    accBiasNoiseStd_ = acc_bias_noise;
    contactNoiseStd_ = contact_noise;
    measurementNoiseStd_ = measurement_noise;
}

void LieEKF::setSensorData(Vector3d gyro, Vector3d acc){
    BGyro_ = gyro;
    BAcc_ = acc;
}

void LieEKF::setMeasuredData(Vector3d l_kynematic, Vector3d r_kynematic, Matrix3d l_kynematic_rot, Matrix3d r_kynematic_rot){
    BLFootMeasured_ = l_kynematic;
    BRFootMeasured_ = r_kynematic;
    
    BLFootRotMeasured_ = l_kynematic_rot;
    BRFootRotMeasured_ = r_kynematic_rot;
}

void LieEKF::seprateStates(const MatrixXd &x, const VectorXd &theta){

    GBaseRot_ = x.block(0, 0, 3, 3);
    GBaseVel_ = x.block(0, 3, 3, 1);
    GBasePos_ = x.block(0, 4, 3, 1);
    GLeftFootPos_ = x.block(0, 5, 3, 1);
    GRightFootPos_ = x.block(0, 6, 3, 1);

    BAccBias_ = theta.segment(0, 3);
    BGyroBias_ = theta.segment(3, 3);
}

void LieEKF::predict(){
    
    this->seprateStates(xPrev_, thetaPrev_);
    // remove bias from IMU data
    BAcc_ = BAcc_ - BAccBias_;
    BGyro_ = BGyro_ - BGyroBias_;

    Matrix3d acc_skew = skewSym(BAcc_);
    
    Matrix3d gamma_0 = gamma(dt_ * BGyro_, 0);
    Matrix3d gamma_1 = gamma(dt_ * BGyro_, 1);
    Matrix3d gamma_2 = gamma(dt_ * BGyro_, 2);

    // predict rotation matrix
    Matrix3d predicted_R = GBaseRot_ * gamma_0;

    // predict position and velocity
    Vector3d predicted_v = GBaseVel_ + dt_ * (GBaseRot_ * gamma_1 * BAcc_ + Gravity_);
    Vector3d predicted_p = GBasePos_ + dt_ * GBaseVel_ + pow(dt_, 2) * (GBaseRot_ * gamma_2 * BAcc_ + 0.5 * Gravity_);

    // predict foot positions
    // what happens if the measurement is not available?
    Vector3d predicted_lf_p;
    Vector3d predicted_rf_p;
    predicted_lf_p = predicted_p + predicted_R * BLFootMeasured_;
    predicted_rf_p = predicted_p + predicted_R * BRFootMeasured_;
    
    predicted_lf_p = contact_[0] * GLeftFootPos_ + (1 - contact_[0]) * predicted_lf_p;
    predicted_rf_p = contact_[1] * GRightFootPos_ + (1 - contact_[1]) * predicted_rf_p;
    
    // predict biases
    Vector3d predicted_gyro_bias = BGyroBias_;
    Vector3d predicted_acc_bias = BAccBias_;

    Matrix3d g_skew = skewSym(Gravity_);
    
    Matrix3d phi_skew = skewSym(dt_ * BGyro_);
    double phi_norm = (dt_ * BGyro_).norm();
    Matrix3d psi_1;
    Matrix3d psi_2;

    if(BGyro_.norm() > 1e-6){

        psi_1 = acc_skew * gamma(-dt_ * BGyro_, 2)
        + ((sin(phi_norm) - phi_norm * cos(phi_norm))/(pow(phi_norm, 3))) * (phi_skew * acc_skew)
        - ((cos(2 * phi_norm) - 4 * cos(phi_norm) + 3) / (4 * pow(phi_norm, 4)))*(phi_skew * acc_skew * phi_skew)
        + ((4 * sin(phi_norm) + sin(2 * phi_norm) - 4 * phi_norm * cos(phi_norm) - 2 * phi_norm) / (4 * pow(phi_norm, 5))) * (phi_skew * acc_skew * phi_skew * phi_skew)
        + ((pow(phi_norm, 2) - 2 * phi_norm * sin(phi_norm) - 2 * cos(phi_norm) + 2) / (2 * pow(phi_norm, 4))) * (phi_skew * phi_skew * acc_skew)
        - ((6 * phi_norm - 8 * sin(phi_norm) + sin(2 * phi_norm)) / (4 * pow(phi_norm, 5))) * (phi_skew * phi_skew * acc_skew * phi_skew)
        + ((2 * pow(phi_norm, 2) - 4 * phi_norm * sin(phi_norm) - cos(2 * phi_norm) + 1) / (4 * pow(phi_norm, 6))) * (phi_skew * phi_skew * acc_skew * phi_skew * phi_skew);
        
        psi_2 = acc_skew * gamma(-dt_ * BGyro_, 3)
        - ((phi_norm * sin(phi_norm) + 2 * cos(phi_norm) - 2) / (pow(phi_norm, 4))) * (phi_skew * acc_skew)
        - ((6 * phi_norm - 8 * sin(phi_norm) + sin(2 * phi_norm)) / (8 * pow(phi_norm, 5))) * (phi_skew * acc_skew * phi_skew)
        - ((2 * pow(phi_norm, 2) + 8 * phi_norm * sin(phi_norm) + 16 * cos(phi_norm) + cos(2 * phi_norm) - 17) / (8 * pow(phi_norm, 6))) * (phi_skew * acc_skew * phi_skew * phi_skew)
        + ((pow(phi_norm, 3) + 6 * phi_norm - 12 * sin(phi_norm) + 6 * phi_norm * cos(phi_norm)) / (6 * pow(phi_norm, 5))) * (phi_skew * phi_skew * acc_skew)
        - ((6 * pow(phi_norm, 2) + 16 * cos(phi_norm) - cos(2 * phi_norm) - 15) / (8 * pow(phi_norm, 6))) * (phi_skew * phi_skew * acc_skew * phi_skew)
        + ((4 * pow(phi_norm, 3) + 6 * phi_norm - 24 * sin(phi_norm) - 3 * sin(2 * phi_norm) + 24 * phi_norm * cos(phi_norm)) / (24 * pow(phi_norm, 7))) * (phi_skew * phi_skew * acc_skew * phi_skew * phi_skew);

    }else{
        psi_1 = (1.0/2.0) * acc_skew;
        psi_2 = (1.0/6.0) * acc_skew;
    }
    
    Phi_ = MatrixXd::Identity(statesErrDim_, statesErrDim_);

    Phi_.block(3, 0, 3, 3) = dt_ * g_skew;
    Phi_.block(6, 0, 3, 3) = 0.5 * pow(dt_, 2) * g_skew;
    Phi_.block(6, 3, 3, 3) = dt_ * Matrix3d::Identity();
    Phi_.block(0, 15, 3, 3) = -dt_ * GBaseRot_ * gamma_1;
    Phi_.block(3, 15, 3, 3) = -dt_ * skewSym(predicted_v) * GBaseRot_ * gamma_1 + pow(dt_, 2) * GBaseRot_ * psi_1;
    Phi_.block(6, 15, 3, 3) = -dt_ * skewSym(predicted_p) * GBaseRot_ * gamma_1 + pow(dt_, 3) * GBaseRot_ * psi_2;
    Phi_.block(9, 15, 3, 3) = -dt_ * skewSym(predicted_lf_p) * GBaseRot_ * gamma_1;
    Phi_.block(12, 15, 3, 3) = -dt_ * skewSym(predicted_rf_p) * GBaseRot_ * gamma_1;
    Phi_.block(3, 18, 3, 3) = -dt_ * GBaseRot_ * gamma_1;
    Phi_.block(6, 18, 3, 3) = -pow(dt_, 2) * GBaseRot_ * gamma_2;

    this->updateQd();

    this->concatStates(predicted_R, predicted_v, predicted_p, predicted_lf_p, predicted_rf_p, predicted_gyro_bias, predicted_acc_bias, xPred_, thetaPred_);
    Ppred_ = Phi_ * P_ * Phi_.transpose() + Qd_;
    P_ = Ppred_;
    x_ = xPred_;
    theta_ = thetaPred_;
}

void LieEKF::updateQd() {
    Matrix3d hR_L = BLFootRotMeasured_;
    Matrix3d hR_R = BRFootRotMeasured_;
    Qd_ = MatrixXd::Zero(statesErrDim_, statesErrDim_);
    Qd_.block(0, 0, 3, 3) = pow(gyroNoiseStd_, 2) * Matrix3d::Identity();
    Qd_.block(3, 3, 3, 3) = pow(accNoiseStd_, 2) * Matrix3d::Identity();
    Qd_.block(6, 6, 3, 3) = Matrix3d::Zero();
    Qd_.block(9, 9, 3, 3) = hR_L * (pow(contactNoiseStd_, 2) * Matrix3d::Identity() + (1e4 * (1 - contact_[0])) * Matrix3d::Identity()) * hR_L.transpose();
    Qd_.block(12, 12, 3, 3) = hR_R * (pow(contactNoiseStd_, 2) * Matrix3d::Identity() + (1e4 * (1 - contact_[1])) * Matrix3d::Identity()) * hR_R.transpose();
    Qd_.block(15, 15, 3, 3) = pow(gyroBiasNoiseStd_, 2) * Matrix3d::Identity();
    Qd_.block(18, 18, 3, 3) = pow(accBiasNoiseStd_, 2) * Matrix3d::Identity();

    MatrixXd G = MatrixXd::Zero(statesErrDim_, statesErrDim_);
    G.block(0, 0, 15, 15) = adjoint(xPrev_); // adjoint of xPrev_
    G.block(15, 15, 6, 6) = MatrixXd::Identity(6, 6);

    Qd_ = dt_ * Phi_ * G * Qd_ * G.transpose() * Phi_.transpose();
}

void LieEKF::updateState(VectorXd delta, const MatrixXd &prev_x, const VectorXd &prev_theta, MatrixXd &x, VectorXd &theta) {
    x = SEK3Exp(delta.segment(0, 15)) * prev_x;
    theta = prev_theta + delta.segment(15, 6);
}

void LieEKF::update() {

    if(contact_[0] == 1 && contact_[1] == 1){

        Hd_ = MatrixXd::Zero(6, statesErrDim_);
        Y_ = MatrixXd::Zero(14, 1);
        Rd_ = MatrixXd::Zero(6, 6);
        b_ = MatrixXd::Zero(14, 1);
        PI_ = MatrixXd::Zero(6, 14);
        BigX_ = MatrixXd::Zero(14, 14);

        Y_.segment(0, GLeftFootPos_.size()) = BLFootMeasured_;
        Y_.segment(GLeftFootPos_.size(), 4) = Vector4d(0, 1, -1, 0);

        b_.segment(4, 3) = Vector3d(1, -1, 0);

        Hd_.block(0, 0, 3, 3) = Matrix3d::Zero();
        Hd_.block(0, 3, 3, 3) = Matrix3d::Zero();
        Hd_.block(0, 6, 3, 3) = -Matrix3d::Identity();
        Hd_.block(0, 9, 3, 3) = Matrix3d::Identity();
        Hd_.block(0, 12, 3, 3) = Matrix3d::Zero();
        Hd_.block(0, 15, 3, 3) = Matrix3d::Zero();
        Hd_.block(0, 18, 3, 3) = Matrix3d::Zero();
        
        Rd_.block(0, 0, 3, 3) = pow(measurementNoiseStd_, 2) * Matrix3d::Identity();

        PI_.block(0, 0, 3, 3) = Matrix3d::Identity();

        BigX_.block(0, 0, 7, 7) = xPred_;

        Y_.segment(7, GRightFootPos_.size()) = BRFootMeasured_;
        Y_.segment(10, 4) = Vector4d(0, 1, 0, -1);

        b_.segment(11, 3) = Vector3d(1, 0, -1);

        Hd_.block(3, 0, 3, 3) = Matrix3d::Zero();
        Hd_.block(3, 3, 3, 3) = Matrix3d::Zero();
        Hd_.block(3, 6, 3, 3) = -Matrix3d::Identity();
        Hd_.block(3, 9, 3, 3) = Matrix3d::Zero();
        Hd_.block(3, 12, 3, 3) = Matrix3d::Identity();
        Hd_.block(3, 15, 3, 3) = Matrix3d::Zero();
        Hd_.block(3, 18, 3, 3) = Matrix3d::Zero();
        
        Rd_.block(3, 3, 3, 3) = pow(measurementNoiseStd_, 2) * Matrix3d::Identity();

        PI_.block(3, 7, 3, 3) = Matrix3d::Identity();

        BigX_.block(7, 7, 7, 7) = xPred_;

    }else if(contact_[0] == 1 && contact_[1] == 0){
        
        Hd_ = MatrixXd::Zero(3, statesErrDim_);
        Y_ = MatrixXd::Zero(7, 1);
        Rd_ = MatrixXd::Zero(3, 3);
        b_ = MatrixXd::Zero(7, 1);
        PI_ = MatrixXd::Zero(3, 7);
        BigX_ = MatrixXd::Zero(7, 7);

        Y_.segment(0, GLeftFootPos_.size()) = BLFootMeasured_;
        Y_.segment(GLeftFootPos_.size(), 4) = Vector4d(0, 1, -1, 0);

        b_.segment(4, 3) = Vector3d(1, -1, 0);

        Hd_.block(0, 0, 3, 3) = Matrix3d::Zero();
        Hd_.block(0, 3, 3, 3) = Matrix3d::Zero();
        Hd_.block(0, 6, 3, 3) = -Matrix3d::Identity();
        Hd_.block(0, 9, 3, 3) = Matrix3d::Identity();
        Hd_.block(0, 12, 3, 3) = Matrix3d::Zero();
        Hd_.block(0, 15, 3, 3) = Matrix3d::Zero();
        Hd_.block(0, 18, 3, 3) = Matrix3d::Zero();
        
        Rd_.block(0, 0, 3, 3) = pow(measurementNoiseStd_, 2) * Matrix3d::Identity();

        PI_.block(0, 0, 3, 3) = Matrix3d::Identity();

        BigX_.block(0, 0, 7, 7) = xPred_;

    }else if(contact_[0] == 0 && contact_[1] == 1){
        
        Hd_ = MatrixXd::Zero(3, statesErrDim_);
        Y_ = MatrixXd::Zero(7, 1);
        Rd_ = MatrixXd::Zero(3, 3);
        b_ = MatrixXd::Zero(7, 1);
        PI_ = MatrixXd::Zero(3, 7);
        BigX_ = MatrixXd::Zero(7, 7);

        Y_.segment(0, GRightFootPos_.size()) = BRFootMeasured_;
        Y_.segment(GRightFootPos_.size(), 4) = Vector4d(0, 1, 0, -1);

        b_.segment(4, 3) = Vector3d(1, 0, -1);

        Hd_.block(0, 0, 3, 3) = Matrix3d::Zero();
        Hd_.block(0, 3, 3, 3) = Matrix3d::Zero();
        Hd_.block(0, 6, 3, 3) = -Matrix3d::Identity();
        Hd_.block(0, 9, 3, 3) = Matrix3d::Zero();
        Hd_.block(0, 12, 3, 3) = Matrix3d::Identity();
        Hd_.block(0, 15, 3, 3) = Matrix3d::Zero();
        Hd_.block(0, 18, 3, 3) = Matrix3d::Zero();
        
        Rd_.block(0, 0, 3, 3) = pow(measurementNoiseStd_, 2) * Matrix3d::Identity();

        PI_.block(0, 0, 3, 3) = Matrix3d::Identity();

        BigX_.block(0, 0, 7, 7) = xPred_;
    }

    Sd_ = Hd_ * Ppred_ * Hd_.transpose() + Rd_;
    Kd_ = Ppred_ * Hd_.transpose() * Sd_.inverse();
    deltaX_ = Kd_ * PI_ * (BigX_ * Y_ - b_);

    this->updateState(deltaX_, xPred_, thetaPred_, x_, theta_);

    MatrixXd I = MatrixXd::Identity(statesErrDim_, statesErrDim_);
    P_ = (I - Kd_ * Hd_) * Ppred_ * (I - Kd_ * Hd_).transpose() + Kd_ * Rd_ * Kd_.transpose();
}

void LieEKF::runFilter(Vector3d gyro, Vector3d acc, Vector3d lfpmeasured, Vector3d rfpmeasured, Matrix3d lfrot, Matrix3d rfrot, int* contact, bool update_enaled) {
    updateEnabled_ = update_enaled;
    contact_[0] = contact[0];
    contact_[1] = contact[1];
    this->setSensorData(gyro, acc);
    this->setMeasuredData(lfpmeasured, rfpmeasured, lfrot, rfrot);
    this->predict();

    if(updateEnabled_){
        this->update();
    }
    xPrev_ = x_;
    thetaPrev_ = theta_;

    cout << GBasePos_(0) << ", " << GBasePos_(1) << ", " << GBasePos_(2) << ", ";
    cout << getGBaseQuat().w() << ", " << getGBaseQuat().x() << ", "<<getGBaseQuat().y() << ", "<<getGBaseQuat().z() << endl;
    // for(int i=0; i < 21; i++){
    //     cout << P_(i, i) << ", ";
    // }
    // cout << endl;
}

// vector<string> parseString(string str, char delimiter) {
//     vector<string> result;
//     stringstream ss(str);
//     string item;
//     while (getline(ss, item, delimiter)) {
//         result.push_back(item);
//     }
//     return result;
// }

// int main() {
//     LieEKF estimator;
//     estimator.setDt(0.002);
//     ifstream file;
//     string line;
//     file.open("turnData'.csv");
//     int i = 0;
//     std::default_random_engine generator;
//     std::normal_distribution<double> dist(0.0, 0.1);
//     while(i < 6999){
//         getline(file, line);
//         vector<string> data = parseString(line, ',');
//         Vector3d gyro(stod(data[0]) + dist(generator), stod(data[1]) + dist(generator), stod(data[2]) + dist(generator));
//         // Vector3d gyro(1, 2, 3);
//         Vector3d acc(stod(data[3]) + dist(generator), stod(data[4]) + dist(generator), stod(data[5]) + dist(generator));
//         // Vector3d acc(4,5,6);
//         Vector3d rfmeasured(stod(data[6]), stod(data[7]), stod(data[8]));
//         Vector3d lfmeasured(stod(data[9]), stod(data[10]), stod(data[11]));
//         Matrix3d rfrot = Quaterniond(stod(data[12]), stod(data[13]), stod(data[14]), stod(data[15])).toRotationMatrix();
//         Matrix3d lfrot = Quaterniond(stod(data[16]), stod(data[17]), stod(data[18]), stod(data[19])).toRotationMatrix();
//         int contact[] = {stoi(data[21]), stoi(data[20])};
//         estimator.runFilter(gyro, acc, lfmeasured, rfmeasured, lfrot, rfrot, contact, true);
//         // if((i + 1) % 80 == 0)
//         //     estimator.initializeCovariance(0.1, 0.1, 0.1, 0.1, 0.1, 0.1);
//         i++;
//     }
//     return 0;
// }