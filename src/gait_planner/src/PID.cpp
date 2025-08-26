#include "PID.h"

PID::PID(double timeStep)
{
    kp_ << 1.0, 1.0, 1.0,
        1.0, 1.0, 1.0,
        1.0, 1.0, 1.0;
    ki_ << 1.0, 1.0, 1.0,
        1.0, 1.0, 1.0,
        1.0, 1.0, 1.0;
    kd_ << 1.0, 1.0, 1.0,
        1.0, 1.0, 1.0,
        1.0, 1.0, 1.0;
    kzmp_ << 1.0, 1.0, 1.0,
        1.0, 1.0, 1.0,
        1.0, 1.0, 1.0;
    kcom_ << 1.0, 1.0, 1.0,
        1.0, 1.0, 1.0,
        1.0, 1.0, 1.0;
    this->intI_ = 0.0;
    this->prevoiusError_ = 0.0;
    this->dt_ = timeStep;
    xi_error_ << 0.0, 0.0, 0.0;
    // ros::init(argc, argv, "controller");
    // ros::ServiceServer dcm_service = nh.advertise("dcmcontroller", dcmController);
    // ros::ServiceServer com_service = nh.advertise("comcontroller", comController);
}

// double PID::getOutput(double desiredValue, double currentValue){
//     /*
//         computes next plant input
//     */
//     double error = currentValue - desiredValue;
//     this->intI_ += error * dt_;
//     double deriv = (error - this->prevoiusError_) / dt_;
//     this->prevoiusError_ = error;
//
//     return kp_ * error + ki_ * this->intI_ + kd_ * deriv;
// }
/*
bool PID::dcmController(gait_planner::DCMController::Request &req,
                        gait_planner::DCMController::Response &res){
                            Matrix3d Kp = kp_*kp_;
                            Matrix3d Ki = ki_*ki_;

                            xi_error += req.xi_real - req.xi_ref;

                            res.r_zmp_ref = req.xi_ref - ((req.xi_dot_ref)/sqrt(9.81/req.deltaZVRP)) +
                            Kp*(req.xi_real-req.xi_ref) + Ki*xi_error_;

                            return true;
                        }

bool PID::comController(gait_planner::COMController::Request &req,
                        gait_planner::COMController::Response &res){
                            res.x_dot_star = req.x_dot_ref - kzmp_*(req.r_zmp_ref - req.r_zmp_real) + kcom_*(req.x_ref - req.x_real);
                            return true;
                        }
*/