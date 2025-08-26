#pragma once

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include "gait_planner/JntAngs.h"
#include "gait_planner/Trajectory.h"
#include "gait_planner/GeneralTraj.h"
#include <std_msgs/Float64MultiArray.h>

#include "json.hpp"
#include "DCM.h"
#include "Link.h"
#include "PID.h"
#include "Controller.h"
#include "Ankle.h"
#include "MinJerk.h"
#include "GeneralMotion.h"
#include "Collision.h"
#include "Estimator.h"
#include "LieEKF.h"
#include "QuatEKF.h"
#include "StepPlanner.h"
#include "Butterworthfilter.h"

#include <deque>
#include "fstream"
#include <random>
#include <chrono>

using namespace std;
using json = nlohmann::json;

enum ControlState
{
    IDLE,
    WALK
};

class Robot
{
public:
    Robot(ros::NodeHandle *nh, std::string config_path, bool simulation=false);
    ~Robot();

    void initROSCommunication();
    void initializeRobotParams();
    void initializeLinkObjects(Vector3d a[], Vector3d b[], Vector3d com_pos[], double links_mass[]);
    void resetRobotParams();

    void spinOnline(double config[], double jnt_vel[], Vector3d torque_r, Vector3d torque_l,
                    double f_r, double f_l, Vector3d gyro, Vector3d accelerometer, int bump_r[], int bump_l[],
                    double *joint_angles, ControlState robot_cs, int &status);

    void runFootLenController(double f_l, double f_r, ControlState robot_cs);

    void runBumpFootOrientController(int bump_r[], int bump_l[]);

    void runEarlyContactController(int bump_r[], int bump_l[]);

    void runFootOrientController();

    void runZMPAdmitanceController();

    bool getJointAngs(int iter, double config[12], double jnt_vel[12], double right_ft[3],
                      double left_ft[3], int right_bump[4], int left_bump[4], double gyro[3],
                      double accelerometer[3], double jnt_command[12], int &status);
    bool trajGen(int step_count, double t_step, double alpha, double t_double_support,
                 double COM_height, double step_length, double step_width, double dt,
                 double theta, double ankle_height, double step_height, double slope, double com_offset, bool is_config);
    bool generalTrajGen(double dt, double time, double init_com_pos[3], double final_com_pos[3], double init_com_orient[3], double final_com_orient[3],
                        double init_lankle_pos[3], double final_lankle_pos[3], double init_lankle_orient[3], double final_lankle_orient[3],
                        double init_rankle_pos[3], double final_rankle_pos[3], double init_rankle_orient[3], double final_rankle_orient[3]);
    
    int OnlineGeneralTrajGen(double dt, double time, double final_com_pos[3], double final_com_orient[3],
                                                     double final_lankle_pos[3], double final_lankle_orient[3],
                                                     double final_rankle_pos[3], double final_rankle_orient[3]);

    void getGeneralTrajJointAngs(int index, double config[12], double jnt_vel[12], double right_ft[3],
                                 double left_ft[3], int right_bump[4], int left_bump[4], double gyro[3],
                                 double accelerometer[3], double jnt_command[12], int &status);

    int OnlineDCMTrajGen(int step_count, double t_step, double alpha, double t_double_support,
                         double COM_height, double step_length, double step_width, double dt,
                         double theta, double ankle_height, double step_height, double slope,
                         double com_offset, bool is_config);

    void getDCMTrajJointAngs(int index, double config[12], double jnt_vel[12], double right_ft[3],
                             double left_ft[3], int right_bump[4], int left_bump[4], double gyro[3],
                             double accelerometer[3], double jnt_command[12], int &status);

    int changeStep();                     

    ControlState getCurrentWalkState() {return currentWalkState_;}

    bool resetTraj();

    void generateStraightFootStep(vector<Vector3d>& ankle_rf, vector<Vector3d>& dcm_rf, const double &step_width,
                                  const double &step_length, const double &step_height, const int &step_count, const double &com_offset);

    void generateTurnFootStep(vector<Vector3d>& ankle_rf, vector<Vector3d>& dcm_rf, const double &step_length,
                              const double &step_height, const int &step_count, const double &theta, const double &com_offset);

    int findTrajIndex(vector<int> arr, int n, int K);

    void distributeFT(Vector3d zmp_y, Vector3d r_foot_y, Vector3d l_foot_y, Vector3d &r_wrench, Vector3d &l_wrench);
    void distributeBump(double r_foot_z, double l_foot_z, double &r_bump, double &l_bump);

    void publishCoMPose(Vector3d com);
    void publishFootStep(const vector<Vector3d>& ankle_rf, const int &step_count);
    void publishZMPPose();

    void loadConfig(json& walk_config, int& step_count, vector<Vector3d>& ankle_rf, vector<Vector3d>& dcm_rf, vector<double>& theta_rf,
                    double& COM_height, double& t_double_support, double& t_step, double& alpha, double& ankle_height, double& com_offset);
    void generateFootSteps(vector<Vector3d>& ankle_rf, vector<Vector3d>& dcm_rf, double step_length, double step_width, double step_height, int step_count, double theta, double com_offset);

    inline int getTrajSize(){
        return dataSize_;
    }

private:
    GeneralMotion *generalPlanner_;
    DCMPlanner *DCMPlanner_;
    Ankle *anklePlanner_;

    vector<ControlState> robotControlState_;

    ros::NodeHandle *nh_;
    std::string robotConfigPath_;
    bool simulation_;
    std_msgs::Float64MultiArray joint_angles_gazebo_;

    double thigh_;
    double shank_;
    double torso_;
    double soleXFront_;
    double soleXBack_;
    double soleY_;
    double soleMinDist_;
    double dt_;
    double totalMass_;

    double joints_[12];

    Controller *onlineWalk_;

    void doIK(MatrixXd pelvisP, Matrix3d pelvisR, MatrixXd leftAnkleP, Matrix3d leftAnkleR, MatrixXd rightAnkleP, Matrix3d rightAnkleR);
    vector<double> geometricIK(MatrixXd p1, MatrixXd r1, MatrixXd p7, MatrixXd r7, bool isLeft);
    Matrix3d Rroll(double phi);
    Matrix3d RPitch(double theta);

    Vector3d currentCommandedCoMPos_;
    Matrix3d currentCommandedCoMRot_;
    Vector3d currentCommandedLeftAnklePos_;
    Matrix3d currentCommandedLeftAnkleRot_;
    Vector3d currentCommandedRightAnklePos_;
    Matrix3d currentCommandedRightAnkleRot_;
    Vector3d currentZMPPos_;
    int currentRobotPhase_;
    ControlState currentWalkState_;

    Vector3d prevCommandedCoMPos_;
    Matrix3d prevCommandedCoMRot_;
    Vector3d prevCommandedLeftAnklePos_;
    Matrix3d prevCommandedLeftAnkleRot_;
    Vector3d prevCommandedRightAnklePos_;
    Matrix3d prevCommandedRightAnkleRot_;

    vector<Vector3d> CoMPos_;
    vector<Matrix3d> CoMRot_;
    vector<Vector3d> zmpd_;
    vector<Vector3d> CoMDot_;
    vector<Vector3d> xiDesired_;
    vector<Vector3d> rAnklePos_;
    vector<Vector3d> lAnklePos_;
    vector<Matrix3d> rAnkleRot_;
    vector<Matrix3d> lAnkleRot_;
    vector<int> robotPhase_;
    double bumpBiasR_;
    double bumpBiasL_;
    bool bumpSensorCalibrated_;

    Vector3d rSole_;   // current position of right sole
    Vector3d lSole_;   // current position of left sole
    deque<Vector3d> FKBase_{3, Vector3d(0, 0, 0)};
    Vector3d FKBaseDot_;
    deque<Vector3d> FKCoM_{3, Vector3d(0, 0, 0)};
    Vector3d FKCoMDot_;
    Vector3d realXi_;
    Vector3d realZMP_; // current ZMP of robot
    bool leftSwings_;
    bool rightSwings_;

    _Link *links_[13];

    Vector3d CoMEstimatorFK(double config[]);
    void updateRobotState(double config[], double jnt_vel[], Vector3d torque_r, Vector3d torque_l, double f_r, double f_l, Vector3d gyro, Vector3d accelerometer);
    Matrix3d rDot_(Matrix3d R);
    void updateSolePosition();
    Vector3d getZMPLocal(Vector3d torque, double fz);
    Vector3d ZMPGlobal(Vector3d zmp_r, Vector3d zmp_l, double f_r, double f_l);
    Vector3d CoM2Base();
    Vector3d CoM2BaseVel();

    ros::ServiceServer jntAngsServer_;
    ros::ServiceServer trajGenServer_;
    ros::ServiceServer generalTrajServer_;
    ros::ServiceServer resetTrajServer_;
    ros::Publisher zmpDataPub_;
    geometry_msgs::Point zmpPosition_;
    ros::Publisher comDataPub_;
    ros::Publisher xiDataPub_;
    ros::Publisher footStepPub_;
    ros::Publisher gazeboJointStatePub_;
    bool isTrajAvailable_;
    bool useController_;

    int index_;
    int dataSize_;
    vector<int> trajSizes_;
    double COM_height_;

    Collision *ankleColide_;
    Estimator *estimator_;
    QuatEKF* quatEKF_;
    LieEKF* lieEKF_;
    StepPlanner* stepPlanner_;
    ButterworthFilter* butterworthfilter_;

    Vector3d lZMP_;
    Vector3d rZMP_;
};
