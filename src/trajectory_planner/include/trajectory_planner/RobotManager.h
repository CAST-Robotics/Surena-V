#pragma once

#include <iostream>
#include "fstream"
#include <math.h>
#include <vector>
#include <string>

#include <ros/ros.h>
#include <ros/package.h>

#include <std_srvs/Empty.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/JointState.h>
#include "sensor_msgs/Imu.h"
#include <std_msgs/Int32MultiArray.h>
#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/Vector3Stamped.h"

#include "trajectory_planner/command.h"
#include "trajectory_planner/getdata.h"
#include "trajectory_planner/JntAngs.h"
#include "trajectory_planner/Trajectory.h"
#include "trajectory_planner/GeneralTraj.h"
#include "trajectory_planner/move_hand_single.h"
#include "trajectory_planner/move_hand_both.h"

#include "MinimumJerkInterpolation.h"
#include "S5mod_right_hand.h"
#include "S5mod_left_hand.h"

#include <Robot.h>

using namespace std;
using namespace Eigen;

class RobotManager
{
public:
    RobotManager(ros::NodeHandle *n);

    // Lower Body
    bool sendCommand();
    bool setPos(int jointID, int dest);
    bool home(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    void qcInitial(const sensor_msgs::JointState &msg);
    void ftCallbackLeft(const geometry_msgs::Wrench &msg);
    void bumpCallback(const std_msgs::Int32MultiArray &msg);
    void ftCallbackRight(const geometry_msgs::Wrench &msg);
    void IMUCallback(const sensor_msgs::Imu &msg);
    void AccCallback(const geometry_msgs::Vector3Stamped &msg);
    void GyroCallback(const geometry_msgs::Vector3Stamped &msg);
    bool setFTZero();
    bool setBumpZero();
    bool dummyCallback(trajectory_planner::getdata::Request &req,
                       trajectory_planner::getdata::Response &res);
    bool emptyCommand();
    bool sendCommand(trajectory_planner::command::Request &req,
                     trajectory_planner::command::Response &res);
    void absReader(const sensor_msgs::JointState &msg);
    void incReader(const sensor_msgs::JointState &msg);
    bool absPrinter(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    bool ankleHome(bool is_left, int roll_dest, int pitch_dest);
    bool walk(trajectory_planner::Trajectory::Request &req,
              trajectory_planner::Trajectory::Response &res);
    bool keyboardWalk(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    void keyboardHandler(const std_msgs::Int32 &msg);
    bool computeLowerLimbJointMotion(double jnt_command[], int iter);
    int sgn(double v);
    double abs2rad(int abs);
    double inc2rad(int inc);
    bool checkAngle(int joint_ID, double angle, double &ang_dif);
    void ankleMechanism(double &theta_inner, double &theta_outer,
                        double teta_p, double teta_r, bool is_left);
    void yawMechanism(double &theta, double alpha1, double R, double B, bool is_left);
    void yawMechanismFK(double &alpha, double theta, double R, double B, bool is_left);

    // Upper Body
    MatrixXd scenario_target_R(string scenario, int i, VectorXd ee_pos, string ee_ini_pos);
    MatrixXd scenario_target_L(string scenario, int i, VectorXd ee_pos, string ee_ini_pos);
    VectorXd reach_target_L(MatrixXd targets, string scenario, int M);
    VectorXd reach_target_R(MatrixXd targets, string scenario, int M);
    bool single(trajectory_planner::move_hand_single::Request &req,
                trajectory_planner::move_hand_single::Response &res);
    bool both(trajectory_planner::move_hand_both::Request &req,
              trajectory_planner::move_hand_both::Response &res);

    void handMotion(vector<double> &right_motion, vector<double> &left_motion, 
                    double t_step, int step_count, double max_angle, double dt, bool isLeftFirst=true);

private:
    Robot *robot;
    // Lower Body
    ros::Publisher motorDataPub_;
    ros::Subscriber incSub_;
    ros::Subscriber offsetSub_;
    ros::Subscriber absSub_;
    ros::Subscriber lFT_;
    ros::Subscriber rFT_;
    ros::Subscriber IMUSub_;
    ros::Subscriber AccSub_;
    ros::Subscriber GyroSub_;
    ros::Subscriber bumpSub_;
    ros::Subscriber keyboardCommandSub_;
    ros::ServiceServer jointCommand_;
    ros::ServiceServer absPrinter_;
    ros::ServiceServer walkService_;
    ros::ServiceServer keyboardWalkService_;
    ros::ServiceServer homeService_;
    ros::ServiceServer dummyCommand_;
    bool isWalkingWithKeyboard;
    bool isKeyboardTrajectoryEnabled;
    bool qcInitialBool_;
    int homeOffset_[32];
    std_msgs::Int32MultiArray motorCommand_;
    double motorCommandArray_[23];
    int harmonicRatio_[12];
    float absData_[32];
    int incData_[32];
    int homeAbs_[12];
    int absHigh_[12];
    int absLow_[12];
    int abs2incDir_[12];
    int absDir_[12];
    int motorDir_[12];
    bool collision_;
    int bumpOrder_[8];

    // Ankle Mechanism Parameters

    float b;
    float c;
    float r1;
    float r0;
    Vector3d r30_inner;
    Vector3d r30_outer;
    MatrixXd r4_inner;
    MatrixXd r4_outer;

    ofstream leftFTFile_;
    ofstream rightFTFile_;

    Vector3d rFTOffset_;
    Vector3d lFTOffset_;
    int rBumpOffset_[4];
    int lBumpOffset_[4];
    vector<Vector3d> recentLFT_;
    vector<Vector3d> recentRFT_;

    double currentLFT_[3];
    double currentRFT_[3];
    int currentLBump_[4];
    int currentRBump_[4];
    int realRBump_[4];
    int realLBump_[4];
    double baseAcc_[3];
    double baseOrient_[3];
    double baseAngVel_[3];
    double commandConfig_[3][12];

    int FTOffsetPeriod_;

    // Upper Body
    ros::ServiceServer move_hand_single_service;
    ros::ServiceServer move_hand_both_service;

    MinimumJerkInterpolation coef_generator;

    MatrixXd X_coef_r;
    MatrixXd X_coef_l;
    MatrixXd Y_coef_r;
    MatrixXd Y_coef_l;
    MatrixXd Z_coef_r;
    MatrixXd Z_coef_l;
    VectorXd P_r;
    VectorXd V_r;
    VectorXd P_l;
    VectorXd V_l;

    VectorXd r_start_r;
    VectorXd r_start_l;
    VectorXd r_middle_r;
    VectorXd r_middle_l;
    VectorXd r_target_r;
    VectorXd r_target_l;
    MatrixXd R_target_r;
    MatrixXd R_target_l;
    VectorXd r_midpoint_r;
    VectorXd r_midpoint_l;
    MatrixXd r_midpoint_b;
    VectorXd r_right_palm;
    VectorXd r_left_palm;

    MatrixXd P_x_r;
    MatrixXd P_x_l;
    MatrixXd V_x_r;
    MatrixXd V_x_l;
    MatrixXd A_x_r;
    MatrixXd A_x_l;
    MatrixXd P_y_r;
    MatrixXd P_y_l;
    MatrixXd V_y_r;
    MatrixXd V_y_l;
    MatrixXd A_y_r;
    MatrixXd A_y_l;
    MatrixXd P_z_r;
    MatrixXd P_z_l;
    MatrixXd V_z_r;
    MatrixXd V_z_l;
    MatrixXd A_z_r;
    MatrixXd A_z_l;

    double d0_r;
    double d0_l;
    double d_r;
    double d_l;
    double d_des_r;
    double d_des_l;
    double theta_r;
    double theta_l;
    double theta_target_r;
    double theta_target_l;
    double sai_r;
    double sai_l;
    double sai_target_r;
    double sai_target_l;
    double phi_r;
    double phi_l;
    double phi_target_r;
    double phi_target_l;
    double v0_r = 0;
    double v0_l = 0;
    double v_target_r = .4;
    double v_target_l = .4;

    // define joint variables
    VectorXd q_ra;
    VectorXd q_la;
    VectorXd q_init_r;
    VectorXd q_init_l;
    MatrixXd qref;
    MatrixXd ee_pos;
    // vector<double> q_motor;
    vector<double> q_gazebo;

    double sum_r = 0;
    double sum_l = 0;
    double T = 0.005;

    VectorXd q_end;
    MatrixXd qref_r;
    MatrixXd qref_l;
    double time_r;
    left_hand hand_func_L;
    right_hand hand_func_R;
    double total;
    MatrixXd next_ini_ee_posR;
    MatrixXd next_ini_ee_posL;
    int M;
    int rate = 200;
    bool simulation = false;
    int encoderResolution[2] = {4096 * 4, 2048 * 4};
    int harmonicRatio[4] = {100, 100, 100, 400};
};