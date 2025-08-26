#ifndef S5_HAND_H
#define S5_HAND_H

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "QuadProg.h"
#include "eiquadprog.h"
#include "eigen3/Eigen/Core"
#include "ros/ros.h"
#include <vector>
#include <std_msgs/Float64.h>
#include <math.h>
#include <iostream>

using namespace Eigen;
using namespace std;

// Enum to specify which hand the instance represents
enum HandType {
    RIGHT,
    LEFT
};

class S5_hand {
public:
    // --- CONSTRUCTOR ---
    S5_hand(HandType type); // Main constructor to set the hand type

    HandType hand_type; // Stores whether the instance is RIGHT or LEFT

    // --- PUBLIC MEMBER VARIABLES ---
    // Physical Parameters
    double L_arm = .25747;
    double L_forearm = .23113;
    double L_palm = 0;
    double angle_fix_elbow = 0;

    // Solver Parameters
    double T = .005;
    double d_des = 0.01;
    double d_orient = 0.2;
    double dist;
    double power = 1e-4;
    double qdot_max = 1;
    double v0;
    double v_des = 0.3;
    double v_target;
    VectorXd V;

    // State Variables
    VectorXd qdot;
    VectorXd q_next;
    MatrixXd qref;
    MatrixXd T_palm;
    MatrixXd R_palm;
    VectorXd r_palm;
    double sai, theta, phi;
    double sai_target, theta_target, phi_target;
    Vector3d w_palm;
    double sai_dot, phi_dot, theta_dot;

    // --- PUBLIC MEMBER FUNCTIONS ---
    // Update Methods (replaces update_right_hand and update_left_hand)
    void update_hand(VectorXd q_in, VectorXd r_target, MatrixXd R_target, double d0, double v_0, double v__target);
    void update_hand(VectorXd q_in, VectorXd r_target, MatrixXd R_target, int i, double d0);
    void update_hand(VectorXd q_in, VectorXd v, VectorXd r_target, MatrixXd R_target);
    void update_hand(VectorXd q_in, VectorXd r_target, MatrixXd R_target);

    // Core Kinematics and Control
    void HO_FK_palm(VectorXd q_in);
    void doQP(VectorXd q_in);

    // Forward declaration for the Jacobian function. The implementation will be in S5_hand_jacobian.cpp
    void calculate_jacobian(const VectorXd& q_in, double angle_fix_shd, MatrixXd& J_palm, MatrixXd& J_w_palm);

    // Utility & Transformation Functions
    double toRad(double d);
    MatrixXd rot(int axis, double q, int dim);
    MatrixXd trans(int axis, double d);
    MatrixXd trans(Vector3d d);
    MatrixXd ObjToNeck(double h_pitch, double h_roll, double h_yaw);
    MatrixXd returnAngles(MatrixXd T_EEtobase);

    // Wrist Inverse Kinematics (These are mechanism-specific)
    double wrist_left_calc(double alpha, double beta);
    double wrist_right_calc(double alpha, double beta);

    // Simulation and Hardware Interface
    double move2pose(double max, double t_local, double T_start, double T_end);
    vector<int> data2qc(vector<double> cntrl);

private:
    // --- PRIVATE MEMBER VARIABLES ---
    // Hand-specific parameters set by the constructor
    double angle_fix_shd;
    vector<double> minimum;
    vector<double> maximum;
    double wrist_clip_value;
    double palm_position_power = 1e6;
    double palm_orientation_power = 1e6;

    // Internal Kinematic Parameters
    MatrixXd R1_fix_shd, R2_fix_shd, R1_ra, R2_ra, R3_ra;
    MatrixXd R1_fix_elbow, R2_fix_elbow, R4_ra, R5_ra, R6_ra, R7_ra;
    MatrixXd P_arm_ra, P_forearm_ra, P_palm_ra;
    MatrixXd T0, T1, T2, T3, T4, T5, T_EEtobase;
    
    // Internal Jacobian and QP Parameters
    MatrixXd J_palm, J_w_palm;
    MatrixXd G;
    VectorXd g;
    MatrixXd CI, CE;
    VectorXd ci0, ce0;

    // Camera to Shoulder Transformation Parameters
    double head_PtoR = 0.0825;
    double head_YtoP = 0.06025;
    double Shoulder2Head[3] = {0, 0.23, 0.08};
    double camera[3] = {0.1248, 0, 0.06746};

    // --- PRIVATE MEMBER FUNCTIONS ---
    void euler2w();
    double distance(VectorXd V1, VectorXd V2);
    double velocity(double d, double d0);
    double phi_calc(MatrixXd R), theta_calc(MatrixXd R), sai_calc(MatrixXd R);
    VectorXd solveQuadratic(double a, double b, double c);
    double wrist_pos2mot(double pos);
};

#endif // S5_HAND_H