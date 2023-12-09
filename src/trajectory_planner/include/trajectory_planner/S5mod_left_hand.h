#ifndef LEFT_HAND_H
#define LEFT_HAND_H
#include "Eigen/Dense"
#include "Eigen/Geometry"
#include "Eigen/QuadProg.h"
#include "Eigen/testQPsolvers.hpp"
#include "Eigen/eiquadprog.hpp"
#include "Eigen/Core"
#include "Eigen/Cholesky"
#include "Eigen/LU"
#include <math.h>
#include <iostream>
#include <vector>
#include "fstream"
#include <string>
#include "ros/ros.h"
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float64.h>
#include <gazebo_msgs/LinkStates.h>

using namespace Eigen;
using namespace std;

class left_hand
{
public:
    // physical parameters
    double L_arm = .25747;
    double L_forearm = .23113;
    double L_palm = 0;
    double angle_fix_shd = toRad(-10);
    double angle_fix_elbow = 0;

    // solver parameters
    double T = .005;
    double d_des = 0.01;
    double d_orient = 0.2;
    double dist;
    double dist_or;
    double power = 1e-4;
    double left_palm_position_power = 1e6;
    double left_palm_orientation_power = 1e6; // 1e6
    double qdot_max = 1;
    double v0;
    double v_des = 0.3;
    double v_target;
    VectorXd V;

    VectorXd qdot;
    VectorXd q_next;
    MatrixXd qref;

    // FK parameters
    MatrixXd R1_fix_shd;
    MatrixXd R2_fix_shd;
    MatrixXd R1_ra;
    MatrixXd R2_ra;
    MatrixXd R3_ra;
    MatrixXd R1_fix_elbow;
    MatrixXd R2_fix_elbow;
    MatrixXd R4_ra;
    MatrixXd R5_ra;
    MatrixXd R6_ra;
    MatrixXd R7_ra;

    MatrixXd P_arm_ra;
    MatrixXd P_forearm_ra;
    MatrixXd P_palm_ra;

    MatrixXd T_left_palm;
    MatrixXd R_left_palm;
    VectorXd r_left_palm;

    double sai;
    double theta;
    double phi;

    double thetaY;
    double thetaZ;
    double thetaX;

    double sai_target;
    double theta_target;
    double phi_target;

    double Vx_left_palm;
    double Vy_left_palm;
    double Vz_left_palm;
    Vector3d w_left_palm;

    double sai_dot;
    double phi_dot;
    double theta_dot;

    // wrist IK
    VectorXd wrist_right_calc(double alpha, double beta);
    VectorXd wrist_left_calc(double alpha, double beta);

    // Jacobian
    MatrixXd Jha;
    MatrixXd J_left_palm;
    MatrixXd J_w_left_palm;

    // constructors
    left_hand();
    left_hand(VectorXd q_ra, VectorXd r_target, MatrixXd R_target);
    left_hand(VectorXd q_ra, VectorXd r_target, MatrixXd R_target, int i, double d0);
    left_hand(VectorXd q_ra, VectorXd r_target, MatrixXd R_target, double d0, double v_0, double v__target);
    left_hand(VectorXd q_ra, VectorXd v, VectorXd r_target, MatrixXd R_target);
    // methods
    double toRad(double d);
    double sai_calc(MatrixXd R);
    double phi_calc(MatrixXd R);
    double theta_calc(MatrixXd R);
    Vector3d euler_calc(MatrixXd R);
    MatrixXd trans(int axis, double d);
    MatrixXd trans(Vector3d d);
    MatrixXd rot(int axis, double q, int dim);
    double distance(VectorXd V1, VectorXd V2);
    double velocity(double d, double d0); // finding coef for v
    void euler2w();

    void HO_FK_left_palm(VectorXd q_ra);
    void jacob(VectorXd q_ra);

    // QP parameters
    MatrixXd G;
    VectorXd g;
    MatrixXd CI;
    VectorXd ci0;
    MatrixXd CE;
    VectorXd ce0;
    VectorXd upbound;
    VectorXd lowbound;

    void doQP(VectorXd q_ra);

    // funcs for talking motors
    double wrist_pos2mot(double pos);
    vector<int> data2qc_without_wrist(vector<double> cntrl);
    vector<int> data2qc(vector<double> cntrl);

    // solve for each time step
    void update_left_hand(VectorXd q_ra, VectorXd r_target, MatrixXd R_target, double d0, double v_0, double v__target);
    void update_left_hand(VectorXd q_ra, VectorXd r_target, MatrixXd R_target, int i, double d0);
    void update_left_hand(VectorXd q_ra, VectorXd v, VectorXd r_target, MatrixXd R_target);
    void update_left_hand(VectorXd q_ra, VectorXd r_target, MatrixXd R_target);

    void matrix_view(MatrixXd M);
    void matrix_view(VectorXd M);

    // move_to_pose
    double move2pose(double max, double t_local, double T_start, double T_end);
    void SendGazebo(vector<double> q);
};

#endif // LEFT_HAND_H
