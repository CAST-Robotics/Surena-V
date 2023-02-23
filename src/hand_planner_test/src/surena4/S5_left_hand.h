#ifndef LEFT_HAND_H
#define LEFT_HAND_H
#include "../Eigen/Dense"
#include "../Eigen/Geometry"
#include "../Eigen/QuadProg.h"
#include "../Eigen/testQPsolvers.hpp"
#include "../Eigen/eiquadprog.hpp"
#include "../Eigen/Core"
#include "../Eigen/Cholesky"
#include "../Eigen/LU"
//#include "qdebug.h"

#include<math.h>

using namespace Eigen;
using namespace std;

class left_hand
{
public:


    double T=.01;

    double L_arm=.25747;
    double L1_forearm=.07175;
    double L2_forearm=.15938;
    double L_palm=0;
    double angle_fix_shd=toRad(20);

    double v_des=0.3;
    double d_des=0.01;
    double d_orient=0.1; //0.1
    double power=1e-4;
    double left_palm_position_power=1e6;
    double left_palm_orientation_power=1e2;
    double qdot_max=10;
    VectorXd qdot;
    //    double q1_le;
    //    double q2_le;
    //    double q3_le;
    //    double q4_le;
    //    double q5_le;
    //    double q6_le;
    //    double q7_le;

    MatrixXd R1_fix_shd;
    MatrixXd R1_le;
    MatrixXd R2_le;
    MatrixXd R3_le;
    MatrixXd R4_le;
    MatrixXd R5_le;
    MatrixXd R6_le;
    MatrixXd R7_le;

    MatrixXd P_arm_le;
    MatrixXd P1_forearm_le;
    MatrixXd P2_forearm_le;
    MatrixXd P_palm_le;

    MatrixXd T_left_palm; //???????????????????????????????????
    MatrixXd R_left_palm;
    VectorXd r_left_palm;

    double v0;              //???????????
    double v_target;        //??????????? update V for QP

    double sai;
    double theta;
    double phi;

    double sai_target;
    double theta_target;
    double phi_target;

    MatrixXd Jha;
    MatrixXd J_left_palm;
    MatrixXd J_w_left_palm;

    double Vx_left_palm;
    double Vy_left_palm;
    double Vz_left_palm;
    Vector3d w_left_palm;
    Vector3d Eul_Angles;
    Vector3d Eul_Angles_target;
    double sai_dot;
    double phi_dot;
    double theta_dot;

    VectorXd V; //???????????
    VectorXd q_next;
    MatrixXd qref;

    left_hand();
    left_hand(VectorXd q_le, VectorXd r_target, MatrixXd R_target);
    left_hand(VectorXd q_le, VectorXd v, VectorXd r_target, MatrixXd R_target);
    left_hand(VectorXd q_le, VectorXd r_target, MatrixXd R_target, int i, double d0); //d0??? (r_target-r0)
    left_hand(VectorXd q_le, VectorXd r_target, MatrixXd R_target, double d0, double v_0, double v__target); 
    double toRad(double d);
    double sai_calc(MatrixXd R);
    double phi_calc(MatrixXd R);
    double theta_calc(MatrixXd R);
    MatrixXd trans(int axis, double d);
    MatrixXd trans(Vector3d d);
    MatrixXd rot(int axis, double q, int dim);

    void HO_FK_left_palm(VectorXd q_le);
    void euler2w(); 
    void jacob(VectorXd q_le);
    double dist; 
    double dist_or;

    VectorXd upbound;
    VectorXd lowbound;


    MatrixXd G;
    VectorXd g;
    MatrixXd CI;
    VectorXd ci0;
    MatrixXd CE;
    VectorXd ce0;
    double distance(VectorXd V1, VectorXd V2);
    double distance_orien(MatrixXd M1, MatrixXd M2);

    void doQP(VectorXd q_le); //????


    double wrist_pos2mot(double pos); //?????
    vector<int> data2qc_without_wrist(vector<double> cntrl); //????
    vector<int> data2qc(vector<double> cntrl); //???
    double velocity(double d, double d0); //?? update for v?
    

    void update_left_hand(VectorXd q_le, VectorXd r_target, MatrixXd R_target, double d0, double v_0, double v__target);
    void update_left_hand(VectorXd q_le, VectorXd r_target, MatrixXd R_target, int i, double d0);
    void update_left_hand(VectorXd q_le, VectorXd v, VectorXd r_target, MatrixXd R_target);
    void update_left_hand(VectorXd q_le, VectorXd r_target, MatrixXd R_target);
    void matrix_view(MatrixXd M); //???
    void matrix_view(VectorXd M); //???
};

#endif // RIGHT_HAND_H
