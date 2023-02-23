#include "../Eigen/Dense"
#include "../Eigen/Geometry"
#include "../Eigen/QuadProg.h"
#include "../Eigen/testQPsolvers.hpp"
#include "../Eigen/eiquadprog.hpp"
#include "../Eigen/Core"
#include "../Eigen/Cholesky"
#include "../Eigen/LU"
#include <iostream>
#include <vector>
#include "fstream"
#include <string>
#include<math.h>

using namespace Eigen;
using namespace std;
MatrixXd rot(int axis , double q ,int dim){
    if (dim==3){
        MatrixXd R(3,3);
    if (axis==1){
        R<<1,0,0,
                0,cos(q),-sin(q),
                0,sin(q),cos(q);
    }

    if (axis==2){
        R<<cos(q),0,sin(q),
                0,1,0 ,
                -sin(q),0,cos(q);
    }

    if (axis==3){
                R<<cos(q),-sin(q),0,
                sin(q),cos(q),0,
                0,0,1;
    }
    return R;
    }

    if(dim==4){
                    MatrixXd R(4,4);
        if (axis==1){
            R<<1,0,0,0,
                    0,cos(q),-sin(q),0,
                    0,sin(q),cos(q),0,
                    0,0,0,1;
        }

        if (axis==2){
            R<<cos(q),0,sin(q),0,
                    0,1,0,0,
                    -sin(q),0,cos(q),0,
                    0,0,0,1;
        }

        if (axis==3){
            R<<cos(q),-sin(q),0,0,
                    sin(q),cos(q),0,0,
                    0,0,1,0,
                    0,0,0,1;
        }
        return R;
    }
};

double toRad(double d){
    double r;
    r=d*M_PI/180;
    return r;
};

MatrixXd trans(int axis, double d){
    MatrixXd H(4,4);
    H=MatrixXd::Identity(4,4);
    H(axis-1,3)=d;
    return H;
}

int main(){
    double L_arm=.25747;
    double L_forearm=.23113;
    double L_palm=0;

    MatrixXd P_arm_ra(4,4);
    MatrixXd P_forearm_ra(4,4);
    MatrixXd P_palm_ra(4,4);

    P_arm_ra = trans(3,-L_arm);
    P_forearm_ra = trans(3,-L_forearm);
    P_palm_ra = trans(3,-L_palm);

    MatrixXd R_target_r(3,3);
    VectorXd r_target_r(3);
    VectorXd q_ra(7);
    q_ra<<0,-5*M_PI/180,0,0,0.0,0.0,0.0;

    double angle_fix_shd=toRad(0);
    double angle_fix_elbow=0;

    MatrixXd R1_fix_shd(4,4);
    MatrixXd R2_fix_shd(4,4);
    MatrixXd R1_ra(4,4);
    MatrixXd R2_ra(4,4);
    MatrixXd R3_ra(4,4);
    MatrixXd R1_fix_elbow(4,4);
    MatrixXd R2_fix_elbow(4,4);
    MatrixXd R4_ra(4,4);
    MatrixXd R5_ra(4,4);
    MatrixXd R6_ra(4,4);
    MatrixXd R7_ra(4,4);

    R1_fix_shd=rot(1,-angle_fix_shd,4);
    R2_fix_shd=rot(1,angle_fix_shd,4);
    R1_ra = rot(2,q_ra(0),4);
    R2_ra = rot(1,q_ra(1),4);
    R3_ra = rot(3,q_ra(2),4);
    R1_fix_elbow=rot(3,+angle_fix_elbow,4);
    R2_fix_elbow=rot(3,-angle_fix_elbow,4);
    R4_ra = rot(2,q_ra(3),4);
    R5_ra = rot(3,q_ra(4),4);
    R6_ra = rot(1,q_ra(5),4);
    R7_ra = rot(2,q_ra(6),4);

    MatrixXd T_right_palm(4,4); 
    MatrixXd R_right_palm(3,3);
    VectorXd r_right_palm(3,1);
    T_right_palm=R1_fix_shd*R1_ra*R2_fix_shd*R2_ra*R3_ra*P_arm_ra*R1_fix_elbow*R4_ra*R5_ra*P_forearm_ra*R2_fix_elbow*R6_ra*R7_ra*P_palm_ra;
    r_right_palm<<T_right_palm.block(0,3,3,1);
    R_right_palm<<T_right_palm.block(0,0,3,3);
    cout << R_right_palm;

}