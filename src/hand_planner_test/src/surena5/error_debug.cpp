#include "S5mod_right_hand.h"
#include"MinimumJerkInterpolation.h"
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

using namespace  std;
using namespace  Eigen;


    int main() {

    MinimumJerkInterpolation coef_generator;

    MatrixXd P_x_r(1,3);      
    MatrixXd V_x_r(1,3);      
    MatrixXd A_x_r(1,3); 
    MatrixXd P_y_r(1,3);   
    MatrixXd V_y_r(1,3);      
    MatrixXd A_y_r(1,3);      
    MatrixXd P_z_r(1,3);      
    MatrixXd V_z_r(1,3);      
    MatrixXd A_z_r(1,3);

    MatrixXd X_coef_r;        
    MatrixXd Y_coef_r;        
    MatrixXd Z_coef_r; 
    VectorXd P_r(3); VectorXd V_r(3);
    
    // define time parameters
    int count = 0;
    double T = 0.005; 
    double time_r;
    time_r=count*T;
    MatrixXd t_r(1,3);
    t_r<<0,5,10;
    int M = t_r(t_r.size()-1)/T;

    // define joint variables
    VectorXd qr_cyc(7);
    qr_cyc<<0.0,-5*M_PI/180,0.0,-10*M_PI/180,0.0,0.0,0.0; // initial condition -10*M_PI/180
    vector<double> q_motor(29,0);
    vector<double> q_gazebo(29,0);
    VectorXd qr_initial(7);
    qr_initial = qr_cyc;
    MatrixXd qref(7,M);
    VectorXd q_end(7,1);

    // define right_hand objs
    right_hand hand_func;
    right_hand hand_r;

    VectorXd result_right(2);
    VectorXd result_left(2);

    VectorXd r_right_palm(3);
    MatrixXd R_target_r(3,3);
    VectorXd r_start_r(3); 
    VectorXd r_middle_r(3); 
    VectorXd r_target_r(3);

    right_hand hand0_r( qr_cyc, r_target_r,  R_target_r,0,0);

    // set target values
    r_start_r=hand0_r.r_right_palm;
    // target1
    //r_middle_r<<0.09,-0.04,-0.43;
    //r_target_r<<0.17,-0.1,-0.38;
    // target2
    //r_middle_r<<0.04,-0.1,-0.48;
    //r_target_r<<0.1,-0.2,-0.4;
    // target3 ok
    r_middle_r<< 0.243,-0.04,-0.4;
    r_target_r<< 0.123,-0.173,-0.45;
    // target4 ok
    //r_middle_r<< 0.238,-0.112,-0.47;
    //r_target_r<< 0.258,0.05,-0.38;
    // target5
    //r_middle_r<< .25,-0.00,-0.42; //bringup
    //r_target_r<< .25,-0.00,-0.35;
    R_target_r=hand_func.rot(2,-90*M_PI/180,3);


    /*
    r_middle_r<< 0.343,0.003,0.084;
    r_target_r<< 0.223,0.173,-0.207; 

    r_start_r=hand0_r.r_right_palm;
    r_middle_r<<-0.005,-0.058,-0.344  ; ///lift up
    r_target_r<<0.226,0.017,-0.31;
    R_target_r=hand_func.rot(3,-90*M_PI/180,3)*hand_func.rot(1,90*M_PI/180,3);

    r_middle_r<< 0.238,-0.112,-0.386;
    r_target_r<< 0.358,0.196,-0.00241;

    r_middle_r<< 0.4,-0.2,-0.05; 
    r_target_r<< 0.27,-0.15,0.37;

    r_middle_r<<.4, -.2, - 0.05;
    r_target_r<<.27, - 0.15, 0.37;
    R_target_r=hand_func.rot(2,-90*M_PI/180,3)*hand_func.rot(1,30*M_PI/180,3)*hand_func.rot(3,90*M_PI/180,3)*hand_func.rot(2,-45*M_PI/180,3);

    r_middle_r<< .45,-0.00,-0.29; //bringup
    r_target_r<< .45,-0.00,-0.22;
    R_target_r=hand_func.rot(2,-45*M_PI/180,3);

    r_middle_r<<.3,-0.1,-0.4;
    r_target_r<<.4, -0.05,-0.4;
    R_target_r=hand_func.rot(2,-65*M_PI/180,3);

    r_middle_r<<.4,-0.1,  -0.4;
    r_target_r<<.35,0, 0.10;   ////byebye
    R_target_r=hand_func.rot(2,-180*M_PI/180,3)*hand_func.rot(3,90*M_PI/180,3);                
*/

    // achive goal in 4 sec time_r = 0:0.005:4  
    while (time_r<t_r(2))
    {
    
    // define minJerk elements to calculate end effector velocity
    P_x_r<< r_start_r(0),r_middle_r(0),r_target_r(0);
    P_y_r<< r_start_r(1),r_middle_r(1),r_target_r(1);
    P_z_r<< r_start_r(2),r_middle_r(2),r_target_r(2);
    /*
    P_x_r<< r_middle_r(0),r_middle_r(0),r_target_r(0);
    P_y_r<< r_middle_r(1),r_middle_r(1),r_target_r(1);
    P_z_r<< r_middle_r(2),r_middle_r(2),r_target_r(2);*/

    V_x_r<<0,INFINITY,0;
    V_y_r<<0,INFINITY,0;
    V_z_r<<0,INFINITY,0;
    A_x_r<<0,INFINITY,0;
    A_y_r<<0,INFINITY,0;
    A_z_r<<0,INFINITY,0;

    X_coef_r=coef_generator.Coefficient(t_r,P_x_r,V_x_r,A_x_r); 
    Y_coef_r=coef_generator.Coefficient(t_r,P_y_r,V_y_r,A_y_r);
    Z_coef_r=coef_generator.Coefficient(t_r,P_z_r,V_z_r,A_z_r);

   if(time_r<t_r(1)&& time_r>=t_r(0))
   {
    P_r<<   coef_generator.GetAccVelPos(X_coef_r.row(0),time_r,t_r(0),5)(0,0),
            coef_generator.GetAccVelPos(Y_coef_r.row(0),time_r,t_r(0),5)(0,0),
            coef_generator.GetAccVelPos(Z_coef_r.row(0),time_r,t_r(0),5)(0,0);
    V_r<<   coef_generator.GetAccVelPos(X_coef_r.row(0),time_r,t_r(0),5)(0,1),
            coef_generator.GetAccVelPos(Y_coef_r.row(0),time_r,t_r(0),5)(0,1),
            coef_generator.GetAccVelPos(Z_coef_r.row(0),time_r,t_r(0),5)(0,1);

    hand_r.update_right_hand(qr_cyc,V_r,r_middle_r,R_target_r);
    r_right_palm=hand_r.r_right_palm;
    hand_r.doQP(qr_cyc);
    qr_cyc=hand_r.q_next;
   }

    else if(time_r<t_r(2)&& time_r>=t_r(1))
    {
    P_r<<   coef_generator.GetAccVelPos(X_coef_r.row(1),time_r,t_r(1),5)(0,0),
            coef_generator.GetAccVelPos(Y_coef_r.row(1),time_r,t_r(1),5)(0,0),
            coef_generator.GetAccVelPos(Z_coef_r.row(1),time_r,t_r(1),5)(0,0);
    V_r<<   coef_generator.GetAccVelPos(X_coef_r.row(1),time_r,t_r(1),5)(0,1),
            coef_generator.GetAccVelPos(Y_coef_r.row(1),time_r,t_r(1),5)(0,1),
            coef_generator.GetAccVelPos(Z_coef_r.row(1),time_r,t_r(1),5)(0,1);

    hand_r.update_right_hand(qr_cyc,V_r,r_target_r,R_target_r);
    r_right_palm=hand_r.r_right_palm;
    hand_r.doQP(qr_cyc);
    qr_cyc=hand_r.q_next;
   }
     
    q_end=qr_cyc;
    qref.block(0,count,7,1)=q_end;

    count++;
    time_r=(count)*.005;

    // pybullet
     ofstream fw("/home/surena/DynCont/SurenaVWalkTest/src/hand_planner_test/src/surena5/errorDebug.txt", std::ofstream::out);
    if (fw.is_open())
    {
    for (int i = 0; i < M; i++) {
        for (int j = 0; j <7; j++){
        fw << qref(j,i) << "\n";
    }
    }
    fw.close();
    }
 
  };    
     return 0 ;
    }

//g++ target_robot_test_RH.cpp MinimumJerkInterpolation.cpp S5mod_right_hand.cpp