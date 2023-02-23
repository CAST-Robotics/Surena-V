// dast rast ro be surat sini gereftan miare bala 
#include "S5mod_left_hand.h"
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
//#include "ros/ros.h"
//#include<std_msgs/Float32MultiArray.h>
//#include<std_msgs/Int32MultiArray.h>
#include"MinimumJerkInterpolation.h"

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
    
    VectorXd q_la(7);
    q_la<<0,0,0,-10*M_PI/180,0,0,0;
    VectorXd q;
    q.resize(7,1);
    VectorXd q_end;
    q_end.resize(7,1);

    left_hand hand_funcs;
    left_hand hand_r;

    VectorXd r_left_palm(3);
    MatrixXd R_target_l;
    R_target_l.resize(3,3);
    //R_target_l=hand_funcs.rot(2,-45*M_PI/180,3);

    
    //R_target_l.fill(0);

    MatrixXd qref;
    qref.resize(7,800);

    double time_r;
    MatrixXd t_r(1,3);
    int count = 0; 
    time_r=count*.005;

    t_r<<0,2,4;
    VectorXd r_target_l(3); 
    VectorXd r_middle_l(3); 
    VectorXd r_start_l(3); 


    left_hand hand0_r( q_la, r_target_l,  R_target_l,0,0);
 

    r_start_l=hand0_r.r_left_palm;
    /*
    r_middle_l<< 0.343,0.003,0.084;
    r_target_l<< 0.223,0.173,-0.207; 
    r_middle_l<< 0.238,-0.112,-0.386;
    r_target_l<< 0.358,0.196,-0.00241;

    r_target_l<< 0.27,-0.15,0.37;
    r_middle_l<< 0.4,-0.2,-0.05; 

    r_middle_l<<.4, -.2, - 0.05;
    r_target_l<<.27, - 0.15, 0.37;
    R_target_l=hand_funcs.rot(2,-90*M_PI/180,3)*hand_funcs.rot(1,30*M_PI/180,3)*hand_funcs.rot(3,90*M_PI/180,3)*hand_funcs.rot(2,-45*M_PI/180,3);

    r_middle_l<< .45,-0.00,-0.29; //bringup
    r_target_l<< .45,-0.00,-0.22;


    R_target_l=hand_funcs.rot(2,-45*M_PI/180,3);

r_target_l<<.4, -0.05,-0.4;
                    R_target_l=hand_funcs.rot(2,-65*M_PI/180,3);
                    r_middle_l<<.3,
                            -0.1,
                            -0.4;
                
*/
    r_middle_l<<-0.005,0.038,-0.344  ; ///lift up
    r_target_l<<0.226,0.017,-0.31;
    //R_target_l=hand_funcs.rot(2,-90*M_PI/180,3); // orientation for shakehands
    R_target_l=hand_funcs.rot(2,-90*M_PI/180,3)*hand_funcs.rot(3,60*M_PI/180,3); // liftup orientation
/*
    r_target_l<<.35,0, 0.10;   ////byebye
    r_middle_l<<.4,-0.1,  -0.4;
    R_target_l=hand_funcs.rot(2,-180*M_PI/180,3)*hand_funcs.rot(3,90*M_PI/180,3);
    */

    while (time_r<4)
    {
    
    P_x_r<< r_start_l(0),r_middle_l(0),r_target_l(0);
    P_y_r<< r_start_l(1),r_middle_l(1),r_target_l(1);
    P_z_r<< r_start_l(2),r_middle_l(2),r_target_l(2);
    /*
    P_x_r<< r_middle_l(0),r_middle_l(0),r_target_l(0);
    P_y_r<< r_middle_l(1),r_middle_l(1),r_target_l(1);
    P_z_r<< r_middle_l(2),r_middle_l(2),r_target_l(2);*/



    V_x_r<<0,INFINITY,0;
    V_y_r<<0,INFINITY,0;
    V_z_r<<0,INFINITY,0;
    A_x_r<<0,INFINITY,0;
    A_y_r<<0,INFINITY,0;
    A_z_r<<0,INFINITY,0;

    X_coef_r=coef_generator.Coefficient(t_r,P_x_r,V_x_r,A_x_r);
    Y_coef_r=coef_generator.Coefficient(t_r,P_y_r,V_y_r,A_y_r);
    Z_coef_r=coef_generator.Coefficient(t_r,P_z_r,V_z_r,A_z_r);

   if(time_r<t_r(1)&& time_r>=t_r(0)){
    P_r<<   coef_generator.GetAccVelPos(X_coef_r.row(0),time_r,t_r(0),5)(0,0),
            coef_generator.GetAccVelPos(Y_coef_r.row(0),time_r,t_r(0),5)(0,0),
            coef_generator.GetAccVelPos(Z_coef_r.row(0),time_r,t_r(0),5)(0,0);
    V_r<<   coef_generator.GetAccVelPos(X_coef_r.row(0),time_r,t_r(0),5)(0,1),
            coef_generator.GetAccVelPos(Y_coef_r.row(0),time_r,t_r(0),5)(0,1),
            coef_generator.GetAccVelPos(Z_coef_r.row(0),time_r,t_r(0),5)(0,1);

    hand_r.update_left_hand(q_la,V_r,r_middle_l,R_target_l);
    r_left_palm=hand_r.r_left_palm;
    hand_r.doQP(q_la);
    q_la=hand_r.q_next;

   }
    else if(time_r<t_r(2)&& time_r>=t_r(1)){
    P_r<<   coef_generator.GetAccVelPos(X_coef_r.row(1),time_r,t_r(1),5)(0,0),
            coef_generator.GetAccVelPos(Y_coef_r.row(1),time_r,t_r(1),5)(0,0),
            coef_generator.GetAccVelPos(Z_coef_r.row(1),time_r,t_r(1),5)(0,0);
    V_r<<   coef_generator.GetAccVelPos(X_coef_r.row(1),time_r,t_r(1),5)(0,1),
            coef_generator.GetAccVelPos(Y_coef_r.row(1),time_r,t_r(1),5)(0,1),
            coef_generator.GetAccVelPos(Z_coef_r.row(1),time_r,t_r(1),5)(0,1);

    hand_r.update_left_hand(q_la,V_r,r_target_l,R_target_l);
    r_left_palm=hand_r.r_left_palm;
    hand_r.doQP(q_la);
    q_la=hand_r.q_next;

   }
 
    q_end=q_la; 
    qref.block(0,count,7,1)=q_end;
  

    count++;
    time_r=(count)*.005;


    
    q[0]=q_la(0);
    q[1]=q_la(1);
    q[2]=q_la(2);
    q[3]=q_la(3);
    q[4]=q_la(4);
    q[5]=q_la(5);
    q[6]=q_la(6);

     ofstream fw("/home/cast/Projects/pybullet/liftup_left.txt", std::ofstream::out);
    //check if file was successfully opened for writing
    if (fw.is_open())
    {
    //store array contents to text file
    for (int i = 0; i < 800; i++) {
        for (int j = 0; j <7; j++){
        fw << qref(j,i) << "\n";
    }
    }
    fw.close();
    }

  };   
     return 0 ;

    }
  /*
    trajectory_data.data.clear();
        for (int i = 0; i <= 7; ++i) {
            trajectory_data.data.push_back(q[i]);

        }
        trajectory_data_pub.publish(trajectory_data);

        ros::spinOnce();
        loop_rate.sleep();
        };
        };
        

   return 0 ;

  };   
*/
//g++ s5Version2_test_l.cpp MinimumJerkInterpolation.cpp S5mod_left_hand.cpp