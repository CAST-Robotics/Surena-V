#include "S5mod_left_hand.h"
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

bool simulation = true;
bool lefthand = true;

    int main(int argc, char **argv) {
    ros::init(argc, argv, "handnode");
    ros::NodeHandle nh;
    ros::Publisher  trajectory_data_pub  = nh.advertise<std_msgs::Int32MultiArray>("jointdata/qc",100);
    std_msgs::Int32MultiArray trajectory_data;
    ros::Rate loop_rate(200);

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
    t_r<<0,2,4;
    int M = t_r(t_r.size()-1)/T;

    // define joint variables
    VectorXd ql_cyc(7);
    ql_cyc<<0,0,0,-10*M_PI/180,0,0,0; // initial condition
    vector<double> q_motor(29,0);
    vector<double> q_gazebo(29,0);
    int qc_offset[29]={0}; // all joint offset calc
    MatrixXd qref(7,M);
    VectorXd q_end(7,1);

    // define left_hand objs
    left_hand hand_func;
    left_hand hand_l;

    VectorXd result_right(2);
    VectorXd result_left(2);

    VectorXd r_left_palm(3);
    MatrixXd R_target_l(3,3);
    VectorXd r_start_l(3);
    VectorXd r_middle_l(3);      
    VectorXd r_target_l(3); 

    left_hand hand0_l( ql_cyc, r_target_l,  R_target_l,0,0);

    // set target values
    r_start_l=hand0_l.r_left_palm;
    r_middle_l<<-0.005,0.058,-0.344  ; ///lift up
    r_target_l<<0.226,-0.017,-0.31;
    R_target_l=hand_func.rot(3,90*M_PI/180,3)*hand_func.rot(1,-90*M_PI/180,3);

    /*
    r_middle_l<<-0.005,0.038,-0.344  ; ///lift up
    r_target_l<<0.226,0.017,-0.31;
    R_target_l=hand_func.rot(2,-90*M_PI/180,3)*hand_func.rot(3,60*M_PI/180,3); // liftup orientation

    r_middle_l<< 0.343,0.003,0.084;
    r_target_l<< 0.223,0.173,-0.207; 
    r_middle_l<< 0.238,-0.112,-0.386;
    r_target_l<< 0.358,0.196,-0.00241;

    r_target_l<< 0.27,-0.15,0.37;
    r_middle_l<< 0.4,-0.2,-0.05; 

    r_middle_l<<.4, -.2, - 0.05;
    r_target_l<<.27, - 0.15, 0.37;
    R_target_l=hand_func.rot(2,-90*M_PI/180,3)*hand_func.rot(1,30*M_PI/180,3)*hand_func.rot(3,90*M_PI/180,3)*hand_func.rot(2,-45*M_PI/180,3);

    r_middle_l<< .45,-0.00,-0.29; //bringup
    r_target_l<< .45,-0.00,-0.22;

    R_target_l=hand_func.rot(2,-45*M_PI/180,3);

    r_target_l<<.4, -0.05,-0.4;
    R_target_l=hand_func.rot(2,-65*M_PI/180,3);
    r_middle_l<<.3, -0.1, -0.4;

    R_target_l=hand_func.rot(2,-90*M_PI/180,3); // orientation for shakehands

    r_target_l<<.35,0, 0.10;   //byebye
    r_middle_l<<.4,-0.1,  -0.4;
    R_target_l=hand_func.rot(2,-180*M_PI/180,3)*hand_func.rot(3,90*M_PI/180,3);
    */

    // achive goal in 4 sec time_r = 0:0.005:4
    while (ros::ok())
    { 
    while (time_r<4)
    {
 
    // define minJerk elements to calculate end effector velocity  
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

   if(time_r<t_r(1)&& time_r>=t_r(0))
   {
    P_r<<   coef_generator.GetAccVelPos(X_coef_r.row(0),time_r,t_r(0),5)(0,0),
            coef_generator.GetAccVelPos(Y_coef_r.row(0),time_r,t_r(0),5)(0,0),
            coef_generator.GetAccVelPos(Z_coef_r.row(0),time_r,t_r(0),5)(0,0);
    V_r<<   coef_generator.GetAccVelPos(X_coef_r.row(0),time_r,t_r(0),5)(0,1),
            coef_generator.GetAccVelPos(Y_coef_r.row(0),time_r,t_r(0),5)(0,1),
            coef_generator.GetAccVelPos(Z_coef_r.row(0),time_r,t_r(0),5)(0,1);

    hand_l.update_left_hand(ql_cyc,V_r,r_middle_l,R_target_l);
    r_left_palm=hand_l.r_left_palm;
    hand_l.doQP(ql_cyc);
    ql_cyc=hand_l.q_next;

   }
    else if(time_r<t_r(2)&& time_r>=t_r(1)){
    P_r<<   coef_generator.GetAccVelPos(X_coef_r.row(1),time_r,t_r(1),5)(0,0),
            coef_generator.GetAccVelPos(Y_coef_r.row(1),time_r,t_r(1),5)(0,0),
            coef_generator.GetAccVelPos(Z_coef_r.row(1),time_r,t_r(1),5)(0,0);
    V_r<<   coef_generator.GetAccVelPos(X_coef_r.row(1),time_r,t_r(1),5)(0,1),
            coef_generator.GetAccVelPos(Y_coef_r.row(1),time_r,t_r(1),5)(0,1),
            coef_generator.GetAccVelPos(Z_coef_r.row(1),time_r,t_r(1),5)(0,1);

    hand_l.update_left_hand(ql_cyc,V_r,r_target_l,R_target_l);
    r_left_palm=hand_l.r_left_palm;
    hand_l.doQP(ql_cyc);
    ql_cyc=hand_l.q_next;

   }
 
    q_end=ql_cyc; 
    qref.block(0,count,7,1)=q_end;
  

    count++;
    time_r=(count)*.005;

    // pybullet
     ofstream fw("/home/cast/Projects/pybullet/liftup_left.txt", std::ofstream::out);
    if (fw.is_open())
    {
    for (int i = 0; i < M; i++) {
        for (int j = 0; j <7; j++){
        fw << qref(j,i) << "\n";
    }
    }
    fw.close();
    }
    
    // gazebo/ros
    // joint 1-12(0-11) upperbody, 13-15(12-14) waist, 16-22(15-21) right hand, 23-29(22-28) left hand
    if (simulation) {
        if(lefthand){
            q_gazebo[22]=ql_cyc[0];  
            q_gazebo[23]=ql_cyc[1];   
            q_gazebo[24]=ql_cyc[2];  
            q_gazebo[25]=ql_cyc[3];   
            q_gazebo[26]=ql_cyc[4];  
            q_gazebo[27]=ql_cyc[5];   
            q_gazebo[28]=ql_cyc[6];
            hand_func.SendGazebo(q_gazebo); 
        }
        else{
            q_gazebo[15]=ql_cyc[0];  
            q_gazebo[16]=ql_cyc[1];   
            q_gazebo[17]=ql_cyc[2];  
            q_gazebo[18]=ql_cyc[3];   
            q_gazebo[19]=ql_cyc[4];  
            q_gazebo[20]=ql_cyc[5];   
            q_gazebo[21]=ql_cyc[6];
            hand_func.SendGazebo(q_gazebo); 

        }

    }

    else {

        result_right = hand_func.wrist_right_calc(ql_cyc[5], ql_cyc[6]);
        result_left = hand_func.wrist_left_calc(ql_cyc[5], ql_cyc[6]);
        if (lefthand){
            q_motor[22]=0;
            q_motor[23]=0;
            q_motor[24]=0;
            q_motor[25]=0;
            q_motor[26]=0;
            q_motor[27]=0;
            q_motor[28]=0;
        }
        else{
            q_motor[15]=-int(10*(ql_cyc[0])*180/M_PI*120/60);
            q_motor[16]=int(10*(ql_cyc[1])*180/M_PI*120/60);
            q_motor[17]=-int(7*(ql_cyc[2])*180/M_PI*100/60);
            q_motor[18]=int(7*(ql_cyc[3])*180/M_PI*100/60);
            q_motor[19]=int((ql_cyc[4])*(2048)/M_PI);
            q_motor[20]=result_right[1];   // wrist joints
            q_motor[21]=result_left[1];
        }
        trajectory_data.data.clear();

        for(int  i = 0; i < 15; i++)
        {
            trajectory_data.data.push_back(q_motor[i]+qc_offset[i]);
        }
        trajectory_data_pub.publish(trajectory_data);
    };
 
    ros::spinOnce();
    loop_rate.sleep();
  };
    };   
     return 0 ;

    }

//g++ target_robot_test_LH.cpp MinimumJerkInterpolation.cpp S5mod_left_hand.cpp