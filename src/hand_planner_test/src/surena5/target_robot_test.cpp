// dast rast ro be surat sini gereftan miare bala 
#include "S5mod_right_hand.h"
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


    int main(int argc, char **argv) {
    ros::init(argc, argv, "handnode");
    ros::NodeHandle nh;
    ros::Publisher  trajectory_data_pub  = nh.advertise<std_msgs::Int32MultiArray>("jointdata/qc",100);
    std_msgs::Int32MultiArray trajectory_data;
    ros::Rate loop_rate(200);

    MinimumJerkInterpolation coef_generator;
    int M = 800;
    MatrixXd qref;
    qref.resize(7,2*M);
    vector<double> q_gazebo(29,0);

// right
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
    
    VectorXd q_ra(7);
    q_ra<<0,0,0,-10*M_PI/180,0,0,0;
    VectorXd q_end_r;
    q_end_r.resize(7,1);

    right_hand hand_funcs;
    right_hand hand_r;

    VectorXd r_right_palm(3);
    MatrixXd R_target_r(3,3);
    
    MatrixXd qref_r;
    qref_r.resize(7,M);

    VectorXd r_target_r(3); 
    VectorXd r_middle_r(3); 
    VectorXd r_start_r(3); 

    right_hand hand0_r( q_ra, r_target_r,  R_target_r,0,0);
 
    r_start_r=hand0_r.r_right_palm;

    r_middle_r<<-0.005,-0.058,-0.344  ; ///lift up
    r_target_r<<0.226,0.017,-0.31;
    R_target_r=hand_funcs.rot(3,-90*M_PI/180,3)*hand_funcs.rot(1,90*M_PI/180,3);


// left
    MatrixXd P_x_l(1,3);      
    MatrixXd V_x_l(1,3);      
    MatrixXd A_x_l(1,3); 
    MatrixXd P_y_l(1,3);   
    MatrixXd V_y_l(1,3);      
    MatrixXd A_y_l(1,3);      
    MatrixXd P_z_l(1,3);      
    MatrixXd V_z_l(1,3);      
    MatrixXd A_z_l(1,3);

    MatrixXd X_coef_l;        
    MatrixXd Y_coef_l;        
    MatrixXd Z_coef_l; 
    VectorXd P_l(3); VectorXd V_l(3);
    
    VectorXd q_la(7);
    q_la<<0,0,0,-10*M_PI/180,0,0,0;
    MatrixXd qref_l;
    qref_l.resize(7,M);
    left_hand hand_l;
    VectorXd q_end_l;
    q_end_l.resize(7,1);

    VectorXd r_left_palm(3);
    MatrixXd R_target_l(3,3);    

    VectorXd r_target_l(3); 
    VectorXd r_middle_l(3); 
    VectorXd r_start_l(3); 

    left_hand hand0_l( q_ra, r_target_r,  R_target_r,0,0);
 
    r_start_l=hand0_l.r_left_palm;

    r_middle_l<<-0.005,0.058,-0.344  ; ///lift up
    r_target_l<<0.226,-0.017,-0.31;
    R_target_l=hand_funcs.rot(3,90*M_PI/180,3)*hand_funcs.rot(1,-90*M_PI/180,3);
    //R_target_l=hand_funcs.rot(2,-90*M_PI/180,3); // orientation for shakehands

    double time_r;
    MatrixXd t_r(1,3);
    int count = 0; 
    time_r=count*.005;

    t_r<<0,2,4;
    while (ros::ok())
    { 
    while (time_r<4)
    {
    
    P_x_r<< r_start_r(0),r_middle_r(0),r_target_r(0);
    P_y_r<< r_start_r(1),r_middle_r(1),r_target_r(1);
    P_z_r<< r_start_r(2),r_middle_r(2),r_target_r(2);
    
    P_x_l<< r_middle_l(0),r_middle_l(0),r_target_l(0);
    P_y_l<< r_middle_l(1),r_middle_l(1),r_target_l(1);
    P_z_l<< r_middle_l(2),r_middle_l(2),r_target_l(2);



    V_x_r<<0,INFINITY,0;
    V_y_r<<0,INFINITY,0;
    V_z_r<<0,INFINITY,0;
    A_x_r<<0,INFINITY,0;
    A_y_r<<0,INFINITY,0;
    A_z_r<<0,INFINITY,0;

    V_x_l<<0,INFINITY,0;
    V_y_l<<0,INFINITY,0;
    V_z_l<<0,INFINITY,0;
    A_x_l<<0,INFINITY,0;
    A_y_l<<0,INFINITY,0;
    A_z_l<<0,INFINITY,0;

    X_coef_r=coef_generator.Coefficient(t_r,P_x_r,V_x_r,A_x_r);
    Y_coef_r=coef_generator.Coefficient(t_r,P_y_r,V_y_r,A_y_r);
    Z_coef_r=coef_generator.Coefficient(t_r,P_z_r,V_z_r,A_z_r);

    X_coef_l=coef_generator.Coefficient(t_r,P_x_l,V_x_l,A_x_l);
    Y_coef_l=coef_generator.Coefficient(t_r,P_y_l,V_y_l,A_y_l);
    Z_coef_l=coef_generator.Coefficient(t_r,P_z_l,V_z_l,A_z_l);

   if(time_r<t_r(1)&& time_r>=t_r(0)){
    P_r<<   coef_generator.GetAccVelPos(X_coef_r.row(0),time_r,t_r(0),5)(0,0),
            coef_generator.GetAccVelPos(Y_coef_r.row(0),time_r,t_r(0),5)(0,0),
            coef_generator.GetAccVelPos(Z_coef_r.row(0),time_r,t_r(0),5)(0,0);
    V_r<<   coef_generator.GetAccVelPos(X_coef_r.row(0),time_r,t_r(0),5)(0,1),
            coef_generator.GetAccVelPos(Y_coef_r.row(0),time_r,t_r(0),5)(0,1),
            coef_generator.GetAccVelPos(Z_coef_r.row(0),time_r,t_r(0),5)(0,1);

    hand_r.update_right_hand(q_ra,V_r,r_middle_r,R_target_r);
    r_right_palm=hand_r.r_right_palm;
    hand_r.doQP(q_ra);
    q_ra=hand_r.q_next;

   //left
    P_l<<   coef_generator.GetAccVelPos(X_coef_l.row(0),time_r,t_r(0),5)(0,0),
            coef_generator.GetAccVelPos(Y_coef_l.row(0),time_r,t_r(0),5)(0,0),
            coef_generator.GetAccVelPos(Z_coef_l.row(0),time_r,t_r(0),5)(0,0);
    V_l<<   coef_generator.GetAccVelPos(X_coef_l.row(0),time_r,t_r(0),5)(0,1),
            coef_generator.GetAccVelPos(Y_coef_l.row(0),time_r,t_r(0),5)(0,1),
            coef_generator.GetAccVelPos(Z_coef_l.row(0),time_r,t_r(0),5)(0,1);

    hand_l.update_left_hand(q_la,V_l,r_middle_l,R_target_l);
    r_left_palm=hand_l.r_left_palm;
    hand_l.doQP(q_la);
    q_la=hand_l.q_next;

   }
    else if(time_r<t_r(2)&& time_r>=t_r(1)){
    P_r<<   coef_generator.GetAccVelPos(X_coef_r.row(1),time_r,t_r(1),5)(0,0),
            coef_generator.GetAccVelPos(Y_coef_r.row(1),time_r,t_r(1),5)(0,0),
            coef_generator.GetAccVelPos(Z_coef_r.row(1),time_r,t_r(1),5)(0,0);
    V_r<<   coef_generator.GetAccVelPos(X_coef_r.row(1),time_r,t_r(1),5)(0,1),
            coef_generator.GetAccVelPos(Y_coef_r.row(1),time_r,t_r(1),5)(0,1),
            coef_generator.GetAccVelPos(Z_coef_r.row(1),time_r,t_r(1),5)(0,1);

    hand_r.update_right_hand(q_ra,V_r,r_target_r,R_target_r);
    r_right_palm=hand_r.r_right_palm;
    hand_r.doQP(q_ra);
    q_ra=hand_r.q_next;


   //left
    P_l<<   coef_generator.GetAccVelPos(X_coef_l.row(1),time_r,t_r(1),5)(0,0),
            coef_generator.GetAccVelPos(Y_coef_l.row(1),time_r,t_r(1),5)(0,0),
            coef_generator.GetAccVelPos(Z_coef_l.row(1),time_r,t_r(1),5)(0,0);
    V_l<<   coef_generator.GetAccVelPos(X_coef_l.row(1),time_r,t_r(1),5)(0,1),
            coef_generator.GetAccVelPos(Y_coef_l.row(1),time_r,t_r(1),5)(0,1),
            coef_generator.GetAccVelPos(Z_coef_l.row(1),time_r,t_r(1),5)(0,1);

    hand_l.update_left_hand(q_la,V_l,r_target_l,R_target_l);
    r_left_palm=hand_l.r_left_palm;
    hand_l.doQP(q_la);
    q_la=hand_l.q_next;
   }
 
    q_end_r=q_ra; 
    q_end_l=q_la;
    qref_r.block(0,count,7,1)=q_end_r;
    qref_l.block(0,count,7,1)=q_end_l;

    count++;
    time_r=(count)*.005;


  };
    qref.block(0,0,7,M)=qref_r;
    qref.block(0,M,7,M)=qref_l;
    ofstream fw("/home/cast/Projects/pybullet/liftup_twohand.txt", std::ofstream::out);
    //check if file was successfully opened for writing
    if (fw.is_open())
    {
    //store array contents to text file
    for (int i = 0; i < 2*M; i++) {
        for (int j = 0; j <7; j++){
        fw << qref(j,i) << "\n";
    }
    }
    fw.close();
    }
    if (simulation) {
            q_gazebo[22]=q_la[0];  
            q_gazebo[23]=q_la[1];   
            q_gazebo[24]=q_la[2];  
            q_gazebo[25]=q_la[3];   
            q_gazebo[26]=q_la[4];  
            q_gazebo[27]=q_la[5];   
            q_gazebo[28]=q_la[6];
            q_gazebo[15]=q_ra[0];  
            q_gazebo[16]=q_ra[1];   
            q_gazebo[17]=q_ra[2];  
            q_gazebo[18]=q_ra[3];   
            q_gazebo[19]=q_ra[4];  
            q_gazebo[20]=q_ra[5];   
            q_gazebo[21]=q_ra[6];
            hand_funcs.SendGazebo(q_gazebo); 
    } 
    };  
     return 0 ;

    }

//g++ s5Version2_test_LR.cpp MinimumJerkInterpolation.cpp S5mod_right_hand.cpp S5mod_left_hand.cpp