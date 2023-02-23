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

bool simulation = false;
bool lefthand = false;

    int main(int argc, char **argv){

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
    VectorXd P_r(3);
    VectorXd V_r(3);
    
    double d0_r;            
    double d_r ;             
    double d_des_r;          
    double theta_r;          
    double theta_target_r;   
    double sai_r;            
    double sai_target_r;    
    double phi_r;            
    double phi_target_r;     
    double v0_r=0;
    double v_target_r =.4;
    int id = 0;

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
    VectorXd P_l(3);
    VectorXd V_l(3);
    
    double d0_l;            
    double d_l ;             
    double d_des_l;          
    double theta_l;          
    double theta_target_l;   
    double sai_l;            
    double sai_target_l;    
    double phi_l;            
    double phi_target_l;     
    double v0_l=0;
    double v_target_l =.4;

    // define time parameters
    int count = 0;
    double T = 0.005; 
    double time_r;
    time_r=count*T;
    MatrixXd t_r(1,3);
    t_r<<0,2,4; //2.335
    int M = t_r(t_r.size()-1)/T;

    // define joint variables
    VectorXd qr_cyc(7);
    qr_cyc<<10*M_PI/180,-10*M_PI/180,0,-25*M_PI/180,0,0,0; // initial condition
    VectorXd q_la(7);
    q_la<<10*M_PI/180,10*M_PI/180,0,-25*M_PI/180,0,0,0; // initial condition
   
    MatrixXd qref_r(7,M);
    MatrixXd qref_r_deal(7,M);
    MatrixXd qref_l(7,M);
    MatrixXd qref_l_deal(7,M);

    vector<double> q_motor(29,0);
    vector<double> q_gazebo(29,0);
    VectorXd qr_initial(7);
    qr_initial = qr_cyc;
    VectorXd ql_initial(7);
    ql_initial = q_la;

    // define right_hand objs
    right_hand hand_func_r;
    right_hand hand_r;
    left_hand hand_func_l;
    left_hand hand_l;

    VectorXd r_right_palm(3);
    MatrixXd R_target_r(3,3);
    VectorXd r_start_r(3); 
    VectorXd r_middle_r(3); 
    VectorXd r_target_r(3);

    VectorXd r_left_palm(3);
    MatrixXd R_target_l(3,3);
    VectorXd r_start_l(3); 
    VectorXd r_middle_l(3); 
    VectorXd r_target_l(3);

    // set target values
    r_middle_r<<0.2,-0.1,-0.35  ; //shakehands right
    r_target_r<<0.3,-0.05,-0.35;
    R_target_r=hand_func_r.rot(2,-65*M_PI/180,3);

    r_middle_l<<0.2,0.1,-0.35  ; //shakehands left
    r_target_l<<0.3,0.05,-0.35;
    R_target_l=hand_func_l.rot(2,-65*M_PI/180,3);

    // r_middle_r<<0.3,-0.1,-0.3  ; //respect
    // r_target_r<<0.25,0.1,-0.35;
    // R_target_r=hand_func_r.rot(2,-70*M_PI/180,3)*hand_func_r.rot(1,50*M_PI/180,3);

    // r_middle_r<<0.35,-0.2,-0.15  ; //ByeBye
    // r_target_r<<0.3,-0.1,0.25;
    // R_target_r=hand_func_r.rot(2,-180*M_PI/180,3)*hand_func_r.rot(3,90*M_PI/180,3);

    right_hand hand0_r(qr_cyc,r_target_r,R_target_r,0,0);
    left_hand hand0_l(q_la,r_target_l,R_target_l,0,0);

    r_start_r=hand0_r.r_right_palm;
    d0_r=hand0_r.dist;
    d_r=d0_r;
    d_des_r=hand0_r.d_des;
    theta_r=hand0_r.theta;
    theta_target_r=hand0_r.theta_target;
    sai_r=hand0_r.sai; 
    sai_target_r=hand0_r.sai_target;
    phi_r=hand0_r.phi; 
    phi_target_r=hand0_r.phi_target; 
    hand0_r.HO_FK_right_palm(qr_cyc);

    r_start_l=hand0_l.r_left_palm;
    d0_l=hand0_l.dist;
    d_l=d0_l;
    d_des_l=hand0_l.d_des;
    theta_l=hand0_l.theta;
    theta_target_l=hand0_l.theta_target;
    sai_l=hand0_l.sai; 
    sai_target_l=hand0_l.sai_target;
    phi_l=hand0_l.phi; 
    phi_target_l=hand0_l.phi_target; 
    hand0_l.HO_FK_left_palm(q_la); 
      
    // define minJerk elements to calculate end effector velocity
    P_x_r<< r_start_r(0),r_middle_r(0),r_target_r(0);
    P_y_r<< r_start_r(1),r_middle_r(1),r_target_r(1);
    P_z_r<< r_start_r(2),r_middle_r(2),r_target_r(2);

    V_x_r<<0,INFINITY,0;
    V_y_r<<0,INFINITY,0;
    V_z_r<<0,INFINITY,0;
    A_x_r<<0,INFINITY,0;
    A_y_r<<0,INFINITY,0;
    A_z_r<<0,INFINITY,0;

    X_coef_r=coef_generator.Coefficient(t_r,P_x_r,V_x_r,A_x_r); 
    Y_coef_r=coef_generator.Coefficient(t_r,P_y_r,V_y_r,A_y_r);
    Z_coef_r=coef_generator.Coefficient(t_r,P_z_r,V_z_r,A_z_r);

    // define minJerk elements to calculate end effector velocity
    P_x_l<< r_start_l(0),r_middle_l(0),r_target_l(0);
    P_y_l<< r_start_l(1),r_middle_l(1),r_target_l(1);
    P_z_l<< r_start_l(2),r_middle_l(2),r_target_l(2);

    V_x_l<<0,INFINITY,0;
    V_y_l<<0,INFINITY,0;
    V_z_l<<0,INFINITY,0;
    A_x_l<<0,INFINITY,0;
    A_y_l<<0,INFINITY,0;
    A_z_l<<0,INFINITY,0;

    X_coef_l=coef_generator.Coefficient(t_r,P_x_l,V_x_l,A_x_l); 
    Y_coef_l=coef_generator.Coefficient(t_r,P_y_l,V_y_l,A_y_l);
    Z_coef_l=coef_generator.Coefficient(t_r,P_z_l,V_z_l,A_z_l);

    // achive goal in 4 sec time_r = 0:0.005:4
 
    while (time_r<t_r(2))
    {
    if(time_r<t_r(1)&& time_r>=t_r(0))
    {
    P_r<<   coef_generator.GetAccVelPos(X_coef_r.row(0),time_r,t_r(0),5)(0,0),
            coef_generator.GetAccVelPos(Y_coef_r.row(0),time_r,t_r(0),5)(0,0),
            coef_generator.GetAccVelPos(Z_coef_r.row(0),time_r,t_r(0),5)(0,0);
    V_r<<   coef_generator.GetAccVelPos(X_coef_r.row(0),time_r,t_r(0),5)(0,1),
            coef_generator.GetAccVelPos(Y_coef_r.row(0),time_r,t_r(0),5)(0,1),
            coef_generator.GetAccVelPos(Z_coef_r.row(0),time_r,t_r(0),5)(0,1);

    P_l<<   coef_generator.GetAccVelPos(X_coef_l.row(0),time_r,t_r(0),5)(0,0),
            coef_generator.GetAccVelPos(Y_coef_l.row(0),time_r,t_r(0),5)(0,0),
            coef_generator.GetAccVelPos(Z_coef_l.row(0),time_r,t_r(0),5)(0,0);
    V_l<<   coef_generator.GetAccVelPos(X_coef_l.row(0),time_r,t_r(0),5)(0,1),
            coef_generator.GetAccVelPos(Y_coef_l.row(0),time_r,t_r(0),5)(0,1),
            coef_generator.GetAccVelPos(Z_coef_l.row(0),time_r,t_r(0),5)(0,1);

    hand_r.update_right_hand(qr_cyc,V_r,r_target_r,R_target_r);
    r_right_palm=hand_r.r_right_palm;
    hand_r.doQP(qr_cyc);
    qr_cyc=hand_r.q_next;
    d_r=hand_r.dist;
    theta_r=hand_r.theta;
    sai_r=hand_r.sai;
    phi_r=hand_r.phi;

    hand_l.update_left_hand(q_la,V_l,r_target_l,R_target_l);
    r_left_palm=hand_l.r_left_palm;
    hand_l.doQP(q_la);
    q_la=hand_l.q_next;
    d_l=hand_l.dist;
    theta_l=hand_l.theta;
    sai_l=hand_l.sai;
    phi_l=hand_l.phi;

   }

    else if(time_r<t_r(2)&& time_r>t_r(1))
    {
    P_r<<   coef_generator.GetAccVelPos(X_coef_r.row(1),time_r,t_r(1),5)(0,0),
            coef_generator.GetAccVelPos(Y_coef_r.row(1),time_r,t_r(1),5)(0,0),
            coef_generator.GetAccVelPos(Z_coef_r.row(1),time_r,t_r(1),5)(0,0);
    V_r<<   coef_generator.GetAccVelPos(X_coef_r.row(1),time_r,t_r(1),5)(0,1),
            coef_generator.GetAccVelPos(Y_coef_r.row(1),time_r,t_r(1),5)(0,1),
            coef_generator.GetAccVelPos(Z_coef_r.row(1),time_r,t_r(1),5)(0,1);

    P_l<<   coef_generator.GetAccVelPos(X_coef_l.row(1),time_r,t_r(1),5)(0,0),
            coef_generator.GetAccVelPos(Y_coef_l.row(1),time_r,t_r(1),5)(0,0),
            coef_generator.GetAccVelPos(Z_coef_l.row(1),time_r,t_r(1),5)(0,0);
    V_l<<   coef_generator.GetAccVelPos(X_coef_l.row(1),time_r,t_r(1),5)(0,1),
            coef_generator.GetAccVelPos(Y_coef_l.row(1),time_r,t_r(1),5)(0,1),
            coef_generator.GetAccVelPos(Z_coef_l.row(1),time_r,t_r(1),5)(0,1);

    hand_r.update_right_hand(qr_cyc,V_r,r_target_r,R_target_r);
    r_right_palm=hand_r.r_right_palm;
    hand_r.doQP(qr_cyc);
    qr_cyc=hand_r.q_next;
    d_r=hand_r.dist;
    theta_r=hand_r.theta;
    sai_r=hand_r.sai;
    phi_r=hand_r.phi;

    hand_l.update_left_hand(q_la,V_l,r_target_l,R_target_l);
    r_left_palm=hand_l.r_left_palm;
    hand_l.doQP(q_la);
    q_la=hand_l.q_next;
    d_l=hand_l.dist;
    theta_l=hand_l.theta;
    sai_l=hand_l.sai;
    phi_l=hand_l.phi;

   }
     
    qref_r.block(0,count,7,1)=qr_cyc;
    qref_l.block(0,count,7,1)=q_la;

    count++;
    time_r=(count)*T;
    
    // cout<<V_r(0)<<','<<V_r(1)<<','<<V_r(2)<<endl;
    };
    // pybullet
    std::ofstream rightfile;
    rightfile.open ("exampleR.csv");
    
    int k = 0;
    for (int i = 0; i < M; i++) {
        // if(i==400){
        //     continue;
        // }
        // else{
            for (int j = 0; j <7; j++){
            qref_r_deal(j,k)=qref_r(j,i);

            rightfile << qref_r_deal(j,i) << ",";
            }
        // }
        k++;
        rightfile <<endl;
    }
    rightfile.close();

    std::ofstream leftfile;
    leftfile.open ("exampleL.csv");
    
    k = 0;

    for (int i = 0; i < M; i++) {
        // if(i==400){
        //     continue;
        // }
        // else{
            for (int j = 0; j <7; j++){
            qref_l_deal(j,k)=qref_l(j,i);
            leftfile << qref_l_deal(j,i) << ",";
            }
        // }
        k++;
        leftfile <<endl;
    }
    leftfile.close();
    
  int encoderResolution[2] = {4096*4, 2048*4};
  int harmonicRatio[4] = {100, 100, 100, 400};


while (ros::ok())
{
    if(id < M) {  
            
            q_motor[12]=int((qref_r_deal(0,id)-qr_initial[0])*encoderResolution[0]*harmonicRatio[0]/M_PI/2); // int((qref_r_deal(0,id)-qr_initial[0])*4096*4*100/M_PI/2)be samte jelo
            q_motor[13]=-int((qref_r_deal(1,id)-qr_initial[1])*encoderResolution[0]*harmonicRatio[1]/M_PI/2); //-int((qref_r_deal(1,id)-qr_initial[1])*4096*4*100/M_PI/2);  // be samte birun
            q_motor[14]=int((qref_r_deal(2,id)-qr_initial[2])*encoderResolution[1]*harmonicRatio[2]/M_PI/2); // be samte birun
            q_motor[15]=-int((qref_r_deal(3,id)-qr_initial[3])*encoderResolution[1]*harmonicRatio[3]/M_PI/2);// be samte bala
            
            q_motor[16]=-int((qref_l_deal(0,id)-ql_initial[0])*encoderResolution[0]*harmonicRatio[0]/M_PI/2);
            q_motor[17]=int((qref_l_deal(1,id)-ql_initial[1])*encoderResolution[0]*harmonicRatio[1]/M_PI/2);
            q_motor[18]=-int((qref_l_deal(2,id)-ql_initial[2])*encoderResolution[1]*harmonicRatio[2]/M_PI/2);
            q_motor[19]=int((qref_l_deal(3,id)-ql_initial[3])*encoderResolution[1]*harmonicRatio[3]/M_PI/2);
 
            
            cout<<q_motor[12]<<','<<q_motor[13]<<','<<q_motor[14]<<','<<q_motor[15]<<','<<q_motor[16]<<','<<q_motor[17]<<','<<q_motor[18]<<','<<q_motor[19]<<endl;
            trajectory_data.data.clear();
            for(int  i = 0; i < 20; i++)
            {
                trajectory_data.data.push_back(q_motor[i]);
            }
            trajectory_data_pub.publish(trajectory_data);    
            ros::spinOnce();
            loop_rate.sleep(); 

    };
    id++;
};  
       
     return 0 ;
    }

//g++ target_robot_test_RH.cpp MinimumJerkInterpolation.cpp S5mod_right_hand.cpp