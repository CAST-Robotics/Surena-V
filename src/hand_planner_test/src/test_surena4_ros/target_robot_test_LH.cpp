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
bool lefthand = true;

    int main(int argc, char **argv){

    ros::init(argc, argv, "handnode");
    ros::NodeHandle nh;
    ros::Publisher  trajectory_data_pub  = nh.advertise<std_msgs::Int32MultiArray>("jointdata/qc",100);
    std_msgs::Int32MultiArray trajectory_data;
    ros::Rate loop_rate(200);

    MinimumJerkInterpolation coef_generator;

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
    int id = 0;
    int count = 0;
    double T = 0.005; 
    double time_r;
    time_r=count*T;
    MatrixXd t_r(1,3);
    t_r<<0,2,4;
    int M = t_r(t_r.size()-1)/T;

    // define joint variables
    VectorXd q_la(7);
    q_la<<10*M_PI/180,10*M_PI/180,0,-25*M_PI/180,0,0,0; // initial condition

    MatrixXd qref(7,M);
    MatrixXd qref_deal(7,M);
    VectorXd q_end(7,1);
    vector<double> q_motor(29,0);
    vector<double> q_gazebo(29,0);
    VectorXd qr_initial(7);
    qr_initial = q_la;

    // define left_hand objs
    left_hand hand_func;
    left_hand hand_l;

    VectorXd r_left_palm(3);
    MatrixXd R_target_l(3,3);
    VectorXd r_start_l(3); 
    VectorXd r_middle_l(3); 
    VectorXd r_target_l(3);

    // set target values
    r_middle_l<<0.2,0.1,-0.35  ; //shakehands
    r_target_l<<0.3,0.05,-0.35;
    R_target_l=hand_func.rot(2,-65*M_PI/180,3);

    // r_middle_l<<0.3,-0.1,-0.3  ; //respect
    // r_target_l<<0.25,0.1,-0.35;
    // R_target_l=hand_func.rot(2,-70*M_PI/180,3)*hand_func.rot(1,50*M_PI/180,3);

    // r_middle_l<<0.35,-0.2,-0.15  ; //ByeBye
    // r_target_l<<0.3,-0.1,0.25;
    // R_target_l=hand_func.rot(2,-180*M_PI/180,3)*hand_func.rot(3,90*M_PI/180,3);

    left_hand hand0_l(q_la,r_target_l,R_target_l,0,0);

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
    P_l<<   coef_generator.GetAccVelPos(X_coef_l.row(0),time_r,t_r(0),5)(0,0),
            coef_generator.GetAccVelPos(Y_coef_l.row(0),time_r,t_r(0),5)(0,0),
            coef_generator.GetAccVelPos(Z_coef_l.row(0),time_r,t_r(0),5)(0,0);
    V_l<<   coef_generator.GetAccVelPos(X_coef_l.row(0),time_r,t_r(0),5)(0,1),
            coef_generator.GetAccVelPos(Y_coef_l.row(0),time_r,t_r(0),5)(0,1),
            coef_generator.GetAccVelPos(Z_coef_l.row(0),time_r,t_r(0),5)(0,1);

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
    d_l=hand_l.dist;
    theta_l=hand_l.theta;
    sai_l=hand_l.sai;
    phi_l=hand_l.phi;
   }
     
    q_end=q_la;
    qref.block(0,count,7,1)=q_end;

    count++;
    time_r=(count)*.005;
    // cout<<V_l(0)<<','<<V_l(1)<<','<<V_l(2)<<endl;
    };

    // pybullet
    std::ofstream myfile;
    myfile.open ("example1.csv");
    
    int k = 0;
    for (int i = 0; i < M; i++) {
        if(i==400){
            continue;
        }
        else{
            for (int j = 0; j <7; j++){
            qref_deal(j,k)=qref(j,i);
            myfile << qref_deal(j,i) << ",";
            }
        }
        k++;
        myfile <<endl;
    }
    myfile.close();

    while (ros::ok()){
        if(id < M-1) {  
            // gazebo
            if (simulation) {
                if(lefthand){
                    q_gazebo[22]=qref_deal(0,id)-qr_initial[0];  
                    q_gazebo[23]=qref_deal(1,id)-qr_initial[1];   
                    q_gazebo[24]=qref_deal(2,id)-qr_initial[2];  
                    q_gazebo[25]=qref_deal(3,id)-qr_initial[3];   
                    q_gazebo[26]=0;  
                    q_gazebo[27]=0;   
                    q_gazebo[28]=0;
                    hand_func.SendGazebo(q_gazebo); 
                    }
                else{
                    q_gazebo[15]=qref_deal(0,id)-qr_initial[0];  
                    q_gazebo[16]=qref_deal(1,id)-qr_initial[1];   
                    q_gazebo[17]=qref_deal(2,id)-qr_initial[2];  
                    q_gazebo[18]=qref_deal(3,id)-qr_initial[3];   
                    q_gazebo[19]=0;  
                    q_gazebo[20]=0;   
                    q_gazebo[21]=0;
                    // cout<<q_gazebo[15]<<','<<q_gazebo[16]<<','<<q_gazebo[17]<<','<<q_gazebo[18]<<','<<q_gazebo[19]<<','<<q_gazebo[20]<<','<<q_gazebo[21]<<endl;
                    hand_func.SendGazebo(q_gazebo);
                    }
                    }

            else{
                    // ROS
                if(lefthand){
                    q_motor[16]=-int((qref_deal(0,id)-qr_initial[0])*4096*4*100/M_PI/2);
                    q_motor[17]=int((qref_deal(1,id)-qr_initial[1])*4096*4*100/M_PI/2);
                    q_motor[18]=-int((qref_deal(2,id)-qr_initial[2])*2048*4*100/M_PI/2);
                    q_motor[19]=int((qref_deal(3,id)-qr_initial[3])*2048*4*4*100/M_PI/2);
                    cout<<q_motor[16]<<','<<q_motor[17]<<','<<q_motor[18]<<','<<q_motor[19]<<endl;
                }
                else{
                    q_motor[12]=int((qref_deal(0,id)-qr_initial[0])*4096*4*100/M_PI/2); // be samte jelo
                    q_motor[13]=-int((qref_deal(1,id)-qr_initial[1])*4096*4*100/M_PI/2);  // be samte birun
                    q_motor[14]=int((qref_deal(2,id)-qr_initial[2])*2048*4*100/M_PI/2); // be samte birun
                    q_motor[15]=-int((qref_deal(3,id)-qr_initial[3])*2048*4*4*100/M_PI/2);// be samte bala
                    // cout<<q_motor[12]<<','<<q_motor[13]<<','<<q_motor[14]<<','<<q_motor[15]<<endl;
                }
                    trajectory_data.data.clear();

                    for(int  i = 0; i < 20; i++)
                    {
                        trajectory_data.data.push_back(q_motor[i]);
                    }
                    trajectory_data_pub.publish(trajectory_data);    
                    ros::spinOnce();
                    loop_rate.sleep(); 
            }
        };
        id++;
    };  

     return 0 ;
    }

//g++ target_robot_test_LH.cpp MinimumJerkInterpolation.cpp S5mod_left_hand.cpp