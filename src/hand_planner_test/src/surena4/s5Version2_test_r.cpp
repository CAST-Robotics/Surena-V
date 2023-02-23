// dast rast ro be surat sini gereftan miare bala 
#include "S5mod_right_hand.h"
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
#include "ros/ros.h"
#include<std_msgs/Float32MultiArray.h>
#include<std_msgs/Int32MultiArray.h>
#include<std_msgs/Float64.h>
#include<gazebo_msgs/LinkStates.h>
#include"MinimumJerkInterpolation.h"

using namespace  std;
using namespace  Eigen;
void  SendGazebo(vector<double> q){
    ros::NodeHandle nh;
    
    ros::Publisher pub1  ;ros::Publisher pub2  ;ros::Publisher pub3  ;ros::Publisher pub4  ;
    ros::Publisher pub5  ;ros::Publisher pub6  ;ros::Publisher pub7  ;ros::Publisher pub8  ;
    ros::Publisher pub9  ;ros::Publisher pub10 ;ros::Publisher pub11 ;ros::Publisher pub12 ;
    ros::Publisher pub13 ;ros::Publisher pub14 ;ros::Publisher pub15 ;ros::Publisher pub16 ;
    ros::Publisher pub17 ;ros::Publisher pub18 ;ros::Publisher pub19 ;ros::Publisher pub20 ;
    ros::Publisher pub21 ;ros::Publisher pub22 ;ros::Publisher pub23 ;ros::Publisher pub24 ;
    ros::Publisher pub25 ;ros::Publisher pub26 ;ros::Publisher pub27 ;ros::Publisher pub28 ;
    ros::Publisher pub29 ;ros::Publisher pub30 ;ros::Publisher pub31 ;


    pub1  = nh.advertise<std_msgs::Float64>("rrbot/joint1_position_controller/command",100);
    pub2  = nh.advertise<std_msgs::Float64>("rrbot/joint2_position_controller/command",100);
    pub3  = nh.advertise<std_msgs::Float64>("rrbot/joint3_position_controller/command",100);
    pub4  = nh.advertise<std_msgs::Float64>("rrbot/joint4_position_controller/command",100);
    pub5  = nh.advertise<std_msgs::Float64>("rrbot/joint5_position_controller/command",100);
    pub6  = nh.advertise<std_msgs::Float64>("rrbot/joint6_position_controller/command",100);
    pub7  = nh.advertise<std_msgs::Float64>("rrbot/joint7_position_controller/command",100);
    pub8  = nh.advertise<std_msgs::Float64>("rrbot/joint8_position_controller/command",100);
    pub9  = nh.advertise<std_msgs::Float64>("rrbot/joint9_position_controller/command",100);
    pub10 = nh.advertise<std_msgs::Float64>("rrbot/joint10_position_controller/command",100);
    pub11 = nh.advertise<std_msgs::Float64>("rrbot/joint11_position_controller/command",100);
    pub12 = nh.advertise<std_msgs::Float64>("rrbot/joint12_position_controller/command",100);
    pub13 = nh.advertise<std_msgs::Float64>("rrbot/joint13_position_controller/command",100);
    pub14 = nh.advertise<std_msgs::Float64>("rrbot/joint14_position_controller/command",100);
    pub15 = nh.advertise<std_msgs::Float64>("rrbot/joint15_position_controller/command",100);
    pub16 = nh.advertise<std_msgs::Float64>("rrbot/joint16_position_controller/command",100);
    pub17 = nh.advertise<std_msgs::Float64>("rrbot/joint17_position_controller/command",100);
    pub18 = nh.advertise<std_msgs::Float64>("rrbot/joint18_position_controller/command",100);
    pub19 = nh.advertise<std_msgs::Float64>("rrbot/joint19_position_controller/command",100);
    pub20 = nh.advertise<std_msgs::Float64>("rrbot/joint20_position_controller/command",100);
    pub21 = nh.advertise<std_msgs::Float64>("rrbot/joint21_position_controller/command",100);
    pub22 = nh.advertise<std_msgs::Float64>("rrbot/joint22_position_controller/command",100);
    pub23 = nh.advertise<std_msgs::Float64>("rrbot/joint23_position_controller/command",100);
    pub24 = nh.advertise<std_msgs::Float64>("rrbot/joint24_position_controller/command",100);
    pub25 = nh.advertise<std_msgs::Float64>("rrbot/joint25_position_controller/command",100);
    pub26 = nh.advertise<std_msgs::Float64>("rrbot/joint26_position_controller/command",100);
    pub27 = nh.advertise<std_msgs::Float64>("rrbot/joint27_position_controller/command",100);
    pub28 = nh.advertise<std_msgs::Float64>("rrbot/joint28_position_controller/command",100);
    pub29 = nh.advertise<std_msgs::Float64>("rrbot/joint29_position_controller/command",100);
    pub30 = nh.advertise<std_msgs::Float64>("rrbot/joint30_position_controller/command",100);
    pub31 = nh.advertise<std_msgs::Float64>("rrbot/joint31_position_controller/command",100);


    std_msgs::Float64 data;
    data.data=q[0];
    pub1.publish(data);
    data.data=q[1];
    pub2.publish(data);
    data.data=q[2];
    pub3.publish(data);
    data.data=q[3];
    pub4.publish(data);
    data.data=q[4];
    pub5.publish(data);
    data.data=q[5];
    pub6.publish(data);
    data.data=q[6];
    pub7.publish(data);
    data.data=q[7];
    pub8.publish(data);
    data.data=q[8];
    pub9.publish(data);
    data.data=q[9];
    pub10.publish(data);
    data.data=q[10];
    pub11.publish(data);
    data.data=q[11];
    pub12.publish(data);
    data.data=q[12];
    pub13.publish(data);
    data.data=q[13];
    pub14.publish(data);
    data.data=q[14];
    pub15.publish(data);
    data.data=q[15];
    pub16.publish(data);
    data.data=q[16];
    pub17.publish(data);
    data.data=q[17];
    pub18.publish(data);
    data.data=q[18];
    pub19.publish(data);
    data.data=q[19];
    pub20.publish(data);
    data.data=q[20];
    pub21.publish(data);
    data.data=q[21];
    pub22.publish(data);
    data.data=q[22];
    pub23.publish(data);
    data.data=q[23];
    pub24.publish(data);
    data.data=q[24];
    pub25.publish(data);
    data.data=q[25];
    pub26.publish(data);
    data.data=q[26];
    pub27.publish(data);
    data.data=q[27];
    pub28.publish(data);
    data.data=q[28];
    pub29.publish(data);
    data.data=q[29];
    pub30.publish(data);
    data.data=q[30];
    pub31.publish(data);
    ROS_INFO("done!");
}
    int main(int argc, char **argv) {

    ros::init(argc, argv, "handnode");
    ros::NodeHandle nh;
    ros::Rate loop_rate(200);

    // define parameters for MinimumJerkInterpolation
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
    double time_r;
    time_r=count*.005;
    MatrixXd t_r(1,3);
    t_r<<0,2,4;
    int M = t_r(t_r.size()-1)/0.005;

    // define joint variables
    VectorXd q_ra(7);
    q_ra<<0,0,0,-10*M_PI/180,0,0,0; // initial condition
    VectorXd q; // set q for motor
    q.resize(7,1);
    VectorXd q_end;
    q_end.resize(7,1);
    MatrixXd qref;
    qref.resize(7,M);

    // define right_hand objs
    right_hand hand_funcs;
    right_hand hand_r;

    VectorXd r_right_palm(3);
    MatrixXd R_target_r;
    R_target_r.resize(3,3);
    VectorXd r_start_r(3); 
    VectorXd r_middle_r(3); 
    VectorXd r_target_r(3); 


    right_hand hand0_r( q_ra, r_target_r,  R_target_r,0,0);

    // set target values
    r_start_r=hand0_r.r_right_palm;

    r_middle_r<<-0.005,-0.058,-0.344  ; ///lift up
    r_target_r<<0.226,-0.017,-0.31;
    R_target_r=hand_funcs.rot(3,-90*M_PI/180,3)*hand_funcs.rot(1,90*M_PI/180,3);

    /*
    r_middle_r<< 0.343,0.003,0.084;
    r_target_r<< 0.223,0.173,-0.207; 

    r_middle_r<< 0.238,-0.112,-0.386;
    r_target_r<< 0.358,0.196,-0.00241;

    r_middle_r<< 0.4,-0.2,-0.05; 
    r_target_r<< 0.27,-0.15,0.37;

    r_middle_r<<.4, -.2, - 0.05;
    r_target_r<<.27, - 0.15, 0.37;
    R_target_r=hand_funcs.rot(2,-90*M_PI/180,3)*hand_funcs.rot(1,30*M_PI/180,3)*hand_funcs.rot(3,90*M_PI/180,3)*hand_funcs.rot(2,-45*M_PI/180,3);

    r_middle_r<< .45,-0.00,-0.29; //bringup
    r_target_r<< .45,-0.00,-0.22;
    R_target_r=hand_funcs.rot(2,-45*M_PI/180,3);

    r_middle_r<<.3,-0.1,-0.4;
    r_target_r<<.4, -0.05,-0.4;
    R_target_r=hand_funcs.rot(2,-65*M_PI/180,3);

    r_middle_r<<.4,-0.1,  -0.4;
    r_target_r<<.35,0, 0.10;   ////byebye
    R_target_r=hand_funcs.rot(2,-180*M_PI/180,3)*hand_funcs.rot(3,90*M_PI/180,3);                
*/

    // achive goal in 4 sec time_r = 0:0.05:4
    while (ros::ok())
    {
    while (time_r<4)
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

    X_coef_r=coef_generator.Coefficient(t_r,P_x_r,V_x_r,A_x_r); // X_coef_r size = 2 x 6 because t_r size is 2 and we need 6 coefficient. in fact row(0) are coef for first interval and row(1) are coef for second interval
    Y_coef_r=coef_generator.Coefficient(t_r,P_y_r,V_y_r,A_y_r);
    Z_coef_r=coef_generator.Coefficient(t_r,P_z_r,V_z_r,A_z_r);

   if(time_r<t_r(1)&& time_r>=t_r(0)){
       // position and velocity of end effector in first time interval in 3 dimention
       // GetAccVelPos output is X,V,A
    P_r<<   coef_generator.GetAccVelPos(X_coef_r.row(0),time_r,t_r(0),5)(0,0), // use first row of coefs for first intervals
            coef_generator.GetAccVelPos(Y_coef_r.row(0),time_r,t_r(0),5)(0,0),
            coef_generator.GetAccVelPos(Z_coef_r.row(0),time_r,t_r(0),5)(0,0);
    V_r<<   coef_generator.GetAccVelPos(X_coef_r.row(0),time_r,t_r(0),5)(0,1),
            coef_generator.GetAccVelPos(Y_coef_r.row(0),time_r,t_r(0),5)(0,1),
            coef_generator.GetAccVelPos(Z_coef_r.row(0),time_r,t_r(0),5)(0,1);
    // calculate position and orientation of hand ee according to q_ra
    // calculate dist and target eular angles
    // set V and W of ee
    // calculate J and J_W 
    hand_r.update_right_hand(q_ra,V_r,r_middle_r,R_target_r);
    // set r_right_palm with its new value calculated from FK
    r_right_palm=hand_r.r_right_palm;
    // QP calculate best 7 next q for joints that satisfies constraint and leads to targets
    hand_r.doQP(q_ra);
    // q_ra is set to next 7 values of joint angles
    q_ra=hand_r.q_next;
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
   }
 
    q_end=q_ra;
    // q_ref contains joints values from beginning to end 
    qref.block(0,count,7,1)=q_end;

    count++;
    time_r=(count)*.005;

    vector<double> q_init(31);
    for (int i = 0; i < 31; ++i) {
        q_init[i]=0;
    }
    q_init[14]=q_end[0];   q_init[14+7]=0;
    q_init[15]=q_end[1];   q_init[15+7]=0;
    q_init[16]=q_end[2];   q_init[16+7]=0;
    q_init[17]=q_end[3];   q_init[17+7]=0;
    q_init[18]=q_end[4];   q_init[18+7]=0;
    q_init[29]=q_end[5];   q_init[19+7]=0;
    q_init[20]=q_end[6];   q_init[20+7]=0;

    SendGazebo(q_init);
    ros::spinOnce();
   
    loop_rate.sleep();

    // write q_ref in txt file
     ofstream fw("/home/cast/Projects/pybullet/liftup4.txt", std::ofstream::out);
    //check if file was successfully opened for writing
    if (fw.is_open())
    {
    //store array contents to text file
    for (int i = 0; i < M; i++) {
        for (int j = 0; j <7; j++){
        fw << qref(j,i) << "\n";
    }
    }
    fw.close();
    }

  };  
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
//g++ s5Version2_test_r.cpp MinimumJerkInterpolation.cpp S5mod_right_hand.cpp