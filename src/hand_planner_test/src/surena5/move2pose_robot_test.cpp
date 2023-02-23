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
using namespace  std;
using namespace  Eigen;

bool simulation = false;
bool lefthand = false;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "handnode");
    ros::NodeHandle nh;
    ros::Publisher  trajectory_data_pub  = nh.advertise<std_msgs::Int32MultiArray>("jointdata/qc",100);
    std_msgs::Int32MultiArray trajectory_data;
    ros::Rate loop_rate(200);

    right_hand hand_func;

    VectorXd q_init;
    q_init.resize(7,1);
    q_init<<0,0,0,0,0,0,0;
    vector<double> qr_cyc(7);
    vector<double> q_motor(29,0);
    vector<double> q_gazebo(29,0);
    int qc_offset[29]={0}; // all joint offset calc

    VectorXd result_right(2);
    VectorXd result_left(2);

    VectorXd q_goal1;
    q_goal1.resize(7,1);
    VectorXd q_goal2;
    q_goal2.resize(7,1);

    // out dir and up dir are positive
    // all joint must be positive to have out and up direction
    q_goal1<<0.3,0.3,0.3,0.3,0.0,0.0,0.0;
    
    //q_goal2<<0.0,0.0,0.0,0.0,0.0,0.0,0.0;

    double time_r=0;
    double n=0;
    int count = 0; 


    while (ros::ok()){

    while(time_r<5){

    if (time_r<5){
        for (int i = 0; i < 7; ++i) {
            qr_cyc[i]=q_init(i)+hand_func.move2pose(q_goal1(i)-q_init(i),time_r,0,5);
        };
    }
        /*
    if(time_r<4){
    
    qr_cyc[3]=qr_cyc[3]-0.5*M_PI/180*sin((time_r)/2*(2*M_PI));
    }
    n++;
    count++;
    time_r=(count)*.005;
    */
   /*
    else{
        for (int i = 0; i < 7; ++i) {
            qr_cyc[i]=q_goal1(i)+hand_func.move2pose(q_goal2(i)-q_goal1(i),time_r,4,8);
        };
    }
    */
    n++;
    count++;
    time_r=(count)*.005;
    
    // joint 1-12(0-11) upperbody, 13-15(12-14) waist, 16-22(15-21) right hand, 23-29(22-28) left hand
    if (simulation) {
        if(lefthand){
            q_gazebo[22]=-qr_cyc[0];  
            q_gazebo[23]=qr_cyc[1];   
            q_gazebo[24]=qr_cyc[2];  
            q_gazebo[25]=-qr_cyc[3];   
            q_gazebo[26]=qr_cyc[4];  
            q_gazebo[27]=qr_cyc[5];   
            q_gazebo[28]=-qr_cyc[6];
            hand_func.SendGazebo(q_gazebo); 
        }
        else{
            q_gazebo[15]=-qr_cyc[0];  
            q_gazebo[16]=-qr_cyc[1];   
            q_gazebo[17]=-qr_cyc[2];  
            q_gazebo[18]=-qr_cyc[3];   
            q_gazebo[19]=-qr_cyc[4];  
            q_gazebo[20]=-qr_cyc[5];   
            q_gazebo[21]=-qr_cyc[6];
            hand_func.SendGazebo(q_gazebo); 

        }

    }

    else {

        result_right = hand_func.wrist_right_calc(qr_cyc[5], qr_cyc[6]);
        result_left = hand_func.wrist_left_calc(qr_cyc[5], qr_cyc[6]);
        if (lefthand){
            q_motor[16]=int(qr_cyc[0]*4096*4*100/M_PI/2);   // be samt jelo
            q_motor[17]=-int(qr_cyc[1]*4096*4*100/M_PI/2);  // be samte birun
            q_motor[18]=int(qr_cyc[2]*2048*4*100/M_PI/2);   // be samte birun
            q_motor[19]=-int(qr_cyc[3]*2048*4*4*100/M_PI/2);// be samte bala
            /*
            q_motor[22]=0;
            q_motor[23]=0;
            q_motor[24]=0;
            q_motor[25]=0;
            q_motor[26]=0;
            q_motor[27]=0;
            q_motor[28]=0;
            */
        }
        else{
            q_motor[12]=-int(qr_cyc[0]*4096*4*100/M_PI/2); // be samt jelo
            q_motor[13]=int(qr_cyc[1]*4096*4*100/M_PI/2);  // be samte birun
            q_motor[14]=-int(qr_cyc[2]*2048*4*100/M_PI/2); // be samte birun
            q_motor[15]=int(qr_cyc[3]*2048*4*4*100/M_PI/2);// be samte bala
        //  q_motor[19]=int((qr_cyc[4])*(2048)/M_PI);
        //  q_motor[20]=result_right[1];   // wrist joints
        //  q_motor[21]=result_left[1];
            // cout<<q_motor[12]<<','<<q_motor[13]<<','<<q_motor[14]<<','<<q_motor[15]<<endl;
        }
        trajectory_data.data.clear();

        for(int  i = 0; i < 29; i++)
        {
            //cout<<"ok";
            trajectory_data.data.push_back(q_motor[i]+qc_offset[i]);
        }
        trajectory_data_pub.publish(trajectory_data);
    };
 
    ros::spinOnce();
    loop_rate.sleep();

  };   
    };
return 0 ;
};