#include "../Eigen/Dense"
#include "../Eigen/Geometry"
#include "../Eigen/QuadProg.h"
#include "../Eigen/testQPsolvers.hpp"
#include "../Eigen/eiquadprog.hpp"
#include "../Eigen/Core"
#include "../Eigen/Cholesky"
#include "../Eigen/LU"
#include <iostream>
#include <math.h>
#include <vector>
#include "fstream"
#include <string>
#include <geometry_msgs/PoseArray.h>
#include<std_msgs/Int32MultiArray.h>
#include "hand_planner_test/DetectionInfoArray.h"
#include "ros/ros.h"
using namespace std;
using namespace Eigen;


double PtoR = 0.0825;
double YtoP = 0.06025;
double h_pitch = 0;
double h_roll = 0;
double h_yaw = 0;
VectorXd camera(3);
VectorXd temp(3);
double Kp = 0.01;
double Ky = -0.01;
double theta_pitch; 
double sai_roll;
double phi_yaw;
double dist;
double y, z;
int obj_id;
double a = 0.5; // 0.55
double b = 0.4; // 0.385
double X0 = 0.5;
double X = 1;
double Y, Z, Y0, Z0, L0;
int L = 640; int W = 480;

void object_detect(const hand_planner_test::DetectionInfoArray & msg){
    if (msg.detections[0].class_id == 41 && msg.detections[0].distance != 0){
        dist = (msg.detections[0].distance)/1000;
        y = (msg.detections[0].x + (msg.detections[0].width)/2);
        z = (msg.detections[0].y + (msg.detections[0].height)/2);

        Y0 = -(y-L/2)/L*a;
        Z0 = -(z-W/2)/W*b;
        L0 = sqrt(pow(X0,2)+pow(Y0,2)+pow(Z0,2));

        X = X0*dist/L0;
        Y = Y0*dist/L0;
        Z = Z0*dist/L0;
        temp<<X,Y,Z;
        //cout<<"X: "<<X<<", Y: "<<Y<<", Z: "<<Z<<endl;
        }
    else{
        X = temp(0);
        Y = temp(1);
        Z = temp(2);
    }
    cout<<"X: "<<X<<", Y: "<<Y<<", Z: "<<Z<<endl;
            }

MatrixXd ObjToNeck(VectorXd camera, double h_pitch, double h_roll, double h_yaw, double PtoR, double YtoP) {
    MatrixXd T0(4, 4);
    T0 << cos(M_PI / 9), 0, sin(M_PI / 9), 0,
        0, 1, 0, 0,
        -sin(M_PI / 9), 0, cos(M_PI / 9), 0,
        0, 0, 0, 1;

    MatrixXd T1(4, 4);
    T1 << 1, 0, 0, camera(0),
        0, 1, 0, camera(1),
        0, 0, 1, camera(2),
        0, 0, 0, 1;

    MatrixXd T2(4, 4);
    T2 << 1, 0, 0, 0,
        0, cos(h_roll), -sin(h_roll), 0,
        0, sin(h_roll), cos(h_roll), PtoR,
        0, 0, 0, 1;

    MatrixXd T3(4, 4);
    T3 << cos(h_pitch), 0, sin(h_pitch), 0,
        0, 1, 0, 0,
        -sin(h_pitch), 0, cos(h_pitch), 0,
        0, 0, 0, 1;

    MatrixXd T4(4, 4);
    T4 << cos(h_yaw), -sin(h_yaw), 0, 0,
        sin(h_yaw), cos(h_yaw), 0, 0,
        0, 0, 1, YtoP,
        0, 0, 0, 1;

    MatrixXd T_EEtobase(4, 4);
    T_EEtobase = T4 * T3 * T2 * T1 * T0;
    return T_EEtobase;
}

MatrixXd returnAngles(MatrixXd T_EEtobase) {
    double theta_pitch;
    double sai_roll;
    double phi_yaw;
    MatrixXd output(3,1);

    if (T_EEtobase(2, 0) != 1 && T_EEtobase(2, 0) != -1)
    {
        theta_pitch = -asin(T_EEtobase(2, 0));
        sai_roll = atan2(T_EEtobase(2, 1) / cos(theta_pitch), T_EEtobase(2, 2) / cos(theta_pitch));
        phi_yaw = atan2(T_EEtobase(1, 0) / cos(theta_pitch), T_EEtobase(0, 0) / cos(theta_pitch));
    }
    else{
            phi_yaw = 0;
            if (T_EEtobase(2, 0) != -1)
            {
                theta_pitch = M_PI / 2;
                sai_roll = atan2(T_EEtobase(0, 1), T_EEtobase(0, 2));
            }
            else{
                theta_pitch = -M_PI / 2;
                sai_roll = atan2(-T_EEtobase(0, 1), -T_EEtobase(0, 2));
                }
    }
    output<< phi_yaw,sai_roll,theta_pitch;
    return output;
}

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "head_node");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/detection_info", 1, object_detect);
    ros::Publisher  trajectory_data_pub  = nh.advertise<std_msgs::Int32MultiArray>("jointdata/qc",100);
    std_msgs::Int32MultiArray trajectory_data;
    ros::Rate rate_(200);

    // MatrixXd T_EEtobase(4, 4);
    // camera << 0.1248, 0, 0.06746;
    // T_EEtobase << ObjToNeck(camera, h_pitch, h_roll, h_yaw, PtoR, YtoP);
    // temp << 0.35, -0.05, -0.2;
    double time = 0;
    temp(0) = 1;
    vector<int> pitch_range = {-30, 30};
    vector<int> roll_range = {-50, 50};
    vector<int> yaw_range = {-90, 90};
    vector<int> pitch_command_range = {180, 110};
    vector<int> roll_command_range = {100, 190};
    vector<int> yaw_command_range = {90, 210};
    vector<double> head_command(23,0);

    ofstream testYaw;
    testYaw.open("/home/surenav/DynCont/Code/WalkTest/src/hand_planner_test/src/testYaw.txt", std::ofstream::out);
    ofstream testPitch;
    testPitch.open("/home/surenav/DynCont/Code/WalkTest/src/hand_planner_test/src/testPitch.txt", std::ofstream::out);
    
    while (time < 10) {

        // if (abs(Y) > 0.02) {

        //     if (abs(h_yaw) < abs(atan2(Y,X))) {
        //         h_yaw += Ky*atan2(Y,X);
        //     }
        //     else {
        //         h_yaw = -atan2(Y,X);
        //     }
        //     if (abs(h_yaw)*180/M_PI>90){
        //         if (h_yaw > 0) {
        //             h_yaw = 90*M_PI/180;
        //         }
        //         else{
        //             h_yaw = -90*M_PI/180;
        //             }
        //     }
        //     cout<<"h_yaw: "<<h_yaw<<endl;
        // }
        

        // if (abs(Z) > 0.03) {
            
        //     if (abs(h_pitch) < abs(atan2(Z,sqrt(pow(Y,2)+pow(X,2))))) {
        //         h_pitch += Kp*atan2(Z,sqrt(pow(Y,2)+pow(X,2)));
        //     }
        //     else {
        //         h_pitch = atan2(Z,sqrt(pow(Y,2)+pow(X,2)));
        //     }
        //     if (abs(h_pitch)*180/M_PI>25){
        //         if (h_pitch > 0) {
        //             h_pitch = 25*M_PI/180;
        //         }
        //         else{
        //             h_pitch = -25*M_PI/180;
        //             }
        //     }
        //     cout<<"h_pitch: "<<h_pitch<<endl; 
        // }

        testYaw <<"time: "<<time<<"/ X: "<<X<<"/ Y: "<<Y<<"/ h_yaw: "<<h_yaw*180/M_PI<<"/ head_command: "<<head_command[22]<<endl;
        testPitch <<"time: "<<time<<"/ X: "<<X<<"/ Z: "<<Z<<"/ h_pitch: "<<h_pitch*180/M_PI<<"/ head_command: "<<head_command[21]<<endl;

        // if (time < 5) {h_pitch -= 0.01*M_PI/180;}
        // else {h_pitch += 0.01*M_PI/180;}
        // h_pitch -= 0.01*M_PI/180; // for 10 sec
        h_yaw -= 0.01*M_PI/180; // for 10 sec

        head_command[21] = int(pitch_command_range[0] + (pitch_command_range[1] - pitch_command_range[0]) * ((-(h_pitch*180/M_PI) - pitch_range[0]) / (pitch_range[1] - pitch_range[0])));
        head_command[20] = int(roll_command_range[0] + (roll_command_range[1] - roll_command_range[0]) * ((-(h_roll*180/M_PI) - (roll_range[0])) / (roll_range[1] - (roll_range[0]))));
        head_command[22] = int(yaw_command_range[0] + (yaw_command_range[1] - yaw_command_range[0]) * ((-(h_yaw*180/M_PI) - yaw_range[0]) / (yaw_range[1] - yaw_range[0])));

        trajectory_data.data.clear();
        for(int  i = 0; i < 23; i++)
        {
            trajectory_data.data.push_back(head_command[i]);
        }
        trajectory_data_pub.publish(trajectory_data);

        time += 0.005;
        ros::spinOnce();
        rate_.sleep();

    }
    testYaw.close();
    testPitch.close();
    return 0;
}