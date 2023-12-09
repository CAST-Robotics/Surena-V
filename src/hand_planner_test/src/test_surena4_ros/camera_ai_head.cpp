#include "ros/ros.h"
#include <geometry_msgs/PoseArray.h>
#include <sstream>
#include "fstream"
#include <sstream>
// using namespace std;

std::ofstream fw ("/home/cast/catkin_ws/src/hand_planner_test/src/test_surena4_ros/Camera_Ai.txt", std::ofstream::out);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "CAMERA_AI");
    ros::NodeHandle n;
    ros::Publisher Ai_pub = n.advertise<geometry_msgs::PoseArray>("Camera_Ai", 100);
    ros::Rate loop_rate(200);
    double time = 0;
    double tempY = 0;
    double tempP = 0;

    while (ros::ok){
    geometry_msgs::PoseArray  posearray;
    geometry_msgs::Pose p;
    ///////////////////* HAND Ai*///////////////////
    // if (time<10){
    //     p.position.x = 0.3; //shakeHands
    //     p.position.y = -0.05;
    //     p.position.z = -0.35;
    // }
    // else if(time>=10 && time<20){
    //     p.position.x = 0.2;
    //     p.position.y = 0.05;
    //     p.position.z = -0.35;
    // }
    // else{
    //     p.position.x = 0.3; //byebye
    //     p.position.y = -0.1;
    //     p.position.z = -0.22;
    // }

    // p.position.x = 0.3; //byebye
    // p.position.y = -0.1;
    // p.position.z = 0.22;

    // p.position.x = 0.3; //shakeHands
    // p.position.y = -0.05;
    // p.position.z = -0.35;


    ///////////////////* HEAD Ai*///////////////////
    if (time<5){
        p.position.x = 1;
        p.position.y = 0.0005;
        p.position.z = 0.0005;
        tempP = p.position.z;
        tempY = p.position.y;
    }
    else if(time>=5 && time<6){
        p.position.x = 1;
        p.position.y = tempY; // + 0.0005;
        p.position.z = tempP + 0.0005;
        tempP = p.position.z;
        tempY = p.position.y;
    }
    else if(time>=6 && time<9){
        p.position.x = 1;
        p.position.y = tempY;
        p.position.z = tempP;
    }
    else if(time>=9 && time<10){
        p.position.x = 1;
        p.position.y = tempY; // + 0.0005;
        p.position.z = tempP + 0.0005;
        tempP = p.position.z;
        tempY = p.position.y;
    }
    else if(time>=10 && time<11.5){
        p.position.x = 1;
        p.position.y = tempY;
        p.position.z = tempP;
    }
    else if(time>=11.5 && time<15){
        p.position.x = 1;
        p.position.y = tempY; // - 0.0005;
        p.position.z = tempP - 0.0005;
        tempP = p.position.z;
        tempY = p.position.y;
    }
    // else if(time>=30 && time<35){
    //     p.position.x = 1;
    //     p.position.y = temp;
    //     p.position.z = 0;
    // }
    // else if(time>=35 && time<40){
    //     p.position.x = 1;
    //     p.position.y = temp + 0.0005;
    //     p.position.z = 0;
    //     temp = p.position.y;
    // }
    else{
        p.position.x = 1;
        p.position.y = tempY;
        p.position.z = tempP;
    }
    fw << p.position.x << "   ,   " << p.position.y <<"   ,   "<< p.position.z <<"\n";
    time += 0.005;
    ROS_INFO("x=%f, y=%f, z=%f", p.position.x, p.position.y, p.position.z);
    posearray.poses.push_back(p);
    Ai_pub.publish(posearray);
    ros::spinOnce();
    loop_rate.sleep();
    }
    return 0;
}
