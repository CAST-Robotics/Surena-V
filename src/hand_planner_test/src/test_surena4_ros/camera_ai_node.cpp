#include "ros/ros.h"
#include <geometry_msgs/PoseArray.h>

#include <sstream>
int main(int argc, char **argv)
{
    ros::init(argc, argv, "CAMERA_AI");
    ros::NodeHandle n;
    ros::Publisher Ai_pub = n.advertise<geometry_msgs::PoseArray>("Camera_Ai", 100);
    ros::Rate loop_rate(200);
    double time = 0;
    while (ros::ok){
    geometry_msgs::PoseArray  posearray;
    geometry_msgs::Pose p;
    if (time<5){
        p.position.x = 0.3; //shakeHands
        p.position.y = -0.05;
        p.position.z = -0.35;
    }
    else if(time>=5 && time<10){
        p.position.x = 0.2;
        p.position.y = 0.05;
        p.position.z = -0.35;
    }
    else{
        p.position.x = 0.3; //byebye
        p.position.y = -0.1;
        p.position.z = -0.22;
    }

    // p.position.x = 0.3; //byebye
    // p.position.y = -0.1;
    // p.position.z = 0.22;

    // p.position.x = 0.3; //shakeHands
    // p.position.y = -0.05;
    // p.position.z = -0.35;

    time += 0.005;
    ROS_INFO("x=%f, y=%f, z=%f", p.position.x, p.position.y, p.position.z);
    posearray.poses.push_back(p);
    Ai_pub.publish(posearray);
    ros::spinOnce();
    loop_rate.sleep();
    }
    return 0;
}
