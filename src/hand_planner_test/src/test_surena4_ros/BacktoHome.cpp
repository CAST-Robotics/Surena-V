#include "ros/ros.h"
#include<std_msgs/Float32MultiArray.h>
#include<std_msgs/Int32MultiArray.h>
#include<std_msgs/Float64.h>
#include <vector>
using namespace  std;
int main(int argc, char **argv) {
    ros::init(argc, argv, "homenode");
    ros::NodeHandle nh;
    ros::Publisher  home_data_pub  = nh.advertise<std_msgs::Int32MultiArray>("jointdata/qc",100);
    std_msgs::Int32MultiArray home_data;
    ros::Rate loop_rate(200);
    vector<int> a={0,0,0,0,0,0,0,0,0,0,0,0,atoi(argv[1]),atoi(argv[2]),atoi(argv[3]),atoi(argv[4])};
    vector<int> b(16,0);
    while (ros::ok())
    {
        for(int  i = 0; i < 16; i++)
        {
            // ROS_INFO("ith: %d", i);
            while (a[i]!=0)
            {   
                home_data.data.clear();
                if (a[i]>0){
                    a[i]=a[i]-200;
                    b[i] = b[i]-200;
                    if (a[i]<0){
                        break;
                    }
                    ROS_INFO("b: %d", b[i]);
                    for(int j=0; j<16; j++){
                        home_data.data.push_back(b[j]);
                    }
                    home_data_pub.publish(home_data);
                    //ROS_INFO("ok1");
                    }
                else{
                    a[i]=a[i]+200;
                    b[i] = b[i]+200;
                    if (a[i]>0){
                        break;
                    }
                    ROS_INFO("b: %d", b[i]);
                    for(int j=0; j<16; j++){
                        home_data.data.push_back(b[j]);
                    }
                    home_data_pub.publish(home_data);
                    //ROS_INFO("ok2");
                }      
                loop_rate.sleep(); 
            } 
            
        }
        ros::spinOnce();

    }
    return 0 ;
}