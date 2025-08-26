#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include <vector>
#include <string>

// Global Publishers
std::vector<ros::Publisher> right_leg_pubs;
std::vector<ros::Publisher> left_leg_pubs;
std::vector<ros::Publisher> right_arm_pubs;
std::vector<ros::Publisher> left_arm_pubs;
std::vector<ros::Publisher> head_pubs;

// callback for whole body
void GazeboCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
    // Right Leg
    for (int i = 0; i < 6; ++i) {
        std_msgs::Float64 cmd;
        cmd.data = msg->data[i];
        right_leg_pubs[i].publish(cmd);
    }
    // Left Leg
    for (int i = 0; i < 6; ++i) {
        std_msgs::Float64 cmd;
        cmd.data = msg->data[i + 6];
        left_leg_pubs[i].publish(cmd);
    }
    // Right Arm
    std::vector<int> right_arm_indices = {12, 13, 14, 15, 23, 24, 25};
    for (size_t i = 0; i < right_arm_pubs.size(); ++i) {
        std_msgs::Float64 cmd;
        cmd.data = (right_arm_indices[i]==23) ? msg->data[right_arm_indices[i]]+1.57/3 : msg->data[right_arm_indices[i]];
        right_arm_pubs[i].publish(cmd);
    }
    // Left Arm 
    std::vector<int> left_arm_indices = {16, 17, 18, 19, 26, 27, 28};
    for (size_t i = 0; i < left_arm_pubs.size(); ++i) {
        std_msgs::Float64 cmd;
        cmd.data = msg->data[left_arm_indices[i]];
        left_arm_pubs[i].publish(cmd);
    }
    // Head
    std::vector<int> head_indices = {20, 21, 22};
    for (size_t i = 0; i < head_pubs.size(); ++i) {
        std_msgs::Float64 cmd;
        cmd.data = msg->data[head_indices[i]];
        head_pubs[i].publish(cmd);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "gazebo_bridge_node");
    ros::NodeHandle nh;

    // Right Leg Publishers
    std::vector<std::string> right_leg_topics = {
        "/surenaV/r_hip_yaw_joint_position_controller/command",
        "/surenaV/r_hip_roll_joint_position_controller/command",
        "/surenaV/r_hip_pitch_joint_position_controller/command",
        "/surenaV/r_knee_joint_position_controller/command",
        "/surenaV/r_ankle_pitch_joint_position_controller/command",
        "/surenaV/r_ankle_roll_joint_position_controller/command"
    };
    for (const auto& topic : right_leg_topics) {
        right_leg_pubs.push_back(nh.advertise<std_msgs::Float64>(topic, 10));
    }

    // Left Leg Publishers
    std::vector<std::string> left_leg_topics = {
        "/surenaV/l_hip_yaw_joint_position_controller/command",
        "/surenaV/l_hip_roll_joint_position_controller/command",
        "/surenaV/l_hip_pitch_joint_position_controller/command",
        "/surenaV/l_knee_joint_position_controller/command",
        "/surenaV/l_ankle_pitch_joint_position_controller/command",
        "/surenaV/l_ankle_roll_joint_position_controller/command"
    };
    for (const auto& topic : left_leg_topics) {
        left_leg_pubs.push_back(nh.advertise<std_msgs::Float64>(topic, 10));
    }

    // Right Arm Publishers
    std::vector<std::string> right_arm_topics = {
        "/surenaV/r_arm_pitch_joint_position_controller/command",
        "/surenaV/r_arm_roll_joint_position_controller/command",
        "/surenaV/r_elbow_pitch_joint_position_controller/command",
        "/surenaV/r_forearm_roll_joint_position_controller/command",
        "/surenaV/r_forearm_link_joint_position_controller/command",
        "/surenaV/r_hand_roll_joint_position_controller/command",
        "/surenaV/r_hand_pitch_joint_position_controller/command"
    };
    for (const auto& topic : right_arm_topics) {
        right_arm_pubs.push_back(nh.advertise<std_msgs::Float64>(topic, 10));
    }

    // Left Arm Publishers
    std::vector<std::string> left_arm_topics = {
        "/surenaV/l_arm_pitch_joint_position_controller/command",
        "/surenaV/l_arm_roll_joint_position_controller/command",
        "/surenaV/l_elbow_pitch_joint_position_controller/command",
        "/surenaV/l_forearm_roll_joint_position_controller/command",
        "/surenaV/l_forearm_link_joint_position_controller/command",
        "/surenaV/l_hand_roll_joint_position_controller/command",
        "/surenaV/l_hand_pitch_joint_position_controller/command"
    };
    for (const auto& topic : left_arm_topics) {
        left_arm_pubs.push_back(nh.advertise<std_msgs::Float64>(topic, 10));
    }

    // Head Publishers
    std::vector<std::string> head_topics = {
        "/surenaV/head_roll_joint_position_controller/command",
        "/surenaV/head_pitch_joint_position_controller/command",
        "/surenaV/head_yaw_joint_position_controller/command"
    };
    for (const auto& topic : head_topics) {
        head_pubs.push_back(nh.advertise<std_msgs::Float64>(topic, 10));
    }

    // Subscribers
    ros::Subscriber joint_sub = nh.subscribe("/joint_angles_gazebo", 10, GazeboCallback);

    ROS_INFO("Surena Gazebo bridge node is running.");
    ros::spin();

    return 0;
}