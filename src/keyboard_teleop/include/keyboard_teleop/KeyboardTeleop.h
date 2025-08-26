# pragma once

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <map>

class KeyboardTeleop{
    // This class has written based on this source:
    // https://github.com/Jayadev22/ubiquitousROS/blob/master/keyboard_pub/src/keyboard.cpp

    public:
        KeyboardTeleop(ros::NodeHandle *nh);
        ~KeyboardTeleop(){}
        void run();
        int getch();

    private:
        ros::NodeHandle nh;
        ros::Publisher keyboardTeleopPub_;
};