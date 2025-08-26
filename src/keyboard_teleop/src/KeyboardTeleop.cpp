#include "KeyboardTeleop.h"

KeyboardTeleop::KeyboardTeleop(ros::NodeHandle *nh){
    keyboardTeleopPub_ = nh->advertise<std_msgs::Int32>("/keyboard_command", 50);
}

int KeyboardTeleop::getch(){
    static struct termios oldt, newt;
    tcgetattr( STDIN_FILENO, &oldt);           // save old settings
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);                 // disable buffering      
    tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new setting
    int ch = getchar();                        // read character (non-blocking
    tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
    return ch;
}

void KeyboardTeleop::run(){
    int data = 0;
    while (ros::ok() && data != 'q')
    {
        std_msgs::Int32 c;
        data = this->getch(); // call your non-blocking input function
        c.data = data;  
        keyboardTeleopPub_.publish(c);
        ros::spinOnce();
    }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "keyboard_teleop");
    ros::NodeHandle nh;
    KeyboardTeleop key(&nh);
    key.run();
    return 0;
}