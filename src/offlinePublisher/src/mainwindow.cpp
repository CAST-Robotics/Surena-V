#include "mainwindow.h"
#include "ui_mainwindow.h"


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);


//     ros::Publisher pub=nh.advertise<std_msgs::Int32MultiArray>("husky/cmd_vel",100);

////ros::start(); // explicitly needed since our nodehandle is going out of scope.
//  //  ros::Publisher pub=nh.advertise<geometry_msgs::Twist>("husky/cmd_vel", 100);

//    //Sets up the random number generator
//    srand(time(0));
//testvalue=0;
//    //Sets the loop to publish at a rate of 10Hz
//    ros::Rate rate(2);

//           std_msgs::Int32MultiArray msg;
//           std_msgs::MultiArrayDimension msg_dim;

//           msg_dim.label = "PLC_input_byte";
//             msg_dim.size = 1;
//             msg.layout.dim.clear();
//             msg.layout.dim.push_back(msg_dim);

//      while(ros::ok()) {
//          //Declares the message to be sent
//        //  geometry_msgs::Twist msg;
//        msg.data.clear();
//        for(size_t i = 0; i < 8; i++)
//          msg.data.push_back(i);

//          //Random x value between -2 and 2
//         // msg.linear.x=4*double(rand())/double(RAND_MAX)-2;
//          //Random y value between -3 and 3
//        //  msg.angular.z= 6*double(rand())/double(RAND_MAX)-3;
//          //Publish the message
//         // pub.publish(msg);
//          pub.publish(msg);
//          ROS_INFO("this is %s","amin");
//          //Delays untill it is time to send another message
//          ros::spinOnce();// let system to receive callback from another nodes
//          rate.sleep();
//        }
//          connect(&_timer,SIGNAL(timeout()),this,SLOT(Timeout()));
//     //     _timer.start(300);
}

MainWindow::~MainWindow()
{
    delete ui;
}
void MainWindow::Timeout()
{
  if(!ros::ok())return;
      //ros::Publisher pub=nh.advertise<geometry_msgs::Twist>("husky/cmd_vel", 100);
 // qDebug()<<"publishing msg";
  geometry_msgs::Twist msg;
 //Random x value between -2 and 2
 msg.linear.x=4*double(rand())/double(RAND_MAX)-2;
 //Random y value between -3 and 3
 msg.angular.z=6*double(rand())/double(RAND_MAX)-3;
 ROS_INFO("this is %s","amin");
 //Publish the message
 pub.publish(msg);
}

void MainWindow::on_pushButton_clicked()
{
    testvalue+=.7;

}
