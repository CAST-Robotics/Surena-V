#include "mainwindow.h"
#include <QApplication>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include<std_msgs/Int32MultiArray.h>
#include <stdlib.h>



int main(int argc, char *argv[])
{
  QApplication a(argc, argv);

MainWindow w;
w.RosInit(argc,argv);
w.show();

  return a.exec();
}
