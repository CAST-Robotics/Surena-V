#include "mainwindow.h"
#include <QApplication>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include<std_msgs/Int32MultiArray.h>
#include <stdlib.h>
#include "datareader.h"
#include "offlinepublisher/srv1.h"
#include "offlinepublisher/num.h"
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>
#include <istream>

using namespace std;

std::vector<std::string> csv_read_row(std::istream &in, char delimiter);
std::vector<std::string> csv_read_row(std::string &in, char delimiter);

std::vector<std::string> csv_read_row(std::string &line, char delimiter)
{
    std::stringstream ss(line);
    return csv_read_row(ss, delimiter);
}
 
std::vector<std::string> csv_read_row(std::istream &in, char delimiter)
{
    std::stringstream ss;
    bool inquotes = false;
    std::vector<std::string> row;//relying on RVO
    while(in.good())
    {
        char c = in.get();
        if (!inquotes && c=='"') //beginquotechar
        {
            inquotes=true;
        }
        else if (inquotes && c=='"') //quotechar
        {
            if ( in.peek() == '"')//2 consecutive quotes resolve to 1
            {
                ss << (char)in.get();
            }
            else //endquotechar
            {
                inquotes=false;
            }
        }
        else if (!inquotes && c==delimiter) //end of field
        {
            row.push_back( ss.str() );
            ss.str("");
        }
        else if (!inquotes && (c=='\r' || c=='\n') )
        {
            if(in.peek()=='\n') { in.get(); }
            row.push_back( ss.str() );
            return row;
        }
        else
        {
            ss << c;
        }
    }
}


int _dataIndex=0;
QList<int> _motorPosition;
bool add(offlinepublisher::srv1::Request  &req,
         offlinepublisher::srv1::Response &res)
{
  res.sum = req.a + req.b;
  ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
  ROS_INFO("sending back response: [%ld]", (long int)res.sum);
  return true;
}
//echo "source  /home/amin/Desktop/catkin_ws/devel/setup.sh" >> ~/.bashrc
//source  /home/amin/Desktop/catkin_ws/devel/setup.sh
//check service with this command rosservice call /add "a: 4 b: 2"
//
int main(int argc, char *argv[])
{
  QApplication a(argc, argv);
  DataReader _dataReader;

  _dataReader.Load("/home/surena/Documents/data.txt");
  //Initializes ROS, and sets up a node
  ros::init(argc, argv, "offline_publisher");
  ros::NodeHandle nh,nh2;
  ros::ServiceServer service = nh.advertiseService("add", add);

  ros::Publisher pub=nh.advertise<std_msgs::Int32MultiArray>("jointdata/qc",100);
  ros::Publisher pub2=nh.advertise<offlinepublisher::num>("custom_msg",100);

  //Sets up the random number generator
  srand(time(0));

  //Sets the loop to publish at a rate of 10Hz
  ros::Rate  rate(100);

  std_msgs::Int32MultiArray msg;
  std_msgs::MultiArrayDimension msg_dim;
  offlinepublisher::num myCustomMsg;
  myCustomMsg.age=40;
  myCustomMsg.first_name="amin";

  myCustomMsg.last_name="amani";
  myCustomMsg.score=20;

  msg_dim.label = "jointdata/qc";
  msg_dim.size = 1;
  msg.layout.dim.clear();
  msg.layout.dim.push_back(msg_dim);

  // std::ifstream in("/home/surena/qref_deal.csv");
  // if (in.fail()) return (std::cout << "File not found" << endl) && 0;
  // std::string line;
  
  int pos=0;
  bool dir=false;
  while(ros::ok()) 
  {
    std::vector<std::string> row;
    // if(getline(in, line)  && in.good())
    // {
      // row = csv_read_row(line, ',');

      msg.data.clear();
      if(dir){
            //_motorPosition[i]=pos;
            pos+=16;
            }
          else{
            //_motorPosition[i]=pos;
            pos-=16;
            }
      //_dataReader.GetData(_motorPosition,_dataIndex++);
      for(size_t i = 0; i < 23; i++)
      {
        if(i == 16 || i == 17 || i == 18 || i == 19)
        {
          msg.data.push_back(pos);
        }
        else{
          msg.data.push_back(0);
        }
      }
    // }
    pub.publish(msg);
     if(pos>(15000))
    dir=false;
     if(pos<(-15000))
    dir=true;
    
   // pub2.publish(myCustomMsg);
    // ROS_INFO("this is %d" ,_motorPosition[0]);
    //Delays untill it is time to send another message
    ros::spinOnce();// let system to receive callback from another nodes
    rate.sleep();
  }

  return 0;
  return a.exec();
}
