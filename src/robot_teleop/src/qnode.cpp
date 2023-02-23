/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date Sep 2018
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <sstream>
#include <iostream>
#include "qnode.h"

using namespace std;

/*****************************************************************************
** Namespaces
*****************************************************************************/


/*****************************************************************************
** Implementation
*****************************************************************************/
QNode::QNode()
{

}
//=============================================================================================================================
QNode::QNode(int argc, char** argv ) :	init_argc(argc),	init_argv(argv)
	{}
//=============================================================================================================================
QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}
//=============================================================================================================================
bool QNode::Init(const std::string &master_url, const std::string &host_url) {
    std::map<std::string,std::string> remappings;
    remappings["__master"] = master_url;
    remappings["__hostname"] = host_url;
    ros::init(remappings,"my_qt_gui_subscriber");
    if ( ! ros::master::check() ) {
        return false;
    }
    ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle n;
    // Add your ros communications here.
   // _chatter_subscriber = n.subscribe("chatter", 1000, &QNode::myCallback, this);
    start();

    return true;
}
//=============================================================================================================================
bool QNode::Init() {
    ros::init(init_argc,init_argv,"robot_teleop");
	if ( ! ros::master::check() ) {
        qDebug()<<"init error!";
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;


	// Add your ros communications here.

chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
_jointPublisher =    n.advertise<std_msgs::Int32MultiArray>("jointdata/qc", 1000);

_getstatusService = n.serviceClient<std_srvs::Trigger>("GetRobotStatus");
_activeCspService = n.serviceClient<robot_teleop::active_csp>("ActiveCSP");
_resetNodeService = n.serviceClient<std_srvs::SetBool>("ResetAllNodes");
std_srvs::TriggerRequest req;
std_srvs::TriggerResponse res;
 _getstatusService.call(req,res);
qDebug()<<"resp="<<QString::fromStdString(res.message);
 PositioningIsActive=false;


    start();
	return true;
}
//=============================================================================================================================
void QNode::DoPositioning()
{
    if(!PositioningIsActive)return;
qDebug()<<"positioning";
double sum=0;

for(int i=0;i<12;i++)
{
  sum+=fabs(IncPositions[i]);

}
if(sum==0){


PositioningIsActive=false;
}
}
//================================================================================================================================================================
void QNode::run() {
    ros::NodeHandle n;
    ros::Rate loop_rate(200);


    while ( ros::ok() ) {
    std_msgs::String msg;



msg.data="salam";
chatter_publisher.publish(msg);
DoPositioning();



    ros::spinOnce();
    loop_rate.sleep();
        }

    //std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}
//================================================================================================================================================================
void QNode::Callback(const std_msgs::Float64& message_holder)
{
	//std::stringstream ss;
	//ss << message_holder.data;
    Log(Info, message_holder);
	ROS_INFO("=============received value is: %f===========",message_holder.data); 
    //Q_EMIT NewDataReceived();
  //really could do something interesting here with the received data...but all we do is print it 
} 
//================================================================================================================================================================
void QNode::Log( const LogLevel &level, const std_msgs::Float64 &msg) {
	logging_model.insertRows(logging_model.rowCount(),1);
	std::stringstream logging_model_msg;
	switch ( level ) {
		case(Debug) : {
				ROS_DEBUG_STREAM(msg);
				logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << "received value is: " << msg.data;
				break;
		}
		case(Info) : {
				ROS_INFO_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << "received value is: " << msg.data;
				break;
		}
		case(Warn) : {
				ROS_WARN_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << "received value is: " << msg.data;
				break;
		}
		case(Error) : {
				ROS_ERROR_STREAM(msg);
				logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << "received value is: " << msg.data;
				break;
		}
		case(Fatal) : {
				ROS_FATAL_STREAM(msg);
				logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << "received value is: " << msg.data;
				break;
		}
	}
	QVariant new_row(QString(logging_model_msg.str().c_str()));
	logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
	Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}
//================================================================================================================================================================
void QNode::ResetAllNodes()
{
    std_srvs::SetBoolRequest req;
    std_srvs::SetBoolResponse resp;

  _resetNodeService.call(req,resp);

}
