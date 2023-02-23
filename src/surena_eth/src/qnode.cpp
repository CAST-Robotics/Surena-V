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
{
RobotStatus="";


}
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
    ros::init(init_argc,init_argv,"surena");
	if ( ! ros::master::check() ) {
        //qDebug()<<"init error!";
		return false;
	}

	ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle n;

    VisualizeInit(&n);

    QLOG_TRACE()<<"Initializing all ros publishers";
    _jointsSubscriber = n.subscribe("jointdata/qc", 1000, &QNode::NewJointDataReady, this);
    chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
    _rigthtFtPublisher= n.advertise<geometry_msgs::Wrench>("surena/ft_r_state", 1000);
    _leftFtPublisher= n.advertise<geometry_msgs::Wrench>("surena/ft_l_state", 1000);
    _imuPublisher =    n.advertise<sensor_msgs::Imu>("surena/imu_state", 1000);
    _jointPublisher =    n.advertise<sensor_msgs::JointState>("surena/abs_joint_state", 1000);
    _incJointPublisher = n.advertise<sensor_msgs::JointState>("surena/inc_joint_state", 1000);
    _bumpPublisher = n.advertise<std_msgs::Int32MultiArray>("surena/bump_sensor_state", 1000);

    QLOG_TRACE()<<"Initializing all ros services";

    _hommingService = n.advertiseService("Home", &QNode::Home, this);
    _resetAllNodesService = n.advertiseService("ResetAllNodes", &QNode::ResetAllNodes, this);
    _resetHandsService = n.advertiseService("ResetHands", &QNode::ResetHands, this);
    _resetLegsService = n.advertiseService("ResetLegs", &QNode::ResetLegs, this);
    _activeCSPService = n.advertiseService("ActivateWholeBody", &QNode::ActiveCSP, this);
    _activateHandsService = n.advertiseService("ActivateHands", &QNode::ActivateHands, this);
    _activateLegsService = n.advertiseService("ActivateLegs", &QNode::ActivateLegs, this);


    _updatePositions = n.advertiseService("UpdatePositions", &QNode::UpdatePositions, this);

     //_resetAllNodesService = n.advertiseService("ResetAllNodes", &QNode::ResetAllNodes, this);
    _getRobotStatus = n.advertiseService("GetRobotStatus", &QNode::GetRobotStatus, this);

    for (int i = 0; i < 30; i++) {
    ActualPositions.append(0);
    IncPositions.append(0);

    }

    QLOG_TRACE()<<"Start ROS::OK loop";
    start();
	return true;
}
//================================================================================================================================================================
bool QNode::GetRobotStatus(std_srvs::TriggerRequest &req,std_srvs::TriggerResponse &res)
{
        QLOG_TRACE()<<"get Robot Status";
    _lastOperationResult=-1;
        Q_EMIT DoReadError();
       // QThread::msleep(1);
        //===============================
    if(!WaitExternalOperation(6000))
    {
      QLOG_TRACE()<<"Wait for external signal failed";
    res.success=false;
    res.message="response timeout!";
    //qDebug()<<"lor:"<<_lastOperationResult;
    return false;

    }
    if(_lastOperationResult!=0)
    {
        QLOG_TRACE()<<"Operation error";
        res.success=false;
        res.message="response with error";
        //qDebug()<<"lor:"<<_lastOperationResult;

        return false;

    }

    //===============================
        //spy.wait(10000);
    //qDebug()<<"lor:"<<_lastOperationResult;
        res.success=true;
        res.message=RobotStatus.toStdString();
        //qDebug()<<"Read errors...>\n"<<RobotStatus;
        return true;

}
//================================================================================================================================================================
bool QNode::ActiveCSP(robot_teleop::active_csp::Request  &req,robot_teleop::active_csp::Response &res)
{


    _lastOperationResult=-1;

    Q_EMIT SetActiveCSP(req.nodeID);
    if(!WaitExternalOperation(10000))
    {
            return false;
    }
    if(_lastOperationResult!=0)
    {
        return false;
    }
    return true;
}
//================================================================================================================================================================
bool QNode::Home(robot_teleop::home::Request  &req,robot_teleop::home::Response &res)
{

    _lastOperationResult=-1;
    Q_EMIT SetHome(req.nodeID);
    if(!WaitExternalOperation(30000))
    {
    return false;

    }
    if(_lastOperationResult!=0)
    {

        return false;

    }
    return true;

}
//================================================================================================================================================================
bool QNode::WaitExternalOperation(int timeoutms=60000)
{
    QTimer timer;
    timer.setSingleShot(true);
    QEventLoop loop;
    connect(this,  SIGNAL(ExternalOperationComleted()), &loop, SLOT(quit()) );
    connect(&timer, SIGNAL(timeout()), &loop, SLOT(quit()));
    timer.start(timeoutms);
    loop.exec();
    if(timer.isActive())return true;
    return false;

}
//================================================================================================================================================================
bool QNode::ResetHands(robot_teleop::node::Request &req, robot_teleop::node::Response &res)
{
    _lastOperationResult=-1;
        Q_EMIT DoResetHands();

        //===============================
    if(!WaitExternalOperation(6000))
    {
    res.result=-1;
    //res.="response timeout!";
    return false;

    }
    if(_lastOperationResult!=0)
    {
        //res.success=false;
        //res.message="response with error";
    res.result=-1;
        return false;
}
}
//================================================================================================================================================================
bool QNode::ResetLegs(robot_teleop::node::Request &req, robot_teleop::node::Response &res)
{
    _lastOperationResult=-1;
        Q_EMIT DoResetLegs();

        //===============================
    if(!WaitExternalOperation(6000))
    {
    res.result=-1;
    //res.="response timeout!";
    return false;

    }
    if(_lastOperationResult!=0)
    {
        //res.success=false;
        //res.message="response with error";
    res.result=-1;
        return false;
}
    return true;
}
//================================================================================================================================================================
bool QNode::UpdatePositions(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
    _lastOperationResult=-1;

    Q_EMIT UpdateAllPositions();//req.nodeID);
    if(!WaitExternalOperation(10000))
    {
            return false;
    }
    if(_lastOperationResult!=0)
    {
        return false;
    }
    return true;
}

//================================================================================================================================================================
bool QNode::ActivateHands(robot_teleop::node::Request &req, robot_teleop::node::Response &res)
{
    _lastOperationResult=-1;
        Q_EMIT DoActivateHands();

        //===============================
    if(!WaitExternalOperation(6000))
    {
    res.result=-1;
    //res.="response timeout!";
    return false;

    }
    if(_lastOperationResult!=0)
    {
        //res.success=false;
        //res.message="response with error";
    res.result=-1;
        return false;
}
}
//================================================================================================================================================================
bool QNode::ActivateLegs(robot_teleop::node::Request &req, robot_teleop::node::Response &res)
{
    _lastOperationResult=-1;
        Q_EMIT DoActivateLegs();

        //===============================
    if(!WaitExternalOperation(6000))
    {
    res.result=-1;
    //res.="response timeout!";
    return false;

    }
    if(_lastOperationResult!=0)
    {
        //res.success=false;
        //res.message="response with error";
    res.result=-1;
        return false;
}
    return true;
}
//================================================================================================================================================================
bool QNode::ResetAllNodes(robot_teleop::node::Request &req, robot_teleop::node::Response &res)
{

_lastOperationResult=-1;
    Q_EMIT DoResetAllNodes(req.nodeID);

    //===============================
if(!WaitExternalOperation(6000))
{
res.result=-1;
//res.="response timeout!";
return false;

}
if(_lastOperationResult!=0)
{
    //res.success=false;
    //res.message="response with error";
res.result=-1;
    return false;

}

//===============================
    //spy.wait(10000);
    //res.success=true;
    //res.message="OK";
    qDebug()<<"Reset all...> "<<teststr;
    res.result=0;
    return true;
}
//================================================================================================================================================================
bool QNode::ReadErrors(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
    //if(!_nodeInitialized)return false;
    // QSignalSpy spysig(this,SIGNAL(ExternalOperationComleted(int)));

_lastOperationResult=-1;
    Q_EMIT DoReadError();
   // QThread::msleep(1);
    //===============================
if(!WaitExternalOperation(6000))
{
res.success=false;

qDebug()<<"lor:"<<_lastOperationResult;
res.message="response timeout!";
return false;

}
if(_lastOperationResult!=0)
{
    res.success=false;
    qDebug()<<"lor:"<<_lastOperationResult;
    res.message="response with error1";

    return false;

}
qDebug()<<"lor:"<<_lastOperationResult;
//===============================
    //spy.wait(10000);
    res.success=true;
    res.message="OK";
    qDebug()<<"Reset all...> "<<teststr;
    return true;
}

//================================================================================================================================================================
void QNode::OperationCompleted(int status)
{
    _lastOperationResult=status;
    emit ExternalOperationComleted();
}
//================================================================================================================================================================
void QNode::NewJointDataReady(const std_msgs::Int32MultiArray & msg)
{

    JointsData=msg;
    Q_EMIT NewjointDataReceived();

}
//================================================================================================================================================================
void QNode::run() {
    ros::NodeHandle n;
    ros::Rate loop_rate(200);
    _jointsSubscriber = n.subscribe("jointdata/qc", 1000, &QNode::NewJointDataReady, this);
    //  std_msgs::Int32MultiArray msg;
//    while (ros::ok()) //Endless loop until Ctrl+c
//        {
//            ROS_DEBUG("This is a DEBUG message");
//            ROS_INFO("This is a INFO message");
//            ROS_WARN("This is a WARN message");
//            ROS_ERROR("This is a ERROR message");
//            ROS_FATAL("This is a FATAL message");

//            loop_rate.sleep();
//            ros::spinOnce();
//        }
    while ( ros::ok() ) {




   // std_msgs::String msg;
    sensor_msgs::JointState ActualJointState,IncJointState;
    std_msgs::Int32MultiArray BumpSensorState;
    std_msgs::MultiArrayDimension msg_dim;
    msg_dim.label = "bump";
    msg_dim.size = 1;
    BumpSensorState.layout.dim.clear();
    BumpSensorState.layout.dim.push_back(msg_dim);

    //chatter_publisher.publish(msg); // publish the value--of type Float64-
    ActualJointState.header.stamp = ros::Time::now();
    IncJointState.header.stamp = ros::Time::now();
    imuSesnsorMsg.header.stamp= ros::Time::now();;
    imuSesnsorMsg.header.frame_id="base_link";

  for(int i=0 ;i<29;i++){
  ActualJointState.position.push_back(ActualPositions[i]);
  IncJointState.position.push_back(IncPositions[i]);
  }
  for(int i=0 ;i<8;i++){
      BumpSensorState.data.push_back(BumpSensor[i]);
  //BumpSensorState.data[i]=i;
  }

    _rigthtFtPublisher.publish(RightFtSensorMessage);
    _leftFtPublisher.publish(LeftFtSensorMessage);
    _bumpPublisher.publish(BumpSensorState);
    _imuPublisher.publish(imuSesnsorMsg);
    _jointPublisher.publish(ActualJointState);
    _incJointPublisher.publish(IncJointState);
    UpdateRobotModel(ActualPositions);
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
