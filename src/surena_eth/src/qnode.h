/**
 * @file /qnode.hpp
 *
 * @brief Communications central!
 *
 * @date sep 2018
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef my_qt_gui_subscriber_QNODE_HPP_
#define my_qt_gui_subscriber_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <string>
#include <std_msgs/Float64.h>
#include <QThread>
#include <QDebug>
#include <QList>
#include <QStringListModel>
#include <std_msgs/Bool.h>
#include "std_msgs/String.h"
#include "robot_teleop/active_csp.h"
#include "robot_teleop/reset_node.h"
#include "robot_teleop/node.h"

#include "robot_teleop/home.h"
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Wrench.h>
#include <tf/transform_broadcaster.h>
#include <QEventLoop>
#include <QTimer>
#include <QApplication>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include "visualize.h"
#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <QtTest/QSignalSpy>
#include <std_srvs/Trigger.h>
#include "QsLog/QsLog.h"

/*****************************************************************************
** Namespaces
*****************************************************************************/

//namespace my_qt_gui_subscriber {

/*****************************************************************************
** Class
*****************************************************************************/
    using namespace visualization_msgs;
class QNode : public QThread {
    Q_OBJECT

private:
    //=================================================================================================
    int init_argc;
    char** init_argv;

    bool _nodeInitialized=false;
    int _lastOperationResult=0;
    ros::Subscriber _jointsSubscriber;
    ros::Publisher chatter_publisher;
    ros::Publisher _imuPublisher,_jointPublisher,_incJointPublisher,_bumpPublisher,_rigthtFtPublisher,_leftFtPublisher,_pressurePublisher;
    QStringListModel logging_model;
    ros::ServiceServer _updatePositions;
    ros::ServiceServer _activeCSPService;
    ros::ServiceServer _hommingService;
    ros::ServiceServer _resetAllNodesService;
    ros::ServiceServer _resetHandsService;
    ros::ServiceServer _resetLegsService;
    ros::ServiceServer _activateHandsService;
    ros::ServiceServer _activateLegsService;
    ros::ServiceServer _getRobotStatus;


public:
    /*********************+
    ** Logging
    **********************/
    enum LogLevel {
             Debug,
             Info,
             Warn,
             Error,
             Fatal
     };
    QString RobotStatus;
    QString teststr="";
    std_msgs::Int32MultiArray  JointsData;
    QList<double> ActualPositions;
    QList<double> IncPositions;
    int BumpSensor[8];
    double Imu[6];
    sensor_msgs::Imu imuSesnsorMsg;
    geometry_msgs::Wrench RightFtSensorMessage,LeftFtSensorMessage;
    float pressureData[6];
     //=================================================================================================
    QNode();
    //=================================================================================================
	QNode(int argc, char** argv );
    //=================================================================================================
	virtual ~QNode();
    //=================================================================================================
	bool Init();
    //=================================================================================================
	bool Init(const std::string &master_url, const std::string &host_url);
    //=================================================================================================
	void run();
    //=================================================================================================
    void Callback(const std_msgs::Float64& message_holder);
    //=================================================================================================
	QStringListModel* loggingModel() { return &logging_model; }
    //=================================================================================================
	void Log( const LogLevel &level, const std_msgs::Float64 &msg);
    //=================================================================================================
    void NewJointDataReady(const std_msgs::Int32MultiArray &msg);
    //=================================================================================================
    bool ActiveCSP(robot_teleop::active_csp::Request &req, robot_teleop::active_csp::Response &res);
    bool ActivateLegs(robot_teleop::node::Request &req, robot_teleop::node::Response &res);
    //=================================================================================================
    bool Home(robot_teleop::home::Request &req, robot_teleop::home::Response &res);
    //=================================================================================================
    bool ResetAllNodes(robot_teleop::node::Request &req, robot_teleop::node::Response &res);
    //=================================================================================================
    void OperationCompleted(int status);
    bool WaitExternalOperation(int timeoutms);
    bool GetRobotStatus(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);
    bool ReadErrors(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
    bool UpdatePositions(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
    bool ResetHands(robot_teleop::node::Request &req, robot_teleop::node::Response &res);
    bool ActivateHands(robot_teleop::node::Request &req, robot_teleop::node::Response &res);

    bool ResetLegs(robot_teleop::node::Request &req, robot_teleop::node::Response &res);
Q_SIGNALS:
    //=================================================================================================
	void loggingUpdated();
    //=================================================================================================
    void rosShutdown();
    //=================================================================================================
    void NewjointDataReceived();
    //=================================================================================================
    void SetActiveCSP(int id);
    //=================================================================================================
    void DoResetAllNodes(int id);
    //=================================================================================================
    void DoResetHands(void);
    //=================================================================================================
    void DoResetLegs(void);
    //=================================================================================================
    void DoActivateHands(void);
    //=================================================================================================
    void DoActivateLegs(void);
    //=================================================================================================
    void SetHome(int id);
   // =================================================================================================
       void DoReadError();
    //=================================================================================================
  void ExternalOperationComleted();
  //=================================================================================================
  void UpdateAllPositions(void);
};

//}  // namespace my_qt_gui_subscriber

#endif /* my_qt_gui_subscriber_QNODE_HPP_ */
