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
#include "robot_teleop/home.h"

#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Wrench.h>
#include <tf/transform_broadcaster.h>
#include <QEventLoop>
#include <QTimer>
#include <QApplication>
#include <std_msgs/Int32MultiArray.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>


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
    ros::Subscriber _jointsSubscriber;
    ros::Publisher chatter_publisher;
    ros::Publisher _jointPublisher;
    QStringListModel logging_model;
    ros::ServiceClient _getstatusService ;
ros::ServiceClient _activeCspService;
ros::ServiceClient _resetNodeService ;

public:
    /*********************
    ** Logging
    **********************/
    enum LogLevel {
             Debug,
             Info,
             Warn,
             Error,
             Fatal
     };

    std_msgs::Int32MultiArray  JointsData;


    bool PositioningIsActive;
    double IncPositions[30];



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
    //=================================================================================================
    bool Home(robot_teleop::home::Request &req, robot_teleop::home::Response &res);
    //=================================================================================================
   // bool ResetAllNodes(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
    //=================================================================================================

    void DoPositioning();
    void ResetAllNodes();
Q_SIGNALS:
    //=================================================================================================
	void loggingUpdated();
    //=================================================================================================
    void rosShutdown();
    //=================================================================================================
    void NewjointDataReceived();
    //=================================================================================================
    void SetActiveCSP();
    //=================================================================================================
    void DoResetAllNodes();
    //=================================================================================================
    void SetHome();
    //=================================================================================================
};

//}  // namespace my_qt_gui_subscriber

#endif /* my_qt_gui_subscriber_QNODE_HPP_ */
