#ifndef VISUALIZE_H
#define VISUALIZE_H
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <tf/transform_broadcaster.h>
#include <QEventLoop>
#include <QTimer>
#include <QDebug>
#include <QObject>
#include <std_msgs/Int32MultiArray.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <tf/transform_datatypes.h>

using namespace visualization_msgs;
//------------------------------------------------------------------------------------------
static boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
static  visualization_msgs::Marker marker;
static ros::Publisher marker_pub ;
static interactive_markers::MenuHandler menu_handler;
//==========================================================================================
void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
//==========================================================================================
InteractiveMarkerControl& MakeBoxControl( InteractiveMarker &msg );
//==========================================================================================
void MakePanTilt2Marker(QString frameName,QString markerName,  const tf::Vector3& position,float roll ,float pitch,float yaw,float scale );
//==========================================================================================
void UpdateRobotModel(QList<int> motorPosition);
void UpdateRobotModel(QList<double> rads);
//==========================================================================================
void DisplayError(QString msg,int markerID,QString frameID,float x,float y,float z);
//==========================================================================================
void VisualizeInit(ros::NodeHandle *nodeHandler);
//==========================================================================================

#endif // VISUALIZE_H
