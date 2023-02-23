#ifndef VISUALIZECONTROL_H
#define VISUALIZECONTROL_H

#include <QObject>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <tf/transform_broadcaster.h>
#include <QEventLoop>
#include <QTimer>
#include <std_msgs/Int32MultiArray.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <tf/transform_datatypes.h>
using namespace visualization_msgs;





class VisualizeControl : public QObject
{
    Q_OBJECT
 boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
    ros::Publisher marker_pub ;

       interactive_markers::MenuHandler menu_handler;
        visualization_msgs::Marker marker;

       //static void ProcessFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

        Marker MakeBox(InteractiveMarker &msg);
       Marker MakeCylinder(InteractiveMarker &msg);
       InteractiveMarkerControl &MakeBoxControl(InteractiveMarker &msg);
       void MakePanTilt2Marker(QString frameName, QString markerName, const tf::Vector3 &position, float roll, float pitch, float yaw, float scale);
       void MakePanTiltMarker(QString frameID, const tf::Vector3 &position, float scale);
public:
    explicit VisualizeControl(QObject *parent = nullptr);
    void Init(ros::NodeHandle *nodeHandler);
    void SendTF(QString frame, QString child, tf::Vector3 location, float roll, float pitch, float yaw);
    void UpdateRobotModel(QList<int> motorPosition);
    QList<double> QctoRadian(QList<int> qc);
     void ProcessFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );

signals:

public slots:
};

#endif // VISUALIZECONTROL_H
