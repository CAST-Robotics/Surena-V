#include "visualize.h"
//==========================================================================================
Marker MakeBox( InteractiveMarker &msg ,float Scale,float r,float g,float b,float a)
{
  Marker marker;

  marker.type = Marker::CUBE;
  marker.scale.x = msg.scale * 0.45;
  marker.scale.y = msg.scale * 0.45;
  marker.scale.z = msg.scale * 0.45;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = a;

  return marker;
}
//==========================================================================================
Marker MakeCylinder( InteractiveMarker &msg )
{
//    visualization_msgs::Marker marker;
//    marker.header.frame_id = "base_link";
//    marker.header.stamp = ros::Time();
//    marker.ns = "my_namespace1";
//    marker.id = 0;
//    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
//    marker.action = visualization_msgs::Marker::ADD;
//    marker.pose.position.x = 1;
//    marker.pose.position.y = 1;
//    marker.pose.position.z = 1;
//    marker.pose.orientation.x = 0.0;
//    marker.pose.orientation.y = 0.0;
//    marker.pose.orientation.z = 0.0;
//    marker.pose.orientation.w = 1.0;
//    marker.scale.x = 1;
//    marker.scale.y = 1;
//    marker.scale.z = 1;
//    marker.color.a = 1.0; // Don't forget to set the alpha!
//    marker.color.r = 0.5;
//    marker.color.g = 0.5;
//    marker.color.b = 0.5;
//    //only if using a MESH_RESOURCE marker type:
//    marker.mesh_resource = "package://robot_sim/meshes/Foot-Left.STL";
//    return marker;
      Marker marker;

  marker.type = Marker::CYLINDER;
 // marker.type = Marker::MESH_RESOURCE;
  //marker.mesh_resource = "package://robot_sim/meshes/Foot_Left.STL";

  marker.scale.x = msg.scale * 0.45;
  marker.scale.y = msg.scale * 0.45;
  marker.scale.z = msg.scale * 0.45;
  marker.color.r = 0.5;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  marker.color.a = 0;

  return marker;
}
//==========================================================================================
InteractiveMarkerControl& MakeBoxControl( InteractiveMarker &msg )
{
  InteractiveMarkerControl control;
  control.always_visible = true;
  //control.markers.push_back( makeBox(msg) );
  control.markers.push_back(MakeCylinder(msg));
  msg.controls.push_back( control );

  return msg.controls.back();
}
//==========================================================================================
void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
 //   tfScalar yaw, pitch, roll;
    std::ostringstream s;
  //  tf::Quaternion q(feedback->pose.orientation.x,feedback->pose.orientation.y,feedback->pose.orientation.z,feedback->pose.orientation.w);

 //   tf::Matrix3x3 mat(q);
   // mat.getRPY(roll, pitch, yaw);

  s << "Feedback from marker '" << feedback->marker_name << "' "
      << " / control '" << feedback->control_name << "'";

  std::ostringstream mouse_point_ss;
  if( feedback->mouse_point_valid )
  {
    mouse_point_ss << " at " << feedback->mouse_point.x
                   << ", " << feedback->mouse_point.y
                   << ", " << feedback->mouse_point.z
                   << " in frame " << feedback->header.frame_id;
  }

  switch ( feedback->event_type )
  {
    case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
      ROS_INFO_STREAM( s.str() << ": button click" << mouse_point_ss.str() << "." );
      break;

    case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
      ROS_INFO_STREAM( s.str() << ": menu item " << feedback->menu_entry_id << " clicked" << mouse_point_ss.str() << "." );
      break;

    case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:

            ROS_INFO_STREAM( s.str() << ": pose changed"

                << "\norientation = "
                << feedback->pose.orientation.w
                << ", " << feedback->pose.orientation.x
                << ", " << feedback->pose.orientation.y
                << ", " << feedback->pose.orientation.z
     );

//      ROS_INFO_STREAM( s.str() << ": pose changed"
//          << "\nposition = "
//          << feedback->pose.position.x
//          << ", " << feedback->pose.position.y
//          << ", " << feedback->pose.position.z
//          << "\norientation = "
//          << feedback->pose.orientation.w
//          << ", " << feedback->pose.orientation.x
//          << ", " << feedback->pose.orientation.y
//          << ", " << feedback->pose.orientation.z
//          << "\nframe: " << feedback->header.frame_id
//          << " time: " << feedback->header.stamp.sec << "sec, "
//          << feedback->header.stamp.nsec << " nsec" );

//      q(0,0, 0, 0);

    //tf::Matrix3x3 mat(q);
    //  mat.getEulerYPR(&yaw, &pitch, &roll);
        //DisplayText(QString::fromStdString( feedback->header.frame_id)+" "+QString::number(roll)+" "+QString::number(pitch)+" "+QString::number(yaw),1,"/base_link",0,0,1.9);
      break;

    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
      ROS_INFO_STREAM( s.str() << ": mouse down" << mouse_point_ss.str() << "." );

      break;

    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
      ROS_INFO_STREAM( s.str() << ": mouse up" << mouse_point_ss.str() << "." );
      //     DisplayText(QString::number(t+=.01),1,"/base_link",0,0,2);

      break;
  }

  server->applyChanges();
}
//==========================================================================================
void MakePanTilt2Marker(QString frameName,QString markerName,  const tf::Vector3& position,float roll ,float pitch,float yaw,float scale )
{
  InteractiveMarker int_marker;
  int_marker.header.frame_id =frameName.toStdString();
  tf::pointTFToMsg(position, int_marker.pose.position);
  int_marker.scale = scale;

  int_marker.name = markerName.toStdString();
  int_marker.description = "Pan / Tilt";

  MakeBoxControl(int_marker);

  InteractiveMarkerControl control;

  tf::Quaternion orien;
  orien.setRPY(roll,pitch,yaw);
  orien.normalize();
  tf::quaternionTFToMsg(orien, control.orientation);
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  control.orientation_mode = InteractiveMarkerControl::FIXED;
  int_marker.controls.push_back(control);

//  orien = tf::Quaternion(0.0, 0.0, 1.0, 1.0);
//  orien.normalize();
//  tf::quaternionTFToMsg(orien, control.orientation);
//  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
//  control.orientation_mode = InteractiveMarkerControl::INHERIT;
 // int_marker.controls.push_back(control);

  server->insert(int_marker);
  server->setCallback(int_marker.name, &processFeedback);
}

void makeMenuMarker( const tf::Vector3& position )
{
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "base_link";
  tf::pointTFToMsg(position, int_marker.pose.position);
  int_marker.scale = .5;

  int_marker.name = "context_menu";
  int_marker.description = "Context Menu\n(Right Click)";

  InteractiveMarkerControl control;

  control.interaction_mode = InteractiveMarkerControl::MENU;
  control.name = "menu_only_control";

  Marker marker = MakeBox( int_marker,.5,0,1,0,1 );
  control.markers.push_back( marker );
  control.always_visible = true;
  int_marker.controls.push_back(control);

  server->insert(int_marker);
  server->setCallback(int_marker.name, &processFeedback);
  menu_handler.apply( *server, int_marker.name );
}
// %EndTag(Menu)%
//==========================================================================================
QList<double> QctoRadian( QList<int> qc)
{
     QList<double> rad;
rad.append(0);
rad.append(double((qc[0])/((1/(2*M_PI))*(2304)*100)));
rad.append(double((qc[1])/((-1)*(1/(2*M_PI))*(2304)*100)));
rad.append(double((qc[2])/((1/(2*M_PI))*(2304)*50)));
rad.append(double((qc[3])/(-1*(1/(2*M_PI))*(2304)*80)));
rad.append(double((qc[4])/((1/(2*M_PI))*(2304)*100)));
rad.append(double((qc[5])/((1/(2*M_PI))*(2304)*100)));
rad.append(double((qc[6])/(-1*(1/(2*M_PI))*(2304)*50)));
rad.append(double((qc[7])/((1/(2*M_PI))*(2304)*80)));
rad.append(double((qc[8])/((-1)*(1/(2*M_PI))*(2304)*120)));
rad.append(double((qc[9])/((1/(2*M_PI))*(2304)*120)));
rad.append(double((qc[10])/((1/(2*M_PI))*(2304)*120)));
rad.append(double((qc[11])/(-1*(1/(2*M_PI))*(2304)*120)));
    return rad;
}
//==========================================================================================
void SendTF( QString frame,QString child,  tf::Vector3 location,float roll,float pitch,float yaw){
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( location );
    tf::Quaternion q;
    q.setRPY(roll,pitch,yaw);
    // rotation.setRPY(0, 0,0.5);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),frame.toStdString(), child.toStdString()));
}
//==========================================================================================
void UpdateRobotModel(QList<int> motorPosition)
{
    QList<double> rads= QctoRadian(motorPosition);
    SendTF( "RARM_LINK4", "RARM_LINK7",tf::Vector3(0 ,0, -0.3625),0,0,0);
    SendTF( "LARM_LINK4", "LARM_LINK7",tf::Vector3(0 ,0, -0.3625),0,0,0);


    SendTF( "LARM_LINK3", "LARM_LINK4",tf::Vector3(0 ,0, -0.234),0,0,0);
    SendTF( "RARM_LINK3", "RARM_LINK4",tf::Vector3(0 ,0, -0.234),0,0,0);

    SendTF( "WAIST_LINK2", "LARM_LINK3",tf::Vector3(0 ,0.207, 0.285),0,0,0);
    SendTF( "WAIST_LINK2", "RARM_LINK3",tf::Vector3(0 ,-0.207, 0.285),0,0,0);
    SendTF( "WAIST_LINK2", "LARM_LINK3",tf::Vector3(0 ,0.207, 0.285),0,0,0);
    SendTF( "base_link", "WAIST_LINK2",tf::Vector3(0,0,1.071),0,0,0);


    SendTF( "base_link", "RLeg_Hip_Thigh_Link",tf::Vector3(0,-0.115,.830),rads[10],rads[4],rads[9]);
    SendTF( "base_link", "LLeg_Hip_Thigh_Link",tf::Vector3(0,0.115,.830),rads[11],rads[8],rads[12]);

    //SendTF( "base_link", "RLeg_Hip_Thigh_Link",tf::Vector3(0,-0.115,.942),0,0,0);
    //SendTF( "base_link", "LLeg_Hip_Thigh_Link",tf::Vector3(0,0.115,.942),0,0,0);

    SendTF( "LLeg_Hip_Thigh_Link", "LLeg_Hip_Shank_Link",tf::Vector3(0,0,-0.37),0,rads[7],0);
    SendTF( "RLeg_Hip_Thigh_Link", "RLeg_Hip_Shank_Link",tf::Vector3(0,0,-0.37),0,rads[3],0);
    SendTF( "LLeg_Hip_Shank_Link", "LLeg_Foot_Link",tf::Vector3(0,0,-.36),rads[6],rads[5],0);
    SendTF( "RLeg_Hip_Shank_Link", "RLeg_Foot_Link",tf::Vector3(0,0,-.36),rads[1],rads[2],0);

}
void UpdateRobotModel(QList<double> rads)
{
//qDebug()<<"rads="<<rads[3];
    SendTF( "RARM_LINK4", "RARM_LINK7",tf::Vector3(0 ,0, -0.3625),0,0,0);
    SendTF( "LARM_LINK4", "LARM_LINK7",tf::Vector3(0 ,0, -0.3625),0,0,0);


    SendTF( "LARM_LINK3", "LARM_LINK4",tf::Vector3(0 ,0, -0.234),0,0,0);
    SendTF( "RARM_LINK3", "RARM_LINK4",tf::Vector3(0 ,0, -0.234),0,0,0);

    SendTF( "WAIST_LINK2", "LARM_LINK3",tf::Vector3(0 ,0.207, 0.285),0,0,0);
    SendTF( "WAIST_LINK2", "RARM_LINK3",tf::Vector3(0 ,-0.207, 0.285),0,0,0);
    SendTF( "WAIST_LINK2", "LARM_LINK3",tf::Vector3(0 ,0.207, 0.285),0,0,0);
    SendTF( "base_link", "WAIST_LINK2",tf::Vector3(0,0,1.071),0,0,0);


    SendTF( "base_link", "RLeg_Hip_Thigh_Link",tf::Vector3(0,-0.115,.830),rads[10],rads[4],rads[9]);
    SendTF( "base_link", "LLeg_Hip_Thigh_Link",tf::Vector3(0,0.115,.830),rads[11],rads[8],rads[12]);

    //SendTF( "base_link", "RLeg_Hip_Thigh_Link",tf::Vector3(0,-0.115,.942),0,0,0);
    //SendTF( "base_link", "LLeg_Hip_Thigh_Link",tf::Vector3(0,0.115,.942),0,0,0);

    SendTF( "LLeg_Hip_Thigh_Link", "LLeg_Hip_Shank_Link",tf::Vector3(0,0,-0.37),0,rads[7],0);
    SendTF( "RLeg_Hip_Thigh_Link", "RLeg_Hip_Shank_Link",tf::Vector3(0,0,-0.37),0,rads[3],0);
    SendTF( "LLeg_Hip_Shank_Link", "LLeg_Foot_Link",tf::Vector3(0,0,-.36),rads[6],rads[5],0);
    SendTF( "RLeg_Hip_Shank_Link", "RLeg_Foot_Link",tf::Vector3(0,0,-.36),rads[1],rads[2],0);

}
//==========================================================================================
void VisualizeInit(ros::NodeHandle*nodeHandler)
{


marker_pub = nodeHandler->advertise<visualization_msgs::Marker>("visualization_marker", 1);
server.reset( new interactive_markers::InteractiveMarkerServer("basic_controls","",false) );

tf::Vector3 position;
position = tf::Vector3(0.5, 0.5, 0);
//menu_handler.insert( "First Entry", &processFeedback );
//menu_handler.insert( "Second Entry", &processFeedback );
interactive_markers::MenuHandler::EntryHandle sub_menu_handle = menu_handler.insert( "MAIN Operations" );
menu_handler.insert( sub_menu_handle, "Home", &processFeedback );
menu_handler.insert( sub_menu_handle, "Init", &processFeedback );
menu_handler.insert( sub_menu_handle, "Reset", &processFeedback );
 makeMenuMarker( position );
//MakePanTilt2Marker( "RLeg_Foot_Link","RLeg_Foot_Link1",position ,0,0.0,0.0,.3);//reds

//MakePanTilt2Marker( "LLeg_Foot_Link","LLeg_Foot_Link1",position ,M_PI,0.0,0.0,.3);//reds

//MakePanTilt2Marker( "RLeg_Foot_Link","RLeg_Foot_Link2",position ,0,0,-M_PI_2+.001,.3);

//MakePanTilt2Marker( "LLeg_Foot_Link","LLeg_Foot_Link2",position ,0,0,-M_PI_2+.001,.3);


//MakePanTilt2Marker( "RLeg_Hip_Shank_Link","RLeg_Hip_Shank_Link",position ,0,0.0,-M_PI_2+.001,.3);

//MakePanTilt2Marker( "LLeg_Hip_Shank_Link","LLeg_Hip_Shank_Link",position ,0,0.0,-M_PI_2+.001,.3);


//MakePanTilt2Marker( "LLeg_Hip_Thigh_Link","LLeg_Hip_Thigh_Link1",position ,0,0.0,-M_PI_2+.001,.3);

//MakePanTilt2Marker( "RLeg_Hip_Thigh_Link","RLeg_Hip_Thigh_Link1",position ,0,0.0,-M_PI_2+.001,.3);

//MakePanTilt2Marker( "LLeg_Hip_Thigh_Link","LLeg_Hip_Thigh_Link2",position ,M_PI,0.0,0,.3);

//MakePanTilt2Marker( "RLeg_Hip_Thigh_Link","RLeg_Hip_Thigh_Link2",position ,M_PI,0.0,0,.3);
server->applyChanges();



}
//==========================================================================================
 void DisplayError(QString msg,int markerID,QString frameID,float x,float y,float z)
 {


     marker.header.frame_id = frameID.toStdString();//"/base_link";
     marker.header.stamp = ros::Time::now();
     marker.ns = "basic_shapes";
     marker.id = markerID;
     marker.type = visualization_msgs::Marker::ARROW;
     marker.action = visualization_msgs::Marker::ADD;


     marker.pose.position.x =x;
     marker.pose.position.y = y;
     marker.pose.position.z = z;
     marker.pose.orientation.x = 0.0;
     marker.pose.orientation.y = 0.0;
     marker.pose.orientation.z = 0.0;
     marker.pose.orientation.w = 1.0;

    // marker.text =msg.toStdString();

     marker.scale.x = 0.2;
     marker.scale.y = 0.05;
     marker.scale.z = 0.05;

     marker.color.r = 1.0f;
     marker.color.g = 0.0f;
     marker.color.b = 0.0f;
     marker.color.a = 1.0;
     //marker.lifetime = ros::Duration();
          marker.lifetime=ros::Duration(1.0);
 marker_pub.publish(marker);
 }
//==========================================================================================
 void DisplayText(QString msg,int markerID,QString frameID,float x,float y,float z)
 {


     marker.header.frame_id = frameID.toStdString();//"/base_link";
     marker.header.stamp = ros::Time::now();
     marker.ns = "basic_shapes";
     marker.id = markerID;
     marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
     marker.action = visualization_msgs::Marker::ADD;


     marker.pose.position.x =x;
     marker.pose.position.y = y;
     marker.pose.position.z = z;
     marker.pose.orientation.x = 0.0;
     marker.pose.orientation.y = 0.0;
     marker.pose.orientation.z = 0.0;
     marker.pose.orientation.w = 1.0;

     marker.text =msg.toStdString();

     marker.scale.x = 0.3;
     marker.scale.y = 0.3;
     marker.scale.z = 0.1;

     marker.color.r = 0.0f;
     marker.color.g = 1.0f;
     marker.color.b = 0.0f;
     marker.color.a = 1.0;
     marker.lifetime = ros::Duration();
 marker_pub.publish(marker);
 }
 //==========================================================================================
 void DisplayLink()
 {
     visualization_msgs::Marker marker;
     marker.header.frame_id = "base_link";
     marker.header.stamp = ros::Time();
     marker.ns = "my_namespace";
     marker.id = 0;
     marker.type = visualization_msgs::Marker::MESH_RESOURCE;
     marker.action = visualization_msgs::Marker::ADD;
     marker.pose.position.x = 1;
     marker.pose.position.y = 1;
     marker.pose.position.z = 1;
     marker.pose.orientation.x = 0.0;
     marker.pose.orientation.y = 0.0;
     marker.pose.orientation.z = 0.0;
     marker.pose.orientation.w = 1.0;
     marker.scale.x = 1;
     marker.scale.y = 1;
     marker.scale.z = 1;
     marker.color.a = 1.0; // Don't forget to set the alpha!
     marker.color.r = 0.5;
     marker.color.g = 0.5;
     marker.color.b = 0.5;
     //only if using a MESH_RESOURCE marker type:
     marker.mesh_resource = "package://robot_sim/meshes/Foot-Left.STL";
 marker_pub.publish(marker);

 }

