#include "ros/ros.h"
#include "std_msgs/String.h"

#include <std_msgs/Empty.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <QEventLoop>
#include <qdebug.h>
#include <QTimer>
#include "robot_teleop/active_csp.h"
#include "robot_teleop/reset_node.h"
#include "robot_teleop/home.h"
#include <QApplication>
#include <std_msgs/Int32MultiArray.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include "visualize.h"
#include "VisualizeControl.h"
#include <tf/transform_datatypes.h>

#include "qnode.h"
#include "robot.h"
#include "QsLog/QsLog.h"


using namespace  std;
using namespace QsLogging;
int _dataIndex=0;
QList<int> _motorPosition;

// 1. init the logging mechanism

void logFunction(const QString &message, QsLogging::Level level)
{
 qDebug()<<  qPrintable(message) << " file:"<<__FILE__<<" function:"<<__FUNCTION__<<" line:"<<__LINE__ ;

}


int main(int argc, char **argv)
{
    /*********************
    ** Qt
    **********************/
    QApplication app(argc, argv);
    Logger& logger = Logger::instance();
    logger.setLoggingLevel(QsLogging::TraceLevel);
    const QString sLogPath(QDir(app.applicationDirPath()).filePath("log.txt"));
    // 2. add two destinations
     DestinationPtr fileDestination(DestinationFactory::MakeFileDestination(
       sLogPath, EnableLogRotation, MaxSizeBytes(512), MaxOldLogCount(2)));
     DestinationPtr debugDestination(DestinationFactory::MakeDebugOutputDestination());

     DestinationPtr functorDestination(DestinationFactory::MakeFunctorDestination(&logFunction));
     logger.addDestination(debugDestination);

     //logger.addDestination(fileDestination);
     //logger.addDestination(functorDestination);

     // 3. start logging
     QLOG_INFO() << "Program started";
     QLOG_INFO() << "Built with Qt" << QT_VERSION_STR << "running on" << qVersion();
     QLOG_TRACE()<<"Instantiate robot";
     Robot w(nullptr,argc,argv);
      QLOG_TRACE()<<"Robot Created !!";
   // w.show();
//   app.connect(&w, SIGNAL(, &app, SLOT(quit()));

   return app.exec();

}
