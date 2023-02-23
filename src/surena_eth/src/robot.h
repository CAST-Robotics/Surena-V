#ifndef ROBOT_H
#define ROBOT_H

#include <QObject>
#include <QDebug>
#include <QTimer>

#include "qnode.h"
#include "epos.h"
#include "pingmodel.h"
#include "pidcontroller.h"
#include"QsLog/QsLog.h"

class Robot : public QObject
{
    Q_OBJECT
    Epos Epos4;
    QNode *_rosNode;
    QTimer timer;
    QTimer _hommingTimer;
    QTimer _initialTimer;
    QTimer _statusCheckTimer;
     QList<int> _motorPosition;
    bool Initialized=false;
    float CurrentAbsPositions[40];
    int HommingState=0;
    float CurrentIncPositions[40];

  // //  const double offset[12]={ -790 ,-86, 315, 433, 614, -266, -33 ,374, -277+39, 593, 748, 339-8};
   // //const double offset[12]={ -770-57   ,-86, 315, 433, 614, -246+57   , -33 ,374, -277+39, 573+57  , 728-57  , 339-8};
//    const double offset[12]={ -779  ,-86, 315, 433, 614, -255   , -33 ,374, -277+39, 582  , 737  , 339-8};
   // const double offset[12]={ -779-57  ,-86, 315, 433, 614, -255+57   , -33 ,374, -277+39, 582+57  , 737-57  , 339-8};
//    const double offset[12]={ -775  ,-86, 315, 433, 614, -245   , -33 ,374, -277, 573  , 728  , 339};
    //using home stand first check
    //const double offset[12]={ -772  ,-143,-127, 443, 669, -264 ,  342 ,399, 324, 575  , 739  , 270};
//    const double offset[13]={ -780  ,-144,-114, 443, 646, -250 ,  337 ,399, 574, 577  , 738  , 294,-727};
const double offset[13]={ -787  ,-114,-113, 446, 638, -247 ,  334 ,397, 573, 576  , 738  , 295,-727};//gooth
//const double offset[13]={ -787, -129, -111, 450, 640 , -244, 336, 392, 575, 578, 736, 295, -883};//stand
//ABS  0 = -787
//ABS  1 = -129
//ABS  2 = -111
//ABS  3 = 450
//ABS  4 = 640
//ABS  5 = -244
//ABS  6 = 336
//ABS  7 = 392
//ABS  8 = 575
//ABS  9 = 578
//ABS  10 = 736
//ABS  11 = 295
//ABS waist = -883

    const double ratio[13]={ 1,-1,1,-1,1,1,-1,1,-1,-1,-1,-1,1};
    //const double Direction[12]={1,-1,1,-1,1,1,-1,1,-1,1,1,-1};

        const double Direction[13]={1,1,1,1,1,1,1,1,1,1,1,1,1};
    const int HomeOrder[13]={0,1,2,3,8,9,5,4,6,7,10,11,12};
    int currentHomeIndex=0;

    int pos;
    bool dir=false;
    PIDController pid;
public:

    //=================================================================================================
    explicit Robot(QObject *parent ,int argc, char **argv);
    //=================================================================================================


    void WaitMs(int ms);


signals:


public slots:
        void ReadErrors();
           bool ReadAllInitialPositions();
    void StatusCheck();
    //=================================================================================================
    void Initialize();
    //=================================================================================================
    void Home(int id);
    //=================================================================================================
    void CleanAndExit();
    //=================================================================================================
    void NewjointDataReceived( );
    //=================================================================================================
    void Timeout();
    //=================================================================================================
    void HommingLoop();
    //=================================================================================================
  //  void FeedBackReceived(QList<int16_t>ft,QList<int32_t>positionAbs,QList<int32_t>positionInc);
   // void FeedBackReceived(QList<int16_t> ft, QList<int32_t> positions, QList<int32_t> positionsInc, QList<int16_t> bump_sensor_list, QList<float> imu_data_list);
    void  FeedBackReceived(QList<int16_t> ft, QList<int32_t> positions,QList<int32_t> positionsInc,QList<uint16_t> bump_sensor_list,QList<float> imu_data_list);

    //=================================================================================================
    void ActiveCSP(int id);
    //=================================================================================================
    void ResetAllNodes(int id);
    //=================================================================================================

    void ResetHands();
    void ActivateHands();
    void ActivateLegs();
    void ResetLegs();
};

#endif // ROBOT_H
