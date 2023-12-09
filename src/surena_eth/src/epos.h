#ifndef EPOS_H
#define EPOS_H

#include <QObject>
#include <QDebug>
#include <qthread.h>
#include <qlist.h>
#include "can.h"
#include "pingmodel.h"
#include "ethernetpackets.h"
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <geometry_msgs/Accel.h>
#include <geometry_msgs/Wrench.h>
#include <tf2/LinearMath/Quaternion.h>

//#include "tcphandler.h"
#define USE_NET

#define RunWithPostDelay(function,delay)\
    function;\
    WaitMs(delay);




enum EPOSErrors { OK=0,USB_ERROR, BOARD_ERROR, CAN_ERROR,SDO_REJECT,SDO_BAD_REPLAY,NO_ANSER,NETWOR_ERROR,EPOS_ERROR};
enum EPOSOperationMode {PPM=1,PVM=3,HMM=6,CSP=8,CST=10};
enum {LeftBumpSensorCanID=0x382,RightBumpSensorCanID=0x383 ,palmCanID=0x0352,ReserveByteCount=180 };

enum {RightShulderPitch=12,RightShulderRoll=13};

class Epos : public QObject
{
    Q_OBJECT
    QMap< int,QString> ErrorCodes;
    const double m_dDecouplingCoefficient [6][6]={
        {1,0,0,0,0,0},
        {0,1,0,0,0,0},
        {0,0,1,0,0,0},
        {0,0,0,1,0,0},
        {0,0,0,0,1,0},
        {0,0,0,0,0,1},



        //{1033.78406318488,0,0,0,0,0},
        //{0,1037.08620260516,0,0,0,0},
        //{0,0,7022.47191011235,0,0,0},
        //{0,0,0,92.2339051835454,0,0},
        //{0,0,0,0,92.6698174404596,0},
        //{0,0,0,0,0,60.819851599562},
    };
    const double sensitivityFTRight[6]=  // #573 Right foot FTsensor
    {
            9.6732*0.0001,
            9.6424*0.0001,
            1.4240*0.0001,
            1.0842*0.01,
            1.0791*0.01,
            1.6442*0.01
};
    const double sensitivityFTLeft[6]= // #574 Left foot FTsensor
    {
            9.5469*0.0001,
            9.6402*0.0001,
            1.4234*0.0001,
            1.0638*0.01,
            1.0505*0.01,
            1.6489*0.01

};
    const double offsetFTRight[6]= // #573 Right foot FTsensor
    {
            //            32778,//32767,
            //            //{32703},
            //            32639,//32625,
            //            //{32720},
            //            32669,//32655,
            //            //{32689},
            //            32650,//32585,
            //            //{32639},
            //            32696,//32750,
            //            //{32663},
            //            32613//32550
            32776,
            32663,
            32700,
            32633,
            32673,
            32612
};
    const double offsetFTLeft[6]= // #574 Left foot FTsensor
    {
            //            //{32689},
            //            32670,//32660,
            //            //{32762},
            //            32773,//32775,
            //            //{32761},
            //            32781,//32785,
            //            //{32691},
            //            32772,//32780,
            //            //{32749},
            //            32647,//32675,
            //            //{32739},
            //            32663//32700
            32684,
            32770,
            32797,
            32749,
            32677,
            32660
};
    const double gainFTRight[6]= // #573 Right foot FTsensor
    {
            123.3246,
            123.3079,
            123.3880,
            123.4881,
            123.5682,
            123.3213
};
    const double gainFTLeft[6]= // #574 Left foot FTsensor
    {
            123.5760,
            123.5726,
            123.5427,
            123.5660,
            123.5992,
            123.4928

};
    const double ExFTRight[6]= // #573 Right foot FTsensor
    {
            4.971,
            4.971,
            4.971,
            4.971,
            4.971,
            4.971

};
    const double ExFTLeft[6]= // #574 Left foot FTsensor
    {
            4.9846,
            4.9846,
            4.9846,
            4.9846,
            4.9846,
            4.9846
};

    const unsigned char _bumpSensorCommand[8]={0x01,0x01,0x01,0x20,0x01,0x01,0x01,0x01};
    uint16_t bump_sensor_left[4];
    uint16_t bump_sensor_right[4];

    QList<uint16_t> bump_sensor_list;
    QList<int32_t> positions;
    QList<int32_t> positionsInc;
    QList<int16_t> ft;
    QList<float> imu_data_list;

    IMUReceivedPacketType *ImuPacket;
    EthernetReceivedPacketType *incommingPacket;
    BumpSensorPacket *bumpPacket;
    QByteArray LastPacketreceived;

    //void CheckCanBoardRequest();
    unsigned char GetSDOCODE(int len);
    void InitErrorMap();

    QByteArray MotorDataToArray(int canID, int position);
    QByteArray CreateHandPacket(QList<int> motorPositions);
    QByteArray CreatePDOPacket(int canID, int value1, int value2);
    QByteArray CreateDynamixelPacket(int canID, int motorID, int motorPosition, int velocity);
    QByteArray CreateBumpRequestCommand();
     QByteArray CreateServoHeadCommand(QList<int> motorPositions);
    QByteArray CreateWaistAndHeadCommand(QList<int> motorPositions);

    bool IsValidRunPacket(QByteArray packet);
    void GetFTSensorDataFromPacket(EthernetReceivedPacketType *packet);
    void GetIMUDataFromPacket(EthernetReceivedPacketType *packet);
    void GetBumpDataFromPacket(BumpSensorPacket *packet);
    void GetPositionDataFromPacket(EthernetReceivedPacketType *packet);
public:
    sensor_msgs::Imu IMU;
    sensor_msgs::MagneticField MagneticSensor;
    geometry_msgs::Accel Acceleration;
    geometry_msgs::Wrench ForceTorqSensorRight,ForceTorqSensorLeft;
    PingModel ping;
    Can can;
    //================================================================================================================================================================
    explicit Epos(QObject *parent = 0);

    //================================================================================================================================================================
    EPOSErrors Init(int tryCount); //
    //================================================================================================================================================================
    EPOSErrors EnableDevice(int devID,EPOSOperationMode mode);
    //================================================================================================================================================================
    void SetPreoperationalMode(int devID, int nodeID=1);
    //================================================================================================================================================================
    void ResetComunication(int devID);
    //================================================================================================================================================================
    void StartNode(int devID);
    //================================================================================================================================================================
    void SetMode(int devID, EPOSOperationMode mode, int canID=1);
    //================================================================================================================================================================
    void StopNode(int devID);
    //================================================================================================================================================================
    void SwitchOn(int devID, int canID=1);
    //================================================================================================================================================================
    void SwitchOff(int devID, int canID=1);
    //================================================================================================================================================================
    void ResetNode(int devID)throw(std::runtime_error);
    //================================================================================================================================================================
    EPOSErrors SDOWriteCommand(int id, unsigned long input, int index, unsigned char sub_index, unsigned char len, char devID);
    //================================================================================================================================================================
    void SDOReadCommand(int id, int index, unsigned char subIndex, char devID, QByteArray &replay);
    //================================================================================================================================================================
    int32_t ReadRegisterValue(int index, int subIndex, int canID, int devID, int timeout)throw(std::runtime_error);
    //================================================================================================================================================================
    int32_t ReadRegister(int index, int subIndex, int canID, int devID, int timeout, int trycount);
    //================================================================================================================================================================
    EPOSErrors WriteRegister(int index, int subIndex, int canID, int devID, int32_t value, int len=4);
    //================================================================================================================================================================
    EPOSErrors ActivePPM(int canID, int devId);
    //================================================================================================================================================================
    bool SetPosition(int canID, int devId, int position, int velocity);
    //================================================================================================================================================================
    QString ReadCurrentError(int canID, int devID);
    //================================================================================================================================================================
    bool ActiveCSP(int nodeID, bool switchOn=true);
    //================================================================================================================================================================
    bool AllActiveCSP();
    //================================================================================================================================================================
    //void SetPositionCST(int position, int velocity);
    //================================================================================================================================================================
    void SetAllPositionCST(QList<int> motorPositions);
    //================================================================================================================================================================
    bool ActiveAllCSP();
    //================================================================================================================================================================
    bool ActiveAllHands(bool switchOn);
    //================================================================================================================================================================
    float QByteArrayToFloat(QByteArray arr);
    //================================================================================================================================================================
    EPOSErrors ReadAllRegisters(int index, int subIndex, int canID, QList<int32_t> &value, int timeout);
    //================================================================================================================================================================
    bool CheckZynq();
    //================================================================================================================================================================
    // EPOSErrors HandsInit(int tryCount);
    //================================================================================================================================================================
    bool ActivePPMPDO(int nodeID, int canID);
    //================================================================================================================================================================
    //bool ActiveHand();
       bool ActiveHand(int nodeID,bool switchOn=true);
 
    //================================================================================================================================================================
    void WaitMs(int ms);
    //================================================================================================================================================================

    ~Epos()
    {
        delete ImuPacket;
        delete incommingPacket;
        delete  bumpPacket;
        QLOG_TRACE()<<"clean and exit Epos";
    }
    //================================================================================================================================================================


    bool ActiveJoint(int joint, bool enableDrive=true);
    bool ActiveWaist(bool enableDrive=true);
    bool ActiveLegs(bool switchOn=true);
signals:

    void NewDataReady();
    //================================================================================================================================================================

    void Dummy();
    //================================================================================================================================================================

    void  FeedBackReceived(QList<int16_t> ft, QList<int32_t> positions,QList<int32_t> positionsInc,QList<uint16_t> bump_sensor_list,QList<float> imu_data_list);
    //================================================================================================================================================================


public slots:


    void DataReceived(QByteArray data);
    //================================================================================================================================================================

};



#endif // EPOS_H
