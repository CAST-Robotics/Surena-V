#include "epos.h"
//#include"QsLog/QsLogDisableForThisFile.h"
//========================================================================
Epos::Epos(QObject *parent) : QObject(parent)
{
    for(int i=0;i<8;i++)
    {
        bump_sensor_list.append(0x1);
    }
    for(int i=0;i<16;i++)
    {
        positions.append((char)0x00);
        positionsInc.append((char)0x00);
        imu_data_list.append((char)0x00);
        ft.append((char)0x00);
    }
    InitErrorMap();
    qDebug()<<"all printed values are hex";
    can.Init();
    //connect(&tcp,SIGNAL(NewDataReceived(QByteArray)),this,SLOT(DataReceived(QByteArray)));
    connect(&can,SIGNAL(NewDataReceived(QByteArray)),this,SLOT(DataReceived(QByteArray)));
}
//========================================================================
bool Epos::ActiveJoint(int joint,bool enableDrive)
{
    ActiveCSP(joint,enableDrive);
    ActiveHand(joint,enableDrive);
}
//========================================================================
bool Epos::ActiveCSP(int nodeID,bool switchOn)
{
    if(nodeID>12 && nodeID!=255)return false;
    RunWithPostDelay(SetPreoperationalMode(nodeID),700);
    RunWithPostDelay(StartNode(nodeID),700);
    RunWithPostDelay(SetMode(nodeID,CSP),700);
    RunWithPostDelay(SwitchOff(nodeID),700);
    if(switchOn) RunWithPostDelay(SwitchOn(nodeID),700);
    return OK;
}
//========================================================================
bool Epos::ActiveHand(int nodeID,bool switchOn) //13,2
{


    qDebug()<<"active node";
    if(nodeID<13)return false;
     if(nodeID==255)
     {
        ActiveAllHands(switchOn); 
        return OK;
    }
    nodeID-=12;
    // qDebug()<<"node:"<<nodeID<<" devid:"<<switchOn;

    // int devid=12;
    // nodeID-=11;
    // if(nodeID>4){nodeID-=8;devid=13;}
    // if(nodeID>4 || nodeID<1)return false;
    int devid=13;


//node 13 works for this
    qDebug()<<"p2 node:"<<nodeID<<" devid:"<<devid;

    WaitMs(700);
    SetPreoperationalMode(devid,nodeID);
    WaitMs(700);
    StartNode(devid);
    WaitMs(700);
    SetMode(devid,PPM,nodeID);
    WaitMs(700);
    SwitchOff(devid,nodeID);
    WaitMs(700);
    if(!switchOn)return OK;
    SwitchOn(devid,nodeID);
    WaitMs(700);
    return OK;
}
//========================================================================
bool Epos::ActiveLegs(bool switchOn) //13,2
{
int nodeID=1;
    WaitMs(700);
    for(int i=0;i<12;i++)
    SetPreoperationalMode(i+0,nodeID);
    WaitMs(700);
    for(int i=0;i<12;i++)
    StartNode(i+0);
    WaitMs(700);
    for(int i=0;i<12;i++)
    SetMode(i+0,CSP,nodeID);
    WaitMs(700);
    for(int i=0;i<12;i++)
    SwitchOff(i+0,nodeID);
    WaitMs(700);
    if(!switchOn)return OK;
    for(int i=0;i<12;i++)
    SwitchOn(i+0,nodeID);
    WaitMs(700);
    return OK;
}
//========================================================================

bool Epos::ActiveWaist(bool enableDrive) //14
{
    int nodeID=1;
    int devID=14;
    SetPreoperationalMode(devID,nodeID);
    WaitMs(700);
    StartNode(devID);
    WaitMs(700);
    SetMode(devID,PPM,nodeID);
    WaitMs(700);
    SwitchOff(devID,nodeID);
    WaitMs(700);
      if(!enableDrive)return OK;
    SwitchOn(devID,nodeID);
    WaitMs(700);
    return OK;
}
//========================================================================
bool Epos::ActiveAllHands(bool switchOn) //13left,12right
{

    WaitMs(700);
    for(int i=0 ;i<4;i++){
        SetPreoperationalMode(12,i+1);
        SetPreoperationalMode(13,i+1);
    }
    WaitMs(700);
     StartNode(12);
    StartNode(13);
    WaitMs(700);
    for(int i=0 ;i<4;i++){
       SetMode(12,PPM,i+1);
        SetMode(13,PPM,i+1);
        WaitMs(700);

    }
    WaitMs(700);
    for(int i=0 ;i<4;i++){
       SwitchOff(12,i+1);
        SwitchOff(13,i+1);
    }
    WaitMs(700);
    if(!switchOn)return OK;
    for(int i=0 ;i<4;i++){
        SwitchOn(12,i+1);
        SwitchOn(13,i+1);
    }
    WaitMs(700);
    return OK;
}
//========================================================================
bool Epos::ActivePPMPDO(int nodeID,int canID) //13,2
{
    RunWithPostDelay(SetPreoperationalMode(nodeID,canID),700);
    RunWithPostDelay(StartNode(nodeID),700);
    RunWithPostDelay(SetMode(nodeID,PPM,canID),700);
    RunWithPostDelay(SwitchOff(nodeID,canID),700);
    RunWithPostDelay(SwitchOn(nodeID,canID),700);
    return OK;
}
//========================================================================
void Epos::ResetNode(int devID)throw(std::runtime_error)
{
    try{
        QByteArray data,replay;
        data.append(0x81);
        data.append(QByteArray(7, Qt::Initialization::Uninitialized));
        replay=can.WriteMessage(0,devID,data);
    }
    catch(std::runtime_error e)
    {
        throw std::runtime_error (e.what());
    }
}
//========================================================================
EPOSErrors Epos::EnableDevice(int devID,EPOSOperationMode mode)
{
    RunWithPostDelay(SetPreoperationalMode(devID),700);
    RunWithPostDelay(StartNode(devID),700);
    RunWithPostDelay(SetMode(devID,mode),700);
    RunWithPostDelay(SwitchOff(devID),700);
    RunWithPostDelay(SwitchOn(devID),700);
    return OK;
}
//========================================================================
void Epos::SetPreoperationalMode(int devID,int nodeID)
{
    try{
        QByteArray data,replay;
        data.append(0x80);     // ???????????? check 0x80 (for 0x01)
        data.append(QByteArray(7, Qt::Initialization::Uninitialized));
        replay=can.WriteMessage(00,devID,data);
    }
    catch(...)
    {}
}
//========================================================================
void  Epos::ResetComunication(int devID)
{    try{
        QByteArray data,replay;
        data.append(0x82);
        data.append(QByteArray(7, Qt::Initialization::Uninitialized));
        replay=can.WriteMessage(0,devID,data);
     }
     catch(...)
     {}

}
//========================================================================
void Epos::StartNode(int devID)
{
    try{
        QByteArray data,replay;
        data.append(0x01);
        data.append(QByteArray(7, Qt::Initialization::Uninitialized));

        replay=can.WriteMessage(0,devID,data);
    }
    catch(...)
    {}

}
//========================================================================
void Epos::StopNode(int devID)
{
    try{
        QByteArray data,replay;
        data.append(0x02);
        data.append(QByteArray(7, Qt::Initialization::Uninitialized));
        replay=can.WriteMessage(0,devID,data);
    }
    catch(...)
    {}

}
//========================================================================
//  SetMode(12,PPM,i+1);
void Epos::SetMode(int devID,EPOSOperationMode mode,int canID)
{
    try{
        QByteArray data,replay;
        data.append(mode);
        data.append((char)0x00);

        data.append(QByteArray(6, Qt::Initialization::Uninitialized));
        replay=can.WriteMessage(0x300+canID,devID,data);
        //if(devID==13)qDebug()<<replay.toHex();
    }
    catch(...)
    {

    }

}
//========================================================================
void Epos::SwitchOn(int devID ,int canID)
{
    try{
        QByteArray data,replay;
        data.append(0x0f);
        data.append( QByteArray(7, Qt::Initialization::Uninitialized));
        replay=can.WriteMessage(0x200+canID,devID,data);
    }
    catch(...)
    {}

}
//========================================================================
void Epos::SwitchOff(int devID,int canID)
{
    try{
        QByteArray data,replay;
        data.append(0x06);
        data.append( QByteArray(7, Qt::Initialization::Uninitialized));
        replay= can.WriteMessage(0x200+canID,devID,data);
    }
    catch(...)
    {}

}
//========================================================================
EPOSErrors Epos::ActivePPM(int canID,int devId){
    qDebug()<<"pvt";
    //return WriteRegister(0x607A,0,canID,devId,4);
    WriteRegister(0x6060,0,canID,devId,PPM,1);//set mode
    QThread::sleep(1);
    WriteRegister(0x6040,0,canID,devId,0x06,2);//set mode
    QThread::sleep(1);
    WriteRegister(0x6040,0,canID,devId,0x0f,2);//set mode
    QThread::sleep(1);
    WriteRegister(0x6040,0,canID,devId,0x010f,2);//set mode
    return OK;
}
//========================================================================
EPOSErrors Epos::SDOWriteCommand( int id,unsigned long int input,int index,unsigned char subIndex,unsigned char len,char devID)
{
    //epos 2 application note collection page 155
    QByteArray replay;
    int16_t inp_index=0;//,inpid,;
    QByteArray data;
    data.append (GetSDOCODE(len));
    data.append((unsigned char)(index & 0xff));
    data.append((index >> 8)&0xff);
    data.append(subIndex&0xff);
    data.append((char)input & 0xff);
    data.append((char)((input >> 8) & 0xff));
    data.append((char)(input >> 16) & 0xff);
    data.append((char)(input >> 24) & 0xff);

    replay=can.WriteMessage(id+0x600, devID,data);
    if (replay.length() < 10)
    {
        return SDO_REJECT;
    }

    inp_index=(replay[8]&0xff);
    inp_index<<=8;
    inp_index+=(replay[9]&0xff);

    return OK;
}
//========================================================================
void Epos::SDOReadCommand( int id,int index,unsigned char subIndex,char devID,QByteArray &replay)
{
    QByteArray data;
    data.append ((unsigned char)GetSDOCODE(0));
    data.append((unsigned char)(index & 0xff));
    data.append((index >> 8)&0xff);
    data.append(subIndex&0xff);
    data.append( QByteArray(4, Qt::Initialization::Uninitialized));
    replay=can.WriteMessage(id,devID,data);
}
//========================================================================
unsigned char Epos::GetSDOCODE(int len)
{
    if (len == 1) { return 0x2f; }
    if (len == 2) { return 0x2b; }
    if (len == 4) { return 0x23; }
    if (len == -1) { return 0x22; }
    if (len == 0) { return 0x40; }
    if (len == 9) { return 0x81; }
    return 0;
}
//========================================================================
int32_t Epos::ReadRegister(int index,int subIndex,int canID, int devID,int timeout,int trycount)
{
    return  ReadRegisterValue(index,subIndex,canID,devID,timeout);
}
//========================================================================
int32_t  Epos::ReadRegisterValue(int index,int subIndex,int canID, int devID,int timeout)throw(std::runtime_error)
{
    try{
        uint16_t inpid;
        QByteArray replay;
        int32_t value;

        can.ReadMessage(devID);
        can.ReadMessage(devID);
        SDOReadCommand(canID+0x600,index,subIndex,devID,replay);
        replay=can.ReadMessage(devID);

        inpid=replay[1]&0xff;
        inpid<<=8;
        inpid+=replay[0]&0xff;
        if((inpid==0x580+canID))
        {
            // PAGE 150 APPLICATION NOTE COLLECTIOIN
            value=0;
            value=replay[9]&0xff;
            value=value<<8;
            value+=replay[8]&0xff;
            value=(value)<<8;
            value+=replay[7]&0xff;
            value=(value)<<8;
            value+=replay[6]&0xff;
        }
        return value;
    }
    catch(const std::runtime_error e)
    {
        QLOG_ERROR()<<e.what();
    }
}
//========================================================================
EPOSErrors Epos::ReadAllRegisters(int index,int subIndex,int canID,QList< int32_t> &value,int timeout)
{
    uint16_t inpid;
    QByteArray replay;

    QLOG_TRACE()<<"Read 255";
    can.ReadMessage(255);
    QLOG_TRACE()<<"Read 255";
    can.ReadMessage(255);

    replay.clear();
    QLOG_TRACE()<<"sdo read";
    SDOReadCommand(canID+0x600,index,subIndex,255,replay);
    //can.ReadMessage(255,replay);
    if(replay.length()!=169)
    {
        //qDebug()<<"Invalid lengh!"<<replay.toHex()<<replay.length();
        QLOG_ERROR()<<"Read All invalid lengh:"<<replay.length()<<" data:"<<replay.toHex();
        return SDO_BAD_REPLAY;
    }
    QLOG_TRACE()<<"befor for:"<<replay.toHex();
    replay=replay.remove(0,4);
    for(int i=0;i<16;i++)

    {
        inpid=replay[i*10+1]&0xff;
        inpid<<=8;
        inpid+=replay[i*10+0]&0xff;
        // if(inpid==(0x580+1))
        // {
        // PAGE 150 APPLICATION NOTE COLLECTIOIN
        int tempValue=0;
        tempValue=replay[i*10+9]&0xff;
        tempValue=tempValue<<8;
        tempValue+=replay[i*10+8]&0xff;
        tempValue=(tempValue)<<8;
        tempValue+=replay[i*10+7]&0xff;
        tempValue=(tempValue)<<8;
        tempValue+=replay[i*10+6]&0xff;
        value.append(tempValue);
        QLOG_TRACE()<<"Read result "<<i<<" ="<<QString::number(tempValue,16)<<QString::number(inpid,16);
    }

    return OK;
}
//========================================================================
EPOSErrors Epos::WriteRegister(int index,int subIndex,int canID, int devID,int32_t value,int len)
{
    QByteArray data_read;
    data_read=can.ReadMessage(devID);
    QThread::msleep(5);
    data_read=can.ReadMessage(devID);
    QThread::msleep(5);
    return SDOWriteCommand(canID,value,index,subIndex,len,devID);
}
//========================================================================
QString Epos::ReadCurrentError(int canID,int devID)
{
    EPOSErrors status;
    int value=0;
    value= ReadRegister(0x603f,0,canID,devID,10,3);

    return ErrorCodes[value];
}
//========================================================================
bool Epos::SetPosition(int canID,int devId,int position,int velocity)
{
    bool status=false;
    status=WriteRegister(0x6081,0,canID,devId,velocity,4);
    if(status!=true)//set velocity
    {
        qDebug()<<"set velocity error";
        return status;

    }
    status=WriteRegister(0x6040,0,canID,devId,0x0f,2);
    if(status !=true)
    {
        qDebug()<<"switch off error";
        return status;
    }
    status=WriteRegister(0x607A,0,canID,devId,position,4);
    if(status!=true)
    {
        qDebug()<<"switch on error";
        return status;
    }
    status=WriteRegister(0x6040,0,canID,devId,0x3f,2);
    if(status!=true){
        qDebug()<<"switch on error";
        return status;
    }

    //set mode
    return true;
}
//========================================================================
inline QByteArray  Epos::MotorDataToArray(int canID,int position)
{
    QByteArray data;
    data.append((canID>>8)&0xff);
    data.append((canID>>0)&0xff);

    data.append((position >> 0) & 0xff);
    data.append((position>> 8) & 0xff);
    data.append((position >> 16) & 0xff);
    data.append((position >> 24) & 0xff);
    return data;
}
//========================================================================
inline QByteArray  Epos::CreatePDOPacket(int canID,int value1,int value2)
{
    QByteArray data;
    data.append((canID>>8)&0xff);
    data.append((canID>>0)&0xff);

    data.append((value1 >> 0) & 0xff);
    data.append((value1 >> 8) & 0xff);
    data.append((value1 >> 16) & 0xff);
    data.append((value1 >> 24) & 0xff);

    data.append((value2 >> 0) & 0xff);
    data.append((value2 >> 8) & 0xff);
    data.append((value2 >> 16) & 0xff);
    data.append((value2 >> 24) & 0xff);
    return data;
}
//========================================================================
inline QByteArray Epos::CreateDynamixelPacket(int canID,int motorID,int motorPosition,int velocity)
{
    QByteArray data;
    data.append((canID>>8)&0xff);
    data.append((canID>>0)&0xff);
    data.append(motorID&0xff);
    data.append((motorPosition >> 8) & 0xff); //pos hi
    data.append((motorPosition >> 0) & 0xff); //pos low
    data.append(velocity&0xff);  //speed
    data.append( QByteArray(4, Qt::Initialization::Uninitialized));
    return data;
}
//========================================================================
inline QByteArray Epos::CreateHandPacket(QList<int> motorPositions)
{

    static int handDeviceID=0;
    QByteArray data;

    if(handDeviceID<4){
        data.append(CreatePDOPacket(0x401+handDeviceID,motorPositions[12+handDeviceID+4],0x3f));
        data.append(CreatePDOPacket(0x401+handDeviceID,motorPositions[12+handDeviceID],0x3f));
        handDeviceID++;
    }
    else if(handDeviceID> 3 && handDeviceID<7)
    {
        data.append(CreateDynamixelPacket(0x581,handDeviceID-3,motorPositions[12+handDeviceID],100));
        data.append(CreateDynamixelPacket(0x581,handDeviceID-3,motorPositions[8+12+handDeviceID],100));
        handDeviceID++;
    }
    else if(handDeviceID==7)
    {
        ///right palm
        data.append(CreatePDOPacket(palmCanID,motorPositions[19] &0xff,0));
        // ///left palm
        data.append(CreatePDOPacket(palmCanID,motorPositions[27] &0xff,0));
        handDeviceID++;
    }
    else if(handDeviceID==8)//start move into controlworld for hands
    {
        data.append(CreatePDOPacket(0x501,0x0f,0x0f));
        data.append(CreatePDOPacket(0x501,0x0f,0x0f));
        handDeviceID=0;

    }
    return data;
}
//========================================================================
inline QByteArray Epos::CreateBumpRequestCommand()
{
    QByteArray command;
    static bool bumpSensorFlag=false;
    command.append(0x03);
    command.append((bumpSensorFlag?0x82:0x83));
    command.insert(command.length(),(const char*)_bumpSensorCommand,sizeof(_bumpSensorCommand));
    bumpSensorFlag=!bumpSensorFlag;
    return command;
}
//========================================================================
inline QByteArray Epos::CreateServoHeadCommand(QList<int> motorPositions)
{
    QByteArray command;
//   QLOG_TRACE()<<"motor:"<<motorPositions[20]<<" "<<motorPositions[21]<<" "<<motorPositions[22];
    command.append(0x02);
    command.append(0x81);
    command.append(motorPositions[20]);
    command.append(motorPositions[21]);
    command.append(motorPositions[22]);
    command.append( QByteArray(5, Qt::Initialization::Uninitialized));
    return command;
}
//========================================================================
inline QByteArray Epos::CreateWaistAndHeadCommand(QList<int> motorPositions)
{
    static int state=0;

    QByteArray command;

    if(state==0)
    {

        command.append(CreatePDOPacket(0x401,motorPositions.at(28),0x3f));//waist max
    }
    else if(state==1){

    command.append(CreatePDOPacket(0x401,motorPositions.at(28),0x3f));//waist max
    }
    else if(state==2){

       command.append(CreatePDOPacket(0x401,motorPositions.at(28),0x3f));//waist max
    }
    else if(state==3){

      command.append(CreatePDOPacket(0x401,motorPositions.at(28),0x3f));//waist max
    }
    else if(state==4){

       command.append(CreatePDOPacket(0x401,motorPositions.at(28),0x3f));//waist max
    }
    else if(state==5){

      command.append(CreatePDOPacket(0x401,motorPositions.at(28),0x3f));//waist max

    }
    else if(state==6){

      command.append(CreatePDOPacket(0x401,motorPositions.at(28),0x3f));//waist max

    }
   // QLOG_TRACE()<<"state:"<<state<<" adad:"<<motorPositions.at(30);
    if(state++>5)state=0;
    return command;
}
//========================================================================
void Epos::SetAllPositionCST(QList<int> motorPositions)
{

    QByteArray command;
    for(int i=0; i< 12; i++)
        command.append(MotorDataToArray(0x401,motorPositions.at(i)));
    command.append(CreateHandPacket(motorPositions));
    command.append(CreateWaistAndHeadCommand(motorPositions));
    command.append(CreateServoHeadCommand(motorPositions));
    //    //packet must be 300 bytes 180 byte zero
    command.append(QByteArray(ReserveByteCount, Qt::Initialization::Uninitialized));
    can.WriteRunCommand(command);
}
//========================================================================
float Epos::QByteArrayToFloat(QByteArray arr)
{
    float output;
    char* point_char = (char*)&output;
    *point_char = (arr[3] & 0xff );
    point_char++;
    *point_char = (arr[2] & 0xff );
    point_char++;
    *point_char = (arr[1] & 0xff );
    point_char++;
    *point_char = (arr[0] & 0xff );
    point_char++;
    return output;
}
//========================================================================
inline bool Epos::IsValidRunPacket(QByteArray packet)
{
    if((packet[0]&0xff)!= 0x01 )
        return false;
    if((packet[1]&0xff)!= 0x55 )
        return false;

    if((packet[2]&0xff)!= 0xAA )
        return false;

    if((packet[3]&0xff)!= 0x55 )
        return false;

    return true;

}
//========================================================================
inline void Epos::GetFTSensorDataFromPacket(EthernetReceivedPacketType*packet)
{
     //21201145
     float calibrationMatrix[3][4]=
 {
    {-1123.0445	,-1033.6627,	-1135.5674,	-1010.5035},
     {29.848	,-0.19658	,-26.7485	,-0.49423},
     { -1.0049	,-26.9313,	-1.0898	,28.8956}

 };
 //21201144

      float calibrationMatrix2[3][4]=
 {
    {-1214.0242,-1015.2706,	-1232.727,	-1009.9608},
{28.3086,	-0.39355,	-27.4125,	-0.62844},
{-0.14038,	-28.0865,	-0.35422,	28.0305}

 };
    float f1=packet->FTsensor[0]*calibrationMatrix[0][0]+packet->FTsensor[1]*calibrationMatrix[0][1]+packet->FTsensor[2]*calibrationMatrix[0][2]+packet->FTsensor[3]*calibrationMatrix[0][3];
    float f2=packet->FTsensor[0]*calibrationMatrix[1][0]+packet->FTsensor[1]*calibrationMatrix[1][1]+packet->FTsensor[2]*calibrationMatrix[1][2]+packet->FTsensor[3]*calibrationMatrix[1][3];
    float f3=packet->FTsensor[0]*calibrationMatrix[2][0]+packet->FTsensor[1]*calibrationMatrix[2][1]+packet->FTsensor[2]*calibrationMatrix[2][2]+packet->FTsensor[3]*calibrationMatrix[2][3];

    float f4=packet->FTsensor[4]*calibrationMatrix2[0][0]+packet->FTsensor[5]*calibrationMatrix2[0][1]+packet->FTsensor[6]*calibrationMatrix2[0][2]+packet->FTsensor[7]*calibrationMatrix2[0][3];
    float f5=packet->FTsensor[4]*calibrationMatrix2[1][0]+packet->FTsensor[5]*calibrationMatrix2[1][1]+packet->FTsensor[6]*calibrationMatrix2[1][2]+packet->FTsensor[7]*calibrationMatrix2[1][3];
    float f6=packet->FTsensor[4]*calibrationMatrix2[2][0]+packet->FTsensor[5]*calibrationMatrix2[2][1]+packet->FTsensor[6]*calibrationMatrix2[2][2]+packet->FTsensor[7]*calibrationMatrix2[2][3];

    ForceTorqSensorLeft.force.x=f1;
    ForceTorqSensorLeft.force.y=f2;
    ForceTorqSensorLeft.force.z=f3;




   ForceTorqSensorRight.force.x=f4;
   ForceTorqSensorRight.force.y=f5;
   ForceTorqSensorRight.force.z=f6;
   
    // ForceTorqSensorLeft.torque.x=packet->FTsensor[3];
    // ForceTorqSensorLeft.torque.y=packet->FTsensor[4];
    // ForceTorqSensorLeft.torque.z=packet->FTsensor[5];

    // ForceTorqSensorRight.force.x=packet->FTsensor[6];
    // ForceTorqSensorRight.force.y=packet->FTsensor[7];
    // ForceTorqSensorRight.force.z=packet->FTsensor[8];
    // ForceTorqSensorRight.torque.x=packet->FTsensor[9];
    // ForceTorqSensorRight.torque.y=packet->FTsensor[10];
    // ForceTorqSensorRight.torque.z=packet->FTsensor[11];
    //5000 * (? / (((double)65535) * (gainFTnow) * (sensitivityFTnow) * (ExFTnow)));
    // ForceTorqSensorRight.force.x -=offsetFTRight[0];
    // ForceTorqSensorRight.force.y -=offsetFTRight[1];
    // ForceTorqSensorRight.force.z -=offsetFTRight[2];
    // ForceTorqSensorRight.torque.x-=offsetFTRight[3];
    // ForceTorqSensorRight.torque.y-=offsetFTRight[4];
    // ForceTorqSensorRight.torque.z-=offsetFTRight[5];

    // ForceTorqSensorLeft.force.x -=offsetFTLeft[0];
    // ForceTorqSensorLeft.force.y -=offsetFTLeft[1];
    // ForceTorqSensorLeft.force.z -=offsetFTLeft[2];
    // ForceTorqSensorLeft.torque.x-=offsetFTLeft[3];
    // ForceTorqSensorLeft.torque.y-=offsetFTLeft[4];
    // ForceTorqSensorLeft.torque.z-=offsetFTLeft[5];

    // ForceTorqSensorRight.force.x/=(65535*gainFTRight[0]*sensitivityFTRight[0]*ExFTRight[0])/5000;
    // ForceTorqSensorRight.force.y/=(65535*gainFTRight[1]*sensitivityFTRight[1]*ExFTRight[1])/5000;
    // ForceTorqSensorRight.force.z/=(65535*gainFTRight[2]*sensitivityFTRight[2]*ExFTRight[2])/5000;
    // ForceTorqSensorRight.torque.x/=(65535*gainFTRight[3]*sensitivityFTRight[3]*ExFTRight[3])/5000;
    // ForceTorqSensorRight.torque.y/=(65535*gainFTRight[4]*sensitivityFTRight[4]*ExFTRight[4])/5000;
    // ForceTorqSensorRight.torque.z/=(65535*gainFTRight[5]*sensitivityFTRight[5]*ExFTRight[5])/5000;

    // ForceTorqSensorLeft.force.x/=(65535*gainFTLeft[0]*sensitivityFTLeft[0]*ExFTLeft[0])/5000;
    // ForceTorqSensorLeft.force.y/=(65535*gainFTLeft[1]*sensitivityFTLeft[1]*ExFTLeft[1])/5000;
    // ForceTorqSensorLeft.force.z/=(65535*gainFTLeft[2]*sensitivityFTLeft[2]*ExFTLeft[2])/5000;
    // ForceTorqSensorLeft.torque.x/=(65535*gainFTLeft[3]*sensitivityFTLeft[3]*ExFTLeft[3])/5000;
    // ForceTorqSensorLeft.torque.y/=(65535*gainFTLeft[4]*sensitivityFTLeft[4]*ExFTLeft[4])/5000;
    // ForceTorqSensorLeft.torque.z/=(65535*gainFTLeft[5]*sensitivityFTLeft[5]*ExFTLeft[5])/5000;



}
//========================================================================
inline void Epos::GetIMUDataFromPacket(EthernetReceivedPacketType*packet)
{
    tf2::Quaternion myQuaternion;
    myQuaternion.setRPY( incommingPacket->IMU.roll,incommingPacket->IMU.pitch,incommingPacket->IMU.yaw);
    //--------------------orientation
//    IMU.orientation.x=myQuaternion.getX();
//    IMU.orientation.y=myQuaternion.getY();
//    IMU.orientation.z=myQuaternion.getZ();
//    IMU.orientation.w=myQuaternion.getW();
    IMU.orientation.x=incommingPacket->IMU.roll;
    IMU.orientation.y=incommingPacket->IMU.pitch;
    IMU.orientation.z=incommingPacket->IMU.yaw;
    IMU.orientation.w=0;
    //--------------------lenear acc
    IMU.linear_acceleration.x=incommingPacket->IMU.ax;
    IMU.linear_acceleration.y=incommingPacket->IMU.ay;
    IMU.linear_acceleration.z=incommingPacket->IMU.az;
    //--------------------angular velocity
    IMU.angular_velocity.x=incommingPacket->IMU.wx;
    IMU.angular_velocity.y=incommingPacket->IMU.wy;
    IMU.angular_velocity.z=incommingPacket->IMU.wz;
    //free acc
    Acceleration.linear.x=incommingPacket->IMU.fax;
    Acceleration.linear.y=incommingPacket->IMU.fay;
    Acceleration.linear.z=incommingPacket->IMU.faz;
    //mag feild
    MagneticSensor.magnetic_field.x=incommingPacket->IMU.mx;
    MagneticSensor.magnetic_field.y=incommingPacket->IMU.my;
    MagneticSensor.magnetic_field.z=incommingPacket->IMU.mz;
}
//========================================================================
inline void Epos::GetBumpDataFromPacket(BumpSensorPacket *packet)
{

        for(int i=0;i<8;i++){
            bump_sensor_list[i]=packet->data[i]&0xffff;
        }
  

}
//========================================================================
void Epos::GetPositionDataFromPacket(EthernetReceivedPacketType*packet)
{
    for(int i=0;i<16;i++){
        positionsInc[i]=(packet->MotorData[i].Valu1);
        positions[i]=(packet->MotorData[i].Valu2);
    //    qDebug()<<positions[i];

    }
   // qDebug()<<"<-------------------->";

}
//========================================================================
void Epos::DataReceived(QByteArray data)
{
    LastPacketreceived=data;
    if(!IsValidRunPacket(data))return;
    incommingPacket=(EthernetReceivedPacketType*)data.data();
    QByteArray mid= data.mid(276,8);
    bumpPacket=(BumpSensorPacket*)mid.data();
    GetPositionDataFromPacket(incommingPacket);
    GetBumpDataFromPacket(bumpPacket);
    GetFTSensorDataFromPacket(incommingPacket);
    GetIMUDataFromPacket(incommingPacket);
    emit FeedBackReceived(ft,positions,positionsInc,bump_sensor_list, imu_data_list);
}
//========================================================================
void Epos::WaitMs(int ms)
{
    QEventLoop q;
    QTimer tT;
    tT.setSingleShot(true);
    connect(&tT, SIGNAL(timeout()), &q, SLOT(quit()));
    tT.start(ms);
    q.exec();
    if(tT.isActive()){

        tT.stop();
    } else {

    }


}
//====================================================================================
void Epos::InitErrorMap()
{
    ErrorCodes.insert(  0x0,"OK");
    ErrorCodes.insert(  0x1000,"Generic error");
    ErrorCodes.insert(  0x1080,"Generic initialization error");
    ErrorCodes.insert(  0x1081,"Generic initialization error");
    ErrorCodes.insert(  0x1082,"Generic initialization error");
    ErrorCodes.insert(  0x1083,"Generic initialization error");
    ErrorCodes.insert(0x1090,"Firmware incompatibility error");
    ErrorCodes.insert(0x2310,"Overcurrent error");
    ErrorCodes.insert(0x2320,"Power stage protection error");
    ErrorCodes.insert(0x3210,"Overvoltage error");
    ErrorCodes.insert(0x3220,"Undervoltage error");
    ErrorCodes.insert(0x4210,"Thermal overload error");
    ErrorCodes.insert(0x5113,"Logic supply voltage too low error");
    ErrorCodes.insert(0x5280,"Hardware defect error");
    ErrorCodes.insert(0x5281,"Hardware incompatibility error");
    ErrorCodes.insert(0x5480,"Hardware error");
    ErrorCodes.insert(0x5481,"Hardware error");
    ErrorCodes.insert(0x5482,"Hardware error");
    ErrorCodes.insert(0x5483,"Hardware error");
    ErrorCodes.insert(0x6080,"Sign of life error");
    ErrorCodes.insert(0x6081,"Extension 1 watchdog error");
    ErrorCodes.insert(0x6320,"Software parameter error");
    ErrorCodes.insert(0x7320,"Position sensor error");
    ErrorCodes.insert(0x7380,"Position sensor breach error");
    ErrorCodes.insert(0x7381,"Position sensor resolution error");
    ErrorCodes.insert(0x7382,"Position sensor index error");
    ErrorCodes.insert(0x7388,"Hall sensor error");
    ErrorCodes.insert(0x7389,"Hall sensor not found error");
    ErrorCodes.insert(0x738A,"Hall angle detection error");
    ErrorCodes.insert(0x738C,"SSI sensor error");
    ErrorCodes.insert(0x7390,"Missing main sensor error");
    ErrorCodes.insert(0x7391,"Missing commutation sensor error");
    ErrorCodes.insert(0x7392,"Main sensor direction error");
    ErrorCodes.insert(0x8110,"CAN overrun error (object lost)");
    ErrorCodes.insert(0x8111,"CAN overrun error");
    ErrorCodes.insert(0x8120,"CAN passive mode error");
    ErrorCodes.insert(0x8130,"CAN heartbeat error");
    ErrorCodes.insert(0x8150,"CAN PDO COB-ID collision");
    ErrorCodes.insert(0x8180,"EtherCAT communication error");
    ErrorCodes.insert(0x8181,"EtherCAT initialization error");
    ErrorCodes.insert(0x81FD,"CAN bus turned off");
    ErrorCodes.insert(0x81FE,"CAN Rx queue overflow");
    ErrorCodes.insert(0x81FF,"CAN Tx queue overflow");
    ErrorCodes.insert(0x8210,"CAN PDO length error");
    ErrorCodes.insert(0x8250,"RPDO timeout");
    ErrorCodes.insert(0x8280,"EtherCAT PDO communication error");
    ErrorCodes.insert(0x8281,"EtherCAT SDO communication error");
    ErrorCodes.insert(0x8611,"Following error");
    ErrorCodes.insert(0x8A80,"Negative limit switch error");
    ErrorCodes.insert(0x8A81,"Positive limit switch error");
    ErrorCodes.insert(0x8A82,"Software position limit error");
    ErrorCodes.insert(0x8A88,"STO error");
    ErrorCodes.insert(0xFF01,"System overloaded error");
    ErrorCodes.insert(0xFF02,"Watchdog error");
    ErrorCodes.insert(0xFF0B,"System peak overloaded error");
    ErrorCodes.insert(0xFF10,"Controller gain error");
    ErrorCodes.insert(0xFF12,"Auto tuning current limit error");
    ErrorCodes.insert(0xFF13,"Auto tuning identification current error");
    ErrorCodes.insert(0xFF14,"Auto tuning buffer overflow error");
    ErrorCodes.insert(0xFF15,"Auto tuning sample mismatch error");
    ErrorCodes.insert(0xFF16,"Auto tuning parameter error");
    ErrorCodes.insert(0xFF17,"Auto tuning amplitude mismatch error");
    ErrorCodes.insert(0xFF18,"Auto tuning period length error");
    ErrorCodes.insert(0xFF19,"Auto tuning timeout error");
    ErrorCodes.insert(0xFF20,"Auto tuning standstill error");
    ErrorCodes.insert(0xFF21,"Auto tuning torque invalid error");
}
//========================================================================
bool Epos::CheckZynq()
{
    //    int i=0;
    //    QByteArray data;
    //    uchar checkPoint[8]={0x00,0x55,0xaa,0x55,0xff,0xcc,0x33,0xcc};
    //    QByteArray ck;

    //    data.append((char)0x00);  // mode 0: Test
    //    data.append(0xAA);  // Header check
    //    data.append(0x55);  // Header check
    //    data.append(0xAA);  // Header check



    //    data.append(0xFF);  // Parity | 0xFF
    //    data.append(0xCC);  // Tailer
    //    data.append(0x33);  // Tailer
    //    data.append(0xCC);  // Tailer
    //bool result;
    //  QByteArray resp=  tcp.SendCommand(data,result,10);  // send 128 Byte Packet
    //  if(LastPacketreceived.toHex()=="0055aa55ffcc33cc")
    //  {

    //  qDebug()<<"valid resp!";
    //  return true;
    //  }
    //  return false;

}
//========================================================================
EPOSErrors Epos::Init(int tryCount)
{
    static bool lastresult=false;


    int numberOfSuccess=0;
    int pingCount=0;

    if(lastresult)return OK;
    qDebug()<<"ping";
    while(! ping.Start("192.168.1.10") && pingCount<tryCount )
    {
        QThread::msleep(500);
        qDebug()<<"waiting zynq ethernet..";
        pingCount++;
    }
    if(pingCount==tryCount)return NETWOR_ERROR;

    qDebug()<<"Ping OK";
    //===========================
    QList<int32_t> values;

    if(ReadAllRegisters(0x1000,0,1,values,10)!=OK)
    {

        qDebug()<<"Read error";
        return NO_ANSER;
    }
    if(values.count()<12){    qDebug()<<"response lengh error"<<values.count();return SDO_REJECT;}
    for(int i=0;i<12;i++){
        qDebug()<<"read val "<<QString::number(i)<<"="<<QString::number(values[i],16);
        if(values[i]!=0x20192)return SDO_BAD_REPLAY;
    }

    lastresult=true;
    return OK;

}
//========================================================================

