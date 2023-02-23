#ifndef ETHERNETPACKETS_H
#define ETHERNETPACKETS_H
#include <QObject>
#pragma pack(push, 1)
class CanPacketType
{
public:
int16_t ID;
int32_t Valu1;
int32_t Valu2;

};
class BumpSensorPacket
{
public:
int8_t data[8];


};
class IMUReceivedPacketType
{
public:
int32_t  Header;
int16_t  valueID1;
uint8_t len1;
float roll;
float pitch;
float yaw;
//linear acc
int16_t  valueID2;
uint8_t len2;
float ax;
float ay;
float az;
//free acc
int16_t  valueID3;
uint8_t len3;
float fax;
float fay;
float faz;

//angular velocity
int16_t  valueID4;
uint8_t len4;
float wx;
float wy;
float wz;
//mag feild
int16_t  valueID5;
uint8_t len5;
float mx;
float my;
float mz;
uint8_t cksum;
};
class EthernetReceivedPacketType
{
public:
   unsigned char Header[4];
   CanPacketType MotorData[16];
float FTsensor[8];
IMUReceivedPacketType IMU;
};
#pragma pack(pop)
#endif // ETHERNETPACKETS_H
