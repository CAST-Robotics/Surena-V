#ifndef CAN_H
#define CAN_H

#include <QObject>
#include <QDebug>
#include <QtCore>
#include <QtGui>
#include <QtDebug>
#include <QTimer>

#include "udphandler.h"
#include "QsLog/QsLog.h"

class Can : public QObject
{
    Q_OBJECT
enum
{   CanRunMessageCommand=1,
  CanWriteCommand=2,
   CanReadMessageCommand=3

};
       const unsigned char _UDPHeader[3]={0xAA,0x55,0xAA};
       const unsigned char _UDPTail[4]={0xff,0xcc,0x33,0xcc};
           UDPHandler _udp;
public:


    bool Init();
    explicit Can(QObject *parent = 0);
    QByteArray WriteMessage(uint16_t canID, unsigned char devID, QByteArray data)throw(std::runtime_error);
    QByteArray ReadMessage(uint8_t devID)throw(std::runtime_error);
    void WriteRunCommand(QByteArray data);
    void SendUDPMessage(QByteArray data);
    //======================================================================== udp????
    void  CheckCanBoardRequest();
signals:
    void NewDataReceived(QByteArray);
public slots:
};

#endif // CAN_H
