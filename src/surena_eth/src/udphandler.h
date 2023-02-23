/*
 *bug:
 * bind return false event all things good
 *
 *
 */
#ifndef UDPHANDLER_H
#define UDPHANDLER_H

#include <QObject>
#include <QDebug>
#include <QtNetwork/QTcpSocket>
#include <QtNetwork/QUdpSocket>
#include <QtNetwork/QtNetwork>
#include <QTimer>
#include <QEventLoop>
#include <exception>

#include "QsLog/QsLog.h"


class UDPHandler : public QObject
{
    Q_OBJECT


private:
    QHostAddress _ipAddress;
    QUdpSocket *_socket;
    QByteArray _incommingData;


public:

    explicit UDPHandler(QObject *parent = 0);
    void Connect(QString IPAddress, int port);
    int WriteData(QByteArray);
    QByteArray ReadBuffer();
    QByteArray SendCommand(QByteArray command, int timeout)throw(std::runtime_error);
    void Disconnect();
    void ClearBuffer();
    bool WaitDataReceive(int ms);

    QString GetLocalIPAddres();
    bool IsConnected();
    ~UDPHandler();
signals:
   void NewDataReceived(QByteArray);
   void Dummy();
private slots:

    void ReadyRead();


};

#endif // TCPHANDLER_H
