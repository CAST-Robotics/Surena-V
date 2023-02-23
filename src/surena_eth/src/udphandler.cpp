#include "udphandler.h"
#include"QsLog/QsLogDisableForThisFile.h"
//================================================================================
UDPHandler::UDPHandler(QObject *parent) : QObject(parent)
{

QLOG_TRACE()<<"Create socket";
_socket = new QUdpSocket(this);

}
//================================================================================
QString UDPHandler::GetLocalIPAddres()
{
 QString result;
foreach (const QHostAddress &address, QNetworkInterface::allAddresses()) {
  QLOG_TRACE()<<"system ip:"<<result;
    if (address.protocol() == QAbstractSocket::IPv4Protocol && address != QHostAddress(QHostAddress::LocalHost))
{     //qDebug() << address.toString();

result=address.toString();
  QLOG_INFO()<<"this is selected ip:"<<result;
}

}
return result;
}
//================================================================================
void UDPHandler::Connect(QString remoteIPAddress,int port)
{

    _ipAddress.setAddress(remoteIPAddress);
    _socket->setSocketOption(QAbstractSocket::LowDelayOption, 1);
    _socket->bind(QHostAddress::Any,port);
    QLOG_INFO()<<"UDP Connecting to"<<remoteIPAddress<<"@port:"<<port;//<<(result?"OK":"Failed");
    connect(_socket,SIGNAL(readyRead()),this,SLOT(ReadyRead()));

}
//================================================================================
bool UDPHandler::WaitDataReceive(int timeout)
{
    QEventLoop q;
    QTimer tT;
    tT.setSingleShot(true);
    connect(&tT, SIGNAL(timeout()), &q, SLOT(quit()));
    connect(this, SIGNAL(NewDataReceived(QByteArray)), &q, SLOT(quit()));
    tT.start(timeout);
    q.exec();
    if(tT.isActive())
    {

    tT.stop();
    return true;
    }
    else
    {

    return false;
    }


}
//================================================================================
int UDPHandler::WriteData(QByteArray data)
{
   QLOG_TRACE()<<"UDP Send "<<data.length()<<" bytes:"<<data.toHex();
    //_socket->waitForBytesWritten();
 int result =1;
 _socket->writeDatagram(data, _ipAddress, 7);
 _socket->flush();
 _socket->waitForBytesWritten();
//qDebug()<< result;
 return  result;
}
//================================================================================
QByteArray UDPHandler::ReadBuffer()
{
    QLOG_TRACE()<<"Request read Buffer";
    return _incommingData;
}
//================================================================================
void UDPHandler::Disconnect()
{

    QLOG_TRACE()<<"Close socket";
    _socket->close();
}
//================================================================================
QByteArray UDPHandler::SendCommand(QByteArray command,int timeout=10)throw(std::runtime_error)
{

WriteData(command);
bool result=WaitDataReceive(timeout);
if(!result)throw std::runtime_error("Send command response timeout") ;
return _incommingData;
}
//================================================================================
bool UDPHandler::IsConnected()
{
return _socket->isWritable();

}
//================================================================================
void UDPHandler::ClearBuffer()
{
    _incommingData.clear();
}
//================================================================================
void UDPHandler::ReadyRead()
{
   // qDebug() <<"my incomming data:"<< _socket->readAll();
   //
    QByteArray datagram;
    datagram.clear();
   // qDebug() << "in !";
   QHostAddress sender;
   quint16 port;
   while (_socket->hasPendingDatagrams())
   {
      //QByteArray datagram;
      datagram.resize(_socket->pendingDatagramSize());
      _socket->readDatagram(datagram.data(),datagram.size(),&sender,&port);
      _incommingData.clear();
      _incommingData.append(datagram);

        QLOG_TRACE()<<"UDP Received "<<_incommingData.length()<<" bytes:"<<_incommingData.toHex();
      //qDebug() <<"Message From Addr: " << sender.toString() <<"; Port: "<< port << "; size: " << datagram.count();
      //qDebug() <<"Message: " << datagram.toHex();
     // qDebug() <<"Message: " << incommingData.toHex();
      emit NewDataReceived(_incommingData);
   }
}
//====================================================================================
UDPHandler::~UDPHandler()
{
    _socket->disconnect();
    _socket->deleteLater();
    QLOG_TRACE()<<"delete tcp handler class";
}
//================================================================================
