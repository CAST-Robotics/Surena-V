#include "can.h"
#include"QsLog/QsLogDisableForThisFile.h"

//========================================================================
Can::Can(QObject *parent ) : QObject(parent)
{
connect(&_udp,SIGNAL(NewDataReceived(QByteArray)),this,SIGNAL(NewDataReceived(QByteArray)));
}
//========================================================================
bool Can::Init()
{
    _udp.Connect("192.168.1.10",7);

}
//========================================================================
QByteArray Can::WriteMessage(uint16_t canID,unsigned char devID,QByteArray data)throw(std::runtime_error)
{
    try{
    QByteArray buffer;
    QByteArray response;
    buffer.append(CanWriteCommand);
    buffer.insert(buffer.length(),(const char*)_UDPHeader,sizeof(_UDPHeader));
    buffer.append(devID);
    buffer.append((canID & 0xff)); //low
    buffer.append(((canID>>8)& 0xff));//hi
    buffer.append(data);
    if(devID==255){
        buffer.insert(buffer.length(),(const char*)_UDPTail,sizeof(_UDPTail));
    }
    response= _udp.SendCommand(buffer,1000);
       return response;
    }
    catch(const std::runtime_error e)
    {
        QLOG_ERROR()<<e.what();
        throw std::runtime_error(e.what());
        return QByteArray(((devID==255)?160:10), Qt::Initialization::Uninitialized);

    }
}
//========================================================================
 QByteArray Can::ReadMessage(uint8_t devID)throw(std::runtime_error)
{
    try {
    QByteArray buffer,response;
    buffer.append(CanReadMessageCommand);
    buffer.insert(buffer.length(),(const char*)_UDPHeader,sizeof(_UDPHeader));
    buffer.append(devID);
    buffer.insert(buffer.length(),QByteArray(((devID==255)?160:10), Qt::Initialization::Uninitialized));
    buffer.insert(buffer.length(),(const char*)_UDPTail,sizeof(_UDPTail));
    response = _udp.SendCommand(buffer,10);
    if(response.length()<((devID==255)?164:14))
    {
        throw std::runtime_error("Invalid UDP Can Packet");
        return QByteArray(((devID==255)?160:10), Qt::Initialization::Uninitialized);
    }
    response=response.mid(4,((devID==255)?160:10));
    QLOG_TRACE()<<"Read Replay:"<<response.toHex();
  return response;

    }
     catch (const std::runtime_error e)
    {
        QLOG_ERROR()<<e.what();
        return QByteArray(((devID==255)?160:10), Qt::Initialization::Uninitialized);
    }
 }
 //========================================================================
 void Can::WriteRunCommand(QByteArray data)
 {
     QByteArray buffer;
     buffer.append(CanRunMessageCommand);  // mode 1: Run
     buffer.insert(buffer.length(),(const char*)_UDPHeader,sizeof(_UDPHeader));
     buffer.append(data);
     buffer.insert(buffer.length(),(const char*)_UDPTail,sizeof(_UDPTail));
     _udp.WriteData(buffer);
 }
 //========================================================================
 void Can::SendUDPMessage(QByteArray data)
 {
     _udp.WriteData(data);
 }
 //========================================================================

 void Can::CheckCanBoardRequest()
 {
     int i=0;
     QByteArray data;
     data.append(0x08);  // mode 8: Test
     data.append(0xAA);  // Header check
     data.append(0x55);  // Header check
     data.append(0xAA);  // Header check



     for(i = 0 ; i < 32; i++)
     {
         data.append(0x01);  // fill data packet with 01 byte
     }

     data.append(0xFF);  // Parity | 0xFF
     data.append(0xCC);  // Tailer
     data.append(0x33);  // Tailer
     data.append(0xCC);  // Tailer
     //qDebug()<<"packet size="<<data.length();
     SendUDPMessage(data);
     //tcp.WriteData(data);  // send 128 Byte Packet

     // just for test
     // qDebug() << "send data: " << data.toHex();
 }
 //========================================================================

