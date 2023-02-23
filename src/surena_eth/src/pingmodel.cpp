#include <QString>
#include <QDebug>
#include "pingmodel.h"

PingModel::PingModel(QObject *parent) :
    QObject(parent), running(false)
{
   // ping = new QProcess(this);
    connect(&ping, SIGNAL(started()), this, SLOT(VerifyStatus()));
    connect(&ping, SIGNAL(finished(int)), this, SLOT(ReadResult()));
//    ping->setProcessChannelMode(QProcess::MergedChannels);
}

PingModel::~PingModel(){
}

void PingModel::VerifyStatus(){
    if(ping.isReadable()){
        //qDebug() << "read on ...";
        connect(&ping, SIGNAL(readyRead()), this, SLOT(ReadResult()));
        if(ping.canReadLine()){
         //   qDebug() << "LINE read on ...";
        }
    }else{
       // qDebug() << "not able to read ...";
    }
}

void PingModel::ReadResult(){

    running = false;
    //_result.append(ping->readLine());
    QByteArray temp=ping.readLine();
  //  qDebug() << "Res:" << temp;
    _result.append(temp);
}

bool PingModel::Start(QString address){
    _result.clear();
    //if(ping){
        QString command = "ping";
        QStringList args;
       // qDebug()<<address;
        args << "-w" <<  "1" << address;// "www.google.com";
        ping.start(command, args);
        ping.waitForStarted(7000);
        running = true;
        ping.waitForFinished(100000);
     // qDebug()<<"resakhar="<<_result;
        if(_result.indexOf("0% packet loss",0)>1)
        {
                  //  qDebug()<<"ping ok";
        return true;

        }
          //  qDebug()<<"ping ERROR!";
        return false;
   // }
   // return false;
}

bool PingModel::IsRunning(){
    return running;
}

bool PingModel::Finished(){
    return ping.atEnd();

}
