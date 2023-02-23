#ifndef PINGMODEL_H
#define PINGMODEL_H

#include <QObject>
#include <QProcess>

class PingModel : public QObject
{
    Q_OBJECT
    QByteArray _result;
public:
    explicit PingModel(QObject *parent = 0);
    ~PingModel();

    bool IsRunning();
    bool Finished();
    
    bool Start(QString address);
signals:
    
public slots:
    void VerifyStatus();
    void ReadResult();

private:
    QProcess ping;
    bool running;
};

#endif // PINGMODEL_H
