#ifndef DATAREADER_H
#define DATAREADER_H

#include <QFile>
#include <QString>
#include <QDebug>
#include <QVector>
#include <QList>

class DataReader
{
public:
    DataReader();
    void Load(QString fileName);
    void GetData(QList<int> &motorsData, int i);
    QList< QList<int> > wholeData;
};

#endif // DATAREADER_H
