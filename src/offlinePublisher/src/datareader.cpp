#include "datareader.h"

DataReader::DataReader()
{

}

void DataReader::Load(QString fileName)
{
    QFile file(fileName);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))// QI0Device ~ windows to C++
    {qDebug()<<"file error"; return ;}
    while (!file.atEnd()) {
        QString line = file.readLine();
        QList<int> temp;
        for (int id=0;id<12;id++)
        {
            temp.append(line.split(",")[id].toInt());
        }
       // qDebug()<<temp[0];
        wholeData.append(temp);
    }
}

void DataReader::GetData(QList<int> &motorsData, int i)
{
    if(i>=wholeData.count())i=wholeData.count()-1;
    motorsData.clear();
    for (int id=0;id<12;id++)
    {
        motorsData.append(wholeData[i][id]);
    }
}
