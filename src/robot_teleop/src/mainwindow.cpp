#include "mainwindow.h"



MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);


}

void MainWindow::RosInit(int argc, char **argv)
{
_rosNode=new QNode(argc,argv);
if(!_rosNode->Init())exit(0);
connect(_rosNode,SIGNAL(rosShutdown()),this,SLOT(CleanAndExit()));
}

MainWindow::~MainWindow()
{
    delete ui;
}
void MainWindow::Timeout()
{

}


void MainWindow::on_BtnHome_clicked()
{

}

void MainWindow::on_BtnSetPosition_clicked()
{

   _rosNode->IncPositions[0]= ui->NumR1->value();
   _rosNode->IncPositions[1]= ui->NumR2->value();
   _rosNode->IncPositions[2]= ui->NumR3->value();
   _rosNode->IncPositions[3]= ui->NumR4->value();
   _rosNode->IncPositions[4]= ui->NumR5->value();
   _rosNode->IncPositions[5]= ui->NumR6->value();
   _rosNode->IncPositions[6]= ui->NumL1->value();
   _rosNode->IncPositions[7]= ui->NumL2->value();
   _rosNode->IncPositions[8]= ui->NumL3->value();
   _rosNode->IncPositions[9]= ui->NumL4->value();
   _rosNode->IncPositions[10]= ui->NumL5->value();
   _rosNode->IncPositions[11]= ui->NumL6->value();

_rosNode->PositioningIsActive=true;
}

void MainWindow::on_BtnStop_clicked()
{

}

void MainWindow::on_BtnResetAll_clicked()
{
_rosNode->ResetAllNodes();
}

void MainWindow::on_BtnActiveAll_clicked()
{

}

void MainWindow::CleanAndExit()
{
qDebug()<<"Clean and exist";
    exit(0);

}
