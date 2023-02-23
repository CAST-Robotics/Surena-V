#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#include "ui_mainwindow.h"
#include <QMainWindow>
#include <stdlib.h>
#include<QtCore>
#include <QDebug>
#include <QTimer>
#include "qnode.h"
namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT
    QNode *_rosNode;
public:
    explicit MainWindow(QWidget *parent = 0);
    void RosInit( int argc, char **argv);
    ~MainWindow();


public slots:

private slots:
  void Timeout();
    void on_BtnHome_clicked();
      void CleanAndExit();

      void on_BtnSetPosition_clicked();
      void on_BtnStop_clicked();
      void on_BtnResetAll_clicked();
      void on_BtnActiveAll_clicked();
private:
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
