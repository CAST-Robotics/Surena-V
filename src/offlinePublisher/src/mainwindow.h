#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include<std_msgs/Int32MultiArray.h>
#include <stdlib.h>
#include<QtCore>
#include <QDebug>
#include <QTimer>
namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT
  ros::NodeHandle nh;
  ros::Publisher pub;
  float testvalue;
  QTimer _timer;
public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
  void Timeout();
  void on_pushButton_clicked();

private:
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
