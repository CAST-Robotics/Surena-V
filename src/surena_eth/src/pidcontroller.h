#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H


class PIDController
{
private:
  double _dt;
  double _max;
  double _min;
  double _Kp;
  double _Kd;
  double _Ki;
  double _pre_error;
  double _integral;
public:



  PIDController();
 PIDController( double dt, double max, double min, double Kp, double Kd, double Ki );
 void Init( double dt, double max, double min, double Kp, double Kd, double Ki );

 double Calculate( double setpoint, double pv );
 double Calculate_direct_D(double setpoint, double pv,double dpv);
};

#endif // PIDCONTROLLER_H
