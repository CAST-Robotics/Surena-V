#pragma once

#include "MinJerk.h"

class Ankle : private MinJerk
{
public:
    Ankle(double step_time, double ds_time, double height, double alpha, short int num_step, double dt, double theta, double slope);
    ~Ankle();
    void updateFoot(const vector<Vector3d>& ankle_rf, int sign);
    void generateTrajectory();
    const vector<Vector3d>& getTrajectoryL();
    const vector<Vector3d>& getTrajectoryR();
    const vector<Matrix3d>& getRotTrajectoryR();
    const vector<Matrix3d>& getRotTrajectoryL();
    const vector<int>& getRobotState();

private:
    double tStep_;
    double tDS_;
    double dt_;
    short int num_step; // ؟؟؟
    double alpha_;
    int stepCount_;
    bool leftFirst_;
    double height_;
    double theta_;
    double slope_;
    int yawSign_;
    int length_;

    vector<Vector3d> footPose_;
    vector<Vector3d> lFoot_;
    vector<Vector3d> rFoot_;
    vector<Matrix3d> lFootRot_;
    vector<Matrix3d> rFootRot_;

    // Robot Movment State Indicator (0:Stance, 1:Double Support, 2:Right Single Support, 3: Left Single Support, 4:None)
    vector<int> stateIndicator_;

    void updateTrajectory(bool left_first);
};