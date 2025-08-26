#pragma once

#include "MinJerk.h"

class Ankle : private MinJerk
{
public:
    Ankle(double step_time, double ds_time, double height, double alpha, short int num_step, double dt, double theta, double slope);
    ~Ankle();
    void updateFoot(const vector<Vector3d>& ankle_rf, int sign);
    void updateOnlineFoot(const vector<Vector3d>& foot_pose, int sign, const vector<Vector3d>& foot_euler = vector<Vector3d>());
    void changeFootStep(int foot_step_idx, const Vector3d& newVRP);
    void generateCoeffs(int step_idx=0);
    void updateCoeffs();
    void generateTrajectory();
    const vector<Vector3d>& getTrajectoryL();
    const vector<Vector3d>& getTrajectoryR();
    const vector<Matrix3d>& getRotTrajectoryR();
    const vector<Matrix3d>& getRotTrajectoryL();
    const vector<int>& getRobotState();
    inline int getStateIndicator(){return stateIndicator_;}
    void getOnlineTrajectory(int index, Vector3d& left_foot_pos, Matrix3d& left_foot_rot,
                             Vector3d& right_foot_pos, Matrix3d& right_foot_rot);
    void handleFirstStep(Vector3d& left_foot_pos, Matrix3d& left_foot_rot, Vector3d& right_foot_pos, Matrix3d& right_foot_rot);
    void handleLastStep(int step, Vector3d& left_foot_pos, Matrix3d& left_foot_rot, Vector3d& right_foot_pos, Matrix3d& right_foot_rot);
    void handleOtherSteps(int step, int step_index, Vector3d& left_foot_pos, Matrix3d& left_foot_rot, Vector3d& right_foot_pos, Matrix3d& right_foot_rot, int& state_indicator);
    void assignFootPosAndRot(int leftIndex, int rightIndex, Vector3d& left_foot_pos, Matrix3d& left_foot_rot, Vector3d& right_foot_pos, Matrix3d& right_foot_rot);
    void handleSupportSwing(int step, int step_index, Vector3d& left_foot_pos, Matrix3d& left_foot_rot, Vector3d& right_foot_pos, Matrix3d& right_foot_rot, int& state_indicator, bool leftSupport);
    void handleSSPhase(int step, int step_index, Vector3d& left_foot_pos, Matrix3d& left_foot_rot, Vector3d& right_foot_pos, Matrix3d& right_foot_rot, bool leftSupport);

private:
    double tStep_;
    double tDS_;
    double dt_;
    double alpha_;
    int stepCount_;
    bool leftFirst_;
    bool leftLast_;
    double height_;
    double theta_;
    double slope_;
    int yawSign_;
    int length_;
    int stepSize_;
    int dsSize_;
    int ssSize_;
    int initDSSize_;
    int finalDSSize_;

    vector<Vector3d> footStepPos_;
    vector<Vector3d> footStepEuler_;
    vector<Vector3d> lFoot_;
    vector<Vector3d> rFoot_;
    vector<Matrix3d> lFootRot_;
    vector<Matrix3d> rFootRot_;
    vector<vector<Vector3d>> coefs_;
    vector<vector<Vector3d>> euler_coefs_;

    // Robot Movment State Indicator (0:Stance, 1:Double Support, 2:Right Single Support, 3: Left Single Support, 4:None)
    vector<int> stateIndicatorArray_;
    int stateIndicator_;
    int currentStepNum_;

    void updateTrajectory(bool left_first);
    
};