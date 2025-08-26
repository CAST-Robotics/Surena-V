#pragma once

#include "MinJerk.h"

class GeneralMotion : private MinJerk
{
public:
    GeneralMotion(double dt);
    ~GeneralMotion();
    void changeInPlace(Vector3d init_com_pos, Vector3d final_com_pos,
                       Vector3d init_com_orient, Vector3d final_com_orient,
                       Vector3d init_lankle_pos, Vector3d final_lankle_pos,
                       Vector3d init_lankle_orient, Vector3d final_lankle_orient,
                       Vector3d init_rankle_pos, Vector3d final_rankle_pos,
                       Vector3d init_rankle_orient, Vector3d final_rankle_orient,
                       double time);
    void generateCoefs(Vector3d init_com_pos, Vector3d final_com_pos,
                       Vector3d init_com_orient, Vector3d final_com_orient,
                       Vector3d init_lankle_pos, Vector3d final_lankle_pos,
                       Vector3d init_lankle_orient, Vector3d final_lankle_orient,
                       Vector3d init_rankle_pos, Vector3d final_rankle_pos,
                       Vector3d init_rankle_orient, Vector3d final_rankle_orient,
                       double time);
    
    void getDataPoint(int index, Vector3d& com_pos, Matrix3d& com_orient, Vector3d& lankle_pos, 
                      Matrix3d& lankle_orient, Vector3d& rankle_pos, Matrix3d& rankle_orient);

    const vector<Vector3d>& getCOMPos()
    {
        return COMPos_;
    }
    const vector<Vector3d>& getLAnklePos()
    {
        return LAnklePos_;
    }
    const vector<Vector3d>& getRAnklePos()
    {
        return RAnklePos_;
    }
    const vector<Matrix3d>& getLAnkleOrient()
    {
        return LAnkleOrient_;
    }
    const vector<Matrix3d>& getRAnkleOrient()
    {
        return RAnkleOrient_;
    }
    const vector<Matrix3d>& getCOMOrient()
    {
        return COMOrient_;
    }
    int getLength()
    {
        return length_;
    }

    int getRobotPhase()
    {
        return currentRobotPhase_;
    }

    const vector<int>& getRobotState()
    {
        return robotState_;
    }

private:
    int length_;
    double dt_;
    vector<Vector3d> COMPos_;
    vector<Vector3d> LAnklePos_;
    vector<Vector3d> RAnklePos_;
    vector<Matrix3d> COMOrient_;
    vector<Matrix3d> LAnkleOrient_;
    vector<Matrix3d> RAnkleOrient_;

    vector<Vector3d> com_pos_coefs_;
    vector<Vector3d> lankle_pos_coefs_;
    vector<Vector3d> rankle_pos_coefs_;

    vector<Vector3d> com_orient_coefs_;
    vector<Vector3d> lankle_orient_coefs_;
    vector<Vector3d> rankle_orient_coefs_;

    // Robot Movment State Indicator (0:Stance, 1:Double Support, 2:Right Single Support, 3: Left Single Support, 4:None)
    vector<int> robotState_;
    int currentRobotPhase_ = 0;
};