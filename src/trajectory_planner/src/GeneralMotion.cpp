#include "GeneralMotion.h"

GeneralMotion::GeneralMotion(double dt)
{
    this->dt_ = dt;
}

GeneralMotion::~GeneralMotion()
{

}

void GeneralMotion::changeInPlace(Vector3d init_com_pos, Vector3d final_com_pos,
                                  Vector3d init_com_orient, Vector3d final_com_orient,
                                  Vector3d init_lankle_pos, Vector3d final_lankle_pos,
                                  Vector3d init_lankle_orient, Vector3d final_lankle_orient,
                                  Vector3d init_rankle_pos, Vector3d final_rankle_pos,
                                  Vector3d init_rankle_orient, Vector3d final_rankle_orient,
                                  double time)
{
    /*
        This function generates robot's trajectories for in-place movements.
        params:
        init_{com, lankle, rankle}_pos, final_{com, lankle, rankle}_pos are
        initial and final position of COM, Left Ankle & Right Ankle.
        init_{com, lankle, rankle}_orient, final_{com, lankle, rankle}_orient are
        initial and final euler angles of COM, Left Ankle & Right Ankle (roll, pitch, yaw).
    */
    // cout << init_lankle_pos << "--------" << init_rankle_pos << endl;
    length_ = time / dt_;
    COMPos_.resize(length_);
    COMOrient_.resize(length_);
    LAnklePos_.resize(length_);
    LAnkleOrient_.resize(length_);
    RAnklePos_.resize(length_);
    RAnkleOrient_.resize(length_);
    robotState_.resize(length_);

    vector<Vector3d> com_pos_coefs = cubicInterpolate<Vector3d>(init_com_pos, final_com_pos, Vector3d::Zero(3), Vector3d::Zero(3), time);
    vector<Vector3d> lankle_pos_coefs = cubicInterpolate<Vector3d>(init_lankle_pos, final_lankle_pos, Vector3d::Zero(3), Vector3d::Zero(3), time);
    vector<Vector3d> rankle_pos_coefs = cubicInterpolate<Vector3d>(init_rankle_pos, final_rankle_pos, Vector3d::Zero(3), Vector3d::Zero(3), time);

    vector<Vector3d> com_orient_coefs = cubicInterpolate<Vector3d>(init_com_orient, final_com_orient, Vector3d::Zero(3), Vector3d::Zero(3), time);
    vector<Vector3d> lankle_orient_coefs = cubicInterpolate<Vector3d>(init_lankle_orient, final_lankle_orient, Vector3d::Zero(3), Vector3d::Zero(3), time);
    vector<Vector3d> rankle_orient_coefs = cubicInterpolate<Vector3d>(init_rankle_orient, final_rankle_orient, Vector3d::Zero(3), Vector3d::Zero(3), time);

    Vector3d temp_com_orient;
    Vector3d temp_lankle_orient;
    Vector3d temp_rankle_orient;

    for (int index = 0; index < length_; index++)
    {
        double t = index * dt_;
        // COM Trajectories
        COMPos_[index] = com_pos_coefs[0] + com_pos_coefs[1] * t + com_pos_coefs[2] * pow(t, 2) + com_pos_coefs[3] * pow(t, 3);
        temp_com_orient = com_orient_coefs[0] + com_orient_coefs[1] * t + com_orient_coefs[2] * pow(t, 2) + com_orient_coefs[3] * pow(t, 3);
        COMOrient_[index] = AngleAxisd(temp_com_orient(2), Vector3d::UnitZ()) * AngleAxisd(temp_com_orient(1), Vector3d::UnitY()) * AngleAxisd(temp_com_orient(0), Vector3d::UnitX());
        // Left Ankle Trajectories
        LAnklePos_[index] = lankle_pos_coefs[0] + lankle_pos_coefs[1] * t + lankle_pos_coefs[2] * pow(t, 2) + lankle_pos_coefs[3] * pow(t, 3);
        temp_lankle_orient = lankle_orient_coefs[0] + lankle_orient_coefs[1] * t + lankle_orient_coefs[2] * pow(t, 2) + lankle_orient_coefs[3] * pow(t, 3);
        LAnkleOrient_[index] = AngleAxisd(temp_lankle_orient(2), Vector3d::UnitZ()) * AngleAxisd(temp_lankle_orient(1), Vector3d::UnitY()) * AngleAxisd(temp_lankle_orient(0), Vector3d::UnitX());
        // Right Ankle Trajectories
        RAnklePos_[index] = rankle_pos_coefs[0] + rankle_pos_coefs[1] * t + rankle_pos_coefs[2] * pow(t, 2) + rankle_pos_coefs[3] * pow(t, 3);
        temp_rankle_orient = rankle_orient_coefs[0] + rankle_orient_coefs[1] * t + rankle_orient_coefs[2] * pow(t, 2) + rankle_orient_coefs[3] * pow(t, 3);
        RAnkleOrient_[index] = AngleAxisd(temp_rankle_orient(2), Vector3d::UnitZ()) * AngleAxisd(temp_rankle_orient(1), Vector3d::UnitY()) * AngleAxisd(temp_rankle_orient(0), Vector3d::UnitX());
        if ((init_lankle_pos == final_lankle_pos) && (init_rankle_pos == final_rankle_pos))
            robotState_[index] = 0;
        else
            robotState_[index] = 4;
    }
}
