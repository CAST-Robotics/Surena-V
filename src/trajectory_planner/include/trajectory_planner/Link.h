#pragma once

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Core"
#include <eigen3/Eigen/Geometry>

#include "iostream"
#include "fstream"
#include <vector>
#include "math.h"
#include "cmath"

using namespace Eigen;
using namespace std;

class _Link
{
    friend class Robot;

public:
    _Link(short int ID, Vector3d a, Vector3d b, double m, Matrix3d inertia, Vector3d com_pose, _Link *parent = NULL);
    _Link() {}
    _Link(const _Link &source);
    ~_Link();

    double q();
    double dq();
    void update(double q, double dq, double ddq);
    void initPose(Vector3d p, Matrix3d r);
    inline void setRot(Matrix3d rot) { R_ = rot; }
    inline void setPos(Vector3d pos) { p_ = pos; }
    inline void setVel(Vector3d vel) { v_ = vel; }
    inline void setOmega(Vector3d omega) { w_ = omega; }
    inline void setEuler(Vector3d euler) { eulerAtitude_ = euler; }

    short int getID();
    _Link *getParent();
    Vector3d getPose();
    Matrix3d getRot();
    double getMass();
    Vector3d getLinkCoM();
    Vector3d getOmega();
    Vector3d getLinkVel();
    inline Vector3d getEuler() { return eulerAtitude_; }

    MatrixXd FK();
    MatrixXd updateJacobian();
    MatrixXd getVel();
    void setParams(short int ID, Vector3d a, Vector3d b, double m, Matrix3d inertia, _Link *parent = NULL);

private:
    ////////////////// Link Properties ///////////////////////
    short int ID_;
    _Link *parent_;
    Vector3d p_;            // Link position (WRT world frame)
    Matrix3d R_;            // Link rotation (WRT world frame)
    Vector3d eulerAtitude_; // Link atitude represented with euler angles (WRT world frame & in the form of roll, pitch, yaw)
    Vector3d v_;            // Linear Velocity of link base in world fram
    Vector3d w_;            // Angular Velocity of link in world frame
    double q_;              // joint angle
    double dq_;             // joint angular velocity
    double ddq_;            // joint angular acceleration

    Vector3d a_; // joint Axis WRT parent frame
    Vector3d b_; // joint position WRT parent frame

    double m_;   // mass
    Matrix3d I_; // Inertia matrix
    Vector3d c_; // CoM position of link relative to its joint

    //////////////////// private methods /////////////////////
    MatrixXd transformation();
    Matrix3d rodrigues(Vector3d w, double dt);
};