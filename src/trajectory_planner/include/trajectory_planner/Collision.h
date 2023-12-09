#pragma once

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Core"
#include <eigen3/Eigen/Geometry>
#include "iostream"

using namespace Eigen;
using namespace std;

class Collision
{
public:
    Collision(double a, double b, double c, double min_dist);
    bool checkColission(Vector3d left_ankle, Vector3d right_ankle, Matrix3d left_orient, Matrix3d right_orient);

private:
    Vector3d centerLeft_;
    Vector3d centerRight_;
    Matrix3d rotLeft_;
    Matrix3d rotRight_;

    // Sole Vertices in Global Coordinate
    Vector3d rightVertice_[4];
    Vector3d leftVertice_[4];

    // Sole Corners Dimensions relative to Ankle Frame
    double a_;
    double b_;
    double c_;
    double minDist_;

    bool insidePoly(Vector3d poly[4], Vector3d point);
};