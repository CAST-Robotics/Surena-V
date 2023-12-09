#include "Collision.h"

Collision::Collision(double a, double b, double c, double min_dist) : a_(a), b_(b), c_(c), minDist_(min_dist) {}

bool Collision::checkColission(Vector3d left_ankle, Vector3d right_ankle,
                               Matrix3d left_orient, Matrix3d right_orient)
{

    leftVertice_[0] = left_ankle + left_orient * Vector3d(a_, -b_, 0);
    leftVertice_[1] = left_ankle + left_orient * Vector3d(a_, b_, 0);
    leftVertice_[2] = left_ankle + left_orient * Vector3d(-c_, b_, 0);
    leftVertice_[3] = left_ankle + left_orient * Vector3d(-c_, -b_, 0);

    rightVertice_[0] = right_ankle + right_orient * Vector3d(a_, -b_, 0);
    rightVertice_[1] = right_ankle + right_orient * Vector3d(a_, b_, 0);
    rightVertice_[2] = right_ankle + right_orient * Vector3d(-c_, b_, 0);
    rightVertice_[3] = right_ankle + right_orient * Vector3d(-c_, -b_, 0);

    for (int i = 0; i < 4; i++)
    {
        if (insidePoly(leftVertice_, rightVertice_[i]))
        {
            return true;
        }

        if (insidePoly(rightVertice_, leftVertice_[i]))
        {
            return true;
        }

        if ((left_ankle - right_ankle).norm() < minDist_)
        {
            return true;
        }
    }

    return false;
}

bool Collision::insidePoly(Vector3d poly[4], Vector3d point)
{

    int i, j;
    bool res = false;
    for (i = 0, j = 4 - 1; i < 4; j = i++)
    {
        if (((poly[i](1) > point(1)) != (poly[j](1) > point(1))) &&
            (point(0) < (poly[j](0) - poly[i](0)) * (point(1) - poly[i](1)) / (poly[j](1) - poly[i](1)) + poly[i](0)))
            res = !res;
    }
    return res;
}

// #include "iostream"
// #include "math.h"

// int main(){
//     Collision temp(0.16, 0.075, 0.09, 0.18);
//     Matrix3d orient;
//     orient = AngleAxisd(M_PI / 6, Vector3d::UnitZ());
//     std::cout << temp.checkColission(Vector3d(0.0, -0.1, 0), Vector3d(0, 0.1, 0), orient, Matrix3d::Identity());
// }