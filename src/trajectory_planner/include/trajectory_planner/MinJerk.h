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

class MinJerk
{
public:
protected:
    template <typename T>
    vector<T> cubicInterpolate(T theta_ini, T theta_f, T theta_dot_ini, T theta_dot_f, double tf)
    {
        /*
            Returns Cubic Polynomial with the Given Boundary Conditions
            https://www.tu-chemnitz.de/informatik//KI/edu/robotik/ws2016/lecture-tg%201.pdf
        */
        vector<T> coefs(4); // a0, a1, a2, a3
        coefs[0] = theta_ini;
        coefs[1] = theta_dot_ini;
        coefs[2] = 3 / pow(tf, 2) * (theta_f - theta_ini) - 1 / tf * (2 * theta_dot_ini + theta_dot_f);
        coefs[3] = -2 / pow(tf, 3) * (theta_f - theta_ini) + 1 / pow(tf, 2) * (theta_dot_ini + theta_dot_f);
        return coefs;
    }
    vector<Vector3d> poly6Interpolate(Vector3d x_ini, Vector3d x_mid, Vector3d x_f, double tf);
    vector<Vector3d> poly6Interpolate(Vector3d x_ini, Vector3d x_mid, Vector3d x_f, double tf, Vector3d x_d_ini, Vector3d _d_f, Vector3d x_dd_ini, Vector3d x_dd_f);
    vector<Vector3d> ankle5Poly(Vector3d x0, Vector3d xf, double z_max, double tf, double final_height);
    void write2File(Vector3d *input, int size, string file_name);
};