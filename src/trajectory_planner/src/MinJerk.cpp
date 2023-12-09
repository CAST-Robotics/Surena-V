#include "MinJerk.h"

vector<Vector3d> MinJerk::poly6Interpolate(Vector3d x_ini, Vector3d x_mid, Vector3d x_f, double tf)
{
    /*
        This function fits a 5th order (C2) Polynomial to given given inputs
        Inputs : X0, Xmid, Xf   !Xd, Xdd at begining and end are assumed to be zero!
    */

    vector<Vector3d> a(7);
    a[0] = x_ini;
    a[1] = Vector3d::Zero(3);
    a[2] = Vector3d::Zero(3);
    a[3] = -2 / pow(tf, 3) * (21 * x_ini + 11 * x_f - 32 * x_mid);
    a[4] = 3 / pow(tf, 4) * (37 * x_ini + 27 * x_f - 64 * x_mid);
    a[5] = -6 / pow(tf, 5) * (17 * x_ini + 15 * x_f - 32 * x_mid);
    a[6] = 3 / pow(tf, 4) * (x_ini + x_f - 2 * x_mid);
    return a;
}

vector<Vector3d> MinJerk::ankle5Poly(Vector3d x0, Vector3d xf, double z_max, double tf, double final_height)
{
    vector<Vector3d> ans(6);
    // XY trajectory 5th order with Vel. and accl. B.C.
    ans[0] = x0;
    ans[1] = Vector3d::Zero(3);
    ans[2] = Vector3d::Zero(3);
    ans[3] = 10 / pow(tf, 3) * (xf - x0);
    ans[4] = -15 / pow(tf, 4) * (xf - x0);
    ans[5] = 6 / pow(tf, 5) * (xf - x0);
    // Z trajectory also 5th order with velocity B.C.
    ans[0](2) = x0(2);
    ans[1](2) = 0.0;
    ans[2](2) = 7 * final_height / pow(tf, 2) + 16 * z_max / pow(tf, 2);
    ans[3](2) = -34 * final_height / pow(tf, 3) - 32 * z_max / pow(tf, 3);
    ans[4](2) = 52 * final_height / pow(tf, 4) + 16 * z_max / pow(tf, 4);
    ans[5](2) = -24 * final_height / pow(tf, 5);

    return ans;
}

void MinJerk::write2File(Vector3d *input, int size, string file_name = "data")
{
    ofstream output_file(file_name + ".csv");
    for (int i = 0; i < size; i++)
    {
        output_file << input[i](0) << " ,";
        output_file << input[i](1) << " ,";
        output_file << input[i](2) << " ,";
        output_file << "\n";
    }
    output_file.close();
}