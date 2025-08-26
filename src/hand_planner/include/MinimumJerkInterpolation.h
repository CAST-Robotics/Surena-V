#ifndef MINIMUMJERKINTERPOLATION_H
#define MINIMUMJERKINTERPOLATION_H

#include <iostream>
#include <fstream>
#include "eigen3/Eigen/Dense"
#include <math.h>
#include <vector>

using namespace Eigen;
using namespace std;
class MinimumJerkInterpolation
{
public:
    MinimumJerkInterpolation();
    MatrixXd Coefficient(MatrixXd time, MatrixXd p, MatrixXd dp, MatrixXd dpp);
void PrintArray(MatrixXd array, string filename);
    MatrixXd diff(MatrixXd E);
    int length(MatrixXd in);
    vector<int> findIsinf(MatrixXd in,bool inf);
    MatrixXd MatrixIndex(MatrixXd in, vector<int> indexes);
    MatrixXd AddtoIndexes(MatrixXd in, MatrixXd val, vector<int> indexes);
    MatrixXd Coefficient1(MatrixXd xp, MatrixXd ord, MatrixXd con, double fig_res);
    MatrixXd PMaker(double t);
    MatrixXd isinf(MatrixXd in);
    MatrixXd isnan(MatrixXd in);
    double SUM1(int first, int last, MatrixXd Mat);
    MatrixXd GetAccVelPos(MatrixXd Coef, double time, double ti, int PolynomialOrder);
};

#endif // MINIMUMJERKINTERPOLATION_H
