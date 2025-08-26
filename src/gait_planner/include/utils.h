#pragma once

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cmath>

using namespace std;
using namespace Eigen;

Matrix3d skewSym(const Vector3d &vec);
void ExpSO3(const Vector3d &vec, Matrix3d &output);
MatrixXd matrixPower(const MatrixXd &A, const int &n);
double factorial(const int &n);
Matrix3d gamma(const Vector3d &vec, int n);
MatrixXd adjoint(const MatrixXd &X);
MatrixXd SEK3Exp(const VectorXd &vec);