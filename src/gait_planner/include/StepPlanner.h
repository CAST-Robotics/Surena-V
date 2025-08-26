#pragma once

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"

#include "iostream"
#include "fstream"
#include <vector>
#include "math.h"
#include "cmath"

using namespace Eigen;
using namespace std;

class StepPlanner
{
public:
    StepPlanner(double torso);
    void generateFootSteps(vector<Vector3d>& ankle_rf, vector<Vector3d>& dcm_rf, double step_length, 
                           double step_width, double step_height, int step_count, double theta, double com_offset);

    void generateStraightFootStep(vector<Vector3d>& ankle_rf, vector<Vector3d>& dcm_rf, const double &step_width,
                                  const double &step_length, const double &step_height, const int &step_count, 
                                  const double &com_offset);

    void generateTurnFootStep(vector<Vector3d>& ankle_rf, vector<Vector3d>& dcm_rf, const double &step_length,
                              const double &step_height, const int &step_count, const double &theta, 
                              const double &com_offset);

private:
    double torso_;
};
