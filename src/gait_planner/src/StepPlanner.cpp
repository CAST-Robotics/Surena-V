#include "StepPlanner.h"

StepPlanner::StepPlanner(double torso) : torso_(torso)
{
    
}

void StepPlanner::generateFootSteps(vector<Vector3d>& ankle_rf, vector<Vector3d>& dcm_rf, double step_length,
                                    double step_width, double step_height, int step_count, double theta,
                                    double com_offset)
{ 
    if (theta == 0.0)
    { // Straight or Diagonal Walk
        generateStraightFootStep(ankle_rf, dcm_rf, step_width, step_length, step_height, step_count-2, com_offset);
    }
    else
    { // Turning Walk
        generateTurnFootStep(ankle_rf, dcm_rf, step_length, step_height, step_count-2, theta, com_offset);
    }
}

void StepPlanner::generateStraightFootStep(vector<Vector3d>& ankle_rf, vector<Vector3d>& dcm_rf, const double &step_width,
                                           const double &step_length, const double &step_height, const int &step_count, 
                                           const double &com_offset)
{
    int lateral_sign;
    if (step_width == 0)
        lateral_sign = 1;
    else
        lateral_sign = (step_width / abs(step_width));

    ankle_rf[0] << 0.0, (torso_ + 0.0) * lateral_sign, 0.0;
    ankle_rf[1] << 0.0, (torso_ + 0.0) * -lateral_sign, 0.0;
    dcm_rf[0] << 0.0, 0.0, 0.0;
    dcm_rf[1] << 0.0, (torso_ - com_offset) * -lateral_sign, 0.0;

    for (int i = 2; i <= step_count + 1; i++)
    {
        if (i == 2 || i == step_count + 1)
        {
            ankle_rf[i] = ankle_rf[i - 2] + Vector3d(step_length, step_width, step_height);
            dcm_rf[i] << ankle_rf[i - 2] + Vector3d(step_length, step_width, step_height);
        }
        else
        {
            ankle_rf[i] = ankle_rf[i - 2] + Vector3d(2 * step_length, step_width, step_height);
            dcm_rf[i] << ankle_rf[i - 2] + Vector3d(2 * step_length, step_width, step_height);
        }
        dcm_rf[i](1) -= pow(-1, i) * com_offset;
    }
    
    dcm_rf[step_count + 1] = 0.5 * (ankle_rf[step_count] + ankle_rf[step_count + 1]);
    ankle_rf[0] << 0.0, torso_ * lateral_sign, 0.0;
    ankle_rf[1] << 0.0, torso_ * -lateral_sign, 0.0;
}

void StepPlanner::generateTurnFootStep(vector<Vector3d>& ankle_rf, vector<Vector3d>& dcm_rf, const double &step_length,
                                       const double &step_height, const int &step_count, const double &theta, 
                                       const double &com_offset)
{
    double r = abs(step_length / theta);
    int turn_sign = abs(step_length) / step_length;
    ankle_rf[0] = Vector3d(0.0, -turn_sign * torso_, 0.0);
    dcm_rf[0] = Vector3d::Zero(3);
    ankle_rf[step_count + 1] = (r + pow(-1, step_count) * torso_) * Vector3d(sin(theta * (step_count - 1)), turn_sign * cos(theta * (step_count - 1)), 0.0) +
                               Vector3d(0.0, -turn_sign * r, 0.0);
    for (int i = 1; i <= step_count; i++)
    {
        ankle_rf[i] = (r + pow(-1, i - 1) * torso_) * Vector3d(sin(theta * (i - 1)), turn_sign * cos(theta * (i - 1)), 0.0) +
                      Vector3d(0.0, -turn_sign * r, 0.0);
        dcm_rf[i] = ankle_rf[i];
        dcm_rf[i](1) += turn_sign * pow(-1, i) * com_offset;
    }
    dcm_rf[step_count + 1] = 0.5 * (ankle_rf[step_count] + ankle_rf[step_count + 1]);
}