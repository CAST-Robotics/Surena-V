#include "DCM.h"
#include "Ankle.h"
#include <thread>

void generateStraightFootStep(vector<Vector3d>& ankle_rf, vector<Vector3d>& dcm_rf, const double &step_width, const double &step_length, 
                              const double &step_height, const int &step_count, const double &com_offset)
{
    int lateral_sign;
    double torso_ = 0.0975;

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

int main()
{
    int step_count = 0;
    vector<Vector3d> ankle_foot_steps(step_count + 2);
    vector<Vector3d> base_foot_steps(step_count + 2);

    generateStraightFootStep(ankle_foot_steps, base_foot_steps, 0.0, 0.17, 0.0, step_count, 0.0);

    DCMPlanner base_planner(0.68, 1, 0.1, 0.005, step_count + 2, 0.44, 0.0);
    base_planner.setOnlineFoot(base_foot_steps, 1);

    Ankle ankle_planner(1, 0.1, 0.025, 0.44, step_count, 0.005, 0.0, 0.0);
    ankle_planner.updateOnlineFoot(ankle_foot_steps, 1);
    Vector3d lanklepos, ranklepos;
    Matrix3d lanklerot, ranklerot;

    std::thread ifThread;

    int length = base_planner.getLength();
    for (int i = 0; i < length; i++)
    {
        Vector3d com = base_planner.computeCoM(i);
        ankle_planner.getOnlineTrajectory(i, lanklepos, lanklerot, ranklepos, ranklerot);
        cout << com(0) << ", " << com(1) << ", " << com(2) << ", ";
        cout << lanklepos(0) << ", " << lanklepos(1) << ", " << lanklepos(2) << ", ";
        cout << ranklepos(0) << ", " << ranklepos(1) << ", " << ranklepos(2) << endl;
        // if(i == 390)
        // {
        //     ifThread = std::thread([&](){ // Start a new thread for the conditional logic
        //         base_planner.changeVRP(3, Vector3d(0.34, -0.0975, 0));
        //         base_planner.changeVRP(4, Vector3d(0.51, 0.0975, 0));
        //         base_planner.changeVRP(5, Vector3d(0.51, 0, 0));
        //         base_planner.updateXiPoints();
        //         length = base_planner.getLength();

        //         ankle_planner.changeFootStep(3, Vector3d(0.34, -0.0975, 0));
        //         ankle_planner.changeFootStep(4, Vector3d(0.51, 0.0975, 0));
        //         ankle_planner.changeFootStep(5, Vector3d(0.51, -0.0975, 0));
        //         ankle_planner.updateCoeffs();
        //     });
        // }
    }
    // ifThread.join();
    return 0;
}