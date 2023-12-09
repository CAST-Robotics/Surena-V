#include "Estimator.h"

void Estimator::atitudeEulerEstimator(Vector3d &base_attitude, Vector3d gyro)
{
    Vector3d change_attitude;
    change_attitude[0] = gyro(0) + tan(base_attitude(1)) * (gyro(1) * sin(base_attitude(0)) + gyro(2) * cos(base_attitude(0)));
    change_attitude[1] = gyro(1) * cos(base_attitude(0)) - gyro(2) * sin(base_attitude(0));
    change_attitude[2] = 1 / cos(base_attitude(1)) * (gyro(1) * sin(base_attitude(0)) + gyro(2) * cos(base_attitude(0)));
    base_attitude += this->dt_ * change_attitude;
}

void Estimator::poseVelEstimator(Vector3d &base_vel, Vector3d &base_pos, Vector3d acc)
{
    base_vel += dt_ * (acc - Vector3d(-0.06331, 0.2084210, 10.0047));
    base_pos += dt_ * base_vel;
}

void Estimator::gaussianPredict(Vector3d &posterior, double &P, double Q, Vector3d delta)
{
    posterior += delta;
    P += Q;
}

void Estimator::gaussianUpdate(Vector3d &prior, double &P, Vector3d z, double R)
{
    Vector3d y = z - prior;
    double K = P / (P + R);
    prior += K * y;
    P = (1 - K) * P;
}