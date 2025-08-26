/**
 * @file DCM.h
 * @brief Three-dimensional trajectory generation based on Divergent Component of Motion (DCM).
 * @details This file defines the DCMPlanner class, which generates trajectories for a humanoid 
 * robot's center of mass (CoM) and zero moment point (ZMP) based on DCM.
 * Reference Paper: https://ieeexplore.ieee.org/abstract/document/7063218/
 * @version 0.1
 * @date 2023-07-12
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"

#include "iostream"
#include "fstream"
#include <vector>
#include "math.h"
#include "cmath"

#include "MinJerk.h"

using namespace Eigen;
using namespace std;

const double K_G = 9.81;

/**
 * @class DCMPlanner
 * @brief Generates three-dimensional trajectories for a humanoid robot's center of mass (CoM) 
 * and zero moment point (ZMP) based on DCM.
 */
class DCMPlanner : private MinJerk
{
public:
    /**
     * @brief Constructor for DCMPlanner.
     * @param deltaZ The robot height in meters.
     * @param stepTime The time duration of each step in seconds.
     * @param doubleSupportTime The time duration of double support phase in seconds.
     * @param dt The time interval for trajectory generation in seconds.
     * @param stepCount The number of steps for which the trajectories will be generated.
     * @param alpha The ratio of double support time to single support time.
     * @param theta The Turn angle in radians.
     */
    DCMPlanner(double deltaZ, double stepTime, double doubleSupportTime, double dt, 
               int stepCount = 6, double alpha = 0.5, double theta = 0.0);
    
    /**
     * @brief Destructor for DCMPlanner.
     */
    ~DCMPlanner();
    
    /**
     * @brief Sets the position of the robot's feet.
     * @param rF A vector of Vector3d representing the position of the robot's feet.
     * @param sign The sign of the yaw angle.
     */
    void setFoot(const vector<Vector3d>& rF, int sign);
    void setOnlineFoot(const vector<Vector3d>& rF, int sign);
    
    /**
     * @brief Returns the trajectory of the divergent component of motion (Xi) in 3D.
     * @return Reference to a vector of Vector3d representing the trajectory of Xi.
     */
    const std::vector<Vector3d>& getXiTrajectory();

    /**
     * @brief Returns the velocity of the divergent component of motion (XiDot) in 3D.
     * @return Reference to a vector of Vector3d representing the velocity of XiDot.
     */
    const vector<Vector3d>& getXiDot();

    /**
     * @brief Returns the trajectory of the center of mass (COM) in 3D.
     * @return Reference to a vector of Vector3d representing the trajectory of the COM.
     */
    const vector<Vector3d>& getCoM();

    /**
     * @brief Returns the trajectory of the zero moment point (ZMP) in 3D.
     * @return Reference to a vector of Vector3d representing the trajectory of the ZMP.
     */
    const vector<Vector3d>& getZMP();

    /**
     * @brief Returns the velocity of the center of mass (CoMDot) in 3D.
     * @return Reference to a vector of Vector3d representing the velocity of the CoMDot.
     */
    const vector<Vector3d>& get_CoMDot();

    /**
     * @brief Returns the yaw rotation matrix.
     * @return Reference to a vector of Matrix3d representing the yaw rotation.
     */
    const vector<Matrix3d>& yawRotGen();

    Vector3d computeCoM(int iter);
    Vector3d getCurrentZMP(){return currentZMP_;}
    Vector3d getCurrentDCM(){return currentXi_;}

    int getLength(){return length_;}

    void calculateRotCoeffs();
    Matrix3d getOnlineRot(int iter);
    void changeVRP(int foot_step_idx, const Vector3d& newVRP);
    void updateXiPoints();
    void setInitCoM(Vector3d init_com){CoMInit_ = init_com;}

private:
    // Design Parameters
    double deltaZ_;
    double tStep_;
    double tDS_;
    double alpha_;
    double theta_;

    double dt_;
    int stepCount_;

    // Trajectory Arrays
    vector<Vector3d> xi_;
    vector<Vector3d> xiDot_;
    vector<Vector3d> COM_;
    vector<Vector3d> CoMDot_;
    vector<Vector3d> ZMP_;

    // Other Points required for Generating Trajectories
    vector<Vector3d> rF_;
    vector<Vector3d> rVRP_;
    vector<Vector3d> xiEOS_;
    vector<Vector3d> xiDSI_;
    vector<Vector3d> xiDSE_;
    vector<Matrix3d> yawRotation_;
    vector<vector<Vector3d>> DSXiCoef_;
    int yawSign_;
    int length_;
    Vector3d CoMIntegral_;
    Vector3d CoMInit_;
    Vector3d currentXi_;
    Vector3d currentXiDot_;
    Vector3d currentZMP_;
    Vector3d prevXi_;
    Vector3d prevXiDot_;

    int currentStepNum_;

    vector<vector<double>> rotCoeffs_;
    // Functions for generating trajectories
    /**
     * @brief Updates the position of the virtual repulsive point (VRP) 
     * based on the current foot position.
     */
    void updateVRP();

    /**
     * @brief Generates trajectories for the single support phase.
     */
    void updateSS();

    /**
     * @brief Generates trajectories for the double support phase.
     */
    void updateDS();
    void updateOnlineDS(Vector3d xi_0, int init_step=0);

    /**
     * @brief Updates DCM position at the end of single support (xiEOS).
     */
    void updateXiEoS(int init_step=0);

    /**
     * @brief Updates DCM position at the start and end positions of the 
     * double support phase (xiDSI and xiDSE).
     */
    void updateXiDSPositions();
    
    /**
     * @brief Generates a minimum jerk trajectory between two given positions.
     * @param theta_ini The initial position.
     * @param theta_f The final position.
     * @param theta_dot_ini The initial velocity.
     * @param theta_dot_f The final velocity.
     * @param tf The time duration of the trajectory.
     * @return A pointer to an array of Vector3d representing the minimum jerk trajectory.
     */
    vector<Vector3d> minJerkInterpolate(Vector3d theta_ini, Vector3d theta_f, 
                                        Vector3d theta_dot_ini, Vector3d theta_dot_f, double tf);
};