#ifndef ROBOT_MANAGER_H
#define ROBOT_MANAGER_H

#include "ros/ros.h"
#include <memory>
#include <yaml-cpp/yaml.h>
#include "ros/callback_queue.h"

// Include managers from library packages
#include "HandManager.h"
#include "GaitManager.h"

// Include our new service definition
#include "robot_manager/ExecuteScenario.h"

class RobotManager {
public:
    RobotManager(ros::NodeHandle *n);

private:
    // --- ROS Communication ---
    ros::NodeHandle* nh_;
    ros::ServiceServer execute_scenario_service_;

    // --- Pointers to Specialized Managers ---
    std::unique_ptr<HandManager> hand_manager_;
    std::unique_ptr<GaitManager> gait_manager_;

    // --- Member to hold the parsed scenario data ---
    YAML::Node scenarios_config_;

    // --- Service Handlers ---
    bool execute_scenario_callback(robot_manager::ExecuteScenario::Request &req, robot_manager::ExecuteScenario::Response &res);
    
    // --- Helper Function ---
    void load_scenarios_from_file();
    bool execute_step(const YAML::Node& step);
};

#endif // ROBOT_MANAGER_H