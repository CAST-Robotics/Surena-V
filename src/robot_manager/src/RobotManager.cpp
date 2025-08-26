#include "RobotManager.h"

// --- CONSTRUCTOR ---
RobotManager::RobotManager(ros::NodeHandle *n) : nh_(n) {
    // Instantiate the managers, passing the node handle so they can create their own services/pubs/subs
    hand_manager_ = std::make_unique<HandManager>(n);
    gait_manager_ = std::make_unique<GaitManager>(n);

    load_scenarios_from_file();

    // ROS Communication Setup
    execute_scenario_service_ = nh_->advertiseService("execute_scenario_srv", &RobotManager::execute_scenario_callback, this);
}

// --- YAML FILE LOADER ---
void RobotManager::load_scenarios_from_file() {
    std::string file_path = ros::package::getPath("robot_manager") + "/config/scenarios.yaml";
    try {
        scenarios_config_ = YAML::LoadFile(file_path);
        YAML::Node scenarios_node = scenarios_config_["scenarios"];
        ROS_INFO("Successfully loaded scenarios. Available scenarios:");
        for (const auto& scenario : scenarios_node) {
            // scenario.first is the key (the scenario name) as a YAML::Node
            ROS_INFO("  - %s", scenario.first.as<std::string>().c_str());
        }
    } catch (const YAML::Exception& e) {
        ROS_FATAL("Failed to load or parse scenarios.yaml: %s", e.what());
    }
}

// --- MAIN SERVICE HANDLER ---
bool RobotManager::execute_scenario_callback(robot_manager::ExecuteScenario::Request &req, robot_manager::ExecuteScenario::Response &res) {
    ROS_INFO("Executing scenario: %s", req.scenario_name.c_str());
    YAML::Node steps;
    try {
        steps = scenarios_config_["scenarios"][req.scenario_name];
        if (!steps || !steps.IsSequence()) { throw std::runtime_error("is not a valid sequence."); }
    } catch (const std::exception& e) {
        res.success = false;
        res.message = "Scenario '" + req.scenario_name + "' error: " + e.what();
        ROS_ERROR("%s", res.message.c_str());
        return true;
    }

    for (int i = 0; i < steps.size(); ++i) {
        ROS_INFO("Executing step %d of %d...", i + 1, (int)steps.size());
        if (!execute_step(steps[i])) {
            res.success = false;
            res.message = "Failed at step " + std::to_string(i + 1) + ". Aborting scenario.";
            ROS_ERROR("%s", res.message.c_str());
            return true;
        }
    }

    res.success = true;
    res.message = "Scenario '" + req.scenario_name + "' completed successfully.";
    ROS_INFO("%s", res.message.c_str());
    return true;
}

// --- HELPER TO EXECUTE A SINGLE STEP ---
bool RobotManager::execute_step(const YAML::Node& step) {
    std::string service_name = step["service"].as<std::string>();
    YAML::Node params = step["params"];

    if (service_name == "/set_target_class_srv") {
        ros::ServiceClient client = nh_->serviceClient<hand_planner::SetTargetClass>(service_name);
        hand_planner::SetTargetClass srv;
        srv.request.class_name = params["class_name"].as<std::string>();
        if (!client.call(srv)) { ROS_ERROR("Service call to %s failed.", service_name.c_str()); return false; }
        return srv.response.class_id != -1;

    } else if (service_name == "/head_track_srv") {
        ros::ServiceClient client = nh_->serviceClient<hand_planner::head_track>(service_name);
        hand_planner::head_track srv;
        srv.request.duration_seconds = params["duration_seconds"].as<double>();
        if (!client.call(srv)) { ROS_ERROR("Service call to %s failed.", service_name.c_str()); return false; }
        return srv.response.success;

    } else if (service_name == "/move_hand_single_srv") {
        if (!params["mode"] || !params["ee_ini_pos"] || !params["scen_count"] || !params["t_total"] || !params["scenario"]) {
            ROS_ERROR("One or more required parameters are missing for %s", service_name.c_str()); return false;
        }
        ros::ServiceClient client = nh_->serviceClient<hand_planner::move_hand_single>(service_name);
        hand_planner::move_hand_single srv;
        srv.request.mode = params["mode"].as<std::string>();
        srv.request.ee_ini_pos = params["ee_ini_pos"].as<std::string>();
        srv.request.scen_count = params["scen_count"].as<int>();
        srv.request.t_total = params["t_total"].as<int>();
        srv.request.scenario = params["scenario"].as<std::vector<std::string>>();
        if (!client.call(srv)) { ROS_ERROR("Service call to %s failed.", service_name.c_str()); return false; }
        return true;

    } else if (service_name == "/move_hand_both_srv") {
        if (!params["scenarioR"] || !params["ee_ini_posR"] || !params["scenR_count"] || !params["scenarioL"] || !params["ee_ini_posL"] || !params["scenL_count"] || !params["t_total"]) {
            ROS_ERROR("One or more required parameters are missing for %s", service_name.c_str()); return false;
        }
        ros::ServiceClient client = nh_->serviceClient<hand_planner::move_hand_both>(service_name);
        hand_planner::move_hand_both srv;
        srv.request.ee_ini_posR = params["ee_ini_posR"].as<std::string>();
        srv.request.scenarioR = params["scenarioR"].as<std::vector<std::string>>();
        srv.request.scenR_count = params["scenR_count"].as<int>();
        srv.request.ee_ini_posL = params["ee_ini_posL"].as<std::string>();
        srv.request.scenarioL = params["scenarioL"].as<std::vector<std::string>>();
        srv.request.scenL_count = params["scenL_count"].as<int>();
        srv.request.t_total = params["t_total"].as<int>();
        if (!client.call(srv)) { ROS_ERROR("Service call to %s failed.", service_name.c_str()); return false; }
        return true;

    } else if (service_name == "/walk_service") {
        if (!params["alpha"] || !params["t_double_support"] || !params["t_step"] || !params["step_length"] || !params["step_width"] || !params["COM_height"] || !params["step_count"] ||
            !params["ankle_height"] || !params["dt"] || !params["theta"] || !params["step_height"] || !params["com_offset"] || !params["is_config"]) {
            ROS_ERROR("One or more required parameters are missing for %s", service_name.c_str()); return false;
        }
        ros::ServiceClient client = nh_->serviceClient<gait_planner::Trajectory>(service_name);
        gait_planner::Trajectory srv;
        srv.request.alpha = params["alpha"].as<double>();
        srv.request.t_double_support = params["t_double_support"].as<double>();
        srv.request.t_step = params["t_step"].as<double>();
        srv.request.step_length = params["step_length"].as<double>();
        srv.request.step_width = params["step_width"].as<double>();
        srv.request.COM_height = params["COM_height"].as<double>();
        srv.request.step_count = params["step_count"].as<int>();
        srv.request.ankle_height = params["ankle_height"].as<double>();
        srv.request.dt = params["dt"].as<double>();
        srv.request.theta = params["theta"].as<double>();
        srv.request.step_height = params["step_height"].as<double>();
        srv.request.com_offset = params["com_offset"].as<double>();
        srv.request.is_config = params["is_config"].as<bool>();
        if (!client.call(srv)) { ROS_ERROR("Service call to %s failed.", service_name.c_str()); return false; }
        return srv.response.result;
    
    } else if (service_name == "/grip_online_srv") {
        ros::ServiceClient client = nh_->serviceClient<hand_planner::gripOnline>(service_name);
        hand_planner::gripOnline srv;
        srv.request.start = params["start"].as<std::string>();
        if (!client.call(srv)) { ROS_ERROR("Service call to %s failed.", service_name.c_str()); return false; }
        return !srv.response.finish.empty();

    } else if (service_name == "/home_service" || service_name == "/keyboard_walk" || service_name == "/print_absolute") {
        ros::ServiceClient client = nh_->serviceClient<std_srvs::Empty>(service_name);
        std_srvs::Empty srv;
        if (!client.call(srv)) { ROS_ERROR("Service call to %s failed.", service_name.c_str()); return false; }
        return true;

    } else if (service_name == "/joint_command") {
        ros::ServiceClient client = nh_->serviceClient<gait_planner::command>(service_name);
        gait_planner::command srv;
        srv.request.motor_id = params["motor_id"].as<int>();
        srv.request.angle = params["angle"].as<double>();
        if (!client.call(srv)) { ROS_ERROR("Service call to %s failed.", service_name.c_str()); return false; }
        return srv.response.result;

    } else if (service_name == "/get_data") {
        ros::ServiceClient client = nh_->serviceClient<gait_planner::getdata>(service_name);
        gait_planner::getdata srv;
        srv.request.time = params["time"].as<double>();
        if (!client.call(srv)) { ROS_ERROR("Service call to %s failed.", service_name.c_str()); return false; }
        return true; // Assuming this service always succeeds if called
    
    } else {
        ROS_ERROR("Scenario step contains unknown service: %s", service_name.c_str());
        return false;
    }
}

// --- Main Function ---
int main(int argc, char **argv) {
    ros::init(argc, argv, "robot_manager_node");
    ros::NodeHandle n;
    RobotManager robot_manager(&n);
    ROS_INFO("Robot Manager is running and ready to execute commands and scenarios.");
    // Create an AsyncSpinner.
    // The '2' means it will create a pool of 2 threads to process callbacks and prevent deadlocks (one for the scenario client, one for the service provider). You can increase this if needed.
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::waitForShutdown(); // replacement of the old ros::spin().
    return 0;
}