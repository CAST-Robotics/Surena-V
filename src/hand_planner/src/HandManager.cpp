#include "HandManager.h"

// --- CONSTRUCTOR ---
HandManager::HandManager(ros::NodeHandle *n) : 
    hand_func_R(RIGHT), 
    hand_func_L(LEFT),
    // Initialize parameters
    T(0.005),
    rate(200),
    simulation(true),
    X(1.0), Y(0.0), Z(0.0),
    tempX(1.0), tempY(0.0), tempZ(0.0),
    h_pitch(0), h_roll(0), h_yaw(0),
    Kp(0.01), Ky(-0.01),
    t_grip(0),
    sum_r(0), sum_l(0)
{
    // Motor and sensor constants
    encoderResolution[0] = 4096 * 4;
    encoderResolution[1] = 2048 * 4;
    harmonicRatio[0] = 100;
    harmonicRatio[1] = 100;
    harmonicRatio[2] = 100;
    harmonicRatio[3] = 400;
    
    // Parameter vectors
    pitch_range = {-30, 30};
    roll_range = {-50, 50};
    yaw_range = {-90, 90};
    pitch_command_range = {180, 110};
    roll_command_range = {100, 190};
    yaw_command_range = {90, 210};
    wrist_command_range = {50, 250};
    wrist_yaw_range = {85, -95};
    wrist_right_range = {90, -90};
    wrist_left_range = {90, -90};

    q_rad_teleop.resize(14);
    q_rad_teleop.setZero();

    // ROS Communication Setup
    trajectory_data_pub = n->advertise<std_msgs::Int32MultiArray>("jointdata/qc", 100);
    gazeboJointStatePub_ = n->advertise<std_msgs::Float64MultiArray>("/joint_angles_gazebo", 100);
    camera_data_sub = n->subscribe("/detection_info", 1, &HandManager::object_detect_callback, this);
    joint_qc_sub = n->subscribe("jointdata/qc", 100, &HandManager::joint_qc_callback, this);
    teleoperation_data_sub = n->subscribe("teleoperation/angles", 100, &HandManager::teleoperation_callback, this);
    micArray_data_sub = n->subscribe("micarray/angle", 100, &HandManager::micArray_callback, this);
    move_hand_single_service = n->advertiseService("move_hand_single_srv", &HandManager::single_hand, this);
    move_hand_both_service = n->advertiseService("move_hand_both_srv", &HandManager::both_hands, this);
    grip_online_service = n->advertiseService("grip_online_srv", &HandManager::grip_online, this);
    home_service = n->advertiseService("home_srv", &HandManager::home, this);
    set_target_class_service = n->advertiseService("set_target_class_srv", &HandManager::setTargetClassService, this);
    head_track_service = n->advertiseService("head_track_srv", &HandManager::head_track_handler, this);
    teleoperation_service = n->advertiseService("teleoperation_srv", &HandManager::teleoperation_handler, this);
}

// --- Object Detection Callback Implementations ---
bool HandManager::setTargetClassService(hand_planner::SetTargetClass::Request &req, hand_planner::SetTargetClass::Response &res) {
        // Look up class ID from JSON
        string object_classes_path = ros::package::getPath("hand_planner") + "/config/object_classes.json";
        std::ifstream fr(object_classes_path);
        json object_classes = json::parse(fr);
        if (!object_classes.contains(req.class_name)) {
            ROS_ERROR("Class name '%s' not found in object_classes.json!", req.class_name.c_str());
            res.class_id = -1;
        } else {
            target_class_id_ = object_classes[req.class_name];    
            res.class_id = target_class_id_;
            X = 1.0; Y = 0; Z = 0; 
        }
        return true;
}

void HandManager::object_detect_callback(const hand_planner::DetectionInfoArray &msg) {
    std::lock_guard<std::mutex> lock(target_mutex_);
    for (size_t i = 0; i < msg.detections.size(); ++i) {
        if (msg.detections[i].class_id == target_class_id_) {
            double dist = msg.detections[i].distance / 1000.0;
            double y_pixel = msg.detections[i].x + msg.detections[i].width / 2.0;
            double z_pixel = msg.detections[i].y + msg.detections[i].height / 2.0;
            double a = 0.5, b = 0.4, X0 = 0.5;
            int L = 640, W = 480;
            
            double Y0 = -(y_pixel - L / 2.0) / L * a;
            double Z0 = -(z_pixel - W / 2.0) / W * b;
            double L0 = sqrt(pow(X0, 2) + pow(Y0, 2) + pow(Z0, 2));

            X = X0 * dist / L0;
            Y = Y0 * dist / L0;
            Z = Z0 * dist / L0;
            tempX = X; tempY = Y; tempZ = Z;
            return;
        }
    }
    X = tempX; Y = tempY; Z = tempZ;
}

void HandManager::joint_qc_callback(const std_msgs::Int32MultiArray::ConstPtr &qcArray) {
    for (size_t i = 0; i < qcArray->data.size() && i < 29; ++i) {
        QcArr[i] = qcArray->data[i];
    }
}

void HandManager::teleoperation_callback(const std_msgs::Float64MultiArray &q_deg_teleop) {
    assert(q_deg_teleop.data.size() == 14);
    for (size_t i = 0; i < q_deg_teleop.data.size(); ++i) {
        q_rad_teleop(i) = q_deg_teleop.data[i] * M_PI / 180.0;  // deg → rad
    }
}

void HandManager::micArray_callback(const std_msgs::Float64 &msg) {
    int MAX_CAPACITY = 10;
    micArray_data_buffer.push_back(msg.data);
    if (micArray_data_buffer.size() > MAX_CAPACITY) {
        micArray_data_buffer.pop_front();
    }
    // Only check if we have at least 5 values
    if (micArray_data_buffer.size() >= 3) {
        auto start = micArray_data_buffer.end() - 3;    // Take last 3 elements
        bool all_equal = std::all_of(start + 1, micArray_data_buffer.end(), [first = *start](double v){ return fabs(v - first) < 1e-6; });
        if (all_equal) {
            micArray_theta = (msg.data - 90) * M_PI / 180;
        }
    }
}

// --- Refactored Core Logic Implementations ---
MatrixXd HandManager::scenario_target(HandType type, string scenario, int i, VectorXd ee_pos, string ee_ini_pos) {
    MatrixXd result(6, 3);
    VectorXd r_middle, r_target, r_start;
    MatrixXd R_target;

    S5_hand& hand_func = (type == RIGHT) ? hand_func_R : hand_func_L;
    VectorXd& q_arm = (type == RIGHT) ? q_ra : q_la;
    VectorXd& q_init = (type == RIGHT) ? q_init_r : q_init_l;
    VectorXd& next_ee_pos = (type == RIGHT) ? next_ini_ee_posR : next_ini_ee_posL;

    if (scenario == "shakeHands") {
        r_middle = (type == RIGHT) ? Vector3d(0.35, -0.1, -0.2) : Vector3d(0.35, 0.1, -0.2);
        r_target = (type == RIGHT) ? Vector3d(0.3, -0.03, -0.3) : Vector3d(0.3, 0.03, -0.3);
        R_target = hand_func.rot(2, -65 * M_PI / 180, 3);
    } else if (scenario == "respect") {
        r_middle = (type == RIGHT) ? Vector3d(0.3, -0.1, -0.3) : Vector3d(0.3, 0.1, -0.3);
        r_target = (type == RIGHT) ? Vector3d(0.3, 0.1, -0.3) : Vector3d(0.3, -0.1, -0.3);
        double rot_angle = (type == RIGHT) ? 60 * M_PI / 180 : -60 * M_PI / 180;
        R_target = hand_func.rot(2, -80 * M_PI / 180, 3) * hand_func.rot(1, rot_angle, 3);
    } else if (scenario == "byebye") {
        r_middle = (type == RIGHT) ? Vector3d(0.35, -0.2, -0.15) : Vector3d(0.35, 0.2, -0.15);
        r_target = (type == RIGHT) ? Vector3d(0.3, -0.1, 0.22) : Vector3d(0.3, 0.1, 0.22);
        R_target = hand_func.rot(3, 90 * M_PI / 180, 3) * hand_func.rot( (type == RIGHT ? 1 : 2), -180 * M_PI / 180, 3);
    } else if (scenario == "punching") {
        r_middle = (type == RIGHT) ? Vector3d(0.15, -0.1, -0.1) : Vector3d(0.15, 0.1, -0.1);
        r_target = (type == RIGHT) ? Vector3d(0.35, 0.2, 0.1) : Vector3d(0.35, -0.2, 0.1);
        R_target = hand_func.rot(3, 90 * M_PI / 180, 3) * hand_func.rot(1, -40 * M_PI / 180, 3);
    } else if (scenario == "perfect") {
        r_middle = (type == RIGHT) ? Vector3d(0.15, -0.1, -0.3) : Vector3d(0.15, 0.1, -0.3);
        r_target = (type == RIGHT) ? Vector3d(0.25, -0.05, -0.25) : Vector3d(0.25, 0.05, -0.25);
        R_target = hand_func.rot(2, -90 * M_PI / 180, 3) * hand_func.rot(3, 90 * M_PI / 180, 3) * hand_func.rot(1, -45 * M_PI / 180, 3);
    } else if (scenario == "pointing") {
        r_middle = (type == RIGHT) ? Vector3d(0.25, -0.1, -0.1) : Vector3d(0.25, 0.1, -0.1);
        r_target = (type == RIGHT) ? Vector3d(0.45, 0.05, 0.0) : Vector3d(0.45, -0.05, 0.0);
        R_target = hand_func.rot(3, 90 * M_PI / 180, 3) * hand_func.rot(1, -65 * M_PI / 180, 3);
    } else if (scenario == "like") {
        r_middle = (type == RIGHT) ? Vector3d(0.15, -0.1, -0.3) : Vector3d(0.15, 0.1, -0.3);
        r_target = (type == RIGHT) ? Vector3d(0.25, -0.05, -0.25) : Vector3d(0.25, 0.05, -0.25);
        R_target = hand_func.rot(2, -90 * M_PI / 180, 3);
    } else if (scenario == "home") {
        r_middle = (type == RIGHT) ? Vector3d(0.3, -0.1, -0.25) : Vector3d(0.3, 0.1, -0.25);
        r_target = (type == RIGHT) ? Vector3d(0.15, -0.07, -0.43) : Vector3d(0.15, 0.07, -0.43);
        R_target = hand_func.rot(2, -20 * M_PI / 180, 3);
    }

    q_arm.resize(7);
    q_init.resize(7);
    if (i == 0) {
        if (ee_ini_pos == "init") {
            if (type == RIGHT) {
                q_arm << 10, -10, 0, -25, 0, 0, 0;
            } else { // LEFT
                q_arm << 10, 10, 0, -25, 0, 0, 0;
            }
            q_arm *= M_PI / 180.0;
            q_init = q_arm;
            hand_func.HO_FK_palm(q_arm);
            r_start = hand_func.r_palm;
        } else { 
            r_start = next_ee_pos; }
    } else { 
        r_start = ee_pos; }

    result << r_middle.transpose(), r_target.transpose(), R_target.row(0), R_target.row(1), R_target.row(2), r_start.transpose();
    return result;
}

VectorXd HandManager::reach_target(S5_hand& hand_model, VectorXd& q_arm, MatrixXd& qref_arm, double& sum_arm, VectorXd& q_init_arm, MatrixXd targets, string scenario, int M) {
    qref_arm.resize(7, M);
    Vector3d r_middle = targets.row(0), r_target = targets.row(1), r_start = targets.row(5);
    Matrix3d R_target = targets.block(2, 0, 3, 3);
    
    MatrixXd t_r(1, 3); 
    t_r << 0, 2, 4;
    double total_time = (scenario == "byebye" || scenario == "shakeHands") ? t_r(2) + 4 : t_r(2);
    int step_count = total_time / T;
    
    MatrixXd P_x(1, 3), P_y(1, 3), P_z(1, 3);
    P_x << r_start(0), r_middle(0), r_target(0);
    P_y << r_start(1), r_middle(1), r_target(1);
    P_z << r_start(2), r_middle(2), r_target(2);
    MatrixXd V_inf(1,3); V_inf << 0, INFINITY, 0; 
    MatrixXd A_inf(1,3); A_inf << 0, INFINITY, 0;

    // define minJerk elements to calculate end effector velocity
    MatrixXd X_coef = coef_generator.Coefficient(t_r, P_x, V_inf, A_inf);
    MatrixXd Y_coef = coef_generator.Coefficient(t_r, P_y, V_inf, A_inf);
    MatrixXd Z_coef = coef_generator.Coefficient(t_r, P_z, V_inf, A_inf);

    for (int count = 0; count < step_count; ++count) {
        double time_r = count * T;
        Vector3d V_curr;
        if (time_r < t_r(2)) {
            int interval = (time_r < t_r(1)) ? 0 : 1;
            double t_interval = (time_r < t_r(1)) ? t_r(0) : t_r(1);
            V_curr << coef_generator.GetAccVelPos(X_coef.row(interval), time_r, t_interval, 5)(0,1),
                      coef_generator.GetAccVelPos(Y_coef.row(interval), time_r, t_interval, 5)(0,1),
                      coef_generator.GetAccVelPos(Z_coef.row(interval), time_r, t_interval, 5)(0,1);
            hand_model.update_hand(q_arm, V_curr, r_target, R_target);
        } else {
            if (scenario == "byebye") { 
                q_arm(2) += (hand_model.hand_type == RIGHT ? -0.5 : 0.5) * M_PI/180 * cos((time_r - t_r(2)) * (2 * M_PI)); }
            else if (scenario == "shakeHands") { 
                q_arm(3) -= 0.125 * M_PI/180 * cos((time_r - t_r(2)) * M_PI); }
            hand_model.update_hand(q_arm, Vector3d::Zero(), r_target, R_target);
        }
        hand_model.doQP(q_arm);
        q_arm = hand_model.q_next;
        int global_index = count + sum_arm / T;
        if (global_index < qref_arm.cols()) { 
            qref_arm.col(global_index) = q_arm - q_init_arm; }
    }
    sum_arm += total_time;
    hand_model.HO_FK_palm(q_arm);
    return hand_model.r_palm;
}

// --- Service Handler Implementations ---
bool HandManager::single_hand(hand_planner::move_hand_single::Request &req, hand_planner::move_hand_single::Response &res) {
    ros::Rate rate_(rate);
    int M = req.t_total / T;
    vector<double> q_motor(29, 0.0);
    vector<double> q_gazebo(29, 0.0);
    VectorXd ee_pos = Vector3d::Zero();
    HandType type = (req.mode == "righthand") ? RIGHT : LEFT;
    
    if (type == RIGHT) {
        for (int i = 0; i < req.scen_count; i++) {
            MatrixXd result = scenario_target(RIGHT, req.scenario[i], i, ee_pos, req.ee_ini_pos);
            ee_pos = reach_target(hand_func_R, q_ra, qref_r, sum_r, q_init_r, result, req.scenario[i], M);
        }
    } else {
        for (int i = 0; i < req.scen_count; i++) {
            MatrixXd result = scenario_target(LEFT, req.scenario[i], i, ee_pos, req.ee_ini_pos);
            ee_pos = reach_target(hand_func_L, q_la, qref_l, sum_l, q_init_l, result, req.scenario[i], M);
        }
    }

    for (int id = 0; id < M; ++id) {
        if (!simulation) {
            if (type == RIGHT) {
                VectorXd q_rad = qref_r.col(id);
                q_motor[12]=int(q_rad(0)*encoderResolution[0]*harmonicRatio[0]/M_PI/2);
                q_motor[13]=-int(q_rad(1)*encoderResolution[0]*harmonicRatio[1]/M_PI/2);
                q_motor[14]=int(q_rad(2)*encoderResolution[1]*harmonicRatio[2]/M_PI/2);
                q_motor[15]=-int(q_rad(3)*encoderResolution[1]*harmonicRatio[3]/M_PI/2);
                // Note: Right Wrist calculations should be added here
                // q_motor[21] = int(wrist_command_range[0] + (wrist_command_range[1] - wrist_command_range[0]) * (((hand_func_R.wrist_left_calc(q_rad(5), q_rad(6))) - wrist_left_range[0]) / (wrist_left_range[1] - wrist_left_range[0])));
                // q_motor[20] = int(wrist_command_range[0] + (wrist_command_range[1] - wrist_command_range[0]) * (((hand_func_R.wrist_right_calc(q_rad(5), q_rad(6))) - (wrist_right_range[0])) / (wrist_right_range[1] - (wrist_right_range[0]))));
                // q_motor[22] = int(wrist_command_range[0] + (wrist_command_range[1] - wrist_command_range[0]) * (((q_rad(4) * 180 / M_PI) - wrist_yaw_range[0]) / (wrist_yaw_range[1] - wrist_yaw_range[0])));
            } else {
                VectorXd q_rad = qref_l.col(id);
                q_motor[16]=-int(q_rad(0)*encoderResolution[0]*harmonicRatio[0]/M_PI/2);
                q_motor[17]=-int(q_rad(1)*encoderResolution[0]*harmonicRatio[1]/M_PI/2);
                q_motor[18]=int(q_rad(2)*encoderResolution[1]*harmonicRatio[2]/M_PI/2);
                q_motor[19]=-int(q_rad(3)*encoderResolution[1]*harmonicRatio[3]/M_PI/2);
                // Note: Left Wrist calculations should be added here
                q_motor[21] = int(wrist_command_range[0] + (wrist_command_range[1] - wrist_command_range[0]) * (((q_rad(4) * 180 / M_PI) - wrist_yaw_range[0]) / (wrist_yaw_range[1] - wrist_yaw_range[0])));
                q_motor[22] = int(wrist_command_range[0] + (wrist_command_range[1] - wrist_command_range[0]) * (((hand_func_L.wrist_right_calc(q_rad(5), q_rad(6))) - (wrist_right_range[0])) / (wrist_right_range[1] - (wrist_right_range[0]))));
                q_motor[20] = int(wrist_command_range[0] + (wrist_command_range[1] - wrist_command_range[0]) * (((hand_func_L.wrist_left_calc(q_rad(5), q_rad(6))) - wrist_left_range[0]) / (wrist_left_range[1] - wrist_left_range[0])));
            }
            std_msgs::Int32MultiArray trajectory_data;
            for(int i = 0; i < 29; i++) { 
                trajectory_data.data.push_back(q_motor[i]); }
            trajectory_data_pub.publish(trajectory_data);
        }
        else { // simulation
            if (type == RIGHT) {
                VectorXd q_rad = qref_r.col(id);
                q_gazebo[12] = q_rad(0);  
                q_gazebo[13] = q_rad(1);   
                q_gazebo[14] = q_rad(2);  
                q_gazebo[15] = q_rad(3);
                q_gazebo[23] = q_rad(4);   
                q_gazebo[24] = q_rad(5);  
                q_gazebo[25] = q_rad(6);
            } else { // LEFT
                VectorXd q_rad = qref_l.col(id);
                q_gazebo[16] = q_rad(0);  
                q_gazebo[17] = q_rad(1);   
                q_gazebo[18] = q_rad(2);  
                q_gazebo[19] = q_rad(3);
                q_gazebo[26] = q_rad(4);   
                q_gazebo[27] = q_rad(5);  
                q_gazebo[28] = q_rad(6);   
            }
            joint_angles_gazebo_.data.clear();
            for (int i = 0; i < 29; i++) {
                joint_angles_gazebo_.data.push_back(q_gazebo[i]);
            }
            gazeboJointStatePub_.publish(joint_angles_gazebo_);
        }
        ros::spinOnce();
        rate_.sleep();
    }

    sum_r = 0; sum_l = 0;
    if (type == RIGHT) { 
        next_ini_ee_posR = ee_pos; } 
    else { 
        next_ini_ee_posL = ee_pos; }
    res.ee_fnl_pos = req.scenario[req.scen_count - 1];
    return true;
}

bool HandManager::both_hands(hand_planner::move_hand_both::Request &req, hand_planner::move_hand_both::Response &res) {
    ros::Rate rate_(rate);
    int M = req.t_total / T;
    vector<double> q_motor(29, 0.0);
    vector<double> q_gazebo(29, 0.0);
    VectorXd ee_pos_r = Vector3d::Zero();
    VectorXd ee_pos_l = Vector3d::Zero();

    // --- Trajectory Generation Phase ---
    // Generate trajectory for the right hand
    for (int i = 0; i < req.scenR_count; i++) {
        MatrixXd result_r = scenario_target(RIGHT, req.scenarioR[i], i, ee_pos_r, req.ee_ini_posR);
        ee_pos_r = reach_target(hand_func_R, q_ra, qref_r, sum_r, q_init_r, result_r, req.scenarioR[i], M);
    }
    // Generate trajectory for the left hand
    for (int i = 0; i < req.scenL_count; i++) {
        MatrixXd result_l = scenario_target(LEFT, req.scenarioL[i], i, ee_pos_l, req.ee_ini_posL);
        ee_pos_l = reach_target(hand_func_L, q_la, qref_l, sum_l, q_init_l, result_l, req.scenarioL[i], M);
    }

    // --- Execution Phase ---
    for (int id = 0; id < M; ++id) {
        VectorXd q_rad_r = qref_r.col(id);
        VectorXd q_rad_l = qref_l.col(id);

        if (!simulation) {
            // Right hand motors
            q_motor[12] = int(q_rad_r(0) * encoderResolution[0] * harmonicRatio[0] / (2 * M_PI));
            q_motor[13] = -int(q_rad_r(1) * encoderResolution[0] * harmonicRatio[1] / (2 * M_PI));
            q_motor[14] = int(q_rad_r(2) * encoderResolution[1] * harmonicRatio[2] / (2 * M_PI));
            q_motor[15] = -int(q_rad_r(3) * encoderResolution[1] * harmonicRatio[3] / (2 * M_PI));
            // Left hand motors
            q_motor[16] = -int(q_rad_l(0) * encoderResolution[0] * harmonicRatio[0] / (2 * M_PI));
            q_motor[17] = -int(q_rad_l(1) * encoderResolution[0] * harmonicRatio[1] / (2 * M_PI));
            q_motor[18] = int(q_rad_l(2) * encoderResolution[1] * harmonicRatio[2] / (2 * M_PI));
            q_motor[19] = -int(q_rad_l(3) * encoderResolution[1] * harmonicRatio[3] / (2 * M_PI));
            
            // Note: Both Wrist calculations should be added here

            std_msgs::Int32MultiArray trajectory_data;
            for(int i = 0; i < 29; i++) { 
                trajectory_data.data.push_back(q_motor[i]); 
            }
            trajectory_data_pub.publish(trajectory_data);
        }
        else { // simulation
            // Right hand joints
            q_gazebo[12] = q_rad_r(0);  
            q_gazebo[13] = q_rad_r(1);   
            q_gazebo[14] = q_rad_r(2);  
            q_gazebo[15] = q_rad_r(3);
            q_gazebo[23] = q_rad_r(4);   
            q_gazebo[24] = q_rad_r(5);  
            q_gazebo[25] = q_rad_r(6);
            // Left hand joints
            q_gazebo[16] = q_rad_l(0);  
            q_gazebo[17] = q_rad_l(1);   
            q_gazebo[18] = q_rad_l(2);  
            q_gazebo[19] = q_rad_l(3);
            q_gazebo[26] = q_rad_l(4);   
            q_gazebo[27] = q_rad_l(5);  
            q_gazebo[28] = q_rad_l(6); 
            
            joint_angles_gazebo_.data.clear();
            for (int i = 0; i < 29; i++) {
                joint_angles_gazebo_.data.push_back(q_gazebo[i]);
            }
            gazeboJointStatePub_.publish(joint_angles_gazebo_);
        }
        ros::spinOnce();
        rate_.sleep();
    }

    // --- Cleanup and Service Response ---
    sum_r = 0;
    sum_l = 0;
    next_ini_ee_posR = ee_pos_r;
    next_ini_ee_posL = ee_pos_l;
    
    res.ee_fnl_posR = req.scenarioR.empty() ? "" : req.scenarioR[req.scenR_count - 1];
    res.ee_fnl_posL = req.scenarioL.empty() ? "" : req.scenarioL[req.scenL_count - 1];
    
    return true;
}

bool HandManager::home(hand_planner::home_service::Request &req, hand_planner::home_service::Response &res) {
    ROS_WARN("Home service is not fully implemented yet.");
    return true;
}

bool HandManager::grip_online(hand_planner::gripOnline::Request &req, hand_planner::gripOnline::Response &res) {
    ros::Rate rate_(rate);
    vector<double> q_motor(29, 0.0);
    vector<double> q_gazebo(29, 0.0);
    VectorXd current_q_ra(7);
    current_q_ra << 10, -10, 0, -25, 0, 0, 0;
    current_q_ra *= M_PI / 180.0;
    VectorXd initial_q_ra = current_q_ra;
    
    Matrix3d R_target_r = Matrix3d::Identity();

    t_grip = 0;
    while (t_grip <= (120)) {
        Vector3d target2camera(X, Y, Z);
        MatrixXd T_CAM2SH = hand_func_R.ObjToNeck(-h_pitch, h_roll, -h_yaw);
        Vector3d target2shoulder = T_CAM2SH.block(0, 3, 3, 1) + T_CAM2SH.block(0, 0, 3, 3) * target2camera;
        
        // Head Pitch
        if (abs(target2camera(2)) > 0.02) {
            h_pitch += Kp * atan2(target2camera(2), sqrt(pow(target2camera(1),2) + pow(target2camera(0),2)));
            h_pitch = max(-28.0*M_PI/180, min(28.0*M_PI/180, h_pitch));
        }
        // Head Yaw
        if (abs(target2camera(1)) > 0.02) {
            h_yaw += Ky * atan2(target2camera(1), target2camera(0));
            h_yaw = max(-60.0*M_PI/180, min(60.0*M_PI/180, h_yaw));
        }

        if (t_grip >= 15 && Y != 0 && Z != 0) {
        R_target_r = hand_func_R.rot(2, -65 * M_PI / 180, 3);
        hand_func_R.update_hand(current_q_ra, Vector3d::Zero(), target2shoulder, R_target_r);
        Vector3d V_r = 0.7 * (target2shoulder - hand_func_R.r_palm);
        
        hand_func_R.update_hand(current_q_ra, V_r, target2shoulder, R_target_r);
        hand_func_R.doQP(current_q_ra);  // Solve the inverse kinematics
        current_q_ra = hand_func_R.q_next;
        }

        if (!simulation) {
            VectorXd q_delta = current_q_ra - initial_q_ra;
            q_motor[12]=int(q_delta(0)*encoderResolution[0]*harmonicRatio[0]/M_PI/2);
            q_motor[13]=-int(q_delta(1)*encoderResolution[0]*harmonicRatio[1]/M_PI/2);
            q_motor[14]=int(q_delta(2)*encoderResolution[1]*harmonicRatio[2]/M_PI/2);
            q_motor[15]=-int(q_delta(3)*encoderResolution[1]*harmonicRatio[3]/M_PI/2);
            q_motor[20] = int(roll_command_range[0] + (roll_command_range[1] - roll_command_range[0]) * ((-(h_roll*180/M_PI) - roll_range[0]) / (roll_range[1] - roll_range[0])));
            q_motor[21] = int(pitch_command_range[0] + (pitch_command_range[1] - pitch_command_range[0]) * ((-(h_pitch*180/M_PI) - pitch_range[0]) / (pitch_range[1] - pitch_range[0])));
            q_motor[22] = int(yaw_command_range[0] + (yaw_command_range[1] - yaw_command_range[0]) * ((-(h_yaw*180/M_PI) - yaw_range[0]) / (yaw_range[1] - yaw_range[0])));
            
            std_msgs::Int32MultiArray trajectory_data;
            for(int i = 0; i < 29; i++) { 
                trajectory_data.data.push_back(q_motor[i]); }
            trajectory_data_pub.publish(trajectory_data);
        } else { // simulation
            VectorXd q_delta = current_q_ra - initial_q_ra;
            q_gazebo[12] = q_delta(0);  
            q_gazebo[13] = q_delta(1);   
            q_gazebo[14] = q_delta(2);  
            q_gazebo[15] = q_delta(3);
            q_gazebo[23] = q_delta(4);   
            q_gazebo[24] = q_delta(5);  
            q_gazebo[25] = q_delta(6);

            q_gazebo[20] = -h_roll;
            q_gazebo[21] = -h_pitch;
            q_gazebo[22] = -h_yaw;

            joint_angles_gazebo_.data.clear();
            for (int i = 0; i < 29; i++) {
                joint_angles_gazebo_.data.push_back(q_gazebo[i]);
            }
            gazeboJointStatePub_.publish(joint_angles_gazebo_);
        }
        ros::spinOnce();
        rate_.sleep();
        t_grip += T;
    }

    res.finish = "end";
    return true;
}

bool HandManager::head_track_handler(hand_planner::head_track::Request &req, hand_planner::head_track::Response &res) {
    ROS_INFO("Starting head tracking for %.2f seconds.", req.duration_seconds);
    ros::Rate rate_(rate);
    ros::Time start_time = ros::Time::now();
    vector<double> q_motor(29, 0.0);
    vector<double> q_gazebo(29, 0.0);

    while (ros::ok() && (ros::Time::now() - start_time).toSec() < req.duration_seconds) {
        Vector3d target2camera(X, Y, Z);
        
        if (Y != 0 && Z != 0){ // target not in robot's sight; so the head cannot track object yet.
            // Head Pitch
            if (abs(target2camera(2)) > 0.02) {
                h_pitch += Kp * atan2(target2camera(2), sqrt(pow(target2camera(1), 2) + pow(target2camera(0), 2)));
                h_pitch = max(-28.0 * M_PI / 180, min(28.0 * M_PI / 180, h_pitch));
            }
            // Head Yaw
            if (abs(target2camera(1)) > 0.02) {
                h_yaw += Ky * atan2(target2camera(1), target2camera(0));
                h_yaw = max(-60.0 * M_PI / 180, min(60.0 * M_PI / 180, h_yaw));
            }
        } else { // follow direction of arrival (voice) instead.
            h_yaw += Ky * micArray_theta;
            h_yaw = max(-60.0 * M_PI / 180, min(60.0 * M_PI / 180, h_yaw));
        }

        if (!simulation) {
            q_motor[20] = int(roll_command_range[0] + (roll_command_range[1] - roll_command_range[0]) * ((-(h_roll*180/M_PI) - roll_range[0]) / (roll_range[1] - roll_range[0])));
            q_motor[21] = int(pitch_command_range[0] + (pitch_command_range[1] - pitch_command_range[0]) * ((-(h_pitch*180/M_PI) - pitch_range[0]) / (pitch_range[1] - pitch_range[0])));
            q_motor[22] = int(yaw_command_range[0] + (yaw_command_range[1] - yaw_command_range[0]) * ((-(h_yaw*180/M_PI) - yaw_range[0]) / (yaw_range[1] - yaw_range[0])));
            
            std_msgs::Int32MultiArray trajectory_data;
            for(int i = 0; i < 29; i++) { 
                trajectory_data.data.push_back(q_motor[i]); }
            trajectory_data_pub.publish(trajectory_data);
        } else { // simulation
            q_gazebo[20] = -h_roll;
            q_gazebo[21] = -h_pitch;
            q_gazebo[22] = -h_yaw;
            
            joint_angles_gazebo_.data.clear();
            for (int i = 0; i < 29; i++) {
                joint_angles_gazebo_.data.push_back(q_gazebo[i]);
            }
            gazeboJointStatePub_.publish(joint_angles_gazebo_);
        }
        ros::spinOnce();
        rate_.sleep();
    }
    ROS_INFO("Head tracking finished.");
    res.success = true;
    return true;
}

bool HandManager::teleoperation_handler(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
    ros::Rate rate_(rate);
    ros::Time start_time = ros::Time::now();
    vector<double> q_motor(29, 0.0);
    vector<double> q_gazebo(29, 0.0);
    while (ros::ok() && (ros::Time::now() - start_time).toSec() < 120) {
        if (!simulation) {
            // RIGHT
            q_motor[12]=int(q_rad_teleop(0)*encoderResolution[0]*harmonicRatio[0]/M_PI/2);
            q_motor[13]=-int(q_rad_teleop(1)*encoderResolution[0]*harmonicRatio[1]/M_PI/2);
            q_motor[14]=int(q_rad_teleop(2)*encoderResolution[1]*harmonicRatio[2]/M_PI/2);
            q_motor[15]=-int(q_rad_teleop(3)*encoderResolution[1]*harmonicRatio[3]/M_PI/2);
            // q_motor[21] = int(wrist_command_range[0] + (wrist_command_range[1] - wrist_command_range[0]) * (((hand_func_R.wrist_left_calc(q_rad_teleop(5), q_rad_teleop(6))) - wrist_left_range[0]) / (wrist_left_range[1] - wrist_left_range[0])));
            // q_motor[20] = int(wrist_command_range[0] + (wrist_command_range[1] - wrist_command_range[0]) * (((hand_func_R.wrist_right_calc(q_rad_teleop(5), q_rad_teleop(6))) - (wrist_right_range[0])) / (wrist_right_range[1] - (wrist_right_range[0]))));
            // q_motor[22] = int(wrist_command_range[0] + (wrist_command_range[1] - wrist_command_range[0]) * (((q_rad_teleop(4) * 180 / M_PI) - wrist_yaw_range[0]) / (wrist_yaw_range[1] - wrist_yaw_range[0])));
            // LEFT
            q_motor[16]=-int(q_rad_teleop(7)*encoderResolution[0]*harmonicRatio[0]/M_PI/2);
            q_motor[17]=-int(q_rad_teleop(8)*encoderResolution[0]*harmonicRatio[1]/M_PI/2);
            q_motor[18]=int(q_rad_teleop(9)*encoderResolution[1]*harmonicRatio[2]/M_PI/2);
            q_motor[19]=-int(q_rad_teleop(10)*encoderResolution[1]*harmonicRatio[3]/M_PI/2);
            // q_motor[21] = int(wrist_command_range[0] + (wrist_command_range[1] - wrist_command_range[0]) * (((hand_func_L.wrist_left_calc(q_rad_teleop(12), q_rad_teleop(13))) - wrist_left_range[0]) / (wrist_left_range[1] - wrist_left_range[0])));
            // q_motor[20] = int(wrist_command_range[0] + (wrist_command_range[1] - wrist_command_range[0]) * (((hand_func_L.wrist_right_calc(q_rad_teleop(12), q_rad_teleop(13))) - (wrist_right_range[0])) / (wrist_right_range[1] - (wrist_right_range[0]))));
            // q_motor[22] = int(wrist_command_range[0] + (wrist_command_range[1] - wrist_command_range[0]) * (((q_rad_teleop(11) * 180 / M_PI) - wrist_yaw_range[0]) / (wrist_yaw_range[1] - wrist_yaw_range[0])));
        
            std_msgs::Int32MultiArray trajectory_data;
            for(int i = 0; i < 29; i++) { 
                trajectory_data.data.push_back(q_motor[i]); }
            trajectory_data_pub.publish(trajectory_data);
        } else { // simulation
            // RIGHT
            q_gazebo[12] = q_rad_teleop(0);
            q_gazebo[13] = q_rad_teleop(1);
            q_gazebo[14] = q_rad_teleop(2);
            q_gazebo[15] = q_rad_teleop(3);
            q_gazebo[23] = q_rad_teleop(4);
            q_gazebo[24] = q_rad_teleop(5);
            q_gazebo[25] = q_rad_teleop(6);
            // LEFT
            q_gazebo[16] = q_rad_teleop(7);
            q_gazebo[17] = q_rad_teleop(8);
            q_gazebo[18] = q_rad_teleop(9);
            q_gazebo[19] = q_rad_teleop(10);
            q_gazebo[26] = q_rad_teleop(11);
            q_gazebo[27] = q_rad_teleop(12);
            q_gazebo[28] = q_rad_teleop(13);

            joint_angles_gazebo_.data.clear();
            for (int i = 0; i < 29; i++) {
                joint_angles_gazebo_.data.push_back(q_gazebo[i]);
            }
            gazeboJointStatePub_.publish(joint_angles_gazebo_);
        }
        ros::spinOnce();
        rate_.sleep();
    }
    ROS_INFO("Teleoperation time finished.");
    return true;
}

// // --- Main Function ---
// int main(int argc, char **argv) {
//     ros::init(argc, argv, "hand_manager_node");
//     ros::NodeHandle n;
//     HandManager node_handler(&n);
//     ROS_INFO("Hand Manager Node is ready to receive service calls.");
//     ros::spin();
//     return 0;
// }