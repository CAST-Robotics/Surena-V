#include "GaitManager.h"

GaitManager::GaitManager(ros::NodeHandle *n)
{
    string config_path = ros::package::getPath("gait_planner") + "/config/surenav_config.json";
    robot = new Robot(n, config_path);

    motorDataPub_ = n->advertise<std_msgs::Int32MultiArray>("jointdata/qc", 100);
    absSub_ = n->subscribe("/surena/abs_joint_state", 100, &GaitManager::absReader, this);
    offsetSub_ = n->subscribe("/surena/inc_joint_state", 100, &GaitManager::qcInitial, this);
    incSub_ = n->subscribe("/surena/inc_joint_state", 100, &GaitManager::incReader, this);
    jointCommand_ = n->advertiseService("joint_command", &GaitManager::sendCommand, this);
    absPrinter_ = n->advertiseService("print_absolute", &GaitManager::absPrinter, this);
    walkService_ = n->advertiseService("walk_service", &GaitManager::walk, this);
    keyboardWalkService_ = n->advertiseService("keyboard_walk", &GaitManager::keyboardWalk, this);
    homeService_ = n->advertiseService("home_service", &GaitManager::home, this);
    dummyCommand_ = n->advertiseService("get_data", &GaitManager::dummyCallback, this);
    lFT_ = n->subscribe("/surena/ft_l_state", 100, &GaitManager::ftCallbackLeft, this);
    rFT_ = n->subscribe("/surena/ft_r_state", 100, &GaitManager::ftCallbackRight, this);
    IMUSub_ = n->subscribe("/surena/imu_state", 100, &GaitManager::IMUCallback, this);
    AccSub_ = n->subscribe("/imu/acceleration", 100, &GaitManager::AccCallback, this);
    GyroSub_ = n->subscribe("/imu/angular_velocity", 100, &GaitManager::GyroCallback, this);
    bumpSub_ = n->subscribe("/surena/bump_sensor_state", 100, &GaitManager::bumpCallback, this);
    keyboardCommandSub_ = n->subscribe("/keyboard_command", 1, &GaitManager::keyboardHandler, this);

    b = 0.049;
    c = 0.35;
    r1 = 0.36;
    r0 = 0.047;
    r30_inner << 0.035, 0.034, -0.002;
    r30_outer << 0.035, -0.034, -0.002;

    qcInitialBool_ = false;
    isKeyboardTrajectoryEnabled = false;
    isWalkingWithKeyboard = false;

    int temp_ratio[12] = {100, 100, 50, 80, 100, 100, 50, 80, 120, 120, 120, 120};
    // int temp_home_abs[12] = {122378, 141666, 134313, 8607, 132216, 130155, 144401, 146847, 125254, 63296, 130474, 145216};
    // int temp_home_abs[12] = {122378, 141090, 134441, 11167, 132216, 130155, 145297, 146399, 124358, 63552, 130474, 145216};
    //int temp_home_abs[12] = {122378, 141090, 135273, 11167, 132216, 130155, 145297, 146399, 124614, 63552, 130474, 145216};
    // int temp_home_abs[12] = {123722, 141090, 135273, 11167, 136504, 131243, 146513, 146399, 124614, 63552, 129130, 141904};
    int temp_home_abs[12] = {123722, 166000, 135273, 11167, 132676, 131435, 146513, 146399, 124614, 63552, 132714, 141488};

    int temp_abs_high[12] = {108426, 119010, 89733, 136440, 71608, 102443, 119697, 82527, 168562, 160000, 191978, 111376};
    int temp_abs_low[12] = {145354, 183778, 194153, 7000, 203256, 160491, 150225, 180000, 61510, 61000, 61482, 172752};
    int temp_abs2inc_dir[12] = {1, 1, -1, -1, -1, 1, 1, 1, -1, 1, 1, 1};
    int temp_abs_dir[12] = {-1, -1, -1, 1, -1, -1, -1, -1, 1, 1, 1, -1};
    int temp_motor_dir[12] = {1, 1, 1, -1, -1, 1, 1, 1, -1, 1, 1, -1};
    int temp_bump_order[8] = {3, 0, 1, 2, 6, 5, 4, 7};
    int temp_initial_bump[4] = {0, 0, 0, 0};

    collision_ = false;

    for (int i = 0; i < 26; i++)
    {
        if (i < 20)
            motorCommandArray_[i] = 0;
        else if (i == 20) // head roll
            motorCommandArray_[i] = 145;
        else if (i == 21) // head pitch
            motorCommandArray_[i] = 165;
        else if (i >= 22) // head yaw
            motorCommandArray_[i] = 145;
        
    }

    for (int i = 0; i < 12; i++)
    {
        harmonicRatio_[i] = temp_ratio[i];
        homeAbs_[i] = temp_home_abs[i];
        absHigh_[i] = temp_abs_high[i];
        absLow_[i] = temp_abs_low[i];
        abs2incDir_[i] = temp_abs2inc_dir[i];
        absDir_[i] = temp_abs_dir[i];
        motorDir_[i] = temp_motor_dir[i];
        commandConfig_[0][i] = 0.0;
        commandConfig_[1][i] = 0.0;
        commandConfig_[2][i] = 0.0;
        if (i < 8)
        {
            bumpOrder_[i] = temp_bump_order[i];
        }
        if (i < 4)
        {
            rBumpOffset_[i] = temp_initial_bump[i];
            lBumpOffset_[i] = temp_initial_bump[i];
        }
    }

    leftFTFile_.open("/home/surena/SurenaV/log/left_ft.csv");
    rightFTFile_.open("/home/surena/SurenaV/log/right_ft.csv");
    if (!leftFTFile_.is_open())
        ROS_DEBUG("left FT log file not open!");
    if (!rightFTFile_.is_open())
        ROS_DEBUG("right FT log file not open!");

    FTOffsetPeriod_ = 60;
    lFTOffset_ = Vector3d::Zero();
    rFTOffset_ = Vector3d::Zero();

    double temp[3] = {0.0, 0.0, 0.0};
    for (int i = 0; i < 3; i++)
    {
        currentLFT_[i] = temp[i];
        currentRFT_[i] = temp[i];
    }
}

bool GaitManager::sendCommand()
{
    motorCommand_.data.clear();
    for (int i = 0; i < 26; i++) {
        motorCommand_.data.push_back(motorCommandArray_[i]);
        // cout << motorCommandArray_[i] << ","; 
    }
    // cout << endl;
    motorDataPub_.publish(motorCommand_);
}

bool GaitManager::setPos(int jointID, int dest)
{
    // this function is used for changing the position of all joint except ankle joints.
    // the value of dest is on the basis of absolute incs.
    ros::Rate rate_(200);
    int temp_roll = jointID;
    while (abs(abs(absData_[temp_roll]) - dest) > 100)
    {
        if (abs(absData_[temp_roll]) < 262144)
        {
            if (abs(absData_[temp_roll]) > dest)
            {
                motorCommandArray_[temp_roll] -= abs2incDir_[jointID] * (4096 * 4 * 0.01);
            }
            else
            {
                motorCommandArray_[temp_roll] += abs2incDir_[jointID] * (4096 * 4 * 0.01);
            }
        }
        sendCommand();
        ros::spinOnce();
        rate_.sleep();
    }
    return true;
}

bool GaitManager::home(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    // ankleHome(false);
    setPos(6, homeAbs_[6]);
    setPos(0, homeAbs_[0]);
    setPos(1, homeAbs_[1] - 20000);
    setPos(7, homeAbs_[7] + 20000);
    ankleHome(false, homeAbs_[5], homeAbs_[4]);
    setPos(3, homeAbs_[3]);
    setPos(2, homeAbs_[2]);
    setPos(1, homeAbs_[1]);
    ankleHome(true, homeAbs_[11], homeAbs_[10]);
    setPos(9, homeAbs_[9]);
    setPos(8, homeAbs_[8]);
    setPos(7, homeAbs_[7]);
    qcInitialBool_ = true;
    setFTZero();
    setBumpZero();
    ros::spinOnce();
    for (int i = 0; i < 12; i++)
    {
        commandConfig_[0][i] = 0.0;
        commandConfig_[1][i] = 0.0;
        commandConfig_[2][i] = 0.0;
    }
    return true;
}

void GaitManager::qcInitial(const sensor_msgs::JointState &msg)
{

    if (qcInitialBool_)
    {
        this->emptyCommand();
        for (int i = 0; i < 12; ++i)
        {
            homeOffset_[i] = int(msg.position[i + 1]);
            motorCommandArray_[i] = homeOffset_[i];
        }
        qcInitialBool_ = false;
    }
}

void GaitManager::ftCallbackLeft(const geometry_msgs::Wrench &msg)
{
    currentLFT_[0] = msg.force.x - lFTOffset_(0);
    currentLFT_[1] = msg.force.y - lFTOffset_(1);
    currentLFT_[2] = msg.force.z - lFTOffset_(2);
    leftFTFile_ << currentLFT_[0] << "," << currentLFT_[1] << "," << currentLFT_[2] << endl;
    if (recentLFT_.size() < FTOffsetPeriod_)
    {
        recentLFT_.push_back(Vector3d(msg.force.x, msg.force.y, msg.force.z));
    }
    else
    {
        recentLFT_.erase(recentLFT_.begin());
        recentLFT_.push_back(Vector3d(msg.force.x, msg.force.y, msg.force.z));
    }
}

void GaitManager::bumpCallback(const std_msgs::Int32MultiArray &msg)
{
    for (int i = 0; i < 8; ++i)
    {
        if (i < 4)
        {
            currentRBump_[bumpOrder_[i]] = msg.data[i] - rBumpOffset_[bumpOrder_[i]];
            realRBump_[bumpOrder_[i]] = msg.data[i];
        }
        else
        {
            currentLBump_[bumpOrder_[i] - 4] = msg.data[i] - lBumpOffset_[bumpOrder_[i] - 4];
            realLBump_[bumpOrder_[i] - 4] = msg.data[i];
        }
    }
}

void GaitManager::ftCallbackRight(const geometry_msgs::Wrench &msg)
{
    currentRFT_[0] = msg.force.x - rFTOffset_(0);
    currentRFT_[1] = msg.force.y - rFTOffset_(1);
    currentRFT_[2] = msg.force.z - rFTOffset_(2);
    rightFTFile_ << currentRFT_[0] << "," << currentRFT_[1] << "," << currentRFT_[2] << endl;
    if (recentRFT_.size() < FTOffsetPeriod_)
    {
        recentRFT_.push_back(Vector3d(msg.force.x, msg.force.y, msg.force.z));
    }
    else
    {
        recentRFT_.erase(recentRFT_.begin());
        recentRFT_.push_back(Vector3d(msg.force.x, msg.force.y, msg.force.z));
    }
}

void GaitManager::IMUCallback(const sensor_msgs::Imu &msg)
{
    // baseAcc_[0] = msg.linear_acceleration.x;
    // baseAcc_[1] = msg.linear_acceleration.y;
    // baseAcc_[2] = msg.linear_acceleration.z;

    baseOrient_[0] = msg.orientation.x;
    baseOrient_[1] = msg.orientation.y;
    baseOrient_[2] = msg.orientation.z;

    // baseAngVel_[0] = msg.angular_velocity.x;
    // baseAngVel_[1] = msg.angular_velocity.y;
    // baseAngVel_[2] = msg.angular_velocity.z;
}

void GaitManager::AccCallback(const geometry_msgs::Vector3Stamped &msg)
{
    baseAcc_[0] = msg.vector.x;
    baseAcc_[1] = msg.vector.y;
    baseAcc_[2] = msg.vector.z;
}

void GaitManager::GyroCallback(const geometry_msgs::Vector3Stamped &msg)
{
    baseAngVel_[0] = msg.vector.x;
    baseAngVel_[1] = msg.vector.y;
    baseAngVel_[2] = msg.vector.z;
}

bool GaitManager::setFTZero()
{

    for (int i = 0; i < recentRFT_.size(); i++)
    {
        lFTOffset_ += recentLFT_[i];
        rFTOffset_ += recentRFT_[i];
    }
    lFTOffset_ = lFTOffset_ / recentLFT_.size();
    rFTOffset_ = rFTOffset_ / recentRFT_.size();
    return true;
}

bool GaitManager::setBumpZero()
{
    for (int i = 0; i < 4; i++)
    {
        rBumpOffset_[i] = realRBump_[i];
        lBumpOffset_[i] = realLBump_[i];
    }
}

bool GaitManager::dummyCallback(gait_planner::getdata::Request &req,
                                 gait_planner::getdata::Response &res)
{
    for (int i = 0; i < req.time * 100; i++)
    {
        emptyCommand();
    }
    return true;
}

bool GaitManager::emptyCommand()
{
    ros::spinOnce();
    ros::Rate rate_(200);
    for (int i = 0; i < 12; i++)
        motorCommandArray_[i] = incData_[i];

    motorCommandArray_[0] += 1;
    sendCommand();
    rate_.sleep();
    motorCommandArray_[0] -= 1;
    sendCommand();
    rate_.sleep();
    return true;
}

bool GaitManager::sendCommand(gait_planner::command::Request &req,
                               gait_planner::command::Response &res)
{

    ros::Rate rate_(200);
    this->emptyCommand();
    if (req.motor_id == 4)
    {

        double theta_inner;
        double theta_outer;
        double cur_pitch = absDir_[4] * (absData_[4] - homeAbs_[4]) * 2 * M_PI / pow(2, 19);
        double cur_roll = absDir_[5] * (absData_[5] - homeAbs_[5]) * 2 * M_PI / pow(2, 19);

        double desired_pitch = cur_pitch + req.angle;
        this->ankleMechanism(theta_inner, theta_outer, desired_pitch, cur_roll, false);
        double inner_inc = motorDir_[5] * theta_inner * 4096 * 4 * 100 * 1.5 / 2 / M_PI + homeOffset_[5];
        double outer_inc = motorDir_[4] * theta_outer * 4096 * 4 * 100 * 1.5 / 2 / M_PI + homeOffset_[4];

        double inner_delta_inc = inner_inc - incData_[5];
        double outer_delta_inc = outer_inc - incData_[4];

        motorCommandArray_[4] = incData_[4];
        motorCommandArray_[5] = incData_[5];

        for (int i = 0; i < 100; i++)
        {
            motorCommandArray_[4] += outer_delta_inc / 100;
            motorCommandArray_[5] += inner_delta_inc / 100;
            sendCommand();
            ros::spinOnce();
            rate_.sleep();
        }
    }
    else if (req.motor_id == 5)
    {

        double theta_inner;
        double theta_outer;
        double cur_pitch = absDir_[4] * (absData_[4] - homeAbs_[4]) * 2 * M_PI / pow(2, 19);
        double cur_roll = absDir_[5] * (absData_[5] - homeAbs_[5]) * 2 * M_PI / pow(2, 19);

        double desired_roll = cur_roll + req.angle;
        this->ankleMechanism(theta_inner, theta_outer, cur_pitch, desired_roll, false);
        double inner_inc = motorDir_[5] * theta_inner * 4096 * 4 * 100 * 1.5 / 2 / M_PI + homeOffset_[5];
        double outer_inc = motorDir_[4] * theta_outer * 4096 * 4 * 100 * 1.5 / 2 / M_PI + homeOffset_[4];

        double inner_delta_inc = inner_inc - incData_[5];
        double outer_delta_inc = outer_inc - incData_[4];

        motorCommandArray_[4] = incData_[4];
        motorCommandArray_[5] = incData_[5];

        for (int i = 0; i < 100; i++)
        {
            motorCommandArray_[4] += outer_delta_inc / 100;
            motorCommandArray_[5] += inner_delta_inc / 100;
            sendCommand();
            ros::spinOnce();
            rate_.sleep();
        }
    }
    else if (req.motor_id == 10)
    {

        double theta_inner;
        double theta_outer;
        double cur_pitch = absDir_[10] * (absData_[10] - homeAbs_[10]) * 2 * M_PI / pow(2, 19);
        double cur_roll = absDir_[11] * (absData_[11] - homeAbs_[11]) * 2 * M_PI / pow(2, 19);

        double desired_pitch = cur_pitch + req.angle;
        this->ankleMechanism(theta_inner, theta_outer, desired_pitch, cur_roll, true);
        double inner_inc = motorDir_[11] * theta_inner * 4096 * 4 * 100 * 1.5 / 2 / M_PI + homeOffset_[11];
        double outer_inc = motorDir_[10] * theta_outer * 4096 * 4 * 100 * 1.5 / 2 / M_PI + homeOffset_[10];

        double inner_delta_inc = inner_inc - incData_[11];
        double outer_delta_inc = outer_inc - incData_[10];

        motorCommandArray_[10] = incData_[10];
        motorCommandArray_[11] = incData_[11];

        for (int i = 0; i < 100; i++)
        {
            motorCommandArray_[10] += outer_delta_inc / 100;
            motorCommandArray_[11] += inner_delta_inc / 100;
            sendCommand();
            ros::spinOnce();
            rate_.sleep();
        }
    }
    else if (req.motor_id == 11)
    {

        double theta_inner;
        double theta_outer;
        double cur_pitch = absDir_[10] * (absData_[10] - homeAbs_[10]) * 2 * M_PI / pow(2, 19);
        double cur_roll = absDir_[11] * (absData_[11] - homeAbs_[11]) * 2 * M_PI / pow(2, 19);

        double desired_roll = cur_roll + req.angle;
        this->ankleMechanism(theta_inner, theta_outer, cur_pitch, desired_roll, true);
        double inner_inc = motorDir_[11] * theta_inner * 4096 * 4 * 100 * 1.5 / 2 / M_PI + homeOffset_[11];
        double outer_inc = motorDir_[10] * theta_outer * 4096 * 4 * 100 * 1.5 / 2 / M_PI + homeOffset_[10];

        double inner_delta_inc = inner_inc - incData_[11];
        double outer_delta_inc = outer_inc - incData_[10];

        motorCommandArray_[10] = incData_[10];
        motorCommandArray_[11] = incData_[11];

        for (int i = 0; i < 100; i++)
        {
            motorCommandArray_[10] += outer_delta_inc / 100;
            motorCommandArray_[11] += inner_delta_inc / 100;
            sendCommand();
            ros::spinOnce();
            rate_.sleep();
        }
    }
    else if (req.motor_id == 0)
    {
        double theta;
        this->yawMechanism(theta, req.angle, 0.03435, 0.088, false);
        double yaw_inc = motorDir_[0] * theta * 4096 * 4 * 100 / 2 / M_PI;
        cout << theta << ": theta" << endl;
        for (int i = 0; i < 100; i++)
        {
            motorCommandArray_[0] += yaw_inc / 100;
            sendCommand();
            ros::spinOnce();
            rate_.sleep();
        }
    }
    else if (req.motor_id == 6)
    {
        double theta;
        this->yawMechanism(theta, req.angle, 0.03435, 0.088, true);
        double yaw_inc = motorDir_[6] * theta * 4096 * 4 * 100 / 2 / M_PI;
        for (int i = 0; i < 100; i++)
        {
            motorCommandArray_[6] += yaw_inc / 100;
            sendCommand();
            ros::spinOnce();
            rate_.sleep();
        }
    }
    else if (req.motor_id >= 20 && req.motor_id < 26)
    {
        motorCommandArray_[req.motor_id] = req.angle;
        sendCommand();
        ros::spinOnce();
        rate_.sleep();
    }
    else
    {
        double inc = motorDir_[req.motor_id] * req.angle * 4096 * 4 * 160 / 2 / M_PI;
        for (int i = 0; i < int(abs(inc)) / 100; i++)
        {
            motorCommandArray_[req.motor_id] += sgn(inc) * 100;
            sendCommand();
            ros::spinOnce();
            rate_.sleep();
        }
    }
}

void GaitManager::absReader(const sensor_msgs::JointState &msg)
{
    for (int i = 0; i < 32; i++)
    {
        absData_[i] = msg.position[i + 1];

        // if (i == 10)
        //     absData_[10] = msg.position[12];
        // else if (i == 11)
        //     absData_[11] = msg.position[11];

        // Lower body collision detection
        if (i < 12 && abs(absData_[i]) > 262144)
        {
            // cout << "Invalid Abs Data, id: " << i << endl;
            continue;
        }
        if (i < 12 && (absData_[i] >= ((absHigh_[i] - homeAbs_[i]) * absDir_[i] - 100) || absData_[i] <= ((absLow_[i] - homeAbs_[i]) * absDir_[i] + 100)))
        {
            collision_ = true;
            // cout << "Error: Collision detected!!, id: " << i << endl;
        }
    }
}

void GaitManager::incReader(const sensor_msgs::JointState &msg)
{
    for (int i = 0; i < 32; i++)
    {
        incData_[i] = msg.position[i + 1];
    }
}

bool GaitManager::absPrinter(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    for (int i = 0; i < 32; i++)
    {
        cout << " Sensor ID: " << i;
        cout << " value: " << absData_[i] << endl;
    }
}

bool GaitManager::ankleHome(bool is_left, int roll_dest, int pitch_dest)
{

    int inner = 5; // roll
    int outer = 4; // pitch
    int ankle_dir = 1;
    if (is_left)
    {
        ankle_dir = -1;
        inner = 11;
        outer = 10;
    }

    ros::Rate rate_(200);
    while (abs(abs(absData_[inner]) - roll_dest) > 100)
    {
        if (abs(absData_[inner]) < 262144)
        {

            if (abs(absData_[inner]) > roll_dest)
            {
                motorCommandArray_[outer] -= (4096 * 4 * 0.01);
                motorCommandArray_[inner] -= (4096 * 4 * 0.01);
            }
            else
            {
                motorCommandArray_[outer] += (4096 * 4 * 0.01);
                motorCommandArray_[inner] += (4096 * 4 * 0.01);
            }
        }
        sendCommand();
        ros::spinOnce();
        rate_.sleep();
    }
    while (abs(abs(absData_[outer]) - pitch_dest) > 100)
    {

        if (abs(absData_[outer]) < 262144)
        {

            if (abs(absData_[outer]) > pitch_dest)
            {
                motorCommandArray_[outer] -= (4096 * 4 * 0.01);
                motorCommandArray_[inner] += (4096 * 4 * 0.01);
            }
            else
            {
                motorCommandArray_[outer] += (4096 * 4 * 0.01);
                motorCommandArray_[inner] -= (4096 * 4 * 0.01);
            }
        }
        sendCommand();
        ros::spinOnce();
        rate_.sleep();
    }
}

bool GaitManager::walk(gait_planner::Trajectory::Request &req,
                        gait_planner::Trajectory::Response &res)
{
    this->emptyCommand();
    int rate = 200;
    ros::Rate rate_(rate);
    double dt = 0.005;
    double COM_height;

    string walk_config_path = ros::package::getPath("gait_planner") + "/config/walk_config.json";
    std::ifstream f(walk_config_path);
    json walk_config = json::parse(f);
    if (req.is_config)
        COM_height = walk_config["COM_height"];
    else
        COM_height = req.COM_height;

    double init_com_pos[3] = {0, 0, 0.71};
    double init_com_orient[3] = {0, 0, 0};
    double final_com_pos[3] = {0, 0, COM_height};
    double final_com_orient[3] = {0, 0, 0};

    double init_lankle_pos[3] = {0, 0.0975, 0};
    double init_lankle_orient[3] = {0, 0, 0};
    double final_lankle_pos[3] = {0, 0.0975, 0};
    double final_lankle_orient[3] = {0, 0, 0};

    double init_rankle_pos[3] = {0, -0.0975, 0};
    double init_rankle_orient[3] = {0, 0, 0};
    double final_rankle_pos[3] = {0, -0.0975, 0};
    double final_rankle_orient[3] = {0, 0, 0};

    // int final_iter = robot->OnlineGeneralTrajGen(dt, 2, final_com_pos, final_com_orient,
    //                                              final_lankle_pos, final_lankle_orient,
    //                                              final_rankle_pos, final_rankle_orient);

    robot->generalTrajGen(dt, 2, init_com_pos, final_com_pos, init_com_orient, final_com_orient,
                          init_lankle_pos, final_lankle_pos, init_lankle_orient, final_lankle_orient,
                          init_rankle_pos, final_rankle_pos, init_rankle_orient, final_rankle_orient);
    // general_traj.request.time = req.t_step;
    // general_traj.request.init_com_pos = {0, 0, req.COM_height};
    // general_traj.request.final_com_pos = {0, 0, req.COM_height};
    // generalTrajectory_.call(general_traj);

    // general_traj.request.init_rankle_pos = {0, -0.0975, 0.05};
    // general_traj.request.final_rankle_pos = {0, -0.0975, 0.0};
    // generalTrajectory_.call(general_traj);

    // general_traj.request.init_rankle_pos = {0, -0.0975, 0.0};
    // general_traj.request.final_rankle_pos = {0, -0.0975, 0.05};
    // generalTrajectory_.call(general_traj);

    robot->trajGen(req.step_count, req.t_step, req.alpha, req.t_double_support, req.COM_height, req.step_length,
                   req.step_width, dt, req.theta, req.ankle_height, req.step_height, 0, req.com_offset, req.is_config);
    

    init_com_pos[2] = COM_height;
    // final_com_pos[1] = 0.07;
    final_com_pos[2] = 0.71;
    robot->generalTrajGen(dt, 2, init_com_pos, final_com_pos, init_com_orient, final_com_orient,
                          init_lankle_pos, final_lankle_pos, init_lankle_orient, final_lankle_orient,
                          init_rankle_pos, final_rankle_pos, init_rankle_orient, final_rankle_orient);

    // int final_iter = robot->OnlineDCMTrajGen(req.step_count, req.t_step, req.alpha, req.t_double_support, req.COM_height, req.step_length,
    //                                          req.step_width, dt, req.theta, req.ankle_height, req.step_height, 0, req.com_offset, req.is_config);

    int iter = 0;
    int final_iter = robot->getTrajSize();
    // int final_iter = req.t_step + 4;

    // // vector<double> right_arm_traj;
    // // vector<double> left_arm_traj;
    // // handMotion(right_arm_traj, left_arm_traj, req.t_step, req.step_count, 0.2, req.dt);

    double jnt_command[12];
    int status;

    while (iter < final_iter)
    {
        // motorCommandArray_[13] = -int(right_arm_traj[iter] * encoderResolution[0] * harmonicRatio[0] / M_PI / 2);
        // motorCommandArray_[16] = int(left_arm_traj[iter] * encoderResolution[0] * harmonicRatio[0] / M_PI / 2);
        // cout << motorCommandArray_[13] << ", " << motorCommandArray_[16] << endl;
        double config[12];
        double jnt_vel[12];
        double left_ft[3] = {-currentLFT_[0], -currentLFT_[2], -currentLFT_[1]};
        double right_ft[3] = {-currentRFT_[0], -currentRFT_[2], -currentRFT_[1]};
        int right_bump[4] = {currentRBump_[0], currentRBump_[1], currentRBump_[2], currentRBump_[3]};
        int left_bump[4] = {currentLBump_[0], currentLBump_[1], currentLBump_[2], currentLBump_[3]};
        double accelerometer[3] = {baseAcc_[0], baseAcc_[1], baseAcc_[2]};
        double gyro[3] = {baseAngVel_[0], baseAngVel_[1], baseAngVel_[2]};
        for (int i = 0; i < 12; i++)
        {
            config[i] = commandConfig_[2][i];
            jnt_vel[i] = (commandConfig_[0][i] - 4 * commandConfig_[1][i] + 3 * commandConfig_[2][i]) / (2 * dt);
        }
        robot->getJointAngs(iter, config, jnt_vel, right_ft, left_ft, right_bump,
                            left_bump, gyro, accelerometer, jnt_command, status);

        // robot->getDCMTrajJointAngs(iter, config, jnt_vel, right_ft, left_ft, right_bump,
        //                            left_bump, gyro, accelerometer, jnt_command, status);

        // robot->getGeneralTrajJointAngs(iter, config, jnt_vel, right_ft, left_ft, right_bump,
        //                                left_bump, gyro, accelerometer, jnt_command, status);
        if (status != 0)
        {
            cout << "Node was shut down due to Ankle Collision!" << endl;
            return false;
        }
        computeLowerLimbJointMotion(jnt_command, iter);
        sendCommand();
        ros::spinOnce();
        rate_.sleep();
        iter++;
    }
    robot->resetTraj();
    res.result = true;
    return true;
}

bool GaitManager::computeLowerLimbJointMotion(double jnt_command[], int iter)
{
    for (int j = 0; j < 12; j++)
    {
        double dif = 0;
        if (this->checkAngle(j, jnt_command[j], dif))
        {
            switch (j)
            {
                double theta;
                double alpha;
                double theta_inner;
                double theta_outer;
                double desired_pitch;
                double desired_roll;
                double inner_inc;
                double outer_inc;

            case 0:
                this->yawMechanism(theta, jnt_command[j], 0.03435, 0.088, false);
                motorCommandArray_[j] = motorDir_[j] * theta * 4096 * 4 * 100 / 2 / M_PI + homeOffset_[j];

                yawMechanismFK(alpha, inc2rad(incData_[0] - homeOffset_[0]) / 100, 0.03435, 0.088, false);
                commandConfig_[2][j] = alpha;
                break;
            case 6:
                this->yawMechanism(theta, jnt_command[j], 0.03435, 0.088, true);
                motorCommandArray_[j] = motorDir_[j] * theta * 4096 * 4 * 100 / 2 / M_PI + homeOffset_[j];
                yawMechanismFK(alpha, inc2rad(motorDir_[j] * (incData_[j] - homeOffset_[j])) / 100, 0.03435, 0.088, true);
                commandConfig_[2][j] = alpha;
                break;
            case 4:
                desired_roll = jnt_command[5];
                desired_pitch = jnt_command[4];
                this->ankleMechanism(theta_inner, theta_outer, desired_pitch, desired_roll, false);
                inner_inc = motorDir_[5] * theta_inner * 4096 * 4 * 100 * 1.5 / 2 / M_PI + homeOffset_[5];
                outer_inc = motorDir_[4] * theta_outer * 4096 * 4 * 100 * 1.5 / 2 / M_PI + homeOffset_[4];
                motorCommandArray_[4] = outer_inc;
                motorCommandArray_[5] = inner_inc;
                commandConfig_[2][j] = absDir_[j] * abs2rad(absData_[j] - homeAbs_[j]);
                break;
            case 5:
                desired_roll = jnt_command[5];
                desired_pitch = jnt_command[4];
                this->ankleMechanism(theta_inner, theta_outer, desired_pitch, desired_roll, false);
                inner_inc = motorDir_[5] * theta_inner * 4096 * 4 * 100 * 1.5 / 2 / M_PI + homeOffset_[5];
                outer_inc = motorDir_[4] * theta_outer * 4096 * 4 * 100 * 1.5 / 2 / M_PI + homeOffset_[4];
                motorCommandArray_[4] = outer_inc;
                motorCommandArray_[5] = inner_inc;
                commandConfig_[2][j] = absDir_[j] * abs2rad(absData_[j] - homeAbs_[j]);
                break;
            case 10:
                desired_roll = jnt_command[11];
                desired_pitch = jnt_command[10];
                this->ankleMechanism(theta_inner, theta_outer, desired_pitch, desired_roll, true);
                inner_inc = motorDir_[11] * theta_inner * 4096 * 4 * 100 * 1.5 / 2 / M_PI + homeOffset_[11];
                outer_inc = motorDir_[10] * theta_outer * 4096 * 4 * 100 * 1.5 / 2 / M_PI + homeOffset_[10];
                motorCommandArray_[10] = outer_inc;
                motorCommandArray_[11] = inner_inc;
                commandConfig_[2][j] = absDir_[j] * abs2rad(absData_[j] - homeAbs_[j]);
                break;
            case 11:
                desired_roll = jnt_command[11];
                desired_pitch = jnt_command[10];
                this->ankleMechanism(theta_inner, theta_outer, desired_pitch, desired_roll, true);
                inner_inc = motorDir_[11] * theta_inner * 4096 * 4 * 100 * 1.5 / 2 / M_PI + homeOffset_[11];
                outer_inc = motorDir_[10] * theta_outer * 4096 * 4 * 100 * 1.5 / 2 / M_PI + homeOffset_[10];
                motorCommandArray_[10] = outer_inc;
                motorCommandArray_[11] = inner_inc;
                commandConfig_[2][j] = absDir_[j] * abs2rad(absData_[j] - homeAbs_[j]);
                break;
            default:
                motorCommandArray_[j] = motorDir_[j] * jnt_command[j] * 4096 * 4 * 160 / 2 / M_PI + homeOffset_[j];
                commandConfig_[2][j] = motorDir_[j] * inc2rad(incData_[j] - homeOffset_[j]) / 160;
                break;
            }

            if (iter == 0)
            {
                commandConfig_[0][j] = 0;
                commandConfig_[1][j] = 0;
            }
            else if (iter == 1)
            {
                commandConfig_[0][j] = 0;
                commandConfig_[1][j] = commandConfig_[2][j];
            }
            else
            {
                commandConfig_[0][j] = commandConfig_[1][j];
                commandConfig_[1][j] = commandConfig_[2][j];
            }

            // commandConfig_[2][j] = absDir_[j] * abs2rad(absData_[j] - homeAbs_[j]);
        }
        else
        {
            cout << "joint " << j << " out of workspace in iteration " << iter << ", angle difference: " << dif << endl;
            return false;
        }
    }
}

void GaitManager::keyboardHandler(const std_msgs::Int32 &msg)
{
    double dt = 0.005;
    double step_width = 0.0;
    double alpha = 0.44;
    double t_double_support = 0.1;
    double t_step = 1.0;
    double step_length = 0.0;
    double COM_height = 0.68;
    double step_count = 2;
    double ankle_height = 0.025;
    double step_height = 0;
    double theta = 0.0;
    double slope = 0.0;
    double offset = 0.01;
    bool is_config = false;

    double init_com_pos[3] = {0, 0, 0.71};
    double init_com_orient[3] = {0, 0, 0};
    double final_com_pos[3] = {0, 0, COM_height};
    double final_com_orient[3] = {0, 0, 0};

    double init_lankle_pos[3] = {0, 0.0975, 0};
    double init_lankle_orient[3] = {0, 0, 0};
    double final_lankle_pos[3] = {0, 0.0975, 0};
    double final_lankle_orient[3] = {0, 0, 0};

    double init_rankle_pos[3] = {0, -0.0975, 0};
    double init_rankle_orient[3] = {0, 0, 0};
    double final_rankle_pos[3] = {0, -0.0975, 0};
    double final_rankle_orient[3] = {0, 0, 0};

    int command = msg.data;

    // if (command == 101) // e
    // {
    //     trajSize_ = robot->changeStep();
    // }

    // if (command == 105) // i
    // {
    //     this->HandInitialCondition();
    //     hasUpperBodyMotion_ = true;
    // }
    // if (command == 106) // j
    // {
    //     this->computeHandLinearMotion(0.15, 4);
    //     hasUpperBodyMotion_ = true;
    // }
    

    if (this->isKeyboardTrajectoryEnabled)
    {
        switch (command)
        {
        case 119: // w: move forward
            step_count = 4;
            step_length = 0.16;
            theta = 0.0;
            robot->trajGen(step_count, t_step, alpha, t_double_support, COM_height, step_length, 
                           step_width, dt, theta, ankle_height, step_height, slope, offset, is_config);
            // trajSize_ = robot->OnlineDCMTrajGen(step_count, t_step, alpha, t_double_support, COM_height, step_length, 
            //                                     step_width, dt, theta, ankle_height, step_height, slope, offset, is_config);

            isKeyboardTrajectoryEnabled = false;
            break;

        case 115: // s: move backward
            step_count = 2;
            step_length = -0.15;
            theta = 0.0;
            robot->trajGen(step_count, t_step, alpha, t_double_support, COM_height, step_length, 
                           step_width, dt, theta, ankle_height, step_height, slope, offset, is_config);
            // trajSize_ = robot->OnlineDCMTrajGen(step_count, t_step, alpha, t_double_support, COM_height, step_length, 
            //                                     step_width, dt, theta, ankle_height, step_height, slope, offset, is_config);
            isKeyboardTrajectoryEnabled = false;
            break;

        case 97: // a: turn left
            step_count = 4;
            step_length = -0.15;
            theta = 0.17;
            robot->trajGen(step_count, t_step, alpha, t_double_support, COM_height, step_length, 
                           step_width, dt, theta, ankle_height, step_height, slope, offset, is_config);
            // trajSize_ = robot->OnlineDCMTrajGen(step_count, t_step, alpha, t_double_support, COM_height, step_length, 
            //                                     step_width, dt, theta, ankle_height, step_height, slope, offset, is_config);
            isKeyboardTrajectoryEnabled = false;
            break;

        case 100: // d: turn right
            step_count = 4;
            step_length = 0.15;
            theta = 0.17;
            robot->trajGen(step_count, t_step, alpha, t_double_support, COM_height, step_length, 
                           step_width, dt, theta, ankle_height, step_height, slope, offset, is_config);
            // trajSize_ = robot->OnlineDCMTrajGen(step_count, t_step, alpha, t_double_support, COM_height, step_length, 
            //                                     step_width, dt, theta, ankle_height, step_height, slope, offset, is_config);
            isKeyboardTrajectoryEnabled = false;
            break;

        case 104: // h: in place turn right
            step_count = 4;  // 14 steps for 90 degrees rotation
            step_length = 0.02;
            theta = 0.10;
            robot->trajGen(step_count, t_step, alpha, t_double_support, COM_height, step_length, 
                           step_width, dt, theta, ankle_height, step_height, slope, offset, is_config);
            isKeyboardTrajectoryEnabled = false;
            break;

        case 103: // g: in place turn left
            step_count = 4;  // 14 steps for 90 degrees rotation
            step_length = -0.02;
            theta = 0.10;
            robot->trajGen(step_count, t_step, alpha, t_double_support, COM_height, step_length, 
                           step_width, dt, theta, ankle_height, step_height, slope, offset, is_config);
            isKeyboardTrajectoryEnabled = false;
            break;

        case 109: // m: height reduction
            step_count = 2;
            step_length = 0.15;
            theta = 0.0;
            robot->trajGen(step_count, t_step, alpha, t_double_support, COM_height, step_length, 
                           step_width, dt, theta, ankle_height, step_height, slope, offset, is_config);

            init_com_pos[2] = 0.68;
            final_com_pos[2] = 0.64;
            robot->generalTrajGen(dt, 2, init_com_pos, final_com_pos, init_com_orient, final_com_orient,
                          init_lankle_pos, final_lankle_pos, init_lankle_orient, final_lankle_orient,
                          init_rankle_pos, final_rankle_pos, init_rankle_orient, final_rankle_orient);

            step_count = 2;
            step_length = 0.15;
            theta = 0.0;
            COM_height = 0.64;
            robot->trajGen(step_count, t_step, alpha, t_double_support, COM_height, step_length, 
                           step_width, dt, theta, ankle_height, step_height, slope, offset, is_config);
            
            init_com_pos[2] = 0.64;
            final_com_pos[2] = 0.68;
            robot->generalTrajGen(dt, 2, init_com_pos, final_com_pos, init_com_orient, final_com_orient,
                          init_lankle_pos, final_lankle_pos, init_lankle_orient, final_lankle_orient,
                          init_rankle_pos, final_rankle_pos, init_rankle_orient, final_rankle_orient);
                          
            COM_height = 0.68;
            isKeyboardTrajectoryEnabled = false;
            break;

        case 117: // u: comming up
            init_com_pos[2] = COM_height;
            final_com_pos[2] = 0.71;
            robot->generalTrajGen(dt, 2, init_com_pos, final_com_pos, init_com_orient, final_com_orient,
                          init_lankle_pos, final_lankle_pos, init_lankle_orient, final_lankle_orient,
                          init_rankle_pos, final_rankle_pos, init_rankle_orient, final_rankle_orient);
            isKeyboardTrajectoryEnabled = false;
            break;

        case 27: // esc: exit
            isKeyboardTrajectoryEnabled = false;
            isWalkingWithKeyboard = false;
            break;

        default:
            break;
        }
    }
}

bool GaitManager::keyboardWalk(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    this->emptyCommand();
    int rate = 200;
    ros::Rate rate_(rate);

    isWalkingWithKeyboard = true;

    double dt = 0.005;
    double init_com_pos[3] = {0, 0, 0.71};
    double init_com_orient[3] = {0, 0, 0};
    double final_com_pos[3] = {0, 0, 0.68};
    double final_com_orient[3] = {0, 0, 0};

    double init_lankle_pos[3] = {0, 0.0975, 0};
    double init_lankle_orient[3] = {0, 0, 0};
    double final_lankle_pos[3] = {0, 0.0975, 0};
    double final_lankle_orient[3] = {0, 0, 0};

    double init_rankle_pos[3] = {0, -0.0975, 0};
    double init_rankle_orient[3] = {0, 0, 0};
    double final_rankle_pos[3] = {0, -0.0975, 0};
    double final_rankle_orient[3] = {0, 0, 0};

    robot->generalTrajGen(dt, 2, init_com_pos, final_com_pos, init_com_orient, final_com_orient,
                          init_lankle_pos, final_lankle_pos, init_lankle_orient, final_lankle_orient,
                          init_rankle_pos, final_rankle_pos, init_rankle_orient, final_rankle_orient);

    double jnt_command[12];
    int status;

    int iter = 0;
    int upper_iter = 0;
    int final_iter;
    // trajSize_ = robot->OnlineGeneralTrajGen(dt, 2, final_com_pos, final_com_orient,
    //                                         final_lankle_pos, final_lankle_orient,
    //                                         final_rankle_pos, final_rankle_orient);
    // ControlState robot_cs = IDLE;

    while (isWalkingWithKeyboard)
    {
        final_iter = robot->getTrajSize();
        
        // if(iter < trajSize_)
        if(iter < final_iter)
        {
            double config[12];
            double jnt_vel[12];
            double left_ft[3] = {-currentLFT_[0], -currentLFT_[2], -currentLFT_[1]};
            double right_ft[3] = {-currentRFT_[0], -currentRFT_[2], -currentRFT_[1]};
            int right_bump[4] = {currentRBump_[0], currentRBump_[1], currentRBump_[2], currentRBump_[3]};
            int left_bump[4] = {currentLBump_[0], currentLBump_[1], currentLBump_[2], currentLBump_[3]};
            double accelerometer[3] = {baseAcc_[0], baseAcc_[1], baseAcc_[2]};
            double gyro[3] = {baseAngVel_[0], baseAngVel_[1], baseAngVel_[2]};

            for (int i = 0; i < 12; i++)
            {
                config[i] = commandConfig_[2][i];
                jnt_vel[i] = (commandConfig_[0][i] - 4 * commandConfig_[1][i] + 3 * commandConfig_[2][i]) / (2 * dt);
            }

            // robot_cs = robot->getCurrentWalkState();
            
            // if(robot_cs == IDLE)
            //     robot->getGeneralTrajJointAngs(iter, config, jnt_vel, right_ft, left_ft, right_bump,
            //                                    left_bump, gyro, accelerometer, jnt_command, status);
            // else
            //     robot->getDCMTrajJointAngs(iter, config, jnt_vel, right_ft, left_ft, right_bump,
            //                                left_bump, gyro, accelerometer, jnt_command, status);

            robot->getJointAngs(iter, config, jnt_vel, right_ft, left_ft, right_bump,
                                left_bump, gyro, accelerometer, jnt_command, status);
            if (status != 0)
            {
                cout << "Node was shut down due to Ankle Collision!" << endl;
                return false;
            }
            computeLowerLimbJointMotion(jnt_command, iter);
            sendCommand();
            iter++;
        }
        // if (iter == trajSize_ - 1)
        if (iter == final_iter - 1)
        {
            robot->resetTraj();
            isKeyboardTrajectoryEnabled = true;
            // trajSize_ = 0;
            iter = 0;
        }

        // if(hasUpperBodyMotion_)
        // {
        //     motorCommandArray_[12]=int(qref_r(0,upper_iter)*encoderResolution[0]*harmonicRatio[0]/M_PI/2); 
        //     motorCommandArray_[13]=-int(qref_r(1,upper_iter)*encoderResolution[0]*harmonicRatio[1]/M_PI/2);
        //     motorCommandArray_[14]=int(qref_r(2,upper_iter)*encoderResolution[1]*harmonicRatio[2]/M_PI/2);
        //     motorCommandArray_[15]=int(qref_r(3,upper_iter)*encoderResolution[1]*harmonicRatio[3]/M_PI/2);
        //     sendCommand();
        //     upper_iter++;
        // }
        // if (upper_iter == qref_r.cols() - 1)
        // {
        //     upper_iter = 0;
        //     hasUpperBodyMotion_ = false;
        // }
        ros::spinOnce();
        rate_.sleep();
    }
    return true;
}

int GaitManager::sgn(double v)
{
    return ((v < 0) ? -1 : (v > 0));
}

double GaitManager::abs2rad(int abs)
{
    double angle = abs / pow(2, 19) * 2 * M_PI;
    return angle;
}

double GaitManager::inc2rad(int inc)
{
    double angle = inc / pow(2, 14) * 2 * M_PI;
    return angle;
}

bool GaitManager::checkAngle(int joint_ID, double angle, double &ang_dif)
{
    if (angle >= abs2rad(absHigh_[joint_ID] - homeAbs_[joint_ID]) * absDir_[joint_ID])
    {
        ang_dif = angle;

        return false;
    }
    else if (angle <= abs2rad(absLow_[joint_ID] - homeAbs_[joint_ID]) * absDir_[joint_ID])
    {
        ang_dif = angle;
        return false;
    }
    else
        return true;
}

void GaitManager::ankleMechanism(double &theta_inner, double &theta_outer,
                                  double teta_p, double teta_r, bool is_left)
{

    Matrix3d pitch_rot;
    pitch_rot << cos(teta_p), 0, sin(teta_p),
        0, 1, 0,
        -sin(teta_p), 0, cos(teta_p);
    Matrix3d roll_rot;
    roll_rot << 1, 0, 0,
        0, cos(teta_r), -sin(teta_r),
        0, sin(teta_r), cos(teta_r);
    Matrix3d rot_mat = pitch_rot * roll_rot;
    Vector3d r3_inner = rot_mat * r30_inner;
    Vector3d r3_outer = rot_mat * r30_outer;
    Vector3d r4_inner(0, -b, -c);
    Vector3d r4_outer(0, b, -c);
    Vector3d r2_inner = r3_inner + r4_inner;
    Vector3d r2_outer = r3_outer + r4_outer;
    Vector3d norm1(r2_inner(0), 0, r2_inner(2));
    Vector3d norm2(r2_outer(0), 0, r2_outer(2));

    if (is_left)
    {
        theta_inner = acos((pow(r2_outer.norm(), 2) + pow(r0, 2) - pow(r1, 2)) / ((norm2.norm() * 2 * r0))) + atan2(r2_outer(0), -r2_outer(2));
        theta_outer = acos((pow(r2_inner.norm(), 2) + pow(r0, 2) - pow(r1, 2)) / ((norm1.norm() * 2 * r0))) + atan2(r2_inner(0), -r2_inner(2));
    }
    else
    {
        theta_inner = acos((pow(r2_inner.norm(), 2) + pow(r0, 2) - pow(r1, 2)) / ((norm1.norm() * 2 * r0))) + atan2(r2_inner(0), -r2_inner(2));
        theta_outer = acos((pow(r2_outer.norm(), 2) + pow(r0, 2) - pow(r1, 2)) / ((norm2.norm() * 2 * r0))) + atan2(r2_outer(0), -r2_outer(2));
    }
    theta_inner = -(theta_inner - M_PI / 2 - 0.160405462422601);
    theta_outer = -(theta_outer - M_PI / 2 - 0.160405462422601);
}

void GaitManager::yawMechanism(double &theta, double alpha1, double R, double B, bool is_left)
{
    double theta_home, alpha_home;
    if (is_left)
    {
        alpha_home = 0.0529;
        theta_home = 0.1868;
    }
    else
    {
        alpha_home = 0.0227;
        theta_home = 0.0809;
    }

    double alpha = alpha_home + alpha1;
    theta = atan2(tan(alpha) * (-(B * pow(tan(alpha), 0.2e1) - sqrt(-B * B * pow(tan(alpha), 0.2e1) + pow(tan(alpha), 0.2e1) * R * R + R * R)) / (pow(tan(alpha), 0.2e1) + 0.1e1) + B) / R, -(B * pow(tan(alpha), 0.2e1) - sqrt(-B * B * pow(tan(alpha), 0.2e1) + pow(tan(alpha), 0.2e1) * R * R + R * R)) / (pow(tan(alpha), 0.2e1) + 0.1e1) / R);
    theta = theta_home - theta;
}

void GaitManager::yawMechanismFK(double &alpha, double theta, double R, double B, bool is_left)
{
    double theta_home, alpha_home;
    if (is_left)
    {
        alpha_home = 0.0529;
        theta_home = 0.1868;
    }
    else
    {
        alpha_home = 0.0227;
        theta_home = 0.0809;
    }
    theta -= theta_home;
    alpha = atan(R * sin(theta) / (B + R * cos(theta)));
    alpha += alpha_home;
}

// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "robot_manager_node");
//     ros::NodeHandle n;
//     GaitManager wt(&n);
//     ros::spin();
//     return 0;
// }