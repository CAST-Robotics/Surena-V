#include "RobotManager.h"

RobotManager::RobotManager(ros::NodeHandle *n)
{
    string config_path = ros::package::getPath("trajectory_planner") + "/config/surenav_config.json";
    robot = new Robot(n, config_path);

    motorDataPub_ = n->advertise<std_msgs::Int32MultiArray>("jointdata/qc", 100);
    absSub_ = n->subscribe("/surena/abs_joint_state", 100, &RobotManager::absReader, this);
    offsetSub_ = n->subscribe("/surena/inc_joint_state", 100, &RobotManager::qcInitial, this);
    incSub_ = n->subscribe("/surena/inc_joint_state", 100, &RobotManager::incReader, this);
    jointCommand_ = n->advertiseService("joint_command", &RobotManager::sendCommand, this);
    absPrinter_ = n->advertiseService("print_absolute", &RobotManager::absPrinter, this);
    walkService_ = n->advertiseService("walk_service", &RobotManager::walk, this);
    keyboardWalkService_ = n->advertiseService("keyboard_walk", &RobotManager::keyboardWalk, this);
    homeService_ = n->advertiseService("home_service", &RobotManager::home, this);
    dummyCommand_ = n->advertiseService("get_data", &RobotManager::dummyCallback, this);
    lFT_ = n->subscribe("/surena/ft_l_state", 100, &RobotManager::ftCallbackLeft, this);
    rFT_ = n->subscribe("/surena/ft_r_state", 100, &RobotManager::ftCallbackRight, this);
    IMUSub_ = n->subscribe("/surena/imu_state", 100, &RobotManager::IMUCallback, this);
    AccSub_ = n->subscribe("/imu/acceleration", 100, &RobotManager::AccCallback, this);
    GyroSub_ = n->subscribe("/imu/angular_velocity", 100, &RobotManager::GyroCallback, this);
    bumpSub_ = n->subscribe("/surena/bump_sensor_state", 100, &RobotManager::bumpCallback, this);
    keyboardCommandSub_ = n->subscribe("/keyboard_command", 1, &RobotManager::keyboardHandler, this);

    move_hand_single_service = n->advertiseService("move_hand_single_srv", &RobotManager::single, this);
    move_hand_both_service = n->advertiseService("move_hand_both_srv", &RobotManager::both, this);

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
    int temp_home_abs[12] = {121482, 141026, 128809, 9439, 134584, 130539, 144401, 145695, 131590, 62400, 130202, 141072};
    int temp_abs_high[12] = {108426, 119010, 89733, 136440, 71608, 102443, 119697, 82527, 168562, 160000, 191978, 111376};
    int temp_abs_low[12] = {145354, 183778, 194153, 7000, 203256, 160491, 150225, 180000, 61510, 61000, 61482, 172752};
    int temp_abs2inc_dir[12] = {1, 1, -1, -1, -1, 1, 1, 1, -1, 1, 1, 1};
    int temp_abs_dir[12] = {-1, -1, -1, 1, -1, -1, -1, -1, 1, 1, 1, -1};
    int temp_motor_dir[12] = {1, 1, 1, -1, -1, 1, 1, 1, -1, 1, 1, -1};
    int temp_bump_order[8] = {3, 0, 1, 2, 6, 5, 4, 7};
    int temp_initial_bump[4] = {0, 0, 0, 0};

    collision_ = false;

    for (int i = 0; i < 23; i++)
    {
        if (i < 20)
            motorCommandArray_[i] = 0;
        else if (i == 20) // head roll
            motorCommandArray_[i] = 145;
        else if (i == 21) // head pitch
            motorCommandArray_[i] = 165;
        else if (i == 22) // head yaw
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

bool RobotManager::sendCommand()
{
    motorCommand_.data.clear();
    for (int i = 0; i < 23; i++)
        motorCommand_.data.push_back(motorCommandArray_[i]);

    motorDataPub_.publish(motorCommand_);
}

bool RobotManager::setPos(int jointID, int dest)
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

bool RobotManager::home(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    // ankleHome(false);
    //  setPos(6, homeAbs_[6]);
    //  setPos(0, homeAbs_[0]);
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

void RobotManager::qcInitial(const sensor_msgs::JointState &msg)
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

void RobotManager::ftCallbackLeft(const geometry_msgs::Wrench &msg)
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

void RobotManager::bumpCallback(const std_msgs::Int32MultiArray &msg)
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

void RobotManager::ftCallbackRight(const geometry_msgs::Wrench &msg)
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

void RobotManager::IMUCallback(const sensor_msgs::Imu &msg)
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

void RobotManager::AccCallback(const geometry_msgs::Vector3Stamped &msg)
{
    baseAcc_[0] = msg.vector.x;
    baseAcc_[1] = msg.vector.y;
    baseAcc_[2] = msg.vector.z;
}

void RobotManager::GyroCallback(const geometry_msgs::Vector3Stamped &msg)
{
    baseAngVel_[0] = msg.vector.x;
    baseAngVel_[1] = msg.vector.y;
    baseAngVel_[2] = msg.vector.z;
}

bool RobotManager::setFTZero()
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

bool RobotManager::setBumpZero()
{
    for (int i = 0; i < 4; i++)
    {
        rBumpOffset_[i] = realRBump_[i];
        lBumpOffset_[i] = realLBump_[i];
    }
}

bool RobotManager::dummyCallback(trajectory_planner::getdata::Request &req,
                                 trajectory_planner::getdata::Response &res)
{
    for (int i = 0; i < req.time * 100; i++)
    {
        emptyCommand();
    }
    return true;
}

bool RobotManager::emptyCommand()
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

bool RobotManager::sendCommand(trajectory_planner::command::Request &req,
                               trajectory_planner::command::Response &res)
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
    else if (req.motor_id == 20 || req.motor_id == 21 || req.motor_id == 22)
    {
        motorCommandArray_[req.motor_id] += req.angle;
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

void RobotManager::absReader(const sensor_msgs::JointState &msg)
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

void RobotManager::incReader(const sensor_msgs::JointState &msg)
{
    for (int i = 0; i < 32; i++)
    {
        incData_[i] = msg.position[i + 1];
    }
}

bool RobotManager::absPrinter(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    for (int i = 0; i < 32; i++)
    {
        cout << " Sensor ID: " << i;
        cout << " value: " << absData_[i] << endl;
    }
}

bool RobotManager::ankleHome(bool is_left, int roll_dest, int pitch_dest)
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

bool RobotManager::walk(trajectory_planner::Trajectory::Request &req,
                        trajectory_planner::Trajectory::Response &res)
{
    this->emptyCommand();
    int rate = 200;
    ros::Rate rate_(rate);

    double init_com_pos[3] = {0, 0, 0.71};
    double init_com_orient[3] = {0, 0, 0};
    double final_com_pos[3] = {0, 0, req.COM_height};
    double final_com_orient[3] = {0, 0, 0};

    double init_lankle_pos[3] = {0, 0.0975, 0};
    double init_lankle_orient[3] = {0, 0, 0};
    double final_lankle_pos[3] = {0, 0.0975, 0};
    double final_lankle_orient[3] = {0, 0, 0};

    double init_rankle_pos[3] = {0, -0.0975, 0};
    double init_rankle_orient[3] = {0, 0, 0};
    double final_rankle_pos[3] = {0, -0.0975, 0};
    double final_rankle_orient[3] = {0, 0, 0};

    robot->generalTrajGen(req.dt, 2, init_com_pos, final_com_pos, init_com_orient, final_com_orient,
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

    robot->trajGen(req.step_count, req.t_step, req.alpha, req.t_double_support, req.COM_height,
                   req.step_length, req.step_width, req.dt, req.theta, req.ankle_height, req.step_height, 0, req.com_offset);
    // if(traj_srv.response.result){

    // init_com_pos[2] = req.COM_height;
    // final_com_pos[2] = req.COM_height;
    // robot->generalTrajGen(req.dt, req.t_step, init_com_pos, final_com_pos, init_com_orient, final_com_orient,
    //                       init_lankle_pos, final_lankle_pos, init_lankle_orient, final_lankle_orient,
    //                       init_rankle_pos, final_rankle_pos, init_rankle_orient, final_rankle_orient);

    init_com_pos[2] = req.COM_height;
    final_com_pos[2] = 0.71;
    robot->generalTrajGen(req.dt, 2, init_com_pos, final_com_pos, init_com_orient, final_com_orient,
                          init_lankle_pos, final_lankle_pos, init_lankle_orient, final_lankle_orient,
                          init_rankle_pos, final_rankle_pos, init_rankle_orient, final_rankle_orient);

    int iter = 0;
    int final_iter = robot->getTrajSize();
    // int final_iter = req.t_step + 4;

    // vector<double> right_arm_traj;
    // vector<double> left_arm_traj;
    // handMotion(right_arm_traj, left_arm_traj, req.t_step, req.step_count, 0.2, req.dt);

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
            jnt_vel[i] = (commandConfig_[0][i] - 4 * commandConfig_[1][i] + 3 * commandConfig_[2][i]) / (2 * req.dt);
        }

        robot->getJointAngs(iter, config, jnt_vel, right_ft, left_ft, right_bump,
                            left_bump, gyro, accelerometer, jnt_command, status);
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

bool RobotManager::computeLowerLimbJointMotion(double jnt_command[], int iter)
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

void RobotManager::handMotion(vector<double> &right_traj, vector<double> &left_traj, 
                              double t_step, int step_count, double max_angle, double dt, bool isLeftFirst)
{
    double delta_q = 2 * max_angle / (t_step / dt);
    int delta_sign = 1;
    if(!isLeftFirst)
        delta_sign = -1;
    
    vector<double> q_r(step_count * t_step / dt, 0);
    vector<double> q_l(step_count * t_step / dt, 0);

    for (int i = 1; i<=step_count; i++){
        for (int j = 0; j<=t_step / dt -1; j++){
            if (i == 1)
            {
                int k = j + 1;
                q_r[k] = q_r[k-1] + delta_sign * delta_q/2;
                q_l[k] = q_l[k-1] - delta_sign * delta_q/2;
            } 
            else if(i == step_count)
            {
                
                int idx = (i-1) * t_step / dt + j;
                q_r[idx] = q_r[idx-1] + delta_sign * pow((-1),(i-1)) * delta_q/2;
                q_l[idx] = q_l[idx-1] - delta_sign * pow((-1),(i-1)) * delta_q/2;
            }
            else 
            {
                int idx = (i-1) * t_step / dt + j;
                q_r[idx] = q_r[idx - 1] + delta_sign * pow((-1),(i-1)) * delta_q;
                q_l[idx] = q_l[idx - 1] - delta_sign * pow((-1),(i-1)) * delta_q;
            }
        }
    }
    right_traj = q_r;
    left_traj = q_l;
}

void RobotManager::keyboardHandler(const std_msgs::Int32 &msg)
{
    double dt = 0.005;
    double step_width = 0.0;
    double alpha = 0.44;
    double t_double_support = 0.1;
    double t_step = 1.0;
    double step_length = 0.15;
    double COM_height = 0.68;
    double step_count = 2;
    double ankle_height = 0.025;
    double step_height = 0;
    double theta = 0.0;
    double slope = 0.0;

    int command = msg.data;

    if (this->isKeyboardTrajectoryEnabled)
    {
        switch (command)
        {
        case 119: // w: move forward
            step_count = 2;
            step_length = 0.15;
            theta = 0.0;
            robot->trajGen(step_count, t_step, alpha, t_double_support, COM_height, step_length, 
                           step_width, dt, theta, ankle_height, step_height, slope, 0);
            isKeyboardTrajectoryEnabled = false;
            break;

        case 115: // s: move backward
            step_count = 2;
            step_length = -0.15;
            theta = 0.0;
            robot->trajGen(step_count, t_step, alpha, t_double_support, COM_height, step_length, 
                           step_width, dt, theta, ankle_height, step_height, slope, 0);
            isKeyboardTrajectoryEnabled = false;
            break;

        case 97: // a: turn left
            step_count = 2;
            step_length = -0.15;
            theta = 0.17;
            robot->trajGen(step_count, t_step, alpha, t_double_support, COM_height, step_length, 
                           step_width, dt, theta, ankle_height, step_height, slope, 0);
            isKeyboardTrajectoryEnabled = false;
            break;

        case 100: // d: turn right
            step_count = 2;
            step_length = 0.15;
            theta = 0.17;
            robot->trajGen(step_count, t_step, alpha, t_double_support, COM_height, step_length, 
                           step_width, dt, theta, ankle_height, step_height, slope, 0);
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

bool RobotManager::keyboardWalk(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
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
    int final_iter;

    while (isWalkingWithKeyboard)
    {
        final_iter = robot->getTrajSize();
        
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
        if (iter == final_iter - 1)
        {
            robot->resetTraj();
            isKeyboardTrajectoryEnabled = true;
            iter = 0;
        }

        ros::spinOnce();
        rate_.sleep();
    }
    return true;
}

int RobotManager::sgn(double v)
{
    return ((v < 0) ? -1 : (v > 0));
}

double RobotManager::abs2rad(int abs)
{
    double angle = abs / pow(2, 19) * 2 * M_PI;
    return angle;
}

double RobotManager::inc2rad(int inc)
{
    double angle = inc / pow(2, 14) * 2 * M_PI;
    return angle;
}

bool RobotManager::checkAngle(int joint_ID, double angle, double &ang_dif)
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

void RobotManager::ankleMechanism(double &theta_inner, double &theta_outer,
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

void RobotManager::yawMechanism(double &theta, double alpha1, double R, double B, bool is_left)
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

void RobotManager::yawMechanismFK(double &alpha, double theta, double R, double B, bool is_left)
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

MatrixXd RobotManager::scenario_target_R(string scenario, int i, VectorXd ee_pos, string ee_ini_pos)
{
    MatrixXd result_r(6, 3);
    r_start_r.resize(3);
    r_middle_r.resize(3);
    r_target_r.resize(3);
    R_target_r.resize(3, 3);
    q_ra.resize(7);
    q_init_r.resize(7);
    if (scenario == "shakeHands")
    {
        r_middle_r << 0.35, -0.1, -0.2; // shakehands
        r_target_r << 0.3, -0.05, -0.35;
        R_target_r = hand_func_R.rot(2, -65 * M_PI / 180, 3);
    }
    else if (scenario == "Respect")
    {
        r_middle_r << 0.3, -0.1, -0.3; // respect
        r_target_r << 0.23, 0.12, -0.3;
        R_target_r = hand_func_R.rot(2, -80 * M_PI / 180, 3) * hand_func_R.rot(1, 60 * M_PI / 180, 3);
    }
    else if (scenario == "byebye")
    {
        r_middle_r << 0.35, -0.2, -0.15; // ByeBye
        r_target_r << 0.3, -0.1, 0.22;
        R_target_r = hand_func_R.rot(3, 90 * M_PI / 180, 3) * hand_func_R.rot(1, -180 * M_PI / 180, 3);
    }
    else if (scenario == "home")
    {
        r_middle_r << 0.3, -0.1, -0.25; // home
        r_target_r << 0.1, -0.05, -0.46;
        R_target_r = hand_func_R.rot(2, -20 * M_PI / 180, 3);
    }
    else if (scenario == "fixed")
    {
        r_middle_r << 0.2, -0.1, -0.35; // fixed
        r_target_r << 0.1, -0.05, -0.46;
        R_target_r = hand_func_R.rot(2, -20 * M_PI / 180, 3);
    }
    if (i == 0)
    {
        if (ee_ini_pos == "init")
        {
            // q_ra<<-12.3*M_PI/180,-5*M_PI/180,38*M_PI/180,-5*M_PI/180,0,0,0; // initial condition
            q_ra << 10 * M_PI / 180, -10 * M_PI / 180, 0, -25 * M_PI / 180, 0, 0, 0; // initial condition
            q_init_r = q_ra;
            // define right_hand objs
            right_hand hand0_r(q_ra, r_target_r, R_target_r, 0, 0);
            r_start_r = hand0_r.r_right_palm;

            d0_r = hand0_r.dist;
            d_r = d0_r;
            d_des_r = hand0_r.d_des;
            theta_r = hand0_r.theta;
            theta_target_r = hand0_r.theta_target;
            sai_r = hand0_r.sai;
            sai_target_r = hand0_r.sai_target;
            phi_r = hand0_r.phi;
            phi_target_r = hand0_r.phi_target;
            hand0_r.HO_FK_right_palm(q_ra);
            cout << r_start_r << endl;
            ;
        }
        else
        {
            r_start_r = next_ini_ee_posR;
            cout << r_start_r << endl;
        }
    }
    else
    {
        r_start_r = ee_pos;
    }
    result_r << r_middle_r(0), r_middle_r(1), r_middle_r(2),
        r_target_r(0), r_target_r(1), r_target_r(2),
        R_target_r(0, 0), R_target_r(0, 1), R_target_r(0, 2),
        R_target_r(1, 0), R_target_r(1, 1), R_target_r(1, 2),
        R_target_r(2, 0), R_target_r(2, 1), R_target_r(2, 2),
        r_start_r(0), r_start_r(1), r_start_r(2);
    return result_r;
}

MatrixXd RobotManager::scenario_target_L(string scenario, int i, VectorXd ee_pos, string ee_ini_pos)
{
    MatrixXd result_l(6, 3);
    r_start_l.resize(3);
    r_middle_l.resize(3);
    r_target_l.resize(3);
    R_target_l.resize(3, 3);
    q_la.resize(7);
    q_init_l.resize(7);
    if (scenario == "shakeHands")
    {
        r_middle_l << 0.35, 0.05, -0.2; // shakehands
        r_target_l << 0.3, 0.05, -0.35;
        R_target_l = hand_func_L.rot(2, -65 * M_PI / 180, 3);
    }
    else if (scenario == "Respect")
    {
        r_middle_l << 0.3, 0.1, -0.3; // respect
        r_target_l << 0.23, -0.12, -0.3;
        R_target_l = hand_func_L.rot(2, -80 * M_PI / 180, 3) * hand_func_L.rot(1, -60 * M_PI / 180, 3);
    }
    else if (scenario == "byebye")
    {
        r_middle_l << 0.35, 0.2, -0.15; // ByeBye
        r_target_l << 0.3, 0.1, 0.22;
        R_target_l = hand_func_L.rot(3, 90 * M_PI / 180, 3) * hand_func_L.rot(2, -180 * M_PI / 180, 3);
    }
    else if (scenario == "home")
    {
        r_middle_l << 0.3, 0.1, -0.25; // home
        r_target_l << 0.1, 0.05, -0.46;
        R_target_l = hand_func_L.rot(2, -20 * M_PI / 180, 3);
    }
    else if (scenario == "fixed")
    {
        r_middle_l << 0.2, 0.1, -0.35; // fixed
        r_target_l << 0.1, 0.05, -0.46;
        R_target_l = hand_func_L.rot(2, -20 * M_PI / 180, 3);
    }
    if (i == 0)
    {
        if (ee_ini_pos == "init")
        {
            // q_la<<-12.3*M_PI/180,5*M_PI/180,-38*M_PI/180,-5*M_PI/180,0,0,0; // initial condition
            q_la << 10 * M_PI / 180, 10 * M_PI / 180, 0, -25 * M_PI / 180, 0, 0, 0; // initial condition
            q_init_l = q_la;
            // define right_hand objs
            left_hand hand0_l(q_la, r_target_l, R_target_l, 0, 0);
            r_start_l = hand0_l.r_left_palm;

            d0_l = hand0_l.dist;
            d_l = d0_l;
            d_des_l = hand0_l.d_des;
            theta_l = hand0_l.theta;
            theta_target_l = hand0_l.theta_target;
            sai_l = hand0_l.sai;
            sai_target_l = hand0_l.sai_target;
            phi_l = hand0_l.phi;
            phi_target_l = hand0_l.phi_target;
            hand0_l.HO_FK_left_palm(q_la);
            cout << r_start_l << endl;
        }
        else
        {
            r_start_l = next_ini_ee_posL;
            cout << r_start_l << endl;
        }
    }
    else
    {
        r_start_l = ee_pos;
    }
    result_l << r_middle_l(0), r_middle_l(1), r_middle_l(2),
        r_target_l(0), r_target_l(1), r_target_l(2),
        R_target_l(0, 0), R_target_l(0, 1), R_target_l(0, 2),
        R_target_l(1, 0), R_target_l(1, 1), R_target_l(1, 2),
        R_target_l(2, 0), R_target_l(2, 1), R_target_l(2, 2),
        r_start_l(0), r_start_l(1), r_start_l(2);
    return result_l;
}

VectorXd RobotManager::reach_target_L(MatrixXd targets, string scenario, int M)
{
    qref_l.resize(7, M);
    P_x_l.resize(1, 3);
    V_x_l.resize(1, 3);
    A_x_l.resize(1, 3);
    P_y_l.resize(1, 3);
    V_y_l.resize(1, 3);
    A_y_l.resize(1, 3);
    P_z_l.resize(1, 3);
    V_z_l.resize(1, 3);
    A_z_l.resize(1, 3);
    P_l.resize(3);
    V_l.resize(3);
    // define time parameters
    int count = 0;
    time_r = count * T;
    MatrixXd t_r(1, 3);
    t_r << 0, 2, 4;

    if (scenario == "byebye" || scenario == "shakeHands")
    {
        total = t_r(2) + 4;
        sum_l += total;
    }
    else
    {
        total = t_r(2);
        sum_l += total;
    }

    // set target values
    r_middle_l << targets(0, 0), targets(0, 1), targets(0, 2); // shakehands
    r_target_l << targets(1, 0), targets(1, 1), targets(1, 2);
    R_target_l << targets(2, 0), targets(2, 1), targets(2, 2),
        targets(3, 0), targets(3, 1), targets(3, 2),
        targets(4, 0), targets(4, 1), targets(4, 2);
    r_start_l << targets(5, 0), targets(5, 1), targets(5, 2);

    left_hand hand_l;

    V_x_l << 0, INFINITY, 0;
    V_y_l << 0, INFINITY, 0;
    V_z_l << 0, INFINITY, 0;
    A_x_l << 0, INFINITY, 0;
    A_y_l << 0, INFINITY, 0;
    A_z_l << 0, INFINITY, 0;

    while (time_r < total)
    {
        if (scenario == "fixed")
        {
        }
        else
        {
            if (time_r < t_r(2))
            {
                // define minJerk elements to calculate end effector velocity
                P_x_l << r_start_l(0), r_middle_l(0), r_target_l(0);
                P_y_l << r_start_l(1), r_middle_l(1), r_target_l(1);
                P_z_l << r_start_l(2), r_middle_l(2), r_target_l(2);

                X_coef_l = coef_generator.Coefficient(t_r, P_x_l, V_x_l, A_x_l);
                Y_coef_l = coef_generator.Coefficient(t_r, P_y_l, V_y_l, A_y_l);
                Z_coef_l = coef_generator.Coefficient(t_r, P_z_l, V_z_l, A_z_l);

                if (time_r < t_r(1) && time_r >= t_r(0))
                {
                    P_l << coef_generator.GetAccVelPos(X_coef_l.row(0), time_r, t_r(0), 5)(0, 0),
                        coef_generator.GetAccVelPos(Y_coef_l.row(0), time_r, t_r(0), 5)(0, 0),
                        coef_generator.GetAccVelPos(Z_coef_l.row(0), time_r, t_r(0), 5)(0, 0);
                    V_l << coef_generator.GetAccVelPos(X_coef_l.row(0), time_r, t_r(0), 5)(0, 1),
                        coef_generator.GetAccVelPos(Y_coef_l.row(0), time_r, t_r(0), 5)(0, 1),
                        coef_generator.GetAccVelPos(Z_coef_l.row(0), time_r, t_r(0), 5)(0, 1);

                    hand_l.update_left_hand(q_la, V_l, r_target_l, R_target_l);
                    r_left_palm = hand_l.r_left_palm;
                    hand_l.doQP(q_la);
                    q_la = hand_l.q_next;
                    d_l = hand_l.dist;
                    theta_l = hand_l.theta;
                    sai_l = hand_l.sai;
                    phi_l = hand_l.phi;
                }

                else if (time_r < t_r(2) && time_r >= t_r(1))
                {
                    P_l << coef_generator.GetAccVelPos(X_coef_l.row(1), time_r, t_r(1), 5)(0, 0),
                        coef_generator.GetAccVelPos(Y_coef_l.row(1), time_r, t_r(1), 5)(0, 0),
                        coef_generator.GetAccVelPos(Z_coef_l.row(1), time_r, t_r(1), 5)(0, 0);
                    V_l << coef_generator.GetAccVelPos(X_coef_l.row(1), time_r, t_r(1), 5)(0, 1),
                        coef_generator.GetAccVelPos(Y_coef_l.row(1), time_r, t_r(1), 5)(0, 1),
                        coef_generator.GetAccVelPos(Z_coef_l.row(1), time_r, t_r(1), 5)(0, 1);

                    hand_l.update_left_hand(q_la, V_l, r_target_l, R_target_l);
                    r_left_palm = hand_l.r_left_palm;
                    hand_l.doQP(q_la);
                    q_la = hand_l.q_next;
                    d_l = hand_l.dist;
                    theta_l = hand_l.theta;
                    sai_l = hand_l.sai;
                    phi_l = hand_l.phi;
                }
            }
            else
            {
                if (scenario == "byebye")
                {
                    q_la(2) = q_la(2) + 0.5 * M_PI / 180 * cos((time_r - t_r(2)) * (2 * M_PI)); // byebye
                }
                else if (scenario == "shakeHands")
                {
                    q_la(3) = q_la(3) - 0.125 * M_PI / 180 * cos((time_r - t_r(2)) * (M_PI)); // shakeHands
                }
            }
        }
        q_end = q_la - q_init_l;
        qref_l.block(0, count + (sum_l - total) / T, 7, 1) = q_end;

        count++;
        time_r = (count)*T;
    };

    r_midpoint_l = r_left_palm;
    return r_midpoint_l;
}

VectorXd RobotManager::reach_target_R(MatrixXd targets, string scenario, int M)
{
    qref_r.resize(7, M);
    r_midpoint_r.resize(3);
    P_x_r.resize(1, 3);
    V_x_r.resize(1, 3);
    A_x_r.resize(1, 3);
    P_y_r.resize(1, 3);
    V_y_r.resize(1, 3);
    A_y_r.resize(1, 3);
    P_z_r.resize(1, 3);
    V_z_r.resize(1, 3);
    A_z_r.resize(1, 3);
    P_r.resize(3);
    V_r.resize(3);
    // define time parameters
    int count = 0;
    time_r = count * T;
    MatrixXd t_r(1, 3);
    t_r << 0, 2, 4;
    double total;
    if (scenario == "byebye" || scenario == "shakeHands")
    {
        total = t_r(2) + 4;
        sum_r += total;
    }
    else
    {
        total = t_r(2);
        sum_r += total;
    }

    // set target values
    r_middle_r << targets(0, 0), targets(0, 1), targets(0, 2); // shakehands
    r_target_r << targets(1, 0), targets(1, 1), targets(1, 2);
    R_target_r << targets(2, 0), targets(2, 1), targets(2, 2),
        targets(3, 0), targets(3, 1), targets(3, 2),
        targets(4, 0), targets(4, 1), targets(4, 2);
    r_start_r << targets(5, 0), targets(5, 1), targets(5, 2);

    right_hand hand_r;

    V_x_r << 0, INFINITY, 0;
    V_y_r << 0, INFINITY, 0;
    V_z_r << 0, INFINITY, 0;
    A_x_r << 0, INFINITY, 0;
    A_y_r << 0, INFINITY, 0;
    A_z_r << 0, INFINITY, 0;

    // cout<<targets<<endl;
    while (time_r < total)
    {
        if (scenario == "fixed")
        {
        }
        else
        {
            if (time_r < t_r(2))
            {
                // define minJerk elements to calculate end effector velocity
                P_x_r << r_start_r(0), r_middle_r(0), r_target_r(0);
                P_y_r << r_start_r(1), r_middle_r(1), r_target_r(1);
                P_z_r << r_start_r(2), r_middle_r(2), r_target_r(2);

                X_coef_r = coef_generator.Coefficient(t_r, P_x_r, V_x_r, A_x_r);
                Y_coef_r = coef_generator.Coefficient(t_r, P_y_r, V_y_r, A_y_r);
                Z_coef_r = coef_generator.Coefficient(t_r, P_z_r, V_z_r, A_z_r);

                if (time_r < t_r(1) && time_r >= t_r(0))
                {
                    P_r << coef_generator.GetAccVelPos(X_coef_r.row(0), time_r, t_r(0), 5)(0, 0),
                        coef_generator.GetAccVelPos(Y_coef_r.row(0), time_r, t_r(0), 5)(0, 0),
                        coef_generator.GetAccVelPos(Z_coef_r.row(0), time_r, t_r(0), 5)(0, 0);
                    V_r << coef_generator.GetAccVelPos(X_coef_r.row(0), time_r, t_r(0), 5)(0, 1),
                        coef_generator.GetAccVelPos(Y_coef_r.row(0), time_r, t_r(0), 5)(0, 1),
                        coef_generator.GetAccVelPos(Z_coef_r.row(0), time_r, t_r(0), 5)(0, 1);

                    hand_r.update_right_hand(q_ra, V_r, r_target_r, R_target_r);
                    r_right_palm = hand_r.r_right_palm;
                    hand_r.doQP(q_ra);
                    q_ra = hand_r.q_next;
                    d_r = hand_r.dist;
                    theta_r = hand_r.theta;
                    sai_r = hand_r.sai;
                    phi_r = hand_r.phi;
                }

                else if (time_r < t_r(2) && time_r >= t_r(1))
                {
                    P_r << coef_generator.GetAccVelPos(X_coef_r.row(1), time_r, t_r(1), 5)(0, 0),
                        coef_generator.GetAccVelPos(Y_coef_r.row(1), time_r, t_r(1), 5)(0, 0),
                        coef_generator.GetAccVelPos(Z_coef_r.row(1), time_r, t_r(1), 5)(0, 0);
                    V_r << coef_generator.GetAccVelPos(X_coef_r.row(1), time_r, t_r(1), 5)(0, 1),
                        coef_generator.GetAccVelPos(Y_coef_r.row(1), time_r, t_r(1), 5)(0, 1),
                        coef_generator.GetAccVelPos(Z_coef_r.row(1), time_r, t_r(1), 5)(0, 1);

                    hand_r.update_right_hand(q_ra, V_r, r_target_r, R_target_r);
                    r_right_palm = hand_r.r_right_palm;
                    hand_r.doQP(q_ra);
                    q_ra = hand_r.q_next;
                    d_r = hand_r.dist;
                    theta_r = hand_r.theta;
                    sai_r = hand_r.sai;
                    phi_r = hand_r.phi;
                }
            }
            else
            {
                if (scenario == "byebye")
                {
                    q_ra(2) = q_ra(2) - 0.5 * M_PI / 180 * cos((time_r - t_r(2)) * (2 * M_PI)); // byebye
                }
                else if (scenario == "shakeHands")
                {
                    q_ra(3) = q_ra(3) - 0.125 * M_PI / 180 * cos((time_r - t_r(2)) * (M_PI)); // shakeHands
                }
            }
        }
        q_end = q_ra - q_init_r;
        qref_r.block(0, count + (sum_r - total) / T, 7, 1) = q_end;

        count++;
        time_r = (count)*T;
    };

    r_midpoint_r = r_right_palm;
    return r_midpoint_r;
}

bool RobotManager::single(trajectory_planner::move_hand_single::Request &req,
                          trajectory_planner::move_hand_single::Response &res)
{
    ros::Rate rate_(rate);
    M = req.t_total / T;
    ee_pos.resize(3, 1);
    // q_motor.resize(29, 0);
    q_gazebo.resize(29, 0);
    int id = 0;
    if (req.mode == "righthand")
    {
        for (int i = 0; i < req.scen_count; i++)
        {
            MatrixXd result_ = scenario_target_R(req.scenario[i], i, ee_pos, req.ee_ini_pos);
            ee_pos = reach_target_R(result_, req.scenario[i], M);
        }
    }
    else if (req.mode == "lefthand")
    {
        for (int i = 0; i < req.scen_count; i++)
        {
            MatrixXd result_ = scenario_target_L(req.scenario[i], i, ee_pos, req.ee_ini_pos);
            ee_pos = reach_target_L(result_, req.scenario[i], M);
            cout << ee_pos << endl;
        }
    }
    // ROS
    while (id < M)
    {
        // gazebo
        if (simulation)
        {
            if (req.mode == "righthand")
            {
                q_gazebo[15] = qref_r(0, id);
                q_gazebo[16] = qref_r(1, id);
                q_gazebo[17] = qref_r(2, id);
                q_gazebo[18] = qref_r(3, id);
                hand_func_R.SendGazebo(q_gazebo);
                cout << q_gazebo[15] << ',' << q_gazebo[16] << ',' << q_gazebo[17] << ',' << q_gazebo[18] << ',' << q_gazebo[19] << ',' << q_gazebo[20] << ',' << q_gazebo[21] << endl;
            }
            else if (req.mode == "lefthand")
            {
                q_gazebo[22] = qref_l(0, id);
                q_gazebo[23] = qref_l(1, id);
                q_gazebo[24] = qref_l(2, id);
                q_gazebo[25] = qref_l(3, id);
                hand_func_L.SendGazebo(q_gazebo);
                cout << q_gazebo[22] << ',' << q_gazebo[23] << ',' << q_gazebo[24] << ',' << q_gazebo[25] << ',' << q_gazebo[26] << ',' << q_gazebo[27] << ',' << q_gazebo[28] << endl;
            }
        }
        else
        {
            if (req.mode == "righthand")
            {
                motorCommandArray_[12] = int(qref_r(0, id) * encoderResolution[0] * harmonicRatio[0] / M_PI / 2);  // be samte jelo
                motorCommandArray_[13] = -int(qref_r(1, id) * encoderResolution[0] * harmonicRatio[1] / M_PI / 2); // be samte birun
                motorCommandArray_[14] = int(qref_r(2, id) * encoderResolution[1] * harmonicRatio[2] / M_PI / 2);  // be samte birun
                motorCommandArray_[15] = -int(qref_r(3, id) * encoderResolution[1] * harmonicRatio[3] / M_PI / 2); // be samte bala
                cout << motorCommandArray_[12] << ',' << motorCommandArray_[13] << ',' << motorCommandArray_[14] << ',' << motorCommandArray_[15] << endl;
            }
            else if (req.mode == "lefthand")
            {
                motorCommandArray_[16] = -int(qref_l(0, id) * encoderResolution[0] * harmonicRatio[0] / M_PI / 2);
                motorCommandArray_[17] = -int(qref_l(1, id) * encoderResolution[0] * harmonicRatio[1] / M_PI / 2);
                motorCommandArray_[18] = int(qref_l(2, id) * encoderResolution[1] * harmonicRatio[2] / M_PI / 2);
                motorCommandArray_[19] = int(qref_l(3, id) * encoderResolution[1] * harmonicRatio[3] / M_PI / 2);
                cout << motorCommandArray_[16] << ',' << motorCommandArray_[17] << ',' << motorCommandArray_[18] << ',' << motorCommandArray_[19] << endl;
            }

            motorCommand_.data.clear();
            for (int i = 0; i < 20; i++)
            {
                motorCommand_.data.push_back(motorCommandArray_[i]);
            }
            motorDataPub_.publish(motorCommand_);
            ros::spinOnce();
            rate_.sleep();
        }
        id++;
    }
    sum_l = 0;
    sum_r = 0;
    cout << ee_pos << endl;
    if (req.mode == "righthand")
    {
        next_ini_ee_posR = ee_pos;
    }
    else if (req.mode == "lefthand")
    {
        next_ini_ee_posL = ee_pos;
    }

    res.ee_fnl_pos = req.scenario[req.scen_count - 1];

    return true;
}

bool RobotManager::both(trajectory_planner::move_hand_both::Request &req,
                        trajectory_planner::move_hand_both::Response &res)
{

    ros::Rate rate_(rate);
    M = req.t_total / T;
    MatrixXd result_(12, 3);
    ee_pos.resize(3, 2);
    // q_motor.resize(29, 0);
    q_gazebo.resize(29, 0);
    int id = 0;

    for (int i = 0; i < req.scenR_count; i++)
    {
        result_.block(0, 0, 6, 3) = scenario_target_R(req.scenarioR[i], i, ee_pos.block(0, 0, 3, 1), req.ee_ini_posR);
        ee_pos.block(0, 0, 3, 1) = reach_target_R(result_.block(0, 0, 6, 3), req.scenarioR[i], M);
        // cout<<ee_pos.block(0,0,3,1)<<endl;
    }
    for (int i = 0; i < req.scenL_count; i++)
    {
        result_.block(6, 0, 6, 3) = scenario_target_L(req.scenarioL[i], i, ee_pos.block(0, 1, 3, 1), req.ee_ini_posL);
        ee_pos.block(0, 1, 3, 1) = reach_target_L(result_.block(6, 0, 6, 3), req.scenarioL[i], M);
        // cout<<ee_pos.block(0,1,3,1)<<endl;
    }
    while (id < M)
    {
        if (simulation)
        {
            q_gazebo[15] = qref_r(0, id);
            q_gazebo[16] = qref_r(1, id);
            q_gazebo[17] = qref_r(2, id);
            q_gazebo[18] = qref_r(3, id);
            q_gazebo[22] = qref_l(0, id);
            q_gazebo[23] = qref_l(1, id);
            q_gazebo[24] = qref_l(2, id);
            q_gazebo[25] = qref_l(3, id);
            hand_func_L.SendGazebo(q_gazebo);
            hand_func_R.SendGazebo(q_gazebo);
        }

        else
        {
            motorCommandArray_[12] = int(qref_r(0, id) * encoderResolution[0] * harmonicRatio[0] / M_PI / 2);  // be samte jelo
            motorCommandArray_[13] = -int(qref_r(1, id) * encoderResolution[0] * harmonicRatio[1] / M_PI / 2); // be samte birun
            motorCommandArray_[14] = int(qref_r(2, id) * encoderResolution[1] * harmonicRatio[2] / M_PI / 2);  // be samte birun
            motorCommandArray_[15] = -int(qref_r(3, id) * encoderResolution[1] * harmonicRatio[3] / M_PI / 2); // be samte bala
            motorCommandArray_[16] = -int(qref_l(0, id) * encoderResolution[0] * harmonicRatio[0] / M_PI / 2);
            motorCommandArray_[17] = -int(qref_l(1, id) * encoderResolution[0] * harmonicRatio[1] / M_PI / 2);
            motorCommandArray_[18] = int(qref_l(2, id) * encoderResolution[1] * harmonicRatio[2] / M_PI / 2);
            motorCommandArray_[19] = int(qref_l(3, id) * encoderResolution[1] * harmonicRatio[3] / M_PI / 2);

            motorCommand_.data.clear();
            for (int i = 0; i < 20; i++)
            {
                motorCommand_.data.push_back(motorCommandArray_[i]);
            }
            motorDataPub_.publish(motorCommand_);
            ros::spinOnce();
            rate_.sleep();
        }

        id++;
    }
    sum_l = 0;
    sum_r = 0;
    cout << ee_pos.block(0, 0, 3, 1) << endl;
    cout << ee_pos.block(0, 1, 3, 1) << endl;
    next_ini_ee_posR = ee_pos.block(0, 0, 3, 1);
    next_ini_ee_posL = ee_pos.block(0, 1, 3, 1);
    res.ee_fnl_posR = req.scenarioR[req.scenR_count - 1];
    res.ee_fnl_posL = req.scenarioL[req.scenL_count - 1];
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "eth_subscriber");
    ros::NodeHandle n;
    RobotManager wt(&n);
    ros::spin();
    return 0;
}