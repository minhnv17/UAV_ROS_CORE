#include "uavlab411/Off_board.h"

OffBoard::OffBoard()
{
    sub_state = nh.subscribe<mavros_msgs::State>("mavros/state", 1, &OffBoard::handleState, this);
    sub_uavpose = nh.subscribe<geometry_msgs::PoseStamped>("uavlab411/uavpose", 1, &OffBoard::handlePoses, this);

    pub_setpoint = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 20);
    pub_navMessage = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 20);

    srv_arming = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    srv_set_mode = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    navigate_srv = nh.advertiseService("uavnavigate", &OffBoard::Navigate, this);
    pid_tuning_srv = nh.advertiseService("uav_pid_tuning", &OffBoard::TuningPID, this);

    // Default PID
    Kp_yaw = 1.2;
    Ki_yaw = 0.2;
    Kd_yaw = 0;

    Kp_vx = 0.2;
    Ki_vx = 0;
    Kd_vx = 0;
    // Default state hold mode
    stateUav = 2;
    stream_point();
}

void OffBoard::handleState(const mavros_msgs::State::ConstPtr &msg)
{
    cur_state = *msg;
}

void OffBoard::handlePoses(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    _uavpose = *msg;
}

void OffBoard::offboardAndArm()
{
    ros::Rate r(10);

    if (cur_state.mode != "OFFBOARD")
    {
        auto start = ros::Time::now();
        ROS_INFO("switch to OFFBOARD");
        static mavros_msgs::SetMode sm;
        sm.request.custom_mode = "OFFBOARD";

        if (!srv_set_mode.call(sm))
            throw std::runtime_error("Error calling set_mode service");

        // wait for OFFBOARD mode
        while (ros::ok())
        {
            ros::spinOnce();
            if (cur_state.mode == "OFFBOARD")
            {
                break;
            }
            else if (ros::Time::now() - start > ros::Duration(3))
            {
                throw std::runtime_error("Offboard timeout!");
            }
            ros::spinOnce();
            r.sleep();
        }
    }

    if (!cur_state.armed)
    {
        ros::Time start = ros::Time::now();
        ROS_INFO("arming");
        mavros_msgs::CommandBool srv;
        srv.request.value = true;
        if (!srv_arming.call(srv))
        {
            throw std::runtime_error("Error calling arming service");
        }

        // wait until armed
        while (ros::ok())
        {
            ros::spinOnce();
            if (cur_state.armed)
            {
                break;
            }
            else if (ros::Time::now() - start > ros::Duration(5))
            {
                throw std::runtime_error("Arming timed out");
            }
            ros::spinOnce();
            r.sleep();
        }
    }
}

void OffBoard::stream_point()
{
    int hz = 40;
    ros::Rate r(hz);
    _navMessage.coordinate_frame = PositionTarget::FRAME_BODY_NED;
    _navMessage.header.seq = 0;
    while (ros::ok())
    {
        ros::spinOnce();
        _navMessage.header.seq++;
        switch (stateUav)
        {
        case 1: // navigate to waypoint mode
            navToWaypoint(targetX, targetY, targetZ, hz);
            pub_navMessage.publish(_navMessage);
            break;
        case 2: // Hold mode
            holdMode();
            break;
        case 0: // Takeoff mode
            if (_uavpose.pose.position.z > _setpoint.pose.position.z - 0.1)
            {
                stateUav = 2; // switch to hold mode
            }
            pub_setpoint.publish(_setpoint);
            break;
        default:
            break;
        }

        ros::spinOnce();
        r.sleep();
    }
}
void OffBoard::holdMode()
{
    ROS_INFO("HOLD MODE!");
    _holdMessage.type_mask = PositionTarget::IGNORE_YAW +
                             PositionTarget::IGNORE_YAW_RATE;
    _holdMessage.coordinate_frame = PositionTarget::FRAME_BODY_NED;
    _holdMessage.position.z = targetZ;
    pub_navMessage.publish(_holdMessage);
}

void OffBoard::navToWaypoint(float x, float y, float z, int rate)
{
    float _targetYaw;
    float _targetVx;
    float _targetVz;

    _targetYaw = PidControl_yaw(_uavpose.pose.position.x, _uavpose.pose.position.y,
                                x, y, _uavpose.pose.orientation.z, 1.0 / rate);
    _targetVx = PidControl_vx(_uavpose.pose.position.x, _uavpose.pose.position.y,
                              x, y, _uavpose.pose.orientation.z, 1.0 / rate);
    _targetVz = Control_vz(_uavpose.pose.position.z, z);
    _navMessage.yaw_rate = _targetYaw;
    _navMessage.velocity.x = _targetVx;
    _navMessage.velocity.z = _targetVz;
    ROS_INFO("Vz= %f", _targetVz);
    _navMessage.header.stamp = ros::Time::now();
    if (_targetYaw == 0)
    {
        stateUav = 2;
        ROS_INFO("Switch to HOLD MODE!");
    }
}

bool OffBoard::Navigate(uavlab411::Navigate::Request &req, uavlab411::Navigate::Response &res)
{
    if (req.auto_arm)
    {
        stateUav = 0;
        _setpoint.pose.position.z = req.z;
        offboardAndArm();
        res.message = "TAKE OFF MODE!";
    }
    else
    {
        Ei_yaw = 0;
        Ei_vx = 0;
        stateUav = 1;
        _navMessage.type_mask = PositionTarget::IGNORE_PX +
                                PositionTarget::IGNORE_PY +
                                PositionTarget::IGNORE_PZ +
                                PositionTarget::IGNORE_AFX +
                                PositionTarget::IGNORE_AFY +
                                PositionTarget::IGNORE_AFZ +
                                PositionTarget::IGNORE_YAW;

        // _navMessage.position.z = req.z;
        targetX = req.x;
        targetY = req.y;
        targetZ = req.z;
        res.success = true;
        res.message = "NAVIGATE TO WAYPOINT!";
    }

    return true;
}

bool OffBoard::TuningPID(uavlab411::PidTuning::Request &req, uavlab411::PidTuning::Response &res)
{
    switch (req.type)
    {
    case 0: // Tuning yawrate PID
        // Response last PID Param
        res.Kp = Kp_yaw;
        res.Ki = Ki_yaw;
        res.Kd = Kd_yaw;
        // New PID param
        Kp_yaw = req.Kp;
        Ki_yaw = req.Ki;
        Kd_yaw = req.Kd;

        res.success = true;
        break;
    case 1: // Tuning velocity PID
        // Response last PID Param
        res.Kp = Kp_vx;
        res.Ki = Ki_vx;
        res.Kd = Kd_vx;
        // New PID param
        Kp_vx = req.Kp;
        Ki_vx = req.Ki;
        Kd_vx = req.Kd;

        res.success = true;
        break;
    default:
        res.success = false;
        return false;
        break;
    }
    return true;
}

float OffBoard::PidControl_yaw(float x_cur, float y_cur, float x_goal, float y_goal, float alpha, float dt)
{

    float e_x;
    float e_y;
    float Error_pre = Error_yaw;
    float alpha_g;
    float Ed_yaw;
    float w;
    // Calculate distance from UAV to goal
    e_x = x_goal - x_cur;
    e_y = y_goal - y_cur;

    // If distance tolen < 0.05m -> return
    if (abs(e_x) < 0.1 && abs(e_y) < 0.1)
    {
        ROS_INFO("Navigate to waypoint success!");
        return 0;
    }

    // Calculate alpha error between robot and waypoint

    // Angle from robot to waypoint
    alpha_g = atan2(e_y, e_x);

    Error_yaw = alpha_g - alpha;
    Error_yaw = atan2(sin(Error_yaw), cos(Error_yaw));

    Ei_yaw += Error_yaw * dt;
    Ed_yaw = (Error_yaw - Error_pre) / dt;

    // PID Function
    w = Kp_yaw * Error_yaw + Ki_yaw * Ei_yaw + Kd_yaw * Ed_yaw;
    w = w > 2 ? 2 : w < -2 ? -2
                           : w;

    ROS_INFO("W: %f", w);
    return w;
}

float OffBoard::PidControl_vx(float x_cur, float y_cur, float x_goal, float y_goal, float alpha, float dt)
{

    float e_x;
    float e_y;
    float Error_pre = Error_vx;
    float alpha_g;
    float Ed_vx;
    float w;
    // Calculate distance from UAV to goal
    e_x = x_goal - x_cur;
    e_y = y_goal - y_cur;

    Error_vx = sqrt(e_x * e_x + e_y * e_y);

    Ei_vx += Error_vx * dt;
    Ed_vx = (Error_vx - Error_pre) / dt;

    // PID Function
    w = Kp_vx * Error_vx + Ki_vx * Ei_vx + Kd_vx * Ed_vx;
    w = w > 0.5 ? 0.5 : w < 0.2 ? 0.2
                                : w;
    ROS_INFO("Vx: %f", w);
    return w;
}

float OffBoard::Control_vz(float z_cur, float z_goal)
{
    float error_z = z_goal - z_cur;
    if (abs(error_z) > 0.2)
        return error_z / 2;
    else
        return 0;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ROS_INFO("OFFBOARD NODE INITIAL!");

    OffBoard ob;

    ros::spin();
    return 0;
}