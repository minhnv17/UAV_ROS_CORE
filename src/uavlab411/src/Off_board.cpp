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

    Kp = 1;
    Ki = 0.1;
    stateUav = 0;
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
            navToWaypoint(targetX, targetY, hz);
            pub_navMessage.publish(_navMessage);
            break;
        case 2: // Takeoff mode

            break;
        case 0: // hold mode
            if (currentZ == 0)
                currentZ = 1;

            _setpoint.pose.position.z = currentZ;
            pub_setpoint.publish(_setpoint);
            break;
        default:
            break;
        }

        ros::spinOnce();
        r.sleep();
    }
}

void OffBoard::takeOff(float z)
{
    if (z - 0.05 < _uavpose.pose.position.z < z + 0.05)
    {
        ROS_INFO("TAKEOFF COMPLETE!!");
    }
}

void OffBoard::navToWaypoint(float x, float y, int rate)
{
    float _targetYaw;

    _targetYaw = PidControl(_uavpose.pose.position.x, _uavpose.pose.position.y,
                            x, y, _uavpose.pose.orientation.z, 1.0 / rate);

    _navMessage.yaw_rate = _targetYaw;
    _navMessage.header.stamp = ros::Time::now();
    if (_targetYaw == 0)
    {
        // stateUav = 0;
        ROS_INFO("Switch to hold mode!");
    }
}

bool OffBoard::Navigate(uavlab411::Navigate::Request &req, uavlab411::Navigate::Response &res)
{
    if (req.auto_arm)
    {
        offboardAndArm();
    }
    stateUav = 1;

    _navMessage.type_mask = PositionTarget::IGNORE_PX +
                            PositionTarget::IGNORE_PY +
                            PositionTarget::IGNORE_PZ +
                            PositionTarget::IGNORE_AFX +
                            PositionTarget::IGNORE_AFY +
                            PositionTarget::IGNORE_AFZ +
                            PositionTarget::IGNORE_YAW;

    _navMessage.position.z = req.z;
    _navMessage.velocity.x = 0.5;
    targetX = req.x;
    targetY = req.y;
    currentZ = req.z;
    res.success = true;
    res.message = "navigate with frame id to waypoint";
    return true;
}

void OffBoard::tunning_pid(float _Kp, float _Ki, float _Kd)
{
    Kp = _Kp;
    Ki = _Ki;
    Kd = _Kd;
}

float OffBoard::PidControl(float x_cur, float y_cur, float x_goal, float y_goal, float alpha, float dt)
{

    float e_x;
    float e_y;
    float E_k = 0;
    float alpha_g;
    float E_i = 0;
    float E_d = 0;
    float e_I;
    float e_D;
    float w;
    // Calculate distance from UAV to goal
    e_x = x_goal - x_cur;
    e_y = y_goal - y_cur;

    // If distance tolen < 0.05m -> return
    if (abs(e_x) < 0.05 && abs(e_y) < 0.05)
    {
        ROS_INFO("Navigate to waypoint success!");
        return 0;
    }

    // Calculate alpha error between robot and waypoint

    // Angle from robot to waypoint
    alpha_g = atan2(e_y, e_x);

    E_k = alpha_g - alpha;
    E_k = atan2(sin(E_k), cos(E_k));

    e_I = E_i + E_k * dt;
    e_D = (E_k - E_d) / dt;

    // ROS_INFO("e_I: %f", e_I);
    // ROS_INFO("e_D: %f", e_D);
    // PID Function
    w = Kp * E_k + Ki * e_I + Kd * e_D;
    // w = Kp * E_k;
    ROS_INFO("W: %f", w);
    return w;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ROS_INFO("OFFBOARD NODE INITIAL!");

    OffBoard ob;

    ros::spin();
    return 0;
}