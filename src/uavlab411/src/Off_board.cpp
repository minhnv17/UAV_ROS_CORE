#include "uavlab411/Off_board.h"

OffBoard::OffBoard()
{
    sub_state = nh.subscribe<mavros_msgs::State>("mavros/state", 1, &OffBoard::handleState, this);
    sub_uavpose = nh.subscribe<geometry_msgs::PoseStamped>("uavlab411/uavpose", 1, &OffBoard::handlePoses, this);
    sub_local_position = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 1, &OffBoard::handleLocalPosition, this);

    pub_setpoint = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 20);
    pub_navMessage = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 20);

    srv_arming = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    srv_set_mode = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    navigate_srv = nh.advertiseService("uavlab411/navigate", &OffBoard::Navigate, this);
    pid_tuning_srv = nh.advertiseService("uavlab411/pid_tuning", &OffBoard::TuningPID, this);
    takeoff_srv = nh.advertiseService("uavlab411/takeoff", &OffBoard::TakeoffSrv, this);
    land_srv = nh.advertiseService("uavlab411/land", &OffBoard::Land, this);

    _uavpose_timemout = ros::Duration(nh.param("uavpose_timeout", 2.0));
    _uavpose_local_position_timeout = ros::Duration(nh.param("rangefinder_timeout", 3.0));
    _land_timeout = ros::Duration(nh.param("land_timeout", 3.0));
    // Default PID
    Kp_yaw = 1;
    Ki_yaw = 0.2;
    Kd_yaw = 0;
    targetZ = 0;
    Kp_vx = 0.2;
    Ki_vx = 0;
    Kd_vx = 0;
    // Default state hold mode
    _holdMessage.type_mask = PositionTarget::IGNORE_YAW +
                             PositionTarget::IGNORE_YAW_RATE;
    _holdMessage.coordinate_frame = PositionTarget::FRAME_LOCAL_NED;
    _curMode = Hold;
    // Initial value
    Vmax = 0.6;
    update_frequency = 35.0;
    _navMessage.coordinate_frame = PositionTarget::FRAME_BODY_NED;
    setpoint_timer = nh.createTimer(ros::Duration(1.0 / update_frequency),
                                    boost::bind(&OffBoard::publish_point, this),
                                    false, false);
}

void OffBoard::handleState(const mavros_msgs::State::ConstPtr &msg)
{
    cur_state = *msg;
}

void OffBoard::handlePoses(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    _uavpose = *msg;
}

void OffBoard::handleLocalPosition(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    _uavpose_local_position = *msg;
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
            else if (ros::Time::now() - start > ros::Duration(5))
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

void OffBoard::publish_point()
{
    switch (_curMode)
    {
    case NavYaw: // navigate to waypoint with yawrate mode
        if (!TIMEOUT(_uavpose, _uavpose_timemout))
        {
            navToWaypoint(targetX, targetY, targetZ, update_frequency);
            _navMessage.header.stamp = ros::Time::now();
            pub_navMessage.publish(_navMessage);
        }
        else
        {
            getCurrentPosition();
            _curMode = Hold;
            ROS_INFO("Switch to HOLD MODE!");
        }
        break;
    case NavNoYaw: // navigate to waypoint without yawrate mode
        if (!TIMEOUT(_uavpose, _uavpose_timemout))
        {
            navToWayPointV2(targetX, targetY, targetZ, update_frequency);
            _navMessage.header.stamp = ros::Time::now();
            pub_navMessage.publish(_navMessage);
        }
        else
        {
            getCurrentPosition();
            _curMode = Hold;
            ROS_INFO("Switch to HOLD MODE!");
        }
        break;
    case Hold: // Hold mode
        holdMode();
        break;
    case Takeoff: // Takeoff mode
        if (!TIMEOUT(_uavpose_local_position, _uavpose_local_position_timeout))
        {
            if (_uavpose.pose.position.z > _setpoint.pose.position.z - 0.1 &&
                _uavpose.pose.position.z < _setpoint.pose.position.z + 0.1)
            {
                getCurrentPosition();
                _curMode = Hold;
                ROS_INFO("Switch to HOLD MODE!");
            }
            pub_setpoint.publish(_setpoint);
        }
        else
        {
            getCurrentPosition();
            _curMode = Hold;
            ROS_INFO("Switch to HOLD MODE!");
        }
        break;
    default:
        break;
    }
}

void OffBoard::holdMode()
{
    pub_navMessage.publish(_holdMessage);
}
void OffBoard::getCurrentPosition()
{
    _holdMessage.header.stamp = ros::Time::now();
    _holdMessage.position = _uavpose_local_position.pose.position;
}
void OffBoard::navToWaypoint(float x, float y, float z, int rate)
{
    float _targetYaw;
    float _targetVx;
    float _targetVz;

    _targetYaw = PidControl_yaw(_uavpose.pose.position.x, _uavpose.pose.position.y,
                                x, y, _uavpose.pose.orientation.z, 1.0 / rate);
    _targetVx = PidControl_vx(_uavpose.pose.position.x, _uavpose.pose.position.y,
                              x, y, 1.0 / rate);
    // _targetVz = Control_vz(_uavpose.pose.position.z, z);
    _navMessage.yaw_rate = _targetYaw * 0.8;
    _navMessage.velocity.x = _targetVx;
    // _navMessage.velocity.z = _targetVz;
    _navMessage.position.z = z_map + z;

    if (_targetYaw == 0)
    {
        _curMode = Hold;
        ROS_INFO("Switch to HOLD MODE!");
    }
}

void OffBoard::navToWayPointV2(float x, float y, float z, int rate)
{
    float _targetV, Vx, Vy, Vz, e_x, e_y, alpha_g, yaw;
    _targetV = PidControl_vx(_uavpose.pose.position.x, _uavpose.pose.position.y,
                             x, y, 1.0 / rate);

    e_x = x - _uavpose.pose.position.x;
    e_y = y - _uavpose.pose.position.y;

    alpha_g = atan2(e_y, e_x);
    yaw = alpha_g - _uavpose.pose.orientation.z;
    yaw = atan2(sin(yaw), cos(yaw));

    Vx = cos(yaw) * _targetV;
    Vy = sin(yaw) * _targetV;
    // Vz = Control_vz(_uavpose.pose.position.z, z);
    // Change nav message
    _navMessage.header.stamp = ros::Time::now();
    _navMessage.velocity.x = Vx;
    _navMessage.velocity.y = Vy;
    _navMessage.position.z = z_map+z;
    if (abs(e_x) < 0.1 && abs(e_y) < 0.1)
    {
        getCurrentPosition();
        _holdMessage.position.z = z_map+z;
        _curMode = Hold;
        ROS_INFO("Switch to HOLD MODE!");
    }
}

bool OffBoard::Navigate(uavlab411::Navigate::Request &req, uavlab411::Navigate::Response &res)
{
    if (!TIMEOUT(_uavpose, _uavpose_timemout) && !TIMEOUT(_uavpose_local_position, _uavpose_local_position_timeout) && checkState())
    {
        switch (req.nav_mode)
        {
        case NavYaw:
            ROS_INFO("NAV TO WP WITH YAW CHANGED");
            Ei_yaw = 0;
            Ei_vx = 0;
            _curMode = NavYaw;
            _navMessage.type_mask = PositionTarget::IGNORE_PX +
                                    PositionTarget::IGNORE_PY +
                                    // PositionTarget::IGNORE_VZ +
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
            res.z_map = z_map;
            return true;
            break;

        case NavNoYaw:
            ROS_INFO("NAV TO WP WITHOUT YAW CHANGED");
            Ei_vx = 0;
            _curMode = NavNoYaw;
            _navMessage.type_mask = PositionTarget::IGNORE_PX +
                                    PositionTarget::IGNORE_PY +
                                    // PositionTarget::IGNORE_VZ +
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
            res.z_map = z_map;
            return true;
            break;

        default:
            res.message = "DONT KNOW THIS NAV MODE!";
            res.success = false;
            res.z_map = z_map;
            return true;
            break;
        }
    }
    else
    {
        res.message = "NO LOCAL POSITION OR/AND NO RANGE FINDER OR/AND NO OFFB MODE - NO ARM, PLS CHECK!";
        res.success = false;
        return true;
    }
}

bool OffBoard::TakeoffSrv(uavlab411::Takeoff::Request &req, uavlab411::Takeoff::Response &res)
{
    ROS_INFO("TAKE OFF MODE");
    publish_point();
    setpoint_timer.start();
    if (!TIMEOUT(_uavpose_local_position, _uavpose_local_position_timeout))
    {
        offboardAndArm();
        if (checkState())
        {           
            z_map = _uavpose_local_position.pose.position.z;
            _setpoint.pose.position.z = req.z + z_map;
            _setpoint.pose.position.x = _uavpose_local_position.pose.position.x;
            _setpoint.pose.position.y = _uavpose_local_position.pose.position.y;
            // targetZ = req.z + _uavpose.pose.position.z;
            _curMode = Takeoff;
            res.success = true;
            res.message = "TAKE OFF MODE!";
            return true;
        }
    }
    else
    {
        res.success = false;
        res.message = "NO LOCAL POSITION, CANT TAKEOFF!";
        return true;
    }
    return true;
}

bool OffBoard::Land(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    try
    {

        static mavros_msgs::SetMode sm;
        sm.request.custom_mode = "AUTO.LAND";

        if (!srv_set_mode.call(sm))
            throw std::runtime_error("Can't call set_mode service");

        if (!sm.response.mode_sent)
            throw std::runtime_error("Can't send set_mode request");

        static ros::Rate r(10);
        auto start = ros::Time::now();
        while (ros::ok())
        {
            if (cur_state.mode == "AUTO.LAND")
            {
                res.success = true;
                return true;
            }
            if (ros::Time::now() - start > _land_timeout)
                throw std::runtime_error("Land request timed out");

            ros::spinOnce();
            r.sleep();
        }
    }
    catch (const std::exception &e)
    {
        res.message = e.what();
        ROS_INFO("%s", e.what());
        return true;
    }
    return false;
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

    return w;
}

float OffBoard::PidControl_vx(float x_cur, float y_cur, float x_goal, float y_goal, float dt)
{

    float e_x;
    float e_y;
    float Error_pre = Error_vx;
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

bool OffBoard::checkState()
{
    if (cur_state.mode != "OFFBOARD")
    {
        setpoint_timer.stop();
        throw std::runtime_error("Copter is not in OFFBOARD mode, use auto_arm?");
        return false;
    }
    else if (!cur_state.armed)
    {
        setpoint_timer.stop();
        throw std::runtime_error("Copter is not armed, use auto_arm?");
        return false;
    }
    else
    {
        return true;
    }
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ROS_INFO("OFFBOARD NODE INITIAL!");

    OffBoard ob;

    ros::spin();
    return 0;
}
