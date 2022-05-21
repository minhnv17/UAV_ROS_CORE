#include "uavlab411/Off_board.h"

OffBoard::OffBoard()
{
    sub_state = nh.subscribe<mavros_msgs::State>("mavros/state", 1, &OffBoard::handleState, this);
    pub_setpoint = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 20);
    pub_yawrate = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local",20);
    sub_uavpose = nh.subscribe<geometry_msgs::PoseStamped>("uavlab411/uavpose", 1, &OffBoard::handlePoses, this);

    srv_arming = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    srv_set_mode = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    navigate_srv = nh.advertiseService("uavnavigate", &OffBoard::Navigate, this);

    Kp = 2;
    _yawrate.type_mask = mavros_msgs::PositionTarget::IGNORE_VX +
                        mavros_msgs::PositionTarget::IGNORE_VY +
                        mavros_msgs::PositionTarget::IGNORE_VZ +
                        mavros_msgs::PositionTarget::IGNORE_AFX +
                        mavros_msgs::PositionTarget::IGNORE_AFY +
                        mavros_msgs::PositionTarget::IGNORE_AFZ +
                        mavros_msgs::PositionTarget::IGNORE_YAW 
                        ;
    _yawrate.header.seq = 1;
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
    int hz = 20;
    float _yaw_rate;
    ros::Rate r(hz);
    while (ros::ok())
    {
        ros::spinOnce();
        _yaw_rate = PidControl(_uavpose.pose.position.x, _uavpose.pose.position.y,
                   3, 3, _uavpose.pose.orientation.z, 1.0/hz);
        
        // _setpoint.pose.orientation.z
        // pub_setpoint.publish(_setpoint);
        _yawrate.yaw_rate = _yaw_rate;
        _yawrate.header.frame_id = "map";
        _yawrate.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        _yawrate.header.stamp = ros::Time::now();
        _yawrate.header.seq++;
        pub_yawrate.publish(_yawrate);
        ros::spinOnce();
        r.sleep();
    }
}

bool OffBoard::Navigate(uavlab411::Navigate::Request &req, uavlab411::Navigate::Response &res)
{
    if (req.auto_arm)
    {
        offboardAndArm();
    }
    _setpoint.pose.position.x = req.x;
    _setpoint.pose.position.y = req.y;
    _setpoint.pose.position.z = req.z;

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