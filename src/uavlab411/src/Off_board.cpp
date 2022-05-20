#include "uavlab411/Off_board.h"

OffBoard::OffBoard()
{
    sub_state = nh.subscribe<mavros_msgs::State>("mavros/state", 1, &OffBoard::handleState, this);
    pub_setpoint = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 20);
    sub_uavpose = nh.subscribe<geometry_msgs::PoseStamped>("uavlab411/uavpose", 1, &OffBoard::handlePoses, this);

    srv_arming = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    srv_set_mode = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    navigate_srv = nh.advertiseService("uavnavigate", &OffBoard::Navigate, this);
    
    stream_point();
}

void OffBoard::handleState(const mavros_msgs::State::ConstPtr &msg)
{
    cur_state = *msg;
}

void OffBoard::handlePoses(const geometry_msgs::PoseStamped::ConstPtr& msg)
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

    if (!cur_state.armed) {
		ros::Time start = ros::Time::now();
		ROS_INFO("arming");
		mavros_msgs::CommandBool srv;
		srv.request.value = true;
		if (!srv_arming.call(srv)) {
			throw std::runtime_error("Error calling arming service");
		}

		// wait until armed
		while (ros::ok()) {
			ros::spinOnce();
			if (cur_state.armed) {
				break;
			} else if (ros::Time::now() - start > ros::Duration(5)) {
				throw std::runtime_error("Arming timed out");
			}
			ros::spinOnce();
			r.sleep();
		}
	}
}
void OffBoard::stream_point()
{
    ros::Rate r(5);
    while (ros::ok())
    {
        ros::spinOnce();
        PidControl(_uavpose.pose.position.x, _uavpose.pose.position.y,
            3, 3, 0, 0);
        pub_setpoint.publish(_setpoint);
        
        ros::spinOnce();
        r.sleep();
    }
}

bool OffBoard::Navigate(uavlab411::Navigate::Request &req, uavlab411::Navigate::Response &res)
{
    if(req.auto_arm){
        offboardAndArm();
    }
    _setpoint.pose.position.x = req.x;
    _setpoint.pose.position.y = req.y;
    _setpoint.pose.position.z = req.z;

    res.success = true;
    res.message = "navigate with frame id to waypoint";
    return true;
}

int OffBoard::PidControl(float x_cur, float y_cur, float x_goal, float y_goal, float alpha, float dt)
{
    // Calculate distance from UAV to goal
    float e_x;
    float e_y;
    float Ek;
    float alpha_g;

    e_x = x_goal - x_cur;
    e_y = y_goal - y_cur;

    // If distance tolen < 0.05m -> return
    if(abs(e_x) < 0.05 && abs(e_y) < 0.05)
    {
        ROS_INFO("Navigate to waypoint success!");
        return 0;
    }

    // Calculate alpha error between robot and waypoint

    // Angle from robot to waypoint
    alpha_g = atan2(e_y, e_x);

    Ek = alpha_g - alpha;
    ROS_INFO("alpha_g: %f", alpha_g);
    ROS_INFO("Ek: %f", Ek);
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