#include "uavlab411/Off_board.h"
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>

OffBoard::OffBoard()
{
    sub_state = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 1, &OffBoard::handleState, this);
    sub_cmd_vel = nh.subscribe<geometry_msgs::Twist>
            ("/cmd_vel", 20, &OffBoard::handleCmdVel, this);
    pub_cmd_vel = nh.advertise<geometry_msgs::Twist>
            ("mavros/setpoint_velocity/cmd_vel_unstamped", 20);

    srv_arming = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    srv_takeoff = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/takeoff");
    srv_set_mode = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    offboardAndArm();

}

void OffBoard::handleState(const mavros_msgs::State::ConstPtr& msg)
{
    cur_state = *msg;
}

void OffBoard::handleCmdVel(const geometry_msgs::Twist::ConstPtr& msg)
{
    // Todo: check last message -> if timeout -> switch to mode hold
    if(cur_state.armed)
    {
        // new_cmd_vel = *msg;
        pub_cmd_vel.publish(*msg);
    }
}

void OffBoard::offboardAndArm()
{
    ros::Rate r(10);
    geometry_msgs::Twist zero_cmd_vel;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        pub_cmd_vel.publish(zero_cmd_vel);
        ros::spinOnce();
        r.sleep();
    }

	mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        if( cur_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( srv_set_mode.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !cur_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( srv_arming.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        pub_cmd_vel.publish(zero_cmd_vel);

        ros::spinOnce();
        r.sleep();
    }
}

void OffBoard::holdPositon()
{
    // Todo: writing hold mode
    // Get local position and publish is a one of most solution
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");

    OffBoard ob;

    ros::spin();
    return 0;
}