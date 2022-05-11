#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>

#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>

#include <uavlab411/Navigate.h>

#define TIMEOUT(msg, timeout) (msg.header.stamp.isZero() || (ros::Time::now() - msg.header.stamp > timeout) )

class OffBoard
{
    public:
        OffBoard();

    private:
        ros::NodeHandle nh;
        // Publisher
        ros::Publisher pub_setpoint;

        // Subscriber
        ros::Subscriber sub_state;

        // Service
        ros::ServiceClient srv_arming, srv_set_mode;
        ros::ServiceServer navigate_srv;

        // Function handle
        void handleState(const mavros_msgs::State::ConstPtr& msg);
        void handleCmdVel(const geometry_msgs::Twist::ConstPtr& msg);

        // Main function
        void offboardAndArm();
        void stream_point();

        // Service func
        bool Navigate(uavlab411::Navigate::Request &req, uavlab411::Navigate::Response &res);
        // Variable
        mavros_msgs::State cur_state;
        geometry_msgs::PoseStamped _setpoint;
};