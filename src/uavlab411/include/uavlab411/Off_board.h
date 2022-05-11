#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>


#define TIMEOUT(msg, timeout) (msg.header.stamp.isZero() || (ros::Time::now() - msg.header.stamp > timeout) )

class OffBoard
{
    public:
        OffBoard();

    private:
        ros::NodeHandle nh;
        // Publisher
        ros::Publisher pub_cmd_vel;

        // Subscriber
        ros::Subscriber sub_state;
        ros::Subscriber sub_cmd_vel;

        // Service client
        ros::ServiceClient srv_arming, srv_set_mode, srv_takeoff;

        // Function handle
        void handleState(const mavros_msgs::State::ConstPtr& msg);
        void handleCmdVel(const geometry_msgs::Twist::ConstPtr& msg);

        // Main function
        void offboardAndArm();
        void holdPositon();
        void servive();

        // Variable
        mavros_msgs::State cur_state;
        geometry_msgs::Twist new_cmd_vel;
};