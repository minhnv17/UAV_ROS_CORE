#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <math.h>

#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/PositionTarget.h>
#include <uavlab411/Navigate.h>

using namespace mavros_msgs;

#define TIMEOUT(msg, timeout) (msg.header.stamp.isZero() || (ros::Time::now() - msg.header.stamp > timeout) )

class OffBoard
{
    public:
        OffBoard();
        void tunning_pid(float Kp, float Ki, float Kd);
    private:
        ros::NodeHandle nh;
        // Publisher
        ros::Publisher pub_setpoint, pub_navMessage;
        // Subscriber
        ros::Subscriber sub_state;
        ros::Subscriber sub_uavpose;

        // Service
        ros::ServiceClient srv_arming, srv_set_mode;
        ros::ServiceServer navigate_srv;

        // Function handle
        void handleState(const mavros_msgs::State::ConstPtr& msg);
        void handlePoses(const geometry_msgs::PoseStamped::ConstPtr& msg);

        // Main function
        void offboardAndArm();
        void stream_point();
        void navToWaypoint(float x, float y, int rate);
        void holdMode();

        // Service func
        bool Navigate(uavlab411::Navigate::Request &req, uavlab411::Navigate::Response &res);
        // Variable
        mavros_msgs::State cur_state;
        mavros_msgs::PositionTarget _navMessage, _holdMessage;
        geometry_msgs::PoseStamped _setpoint;
        geometry_msgs::PoseStamped _uavpose;
        geometry_msgs::Twist _cmdvel_msg;

        // PID Controller parameter
        float Kp = 0;
        float Kd = 0;
        float Ki = 0;

        float E_i = 0;
        float E_d = 0;
        int stateUav;

        float targetX, targetY, currentZ;
        float PidControl(float x_cur, float y_cur, float x_goal, float y_goal, float alpha, float dt);
};