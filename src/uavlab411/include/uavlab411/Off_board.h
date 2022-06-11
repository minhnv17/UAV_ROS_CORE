#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Range.h>
#include <math.h>

#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/PositionTarget.h>

#include <uavlab411/Navigate.h>
#include <uavlab411/PidTuning.h>
#include <uavlab411/Takeoff.h>

using namespace mavros_msgs;

#define TIMEOUT(msg, timeout) (msg.header.stamp.isZero() || (ros::Time::now() - msg.header.stamp > timeout))

enum Mode
{
    Takeoff = 0,
    Hold = 2,
    NavYaw = 1,
    NavNoYaw = 3
};

class OffBoard
{
public:
    OffBoard();
private:
    ros::NodeHandle nh;
    // Publisher
    ros::Publisher pub_setpoint, pub_navMessage;
    // Subscriber
    ros::Subscriber sub_state;
    ros::Subscriber sub_uavpose, sub_rangefinder;

    // Service
    ros::ServiceClient srv_arming, srv_set_mode;
    ros::ServiceServer navigate_srv, pid_tuning_srv, takeoff_srv;

    ros::Timer setpoint_timer;
    // Function handle
    void handleState(const mavros_msgs::State::ConstPtr &msg);
    void handlePoses(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void handleRangefinder(const sensor_msgs::RangeConstPtr &msg);

    // Main function
    void offboardAndArm();
    void publish_point();
    void navToWaypoint(float x, float y, float z, int rate);
    void navToWayPointV2(float x, float y, float z, int rate);
    void holdMode();
    void takeOffMode(float z);
    bool checkState();

    // Service func
    bool Navigate(uavlab411::Navigate::Request &req, uavlab411::Navigate::Response &res);
    bool TakeoffSrv(uavlab411::Takeoff::Request &req, uavlab411::Takeoff::Response &res);
    bool TuningPID(uavlab411::PidTuning::Request &req, uavlab411::PidTuning::Response &res);
    // Variable
    mavros_msgs::State cur_state;
    mavros_msgs::PositionTarget _navMessage, _holdMessage;
    sensor_msgs::Range _rangefinder;
    geometry_msgs::PoseStamped _setpoint;
    geometry_msgs::PoseStamped _uavpose;
    geometry_msgs::Twist _cmdvel_msg;
    ros::Duration _uavpose_timemout, _rangefinder_timeout;

    Mode _curMode;
    int update_frequency;
    // PID Controller parameter
    float Kp_yaw, Kd_yaw, Ki_yaw, Ei_yaw, Error_yaw;
    float Kp_vx, Kd_vx, Ki_vx, Ei_vx, Error_vx;

    float targetX, targetY, targetZ;
    float PidControl_yaw(float x_cur, float y_cur, float x_goal, float y_goal, float alpha, float dt);
    float PidControl_vx(float x_cur, float y_cur, float x_goal, float y_goal, float dt);
    float Control_vz(float z_cur, float z_goal);
};