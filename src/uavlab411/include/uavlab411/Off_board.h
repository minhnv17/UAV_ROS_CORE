#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <std_srvs/Trigger.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/NavSatFix.h>
#include <math.h>
#include <algorithm>

#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/GlobalPositionTarget.h>

#include <uavlab411/Navigate.h>
#include <uavlab411/NavigateGlobal.h>
#include <uavlab411/PidTuning.h>
#include <uavlab411/Takeoff.h>
#include <uavlab411/Telemetry.h>

using namespace mavros_msgs;
using namespace std;

#define TIMEOUT(msg, timeout) (msg.header.stamp.isZero() || (ros::Time::now() - msg.header.stamp > timeout))

enum Mode
{
    Takeoff = 0,
    Hold = 2,
    NavYaw = 1,
    NavNoYaw = 3,
    NavGlobal = 4
};

float ScalePid(float data, float maxData, float minData, float heso)
{
    return (((data - minData) * heso) / (maxData - minData));
}

class OffBoard
{
public:
    OffBoard();

private:
    ros::NodeHandle nh;
    // Publisher
    ros::Publisher pub_navMessage;
    ros::Publisher pub_pointMessage, pub_globalMessage;
    // Subscriber
    ros::Subscriber sub_state;
    ros::Subscriber sub_uavpose, sub_local_position, sub_global_position;

    // Service
    ros::ServiceClient srv_arming, srv_set_mode;
    ros::ServiceServer navigate_srv, navigateGlobal_srv, pid_tuning_srv, takeoff_srv, land_srv, telemetry_srv;

    ros::Timer setpoint_timer;
    // Function handle
    void handleState(const mavros_msgs::State::ConstPtr &msg);
    void handlePoses(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void handleLocalPosition(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void handleGlobalPosition(const sensor_msgs::NavSatFix::ConstPtr &msg);

    // Main function
    void offboardAndArm();
    void publish_point();
    void navToWaypoint(float x, float y, float z, int rate);
    void navToWayPointV2(float x, float y, float z, int rate);
    void holdMode();
    void getCurrentPosition();
    void takeOffMode(float z);
    bool checkState();

    // Service func
    bool Navigate(uavlab411::Navigate::Request &req, uavlab411::Navigate::Response &res);
    bool NavigateGlobal(uavlab411::NavigateGlobal::Request &req, uavlab411::NavigateGlobal::Response &res);
    bool TakeoffSrv(uavlab411::Takeoff::Request &req, uavlab411::Takeoff::Response &res);
    bool TuningPID(uavlab411::PidTuning::Request &req, uavlab411::PidTuning::Response &res);
    bool Land(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    bool GetTelemetry(uavlab411::Telemetry::Request &req, uavlab411::Telemetry::Response &res);
    // Variable
    mavros_msgs::State cur_state;
    mavros_msgs::PositionTarget _navMessage, _holdMessage;
    sensor_msgs::NavSatFix _globalPos;
    geometry_msgs::Point _startGPoint, _endGPoint;
    geometry_msgs::PoseStamped _setpoint, _pointMessage;
    geometry_msgs::PoseStamped _uavpose;
    geometry_msgs::PoseStamped _uavpose_local_position;
    geometry_msgs::Twist _cmdvel_msg;
    mavros_msgs::GlobalPositionTarget nav_sp;
    ros::Duration _uavpose_timemout, _uavpose_local_position_timeout, _land_timeout;
    ros::Time getTime;
    Mode _curMode;
    float update_frequency;
    // z map when booting UAV
    double z_map;
    // PID Controller parameter
    float Kp_yaw, Kd_yaw, Ki_yaw, Ei_yaw, Error_yaw;
    float Kp_vx, Kd_vx, Ki_vx, Ei_vx, Error_vx;

    float targetX, targetY, targetZ, speed, realDistance;
    float PidControl_yaw(float x_cur, float y_cur, float x_goal, float y_goal, float alpha, float dt);
    float PidControl_vx(float x_cur, float y_cur, float x_goal, float y_goal, float dt);
    float Control_vz(float z_cur, float z_goal);
    float getNavigateSetpoint(const ros::Time &stamp, float speed, mavros_msgs::GlobalPositionTarget &nav_setpoint);
};

float getDistance(const geometry_msgs::Point &from, const geometry_msgs::Point &to);