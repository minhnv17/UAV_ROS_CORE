#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include "nav_msgs/Odometry.h"
#include <nav_msgs/Path.h>
#include "sensor_msgs/NavSatFix.h"
#include <sensor_msgs/Range.h>
#include "std_msgs/String.h"
#include "sensor_msgs/BatteryState.h"
#include <sensor_msgs/Range.h>

#include <string>
#include <math.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

#define PI 3.14159265358979323846 
#define TIMEOUT(msg, timeout) (msg.header.stamp.isZero() || (ros::Time::now() - msg.header.stamp > timeout))
ros::Duration _aruco_timemout = ros::Duration(0.2);
std::string value;

class Localization
{
public:
    Localization();

private:
    ros::NodeHandle nh;
    // param for calculate uavpose from local position when loss aruco detection
    double x_negative, y_negative, yaw_negative;
    // define global variable
    double z_barometer;
    geometry_msgs::PoseStamped uav_pose, pose_path, uavpose_of_local_msg, pose_path_GPS;
    nav_msgs::Path path_uav;
    sensor_msgs::Range _rangefinder;
    uint64_t pose_seq = 0;

    // define subscriber and publisher
    ros::Publisher uavpose_pub, path_pub, uavpose_of_local_pub, pub_baro;

    ros::Subscriber main_optical_flow_pose_sub, range_finder_sub;
    ros::Subscriber local_position_sub;

    void handle_main_optical_flow_pose(const geometry_msgs::PoseWithCovarianceStamped &);
    void handle_local_position(const geometry_msgs::PoseStamped &);
    void handle_global_position_local(const nav_msgs::Odometry &);
    void handle_rangefinder(const sensor_msgs::RangeConstPtr &);
};
