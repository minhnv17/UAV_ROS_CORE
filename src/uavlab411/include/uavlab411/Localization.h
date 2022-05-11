#include "ros/ros.h"
#include "mavros_msgs/State.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/NavSatFix.h"
#include "std_msgs/String.h"
#include "sensor_msgs/BatteryState.h"
#include <string>
#include <math.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>

#define PI atan(1)*4
void handle_main_optical_flow_pose(const geometry_msgs::PoseWithCovarianceStamped&);
void handle_local_position(const geometry_msgs::PoseStamped&);
