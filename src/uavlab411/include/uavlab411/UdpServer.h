#include "ros/ros.h"
#include "mavros_msgs/State.h"
#include "std_msgs/String.h"
#include <string>

using std::string;

#define MAVLINK_MSG_ID_MANUAL_CONTROL 69
#define MAVLINK_MSG_ID_SET_MODE 11
#define MAV_CMD_COMPONENT_ARM_DISARM 400
#define TIMEOUT(msg, timeout) (msg.header.stamp.isZero() || (ros::Time::now() - msg.header.stamp > timeout) )
// 
string mode_define[] = {"POSCTL", "OFFBOARD", "AUTO.LAND"};

void socketThread();
int createSocket(int);
void handleState(const mavros_msgs::State&);
void stateTimedOut(const ros::TimerEvent&);
inline int16_t ReadINT16(char *, int32_t);

void handle_msg_set_mode(char buff[]);
void handle_msg_manual_control(int bsize, char buff[]);
void handle_arm_disarm(char buff[]);

struct ControlMessage
{
	uint16_t x, y, z, r;
} __attribute__((packed));
