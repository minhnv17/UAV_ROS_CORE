#include "ros/ros.h"
#include "mavros_msgs/State.h"
#include "std_msgs/String.h"
#include <string>

using std::string;

#define MAVLINK_MSG_ID_MANUAL_CONTROL 69
#define MAVLINK_MSG_ID_SET_MODE 11

string mode_define[] = {"POSCTL", "OFFBOARD", "AUTO.LAND"};

void socketThread();
int createSocket(int port);
void handleState(const mavros_msgs::StateConstPtr& state);
void stateTimedOut(const ros::TimerEvent&);
inline int16_t ReadINT16(char *ByteArray, int32_t Offset);

void handle_msg_set_mode();
void handle_msg_manual_control();

struct ControlMessage
{
	uint16_t x, y, z, r;
} __attribute__((packed));
