#include <ros/ros.h>
#include <string>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <errno.h>
#include <thread>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/BatteryState.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include "mavros_msgs/State.h"
#include "mavros_msgs/ManualControl.h"
#include "mavros_msgs/CommandBool.h"
#include "mavros_msgs/SetMode.h"

#include "uavlab411/control_robot_msg.h"

using std::string;

#define CONTROL_ROBOT_MSG_ID 45
#define MAVLINK_MSG_ID_MANUAL_CONTROL 69
#define UAVLINK_CMD_SET_MODE 21
#define UAVLINK_CMD_TAKEOFF 22
#define UAVLINK_CMD_ARM_DISARM 23
#define UAVLINK_CMD_LAND 24
#define UAVLINK_CMD_FLYTO 25

#define MAX_VOLTAGE 12.6
#define MIN_VOLTAGE 10.8
#define TIMEOUT(msg, timeout) (msg.header.stamp.isZero() || (ros::Time::now() - msg.header.stamp > timeout) )
// 
string mode_define[] = {"MANUAL", "POSCTL", "OFFBOARD", "AUTO.LAND"};

void 			readingSocketThread();
void 			writeSocketMessage(char*, int);
int  			createSocket(int);
void 			handleState(const mavros_msgs::State&);
void 			handleLocalPosition(const nav_msgs::Odometry&);
void 			handleGlobalPosition(const sensor_msgs::NavSatFix&);
void			handleUavPose(const geometry_msgs::PoseStampedConstPtr&);
void 			stateTimedOut(const ros::TimerEvent&);
void 			handleBatteryState(const sensor_msgs::BatteryState&);

// Function handle send msg
void handle_Write_State(char buff[]);

// Working byte library
inline int8_t ReadINT8(char *ByteArray, int32_t Offset)
{
	int8_t result;
	memcpy(&result, ByteArray+Offset, sizeof(int8_t));
	return result;
};
inline int16_t ReadINT16(char *ByteArray, int32_t Offset)
{
	int16_t result;
	memcpy(&result, ByteArray+Offset, sizeof(int16_t));
	return result;
};

inline int32_t ReadINT32(char *ByteArray, int32_t Offset)
{
	int32_t result;
	memcpy(&result, ByteArray+Offset, sizeof(int32_t));
	return result;
};
// Define message uavlink
typedef struct __uavlink_message_t {
	uint8_t 	msgid;
	uint8_t		len;
	uint64_t	payload64[64];
} uavlink_message_t;
#define _MAV_PAYLOAD(msg) ((const char *)(&((msg)->payload64[0])))
#define _MAV_PAYLOAD_NON_CONST(msg) ((char *)(&((msg)->payload64[0])))

typedef struct __uavlink_state_t {
	int8_t 		connected;
	int8_t 		armed;
	int8_t 		mode;
	int8_t 		battery_remaining;
} uavlink_state_t;
#define UAVLINK_MSG_ID_STATE 1
#define UAVLINK_MSG_ID_STATE_LEN 4

static inline uint16_t uavlink_state_encode(uavlink_message_t* msg, const uavlink_state_t* uavlink_state)
{
	uavlink_state_t packet;
	packet.connected = uavlink_state->connected;
	packet.armed = uavlink_state->armed;
	packet.mode = uavlink_state->mode;
	packet.battery_remaining = uavlink_state->battery_remaining;

	memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, UAVLINK_MSG_ID_STATE_LEN);
	msg->msgid = UAVLINK_MSG_ID_STATE;
	msg->len   = UAVLINK_MSG_ID_STATE_LEN;
	return 1;
}

static inline void uavlink_state_decode(const uavlink_message_t* msg, uavlink_state_t* state)
{

	uint8_t len = msg->len < UAVLINK_MSG_ID_STATE_LEN? msg->len : UAVLINK_MSG_ID_STATE_LEN;
	memset(state, 0, UAVLINK_MSG_ID_STATE_LEN);
    memcpy(state, _MAV_PAYLOAD(msg), len);
}

typedef struct __uavlink_global_position_int_t {
	int32_t 	lat; /*< [degE7] Latitude, expressed*/
	int32_t 	lon; /*< [degE7] Longitude, expressed*/
	float 		alt; /*< [m] Subcribe from /mavros/global_position/local -> z */
	float		vx;
	float		vy;
	float		vz;
} uavlink_global_position_int_t;
#define UAVLINK_MSG_ID_GLOBAL_POSITION_INT 2
#define UAVLINK_MSG_ID_GLOBAL_POSITION_INT_LEN 24

static inline uint16_t uavlink_global_position_encode(uavlink_message_t* msg, const uavlink_global_position_int_t* uavlink_global_position)
{
	uavlink_global_position_int_t packet;
	packet.alt = uavlink_global_position->alt;
	packet.lat = uavlink_global_position->lat;
	packet.lon = uavlink_global_position->lon;
	packet.vx = uavlink_global_position->vx;
	packet.vy = uavlink_global_position->vy;
	packet.vz = uavlink_global_position->vz;
	memcpy(_MAV_PAYLOAD_NON_CONST(msg),&packet,UAVLINK_MSG_ID_GLOBAL_POSITION_INT_LEN);
	msg->msgid = UAVLINK_MSG_ID_GLOBAL_POSITION_INT;
	msg->len = UAVLINK_MSG_ID_GLOBAL_POSITION_INT_LEN;
	return 1;
}

static inline void uavlink_global_position_decode(const uavlink_message_t* msg, uavlink_global_position_int_t* uavlink_global_position)
{
    memset(uavlink_global_position, 0, UAVLINK_MSG_ID_GLOBAL_POSITION_INT_LEN);
    memcpy(uavlink_global_position, _MAV_PAYLOAD(msg), UAVLINK_MSG_ID_GLOBAL_POSITION_INT_LEN);
}

typedef struct __uavlink_local_position_int_t {
	float 	posX; /*< [degE7] Latitude, expressed*/
	float 	posY; /*< [degE7] Longitude, expressed*/
	float 	posZ; /*< [m] Subcribe from /rangefinder/range -> z */
	float	vx;
	float	vy;
	float	vz;
} uavlink_local_position_int_t;
#define UAVLINK_MSG_ID_LOCAL_POSITION_INT 3
#define UAVLINK_MSG_ID_LOCAL_POSITION_INT_LEN 24

static inline uint16_t uavlink_local_position_encode(uavlink_message_t* msg, const uavlink_local_position_int_t* uavlink_local_position)
{
	uavlink_local_position_int_t packet;
	packet.posX = uavlink_local_position->posX;
	packet.posY = uavlink_local_position->posY;
	packet.posZ = uavlink_local_position->posZ;
	packet.vx = uavlink_local_position->vx;
	packet.vy = uavlink_local_position->vy;
	packet.vz = uavlink_local_position->vz;
	memcpy(_MAV_PAYLOAD_NON_CONST(msg),&packet,UAVLINK_MSG_ID_LOCAL_POSITION_INT_LEN);
	msg->msgid = UAVLINK_MSG_ID_LOCAL_POSITION_INT;
	msg->len = UAVLINK_MSG_ID_LOCAL_POSITION_INT_LEN;
	return 1;
}

static inline void uavlink_local_position_decode(const uavlink_message_t* msg, uavlink_local_position_int_t* uavlink_local_position)
{
    memset(uavlink_local_position, 0, UAVLINK_MSG_ID_LOCAL_POSITION_INT_LEN);
    memcpy(uavlink_local_position, _MAV_PAYLOAD(msg), UAVLINK_MSG_ID_LOCAL_POSITION_INT_LEN);
}

typedef struct __uavlink_msg_waypoint_t {
	uint16_t 	wpId; /*< [degE7] Latitude, expressed*/
	float 		targetX; /*< [degE7] Longitude, expressed*/
	float 		targetY; /*< [m] Subcribe from /rangefinder/range -> z */
	float		targetZ;
} uavlink_msg_waypoint_t;
#define UAVLINK_MSG_ID_WAYPOINT 4
#define UAVLINK_MSG_ID_WAYPOINT_LEN 14

static inline uint16_t uavlink_waypoint_encode(uavlink_message_t* msg, const uavlink_msg_waypoint_t* uavlink_msg_waypoint)
{
	uavlink_msg_waypoint_t packet;
	packet.wpId = uavlink_msg_waypoint->wpId;
	packet.targetX = uavlink_msg_waypoint->targetX;
	packet.targetY = uavlink_msg_waypoint->targetY;
	packet.targetZ = uavlink_msg_waypoint->targetZ;
	memcpy(_MAV_PAYLOAD_NON_CONST(msg),&packet,UAVLINK_MSG_ID_WAYPOINT_LEN);
	msg->msgid = UAVLINK_MSG_ID_WAYPOINT;
	msg->len = UAVLINK_MSG_ID_WAYPOINT_LEN;
	return 1;
}

static inline void uavlink_waypoint_decode(const uavlink_message_t* msg, uavlink_msg_waypoint_t* uavlink_msg_waypoint)
{
    memset(uavlink_msg_waypoint, 0, UAVLINK_MSG_ID_WAYPOINT_LEN);
    memcpy(uavlink_msg_waypoint, _MAV_PAYLOAD(msg), UAVLINK_MSG_ID_WAYPOINT_LEN);
}

typedef struct __uavlink_command_t {
	uint16_t 	command;
	float 		param1;
	float 		param2;
	float		param3;
	float		param4;
} uavlink_command_t;
#define UAVLINK_MSG_ID_COMMAND 5
#define UAVLINK_MSG_ID_COMMAND_LEN 18

static inline uint16_t uavlink_command_encode(uavlink_message_t* msg, const uavlink_command_t* uavlink_command)
{
	uavlink_command_t packet;
	packet.command = uavlink_command->command;
	packet.param1 = uavlink_command->param1;
	packet.param2 = uavlink_command->param2;
	packet.param4 = uavlink_command->param4;
	memcpy(_MAV_PAYLOAD_NON_CONST(msg),&packet,UAVLINK_MSG_ID_COMMAND_LEN);
	msg->msgid = UAVLINK_MSG_ID_COMMAND;
	msg->len = UAVLINK_MSG_ID_COMMAND_LEN;
	return 1;
}

static inline void uavlink_command_decode(const uavlink_message_t* msg, uavlink_command_t* uavlink_command)
{
    memset(uavlink_command, 0, UAVLINK_MSG_ID_COMMAND_LEN);
    memcpy(uavlink_command, _MAV_PAYLOAD(msg), UAVLINK_MSG_ID_COMMAND_LEN);
}

// Message helper define
uint8_t _mav_trim_payload(const char *payload, uint8_t length)
{
	while (length > 1 && payload[length-1] == 0) {
		length--;
	}
	return length;
}

int8_t mode_to_int(string mode)
{
	for(int8_t i = 0; i < sizeof(mode_define) -1; i++)
	{
		if (mode == mode_define[i]) return i;
	}
	return -1;
}

int8_t battery_remaining_calculate(float voltage)
{
	return (int8_t)((voltage - MIN_VOLTAGE) / (MAX_VOLTAGE - MIN_VOLTAGE)*100);
}

uint16_t uavlink_msg_to_send_buffer(uint8_t *buf, const uavlink_message_t *msg)
{
	buf[0] = msg->msgid;
	buf[1] = msg->len;
	memcpy(&buf[2], _MAV_PAYLOAD(msg), msg->len);
	return msg->len + 1 + 1;
}

// Function handle receiver msg
void 			handle_msg_manual_control(int bsize, char buff[]);
void 			handle_command(uavlink_message_t message);

// Function handle command
void 			handle_cmd_arm_disarm(bool flag);
void 			handle_cmd_set_mode(int mode);
void			handle_cmd_takeoff(float altitude);