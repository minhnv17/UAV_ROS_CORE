#include "ros/ros.h"
#include "mavros_msgs/State.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/NavSatFix.h"
#include "std_msgs/String.h"
#include "sensor_msgs/BatteryState.h"
#include <string>

using std::string;

#define MAVLINK_MSG_ID_MANUAL_CONTROL 69
#define MAVLINK_MSG_ID_SET_MODE 11
#define MAV_CMD_COMPONENT_ARM_DISARM 400
#define MAX_VOLTAGE 12.6
#define MIN_VOLTAGE 10.8
#define TIMEOUT(msg, timeout) (msg.header.stamp.isZero() || (ros::Time::now() - msg.header.stamp > timeout) )
// 
string mode_define[] = {"POSCTL", "OFFBOARD", "AUTO.LAND"};

void 			readingSocketThread();
void 			writeSocketMessage(char*, int);
int  			createSocket(int);
void 			handleState(const mavros_msgs::State&);
void 			handle_Local_Position(const nav_msgs::Odometry&);
void 			handle_Global_Position(const sensor_msgs::NavSatFix&);
void 			stateTimedOut(const ros::TimerEvent&);
void 			handle_Battery_State(const sensor_msgs::BatteryState&);
// Function handle receiver msg
void 			handle_msg_set_mode(char buff[]);
void 			handle_msg_manual_control(int bsize, char buff[]);
void 			handle_arm_disarm(char buff[]);

// Function handle send msg
void handle_Write_State(char buff[]);

// Working byte library
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

static inline void uavlink_global_position_decode(uint8_t* buf, uavlink_global_position_int_t* uavlink_global_position)
{
    memset(uavlink_global_position, 0, UAVLINK_MSG_ID_GLOBAL_POSITION_INT_LEN);
    memcpy(uavlink_global_position, buf+1, UAVLINK_MSG_ID_GLOBAL_POSITION_INT_LEN);
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
	for(int i=0; i<sizeof(mode_define)/sizeof(mode_define[0]);i++)
	{
		if (mode == mode_define[i]) return i;
	}
	return 100;
}
int8_t battery_remaining_calculate(float voltage)
{
	return (int8_t)((voltage - MIN_VOLTAGE) / (MAX_VOLTAGE - MIN_VOLTAGE)*100);
}
uint16_t uavlink_msg_to_send_buffer(uint8_t *buf, const uavlink_message_t *msg)
{
	uint8_t length = msg->len;
	//length = _mav_trim_payload(_MAV_PAYLOAD(msg), length);
	buf[0] = msg->msgid;
	switch(msg->msgid){
		case UAVLINK_MSG_ID_STATE:
			length = UAVLINK_MSG_ID_STATE_LEN; break;
		case UAVLINK_MSG_ID_GLOBAL_POSITION_INT:
			length = UAVLINK_MSG_ID_GLOBAL_POSITION_INT_LEN; break;
		default: 
			length = 0; break;
	}
	memcpy(&buf[1], _MAV_PAYLOAD(msg), length);
	return length;
}
