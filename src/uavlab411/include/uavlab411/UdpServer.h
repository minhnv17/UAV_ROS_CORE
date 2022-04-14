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

void 			readingSocketThread();
void 			writingSocketThread();
int  			createSocket(int);
void 			handleState(const mavros_msgs::State&);
void 			stateTimedOut(const ros::TimerEvent&);

// Function handle receiver msg
void 			handle_msg_set_mode(char buff[]);
void 			handle_msg_manual_control(int bsize, char buff[]);
void 			handle_arm_disarm(char buff[]);

// Function handle send msg
void handle_write_state(char buff[]);

// Working byte library
inline int16_t ReadINT16(char *ByteArray, int32_t Offset)
{
	int16_t result;
	memcpy(&result, ByteArray+Offset, sizeof(int16_t));
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

// Message helper define
uint8_t _mav_trim_payload(const char *payload, uint8_t length)
{
	while (length > 1 && payload[length-1] == 0) {
		length--;
	}
	return length;
}

uint16_t uavlink_msg_to_send_buffer(uint8_t *buf, const uavlink_message_t *msg)
{
	uint8_t length = msg->len;
	length = _mav_trim_payload(_MAV_PAYLOAD(msg), length);
	buf[0] = msg->msgid;
	memcpy(&buf[1], _MAV_PAYLOAD(msg), length);
	return length;
}
