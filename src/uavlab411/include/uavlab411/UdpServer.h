#include <ros/ros.h>
#include <string>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <errno.h>
#include <thread>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ManualControl.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>

#include <vector>
#include <array>
#include <math.h>

#include "uavlab411/drawing_point_msg.h"
#include "uavlab411/control_robot_msg.h"
#include "uavlab411/data_sensor_msg.h"
#include "uavlab411/detect_object_msg.h"
#include "uavlab411/Takeoff.h"
#include "uavlab411/Navigate.h"
#include "uavlab411/NavigateGlobal.h"

#pragma pack(1)
using std::string;


#define UAVLINK_CMD_TAKEOFF 22
#define UAVLINK_CMD_ARM_DISARM 23
#define UAVLINK_CMD_LAND 24
#define UAVLINK_CMD_FLYTO 25
#define UAVLINK_CMD_SET_MODE 26

#define PI 3.14159265358979323846 
#define MAX_VOLTAGE 12.6
#define MIN_VOLTAGE 11.1
#define TIMEOUT(msg, timeout) (msg.header.stamp.isZero() || (ros::Time::now() - msg.header.stamp > timeout))

string mode_define[] = {"MANUAL", "POSCTL", "OFFBOARD", "AUTO.LAND"};

void readingSocketThread();
void writeSocketMessage(char *, int);
int createSocket(int);

void handleState(const mavros_msgs::State &);
void handleLocalPosition(const nav_msgs::Odometry &);
void handleGlobalPosition(const sensor_msgs::NavSatFix &);
void handleUavPose(const geometry_msgs::PoseStampedConstPtr &);
void stateTimedOut(const ros::TimerEvent &);
void handleBatteryState(const sensor_msgs::BatteryState &);
void handleSensorData(const uavlab411::data_sensor_msg &);
void handleControlRobot(const uavlab411::control_robot_msg &);
void handleDetectObject(const uavlab411::detect_object_msg &);

// Function handle send msg
void handle_Write_State(char buff[]);

// Working byte library
inline int8_t ReadINT8(char *ByteArray, int32_t Offset)
{
	int8_t result;
	memcpy(&result, ByteArray + Offset, sizeof(int8_t));
	return result;
};
inline int16_t ReadINT16(char *ByteArray, int32_t Offset)
{
	int16_t result;
	memcpy(&result, ByteArray + Offset, sizeof(int16_t));
	return result;
};

inline int32_t ReadINT32(char *ByteArray, int32_t Offset)
{
	int32_t result;
	memcpy(&result, ByteArray + Offset, sizeof(int32_t));
	return result;
};
// Define message uavlink
typedef struct __uavlink_message_t
{
	uint8_t msgid;
	uint8_t len;
	uint8_t payload64[255];
} uavlink_message_t;
#define _MAV_PAYLOAD(msg) ((const char *)(&((msg)->payload64[0])))
#define _MAV_PAYLOAD_NON_CONST(msg) ((char *)(&((msg)->payload64[0])))


typedef struct __uavlink_state_t
{
	int8_t connected;
	int8_t armed;
	int8_t mode;
	int8_t battery_remaining;
} uavlink_state_t;
#define UAVLINK_MSG_ID_STATE 1
#define UAVLINK_MSG_ID_STATE_LEN 4

static inline uint16_t uavlink_state_encode(uavlink_message_t *msg, const uavlink_state_t *uavlink_state)
{
	uavlink_state_t packet;
	packet.connected = uavlink_state->connected;
	packet.armed = uavlink_state->armed;
	packet.mode = uavlink_state->mode;
	packet.battery_remaining = uavlink_state->battery_remaining;

	memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, UAVLINK_MSG_ID_STATE_LEN);
	msg->msgid = UAVLINK_MSG_ID_STATE;
	msg->len = UAVLINK_MSG_ID_STATE_LEN;
	return 1;
}

static inline void uavlink_state_decode(const uavlink_message_t *msg, uavlink_state_t *state)
{

	uint8_t len = msg->len < UAVLINK_MSG_ID_STATE_LEN ? msg->len : UAVLINK_MSG_ID_STATE_LEN;
	memset(state, 0, UAVLINK_MSG_ID_STATE_LEN);
	memcpy(state, _MAV_PAYLOAD(msg), len);
}

typedef struct __uavlink_global_position_int_t
{
	int32_t lat; /*< [degE7] Latitude, expressed*/
	int32_t lon; /*< [degE7] Longitude, expressed*/
	int16_t alt; /*< [m] Subcribe from /mavros/global_position/local -> z */
	int16_t vx;
	int16_t vy;
	int16_t vz;
	int16_t yaw;
} uavlink_global_position_int_t;
#define UAVLINK_MSG_ID_GLOBAL_POSITION_INT 2
#define UAVLINK_MSG_ID_GLOBAL_POSITION_INT_LEN 18

static inline uint16_t uavlink_global_position_encode(uavlink_message_t *msg, const uavlink_global_position_int_t *uavlink_global_position)
{
	uavlink_global_position_int_t packet;
	packet.alt = uavlink_global_position->alt;
	packet.lat = uavlink_global_position->lat;
	packet.lon = uavlink_global_position->lon;
	packet.vx = uavlink_global_position->vx;
	packet.vy = uavlink_global_position->vy;
	packet.vz = uavlink_global_position->vz;
	packet.yaw = uavlink_global_position->yaw;
	memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, UAVLINK_MSG_ID_GLOBAL_POSITION_INT_LEN);
	msg->msgid = UAVLINK_MSG_ID_GLOBAL_POSITION_INT;
	msg->len = UAVLINK_MSG_ID_GLOBAL_POSITION_INT_LEN;
	return 1;
}

static inline void uavlink_global_position_decode(const uavlink_message_t *msg, uavlink_global_position_int_t *uavlink_global_position)
{
	memset(uavlink_global_position, 0, UAVLINK_MSG_ID_GLOBAL_POSITION_INT_LEN);
	memcpy(uavlink_global_position, _MAV_PAYLOAD(msg), UAVLINK_MSG_ID_GLOBAL_POSITION_INT_LEN);
}

typedef struct __uavlink_local_position_int_t
{
	int16_t posX; /*< [degE7] Latitude, expressed*/
	int16_t posY; /*< [degE7] Longitude, expressed*/
	int16_t posZ; /*< [m] Subcribe from /rangefinder/range -> z */
	int16_t vx;
	int16_t vy;
	int16_t vz;
} uavlink_local_position_int_t;
#define UAVLINK_MSG_ID_LOCAL_POSITION_INT 3
#define UAVLINK_MSG_ID_LOCAL_POSITION_INT_LEN 12

static inline uint16_t uavlink_local_position_encode(uavlink_message_t *msg, const uavlink_local_position_int_t *uavlink_local_position)
{
	uavlink_local_position_int_t packet;
	packet.posX = uavlink_local_position->posX;
	packet.posY = uavlink_local_position->posY;
	packet.posZ = uavlink_local_position->posZ;
	packet.vx = uavlink_local_position->vx;
	packet.vy = uavlink_local_position->vy;
	packet.vz = uavlink_local_position->vz;
	memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, UAVLINK_MSG_ID_LOCAL_POSITION_INT_LEN);
	msg->msgid = UAVLINK_MSG_ID_LOCAL_POSITION_INT;
	msg->len = UAVLINK_MSG_ID_LOCAL_POSITION_INT_LEN;
	return 1;
}

static inline void uavlink_local_position_decode(const uavlink_message_t *msg, uavlink_local_position_int_t *uavlink_local_position)
{
	memset(uavlink_local_position, 0, UAVLINK_MSG_ID_LOCAL_POSITION_INT_LEN);
	memcpy(uavlink_local_position, _MAV_PAYLOAD(msg), UAVLINK_MSG_ID_LOCAL_POSITION_INT_LEN);
}

typedef struct __uavlink_msg_manual_control_t
{
	int x;
	int y;
	int z;
	int r;
} uavlink_msg_manual_control;
#define UAVLINK_MSG_ID_MANUAL_CONTROL 6
#define UAVLINK_MSG_ID_MANUAL_CONTROL_LEN 16

static inline void uavlink_manual_control_decode(const uavlink_message_t *msg, uavlink_msg_manual_control *uavlink_manual_control)
{
	memset(uavlink_manual_control, 0, UAVLINK_MSG_ID_MANUAL_CONTROL_LEN);
	memcpy(uavlink_manual_control, _MAV_PAYLOAD(msg), UAVLINK_MSG_ID_MANUAL_CONTROL_LEN);
}

typedef struct __uavlink_control_robot_t
{
	int32_t step1;
	int32_t step2;
	int32_t step3;
	int32_t step4;
	int32_t step5;

} uavlink_control_robot_t;
#define UAVLINK_CONTROL_ROBOT_MSG_ID 7
#define UAVLINK_CONTROL_ROBOT_MSG_LEN 20

static inline uint16_t uavlink_control_robot_encode(uavlink_message_t *msg, const uavlink_control_robot_t *uavlink_control_robot)
{
	uavlink_control_robot_t packet;
	packet.step1 = uavlink_control_robot->step1;
	packet.step2 = uavlink_control_robot->step2;
	packet.step3 = uavlink_control_robot->step3;
	packet.step4 = uavlink_control_robot->step4;
	packet.step5 = uavlink_control_robot->step5;
	memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, UAVLINK_CONTROL_ROBOT_MSG_LEN);
	msg->msgid = UAVLINK_CONTROL_ROBOT_MSG_ID;
	msg->len = UAVLINK_CONTROL_ROBOT_MSG_LEN;
	return 1;
}

static inline void uavlink_control_robot_decode(const uavlink_message_t *msg, uavlink_control_robot_t *state)
{

	uint8_t len = msg->len < UAVLINK_CONTROL_ROBOT_MSG_LEN ? msg->len : UAVLINK_CONTROL_ROBOT_MSG_LEN;
	memset(state, 0, UAVLINK_CONTROL_ROBOT_MSG_LEN);
	memcpy(state, _MAV_PAYLOAD(msg), len);
}

typedef struct __uavlink_msg_waypoint_t
{
	uint16_t type;
	uint32_t wpId;
	float targetX;
	float targetY;
	float targetZ;
} uavlink_msg_waypoint_t;
#define UAVLINK_MSG_ID_WAYPOINT 4
#define UAVLINK_MSG_ID_WAYPOINT_LEN 18

static inline uint16_t uavlink_waypoint_encode(uavlink_message_t *msg, const uavlink_msg_waypoint_t *uavlink_msg_waypoint)
{
	uavlink_msg_waypoint_t packet;
	packet.type = uavlink_msg_waypoint->type;
	packet.wpId = uavlink_msg_waypoint->wpId;
	packet.targetX = uavlink_msg_waypoint->targetX;
	packet.targetY = uavlink_msg_waypoint->targetY;
	packet.targetZ = uavlink_msg_waypoint->targetZ;
	memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, UAVLINK_MSG_ID_WAYPOINT_LEN);
	msg->msgid = UAVLINK_MSG_ID_WAYPOINT;
	msg->len = UAVLINK_MSG_ID_WAYPOINT_LEN;
	return 1;
}

static inline void uavlink_waypoint_decode(const uavlink_message_t *msg, uavlink_msg_waypoint_t *uavlink_msg_waypoint)
{
	int index = 0;
	memset(uavlink_msg_waypoint, 0, UAVLINK_MSG_ID_WAYPOINT_LEN);
	memcpy(uavlink_msg_waypoint, _MAV_PAYLOAD(msg), UAVLINK_MSG_ID_WAYPOINT_LEN);
}

typedef struct __uavlink_command_t
{
	uint16_t command;
	float param1;
	float param2;
	float param3;
	float param4;
} uavlink_command_t;
#define UAVLINK_MSG_ID_COMMAND 5
#define UAVLINK_MSG_ID_COMMAND_LEN 18

static inline uint16_t uavlink_command_encode(uavlink_message_t *msg, const uavlink_command_t *uavlink_command)
{
	uavlink_command_t packet;
	packet.command = uavlink_command->command;
	packet.param1 = uavlink_command->param1;
	packet.param2 = uavlink_command->param2;
	packet.param4 = uavlink_command->param4;
	memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, UAVLINK_MSG_ID_COMMAND_LEN);
	msg->msgid = UAVLINK_MSG_ID_COMMAND;
	msg->len = UAVLINK_MSG_ID_COMMAND_LEN;
	return 1;
}

static inline void uavlink_command_decode(const uavlink_message_t *msg, uavlink_command_t *uavlink_command)
{
	int index = 0;
	memset(uavlink_command, 0, UAVLINK_MSG_ID_COMMAND_LEN);
	// memcpy(&uavlink_command, _MAV_PAYLOAD(msg),UAVLINK_MSG_ID_COMMAND_LEN);
	memcpy(&uavlink_command->command, _MAV_PAYLOAD(msg), 2);
	memcpy(&uavlink_command->param1, _MAV_PAYLOAD(msg) + (index += 2), 4);
	memcpy(&uavlink_command->param2, _MAV_PAYLOAD(msg) + (index += 4), 4);
	memcpy(&uavlink_command->param3, _MAV_PAYLOAD(msg) + (index += 4), 4);
	memcpy(&uavlink_command->param4, _MAV_PAYLOAD(msg) + (index += 4), 4);
}

typedef struct __uavlink_msg_sensor_data_t
{
	int16_t id;
	int32_t lat;
	int32_t lon;
	int16_t temperature;
	int16_t humidity;
	int16_t gas;
} uavlink_msg_sensor_data_t;
#define UAVLINK_MSG_ID_SENSOR_DATA 8
#define UAVLINK_MSG_ID_SENSOR_DATA_LEN 16

static inline uint16_t uavlink_sensor_data_encode(uavlink_message_t *msg, const uavlink_msg_sensor_data_t *uavlink_sensor_data)
{
	uavlink_msg_sensor_data_t packet;
	packet.id = uavlink_sensor_data->id;
	packet.lat = uavlink_sensor_data->lat;
	packet.lon = uavlink_sensor_data->lon;
	packet.temperature = uavlink_sensor_data->temperature;
	packet.humidity = uavlink_sensor_data->humidity;
	packet.gas = uavlink_sensor_data->gas;
	memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, UAVLINK_MSG_ID_SENSOR_DATA_LEN);
	msg->msgid = UAVLINK_MSG_ID_SENSOR_DATA;
	msg->len = UAVLINK_MSG_ID_SENSOR_DATA_LEN;
	return 1;
}

typedef struct _uavlink_msg_detect_object_t
{
	float x;
	float y;
	float z;
	float distance;
} uavlink_msg_detect_object_t;
#define UAVLINK_MSG_ID_DETECT_OBJECT 9
#define UAVLINK_MSG_ID_DETECT_OBJECT_LEN 16

static inline uint16_t uavlink_detect_object_encode(uavlink_message_t *msg, const uavlink_msg_detect_object_t *uavlink_detect_object)
{
	uavlink_msg_detect_object_t packet;
	packet.x = uavlink_detect_object->x;
	packet.y = uavlink_detect_object->y;
	packet.z = uavlink_detect_object->z;
	packet.distance = uavlink_detect_object->distance;
	memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, UAVLINK_MSG_ID_DETECT_OBJECT_LEN);
	msg->msgid = UAVLINK_MSG_ID_DETECT_OBJECT;
	msg->len = UAVLINK_MSG_ID_DETECT_OBJECT_LEN;
	return 1;	
}

typedef struct _uavlink_msg_drawing_point_t
{
	float x;
	float y;
	float z;
} uavlink_msg_drawing_point_t;
#define UAVLINK_MSG_ID_DRAWING_POINT 10
#define UAVLINK_MSG_ID_DRAWING_POINT_LEN 12

static inline uint16_t uavlink_drawing_point_decode(const uavlink_message_t *msg, uavlink_msg_drawing_point_t *uavlink_drawing_point)
{
	uint8_t len = msg->len < UAVLINK_MSG_ID_DRAWING_POINT_LEN ? msg->len : UAVLINK_MSG_ID_DRAWING_POINT_LEN;
	memset(uavlink_drawing_point, 0, UAVLINK_MSG_ID_DRAWING_POINT_LEN);
	memcpy(uavlink_drawing_point, _MAV_PAYLOAD(msg), UAVLINK_MSG_ID_DRAWING_POINT_LEN);
}


// Message helper define
uint8_t _mav_trim_payload(const char *payload, uint8_t length)
{
	while (length > 1 && payload[length - 1] == 0)
	{
		length--;
	}
	return length;
}

int8_t mode_to_int(string mode)
{
	for (int8_t i = 0; i < (end(mode_define) - begin(mode_define)); i++)
	{
		if (mode == mode_define[i])
			return i;
	}
	return -1;
}

int8_t battery_remaining_calculate(float voltage)
{
	return (int8_t)((voltage - MIN_VOLTAGE) / (MAX_VOLTAGE - MIN_VOLTAGE) * 100);
}

uint16_t uavlink_msg_to_send_buffer(uint8_t *buf, const uavlink_message_t *msg)
{
	buf[0] = msg->msgid;
	buf[1] = msg->len;
	memcpy(&buf[2], _MAV_PAYLOAD(msg), msg->len);
	return msg->len + 1 + 1;
}

float get_distance_GPS(double lat, double lon, double lat_des, double lon_des)
{
	return hypot(lat - lat_des, lon - lon_des) * 1.113195e5;
}

// Function handle receiver msg
void handle_msg_manual_control(uavlink_message_t message);
void handle_command(uavlink_message_t message);
void handle_msg_waypoint(uavlink_message_t message);
void handle_msg_drawing_point(uavlink_message_t message);

// Function handle command
void handle_cmd_arm_disarm(bool flag);
void handle_cmd_set_mode(int mode);
void handle_cmd_takeoff(float altitude);
void handle_cmd_flyto(bool alwp, int wpid);
void handle_cmd_land();

// Function handle nav wp
bool navigate_to_local(uavlink_msg_waypoint_t point, float tolerance);
bool navigate_to_GPS(uavlink_msg_waypoint_t point, float tolerance);

// Function navigate_to_waypoint
bool navigate_to(uavlink_msg_waypoint_t point, float tolerance);
void navigate_points_vector(void *type);

