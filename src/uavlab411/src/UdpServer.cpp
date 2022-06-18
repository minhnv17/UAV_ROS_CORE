#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <errno.h>
#include <thread>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "mavros_msgs/State.h"
#include "mavros_msgs/ManualControl.h"
#include "mavros_msgs/CommandBool.h"
#include "mavros_msgs/SetMode.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/BatteryState.h"
#include "uavlab411/UdpServer.h"
#include "uavlab411/control_robot_msg.h"
/* ---- Global variable ---- */
// Socket server
int sockfd;
sockaddr_in android_addr;
socklen_t android_addr_size = sizeof(android_addr);

// Timing
ros::Timer state_timeout_timer; // Check timeout connecting
ros::Duration arming_timeout;
ros::Duration state_timeout;

// ROS Message
uavlab411::control_robot_msg msg_robot;
mavros_msgs::State state; // State robot
mavros_msgs::ManualControl manual_control_msg; // Manual control msg
sensor_msgs::NavSatFix global_msg; // message from topic "/mavros/global_position/global"
sensor_msgs::BatteryState battery_msg; // message from /mavros/battery

//param
int port;

// Service clients
ros::ServiceClient arming, set_mode;

// Subcriber
ros::Subscriber state_sub;

// Publisher
ros::Publisher manual_control_pub;
ros::Publisher control_robot_pub;
bool check_receiver = false;

void handle_cmd_set_mode(int mode) 
{
	if(state.mode != mode_define[mode])
	{
		static mavros_msgs::SetMode sm;
		sm.request.custom_mode = mode_define[mode];

		if (!set_mode.call(sm))
			ROS_INFO("Error calling set_mode service");
			throw std::runtime_error("Error calling set_mode service");
	}
	else{
		ROS_INFO("Robot already in this mode");
	}
}

void handle_msg_control_robot(char buff[])
{
	msg_robot.step1 = ReadINT16(buff,2);
	msg_robot.step2 = ReadINT16(buff,4);
	msg_robot.tongs = ReadINT16(buff,6);
	control_robot_pub.publish(msg_robot);
}

void handle_cmd_arm_disarm(bool flag)
{
	if(!TIMEOUT(state, state_timeout) && !state.armed && flag == true) // Arming
	{
		ros::Time start = ros::Time::now();
		ROS_INFO("arming");
		mavros_msgs::CommandBool srv;
		srv.request.value = true;
		if (!arming.call(srv)) {
			throw std::runtime_error("Error calling arming service");
		}

		// wait until armed
		while (ros::ok()) {
			if (state.armed) {
				break;
			} else if (ros::Time::now() - start > arming_timeout) {
				string report = "Arming timed out";
				ROS_INFO("ARMING TIMEOUT... TRY AGAIN!!");
				break;
			}
		}
	}
	else if(!TIMEOUT(state, state_timeout) && state.armed && flag == false) // Disarming
	{
		ROS_INFO("DISARM"); // TODO: handle disarm motor
	}
}

void handle_command(uavlink_message_t message)
{
	uavlink_command_t command_msg;
	uavlink_command_decode(&message, &command_msg);
	switch (command_msg.command)
	{
	case UAVLINK_COMMAND_SET_MODE:
		handle_cmd_set_mode((int)command_msg.param1);
		break;
	
	case UAVLINK_CMD_ARM_DISARM:
		handle_cmd_arm_disarm((bool)command_msg.param1);
		break;
		
	default:
		break;
	}
}

void handle_msg_manual_control(int bsize, char buff[])
{
	if (bsize != 10)
	{
		ROS_ERROR_THROTTLE(30, "Wrong UDP packet size: %d", bsize);
	}
	else 
	{
		manual_control_msg.x = ReadINT16(buff, 2);
		manual_control_msg.y = ReadINT16(buff, 4);
		manual_control_msg.z = ReadINT16(buff, 6);
		manual_control_msg.r = ReadINT16(buff, 8);

		manual_control_pub.publish(manual_control_msg);
	}
}

// Handle state from UAV
void handleState(const mavros_msgs::State& s)
{
	state = s;
	uavlink_state_t send_state;
	send_state.armed = s.armed;
	send_state.connected = s.connected;
	send_state.mode = mode_to_int(s.mode);
	send_state.battery_remaining = battery_remaining_calculate(battery_msg.voltage);

	uavlink_message_t msg;
	uavlink_state_encode(&msg, &send_state);

	char buf[300];
	uint16_t len = uavlink_msg_to_send_buffer((uint8_t*)buf, &msg);
	writeSocketMessage(buf, len);
}

//Handle Local Position from UAV
void handleLocalPosition(const nav_msgs::Odometry& o)
{
	ros::Rate r(2);
	uavlink_global_position_int_t global_pos;
	global_pos.vx = (float)o.twist.twist.linear.x;
	global_pos.vy = (float)o.twist.twist.linear.y;
	global_pos.vz = (float)o.twist.twist.linear.z;
	
	//get data from global_position
	global_pos.alt = (float)o.pose.pose.position.z;
	// global_pos.alt = 1;
	global_pos.lat = (int32_t)(global_msg.latitude*10000000);
	global_pos.lon = (int32_t)(global_msg.longitude*10000000);
	uavlink_message_t msg;
	uavlink_global_position_encode(&msg,&global_pos);
	char buf[300];
	uint16_t len = uavlink_msg_to_send_buffer((uint8_t*)buf, &msg);
	writeSocketMessage(buf, len);
	r.sleep();
}

//Handle global Posotion from UAV
void handleGlobalPosition(const sensor_msgs::NavSatFix& n)
{
	global_msg = n;
}
//Handle battery state from UAV
void handle_Battery_State(const sensor_msgs::BatteryState& bat)
{
	battery_msg = bat;
	
}
void init()
{
	// Thread for UDP soket read
	std::thread readThread(&readingSocketThread);
	readThread.detach();
}

int createSocket(int port)
{
	int sockfd = socket(AF_INET, SOCK_DGRAM, 0);

	sockaddr_in sin;
	sin.sin_family = AF_INET;
	sin.sin_addr.s_addr = htonl(INADDR_ANY);
	sin.sin_port = htons(port);

	if (bind(sockfd, (sockaddr *)&sin, sizeof(sin)) < 0) {
		ROS_FATAL("socket bind error: %s", strerror(errno));
		close(sockfd);
		ros::shutdown();
	}

	return sockfd;
}

void readingSocketThread()
{
	char buff[9999];

	// Socket create
	sockfd = createSocket(port);

	ROS_INFO("UDP UdpSocket initialized on port %d", port);
	
	// handle_msg_set_mode();
	while (true) {
		// read next UDP packet
		int bsize = recvfrom(sockfd, &buff[0], sizeof(buff) - 1, 0, (sockaddr *) &android_addr, &android_addr_size);	
		check_receiver = true;
		if (bsize < 0) {
			ROS_ERROR("recvfrom() error: %s", strerror(errno));
		}
		else {
			if(!check_receiver) check_receiver = true;
			uavlink_message_t message;
			memcpy(&message, buff, sizeof(uavlink_message_t));
			switch (message.msgid)
			{
				case MAVLINK_MSG_ID_MANUAL_CONTROL:
					handle_msg_manual_control(bsize, buff);
					break;

				case UAVLINK_MSG_ID_COMMAND:
					handle_command(message);
					break;

				case CONTROL_ROBOT_MSG_ID:
					handle_msg_control_robot(buff);
					break;
				default:
					break;
			}
		}
	}
}

void writeSocketMessage(char buff[], int length)
{
	if (check_receiver) // Need received first
	{
		int len = sendto(sockfd, (const char *)buff, length + 1, 0, (const struct sockaddr *) &android_addr, android_addr_size);
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "UdpSocket");
	ros::NodeHandle nh, nh_priv("~");

	// param
	nh_priv.param("port", port, 12345);

	// Initial publisher
	manual_control_pub = nh.advertise<mavros_msgs::ManualControl>("mavros/manual_control/send", 1);
	control_robot_pub = nh.advertise<uavlab411::control_robot_msg>("control_robot",1);
	
	// Initial subscribe
	auto state_sub = nh.subscribe("mavros/state", 1, &handleState);
	auto global_position_sub = nh.subscribe("/mavros/global_position/global", 1, &handleGlobalPosition);
	auto local_position_sub = nh.subscribe("/mavros/global_position/local", 1, &handleLocalPosition);
	auto battery_sub = nh.subscribe("/mavros/battery", 1, &handle_Battery_State);

	// Service client
	set_mode = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
	arming = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");

	// Timer
	state_timeout = ros::Duration(nh_priv.param("state_timeout", 3.0));
	arming_timeout = ros::Duration(nh_priv.param("arming_timeout", 4.0));
	
	init();
	ros::spin();
}
