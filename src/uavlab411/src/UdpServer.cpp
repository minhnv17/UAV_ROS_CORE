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
#include "uavlab411/UdpServer.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/BatteryState.h"
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
bool check_receiver = false;

void handle_msg_set_mode(char buff[]) 
{
	uint16_t new_mode = ReadINT16(buff, 2);
	if(state.mode != mode_define[new_mode])
	{
		static mavros_msgs::SetMode sm;
		sm.request.custom_mode = mode_define[new_mode];

		if (!set_mode.call(sm))
			ROS_INFO("Error calling set_mode service");
			throw std::runtime_error("Error calling set_mode service");
	}
	else{
		ROS_INFO("Robot already in this mode");
	}
}

void handle_arm_disarm(char buff[]) 
{
	uint16_t new_action = ReadINT16(buff, 2);
	if(!TIMEOUT(state, state_timeout) && !state.armed && new_action == 1) // Arming
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
	else if(!TIMEOUT(state, state_timeout) && state.armed && new_action == 0) // Disarming
	{
		ROS_INFO("DISARM");
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
	//ROS_INFO("mode : %d",send_state.mode);
	//ROS_INFO("vol : %f", battery_msg.voltage);
	send_state.battery_remaining = battery_remaining_calculate(battery_msg.voltage);
	//ROS_INFO("%d", send_state.battery_remaining);

	uavlink_message_t msg;
	uavlink_state_encode(&msg, &send_state);

	char buf[300];
	unsigned len = uavlink_msg_to_send_buffer((uint8_t*)buf, &msg);
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
	
	// global_pos.vx = 1.2;
	// global_pos.vy = 2.6;
	// global_pos.vz = 0.5;
	//get data from global_position
	global_pos.alt = (float)o.pose.pose.position.z;
	// global_pos.alt = 1;
	global_pos.lat = (int32_t)(global_msg.latitude*10000000);
	global_pos.lon = (int32_t)(global_msg.longitude*10000000);
	// global_pos.vx = 17;
	// global_pos.vy = 17;
	// global_pos.vz = 17;
	// //get data from global_position
	// global_pos.alt = 17;
	// global_pos.lat = 17;
	// global_pos.lon = 17;
	uavlink_message_t msg;
	uavlink_global_position_encode(&msg,&global_pos);
	char buf[300];
	unsigned len = uavlink_msg_to_send_buffer((uint8_t*)buf, &msg);
	printf("\n");
	for (int i =0 ;i<25;i++)
	{
	   printf("%d, ",buf[i]);
	}
 	uavlink_global_position_int_t test;
 	uavlink_global_position_decode((uint8_t*)buf, &test);
	// ROS_INFO("\n\n\nlat: %d", test.lat);
	// ROS_INFO("lon: %d", test.lon);
	// ROS_INFO("alt: %f", test.alt);
	// ROS_INFO("vx: %f", test.vx);
	// ROS_INFO("vy: %f", test.vy);
	// ROS_INFO("vz: %f", test.vz);
	// ROS_INFO("LEN: %d", len);
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
			uint16_t number = ReadINT16(buff, 0);
			switch (number)
			{
				case MAVLINK_MSG_ID_SET_MODE:
					handle_msg_set_mode(buff);
					break;

				case MAVLINK_MSG_ID_MANUAL_CONTROL:
					handle_msg_manual_control(bsize, buff);
					break;

				case MAV_CMD_COMPONENT_ARM_DISARM:
					handle_arm_disarm(buff);
					break;
				default:
					// test();
					break;
			}
		}
	}
}

void writeSocketMessage(char buff[], int length)
{
	// sockaddr_in client;
	// client.sin_family = AF_INET;
	// client.sin_port = htons(35602);
	// client.sin_addr.s_addr = inet_addr("192.168.0.132");
	// socklen_t client_size = sizeof(client);
	if (check_receiver) // Need received first
	{
		int len = sendto(sockfd, (const char *)buff, length + 1, 0, (const struct sockaddr *) &android_addr, android_addr_size);
		// ROS_INFO("%d", len);
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "UdpSocket");
	ros::NodeHandle nh, nh_priv("~");

	// param
	nh_priv.param("port", port, 35602);

	// Initial publisher
	manual_control_pub = nh.advertise<mavros_msgs::ManualControl>("mavros/manual_control/send", 1);
	
	// Initial subscribe
	auto state_sub = nh.subscribe("mavros/state", 1, &handleState);
	auto global_position_sub = nh.subscribe("/mavros/global_position/global",1,&handleGlobalPosition);
	auto local_position_sub = nh.subscribe("/mavros/global_position/local",1,&handleLocalPosition);
	auto battery_sub = nh.subscribe("/mavros/battery",1,&handle_Battery_State);
	// Service client
	set_mode = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
	arming = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");

	// Timer
	state_timeout = ros::Duration(nh_priv.param("state_timeout", 3.0));
	arming_timeout = ros::Duration(nh_priv.param("arming_timeout", 4.0));
	
	init();
	ros::spin();
}
