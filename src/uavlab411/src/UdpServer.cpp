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

#include "uavlab411/UdpServer.h"

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
	send_state.mode = 26;
	send_state.battery_remaining = 80;

	uavlink_message_t msg;
	uavlink_state_encode(&msg, &send_state);

	char buf[300];
	unsigned len = uavlink_msg_to_send_buffer((uint8_t*)buf, &msg);
	writeSocketMessage(buf, len);
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
		sendto(sockfd, (const char *)buff, strlen(buff), 0, (const struct sockaddr *) &android_addr, android_addr_size);
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

	// Service client
	set_mode = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
	arming = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");

	// Timer
	state_timeout = ros::Duration(nh_priv.param("state_timeout", 3.0));
	arming_timeout = ros::Duration(nh_priv.param("arming_timeout", 4.0));
	
	init();
	ros::spin();
}
