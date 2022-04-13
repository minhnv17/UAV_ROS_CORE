#include <sys/socket.h>
#include <netinet/in.h>
#include <errno.h>
#include <thread>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "mavros_msgs/State.h"
#include "mavros_msgs/ManualControl.h"
#include "mavros_msgs/SetMode.h"
#include <iostream>

#include "uavlab411/UdpServer.h"

using namespace std;

/* ---- Global variable ---- */
ros::Timer state_timeout_timer; // Check timeout connecting

// ROS Message
mavros_msgs::StateConstPtr state_msg; // State robot
mavros_msgs::ManualControl manual_control_msg; // Manual control msg

//param
int port;

// Service clients
ros::ServiceClient arming, set_mode;

// Subcriber
ros::Subscriber state_sub;

// Publisher
ros::Publisher manual_control_pub;


inline int16_t ReadINT16(char *ByteArray, int32_t Offset)
{
	int16_t result;
	memcpy(&result, ByteArray+Offset, sizeof(int16_t));
	return result;
};

void handle_msg_set_mode(char buff[]) 
{
	uint16_t new_mode = ReadINT16(buff, 2);
	if(state_msg->mode != mode_define[new_mode])
	{
		static mavros_msgs::SetMode sm;
		sm.request.custom_mode = mode_define[new_mode];

		if (!set_mode.call(sm))
			throw std::runtime_error("Error calling set_mode service");
	}
	else{
		ROS_INFO("Robot already in this mode");
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
void handleState(const mavros_msgs::StateConstPtr& state)
{
	state_timeout_timer.setPeriod(ros::Duration(3), true);
	state_timeout_timer.start();

	if (!state_msg || state->connected != state_msg->connected ||
		state->mode != state_msg->mode ||
		state->armed != state_msg->armed)
		{
			state_msg = state;
		}
}

// handle sate timeout
void stateTimedOut(const ros::TimerEvent&)
{
	ROS_INFO("State timeout");
	state_msg = nullptr;
}

void init()
{
	// Thread for UDP soket
	std::thread t(&socketThread);
	t.detach();
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

void socketThread()
{
	int sockfd = createSocket(port);

	char buff[9999];

	sockaddr_in client_addr;
	socklen_t client_addr_size = sizeof(client_addr);

	ROS_INFO("UDP UdpSocket initialized on port %d", port);
	// handle_msg_set_mode();
	while (true) {
		// read next UDP packet
		int bsize = recvfrom(sockfd, &buff[0], sizeof(buff) - 1, 0, (sockaddr *) &client_addr, &client_addr_size);
		
		if (bsize < 0) {
			ROS_ERROR("recvfrom() error: %s", strerror(errno));
			
		}
		// } else if (bsize != sizeof(ControlMessage)) {
		// 	ROS_ERROR_THROTTLE(30, "Wrong UDP packet size: %d", bsize);
		// 	
		// }
		else {
			// for(int i = 0; i < buff.lenght())
			// unpack message
			// warning: ignore endianness, so the code is platform-dependent
			// ControlMessage *msg = (ControlMessage *)buff;
			
			// manual_control_msg.x = msg->x;
			// manual_control_msg.y = msg->y;
			// manual_control_msg.z = msg->z;
			// manual_control_msg.r = msg->r;

			uint16_t number = ReadINT16(buff, 0);
			switch (number)
			{
				case MAVLINK_MSG_ID_SET_MODE:
					handle_msg_set_mode(buff);
					break;

				case MAVLINK_MSG_ID_MANUAL_CONTROL:
					handle_msg_manual_control(bsize, buff);
					break;

				case 5:
					break;
				default:
					
					break;
			}
			// printf("This is %d\n", number);
		}
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
	state_sub = nh.subscribe("mavros/state", 1, &handleState);

	// Service
	set_mode = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

	// Timer
	state_timeout_timer = nh.createTimer(ros::Duration(0), &stateTimedOut, true, false);
	init();
	ros::spin();
}
