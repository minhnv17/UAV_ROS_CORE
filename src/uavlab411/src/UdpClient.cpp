#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <errno.h>
#include <thread>
#include "ros/ros.h"

void read()
{
    char buff[9999];
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);

	ROS_INFO("UDP UdpSocket initialized on port %d", 12345);
	sockaddr_in sin;
	sin.sin_family = AF_INET;
	sin.sin_addr.s_addr = htonl(INADDR_ANY);
	sin.sin_port = htons(12345);

    if (bind(sockfd, (sockaddr *)&sin, sizeof(sin)) < 0) {
		ROS_FATAL("socket bind error: %s", strerror(errno));
		close(sockfd);
		ros::shutdown();
	}

    sockaddr_in client_addr;
    socklen_t client_addr_size = sizeof(client_addr);

	// handle_msg_set_mode();
	while (true) {
		// read next UDP packet
		int bsize = recvfrom(sockfd, &buff[0], sizeof(buff) - 1, 0, (sockaddr *) &client_addr, &client_addr_size);
        printf("BSIZE: %d\n", bsize);
		if (bsize < 0) {
			ROS_ERROR("recvfrom() error: %s", strerror(errno));
		}
		else {
            printf("NO SIZE\n");
            printf("BUF: %d", buff[1]);
		}
	}
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "UdpClient");
    read();
    ros::spin();
}