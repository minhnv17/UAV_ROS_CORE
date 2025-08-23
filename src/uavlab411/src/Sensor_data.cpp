#include "uavlab411/Sensor_data.h"

void lora_data(int serial_port)
{

    struct termios tty;

	if(tcgetattr(serial_port, &tty) != 0)
	{
		printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
		return;
	}

	tty.c_cflag = B9600 | CS8 | CLOCAL | CREAD;
	tty.c_iflag = IGNPAR;
	tty.c_oflag = 0;
	tty.c_lflag = 0;

	if(tcsetattr(serial_port, TCSANOW, &tty) != 0)
	{
		printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
		return;
	}
	
	memset(&read_buf, '\0', sizeof(read_buf));
}

void handle_string(string str, uavlab411::data_sensor_msg &s)
{
    stringstream ss;
	string data;
	for (int i = 0; i < str.length(); i++)
	{
		data = data + str[i];
		if (str[i + 1] == ',')
		{
			data = data + " ";
			i++;
		}
	}
	ss << data;
	ss >> s.id >> s.lat >> s.lon >> s.temp >> s.hum >> s.gas;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "lora_data");
    ros::NodeHandle nh;

    ros::Publisher pub_loraData = nh.advertise<uavlab411::data_sensor_msg>("uavlab411/sensor", 1000);

    auto serial_port = open("/dev/ttyUSB0", O_RDWR);
	FILE *k = fdopen(serial_port,"rb");

	lora_data(serial_port);

	while(ros::ok())
	{
		fgets(read_buf, sizeof(read_buf), k);
		
        handle_string(read_buf, sens_msg);
			// ROS_INFO("%d", sens_msg.id);	
			// ROS_INFO("%f", sens_msg.lat);
			// ROS_INFO("%f", sens_msg.lon);
			// ROS_INFO("%f", sens_msg.tds);
			// ROS_INFO("%f", sens_msg.ph);
			// ROS_INFO("%f", sens_msg.temp);
			// ROS_INFO("%f", sens_msg.hum);
			// ROS_INFO("%f", sens_msg.gas);

			pub_loraData.publish(sens_msg);
			
			//ros::spinOnce();
	}

	close(serial_port);
	
	return 0;
}
