#include "ros/ros.h"
#include "uavlab411/data_sensor_msg.h"
#include <string>
#include <sstream>
#include <stdio.h>
#include <cstring>

#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>

using namespace std;

static char read_buf[256];
uavlab411::data_sensor_msg sens_msg;


void handle_string(string str, uavlab411::data_sensor_msg &s);

void lora_data(int serial_port); 