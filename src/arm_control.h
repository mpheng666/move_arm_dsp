#define DSP_SERVO_BOARD
#define SERIAL_BAUD_RATE 9600

#include <ros/ros.h>
#include <iostream>
#include <sstream>
#include <unistd.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>

#include <std_msgs/Int32.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include "serial/serial.h"

using std::cin;
using std::cout;
using std::endl;
using std::string;
using std::stringstream;

#define PI 3.141592
#define link_1 0.37
#define link_2 0.38
#define h_max 0.01
#define h_min -0.27
#define d_min 0.3
#define d_max 0.53
#define joint_1_min -110.0
#define joint_1_max 110.0
#define joint_1_home 0.0
#define joint_2_min 10.0
#define joint_2_max 85.0
#define joint_2_home 75.0
#define joint_3_min 37.0
#define joint_3_max 127.0
#define joint_3_home 47.0
#define joint_4_min -70.0
#define joint_4_max 70.0
#define joint_4_home 0.0
