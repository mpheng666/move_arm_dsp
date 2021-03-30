#include "arm_control.h"

// create serial object for communication fron ros to dsp
serial::Serial ser;
// default mode is always -1
int mode = -1;
// i_2 is used for appending dsp_array to dsp_cmd
int i_2 = 0;
// j_2 is the number of item in dsp_array
int j_2 = 0;
// swing flag = 1 when it is in swing mode
int swing_flag = 0;
// swing lock = 1 to make the next dsp command is sent after the execution is done
int swing_lock = 0;
// create stringstream object for writing serial
stringstream ss;
// create string dsp_cmd to dsp
string dsp_cmd;
// create dsp_array to store dsp_cmd
string dsp_array[100];
// create move array to store move_input data
double move_array[5];
// create string result to read from serial port
std_msgs::String result;
// status = 1 (task completed)
std_msgs::Int32 status;

// move function (read from user input)
void move_cb(const std_msgs::Float64MultiArray &move_msg)
{
	for (int i = 0; i < 5; i++)
	{
		move_array[i] = move_msg.data[i];
		// sleep(0.05);
	}
	mode = int(move_array[0]);
}

// height control function (pass in h, base_angle and cutter_angle)
void execute_kinematic_h(double h, double base_angle, double cutter_angle)
{
	// user input limit check
	if (h > h_max || h < h_min)
	{
		cout << "Please enter h value (" << h_min << " ~ " << h_max << " )!";
		j_2 = 0;
	}
	else if (base_angle > joint_1_max || base_angle < joint_1_min)
	{
		cout << "Please enter base angle value (" << joint_1_min << " ~ " << joint_1_max << " )!";
		j_2 = 0;
	}
	else if (cutter_angle > joint_4_max || cutter_angle < joint_4_min)
	{
		cout << "Please enter base angle value (" << joint_4_min << " ~ " << joint_4_max << " )!";
		j_2 = 0;
	}
	else
	{
		double joint_1 = base_angle;
		printf("joint_1: %f \n", joint_1);
		double joint_4 = cutter_angle;
		printf("joint_4: %f \n", joint_4);

		// the motion of the final motor is limited to the range of motion to this equation to prevent overloading
		double d = -0.852 * h + 0.3085;
		printf("d: %f \n", d);
		// calculate the radius distance from base point to end point
		double r = sqrt(h * h + d * d);
		printf("r: %f \n", r);
		// calcuate the theta angle
		double theta = (180.0 / PI) * acos((link_1 * link_1 + link_2 * link_2 - r * r) / (2 * link_1 * link_2));
		printf("theta: %f \n", theta);
		// calculate the phi angle
		double phi = (180.0 / PI) * acos((link_1 * link_1 - link_2 * link_2 + r * r) / (2 * link_1 * r));
		printf("phi: %f \n", phi);
		// calculate the alpha angle
		double alpha = (180.0 / PI) * atan(h / d);
		printf("alpha: %f \n", alpha);
		// calculate the actual angle of joint_2 motor
		double joint_2 = phi - abs(alpha);
		printf("joint_2: %f \n", joint_2);
		// calculate the actual angle of joint_3 motor
		double joint_3 = theta;
		printf("joint_3: %f \n", joint_3);

		// for debugging use
		// if (joint_1 > joint_1_max || joint_1 < joint_1_min)
		// {
		// 	ROS_INFO_STREAM("Joint 1 exceeds max/min angle!");
		// }
		// else if (joint_2 > joint_2_max || joint_2 < joint_2_min)
		// {
		// 	ROS_INFO_STREAM("Joint 2 exceeds max/min angle!");
		// }
		// else if (joint_3 > joint_3_max || joint_3 < joint_3_min)
		// {
		// 	ROS_INFO_STREAM("Joint 2 exceeds max/min angle!");
		// }
		// else if (joint_4 > joint_4_max || joint_4 < joint_4_min)
		// {
		// 	ROS_INFO_STREAM("Joint 4 exceeds max/min angle!");
		// }
		// else if (cutter_speed < 0 || cutter_speed > 1)
		// {
		// 	ROS_INFO_STREAM("Cutter speed exceeds max/min speed!");
		// }
		// else

		// passing the angles to dsp_array for execution later (Vs used here are variables in dsp cmd)
		int theta_x = joint_1;
		ss << "V38=" << theta_x;
		dsp_array[0] = ss.str();
		ss.str("");
		int theta_z = joint_2 - 75;
		ss << "V40=" << theta_z;
		dsp_array[1] = ss.str();
		ss.str("");
		int theta_y = joint_3 - 47.0;
		ss << "V39=" << theta_y;
		dsp_array[2] = ss.str();
		ss.str("");
		int theta_r = joint_4;
		ss << "V41=" << theta_r;
		dsp_array[3] = ss.str();
		ss.str("");
		dsp_array[4] = "V50=1";
		ss.str("");
		// declare the size of dsp_array for this loop
		j_2 = 5;
		// sleep is for debugging purposes
		// sleep(0.1);
	}
}

// distnace control function (pass in distance, base_angle and cutter_angle) same as h control except for h and d is reversed
void execute_kinematic_d(double d, double base_angle, double cutter_angle)
{
	if (d > d_max || d < d_min)
	{
		cout << "Please enter h value (" << d_min << " ~ " << d_max << " )!";
		j_2 = 0;
	}
	else if (base_angle > joint_1_max || base_angle < joint_1_min)
	{
		cout << "Please enter base angle value (" << joint_1_min << " ~ " << joint_1_max << " )!";
		j_2 = 0;
	}
	else if (cutter_angle > joint_4_max || cutter_angle < joint_4_min)
	{
		cout << "Please enter base angle value (" << joint_4_min << " ~ " << joint_4_max << " )!";
		j_2 = 0;
	}
	else
	{
		double joint_1 = base_angle;
		printf("joint_1: %f \n", joint_1);
		double joint_4 = cutter_angle;
		printf("joint_4: %f \n", joint_4);

		double h = (d - 0.3085) / (-0.852);
		printf("h: %f \n", h);
		double r = sqrt(h * h + d * d);
		printf("r: %f \n", r);
		double theta = (180.0 / PI) * acos((link_1 * link_1 + link_2 * link_2 - r * r) / (2 * link_1 * link_2));
		printf("theta: %f \n", theta);
		double phi = (180.0 / PI) * acos((link_1 * link_1 - link_2 * link_2 + r * r) / (2 * link_1 * r));
		printf("phi: %f \n", phi);
		double alpha = (180.0 / PI) * atan(h / d);
		printf("alpha: %f \n", alpha);
		double joint_2 = phi - abs(alpha);
		printf("joint_2: %f \n", joint_2);
		double joint_3 = theta;
		printf("joint_3: %f \n", joint_3);

		// if (joint_1 > joint_1_max || joint_1 < joint_1_min)
		// {
		// 	ROS_INFO_STREAM("Joint 1 exceeds max/min angle!");
		// }
		// else if (joint_2 > joint_2_max || joint_2 < joint_2_min)
		// {
		// 	ROS_INFO_STREAM("Joint 2 exceeds max/min angle!");
		// }
		// else if (joint_3 > joint_3_max || joint_3 < joint_3_min)
		// {
		// 	ROS_INFO_STREAM("Joint 2 exceeds max/min angle!");
		// }
		// else if (joint_4 > joint_4_max || joint_4 < joint_4_min)
		// {
		// 	ROS_INFO_STREAM("Joint 4 exceeds max/min angle!");
		// }
		// else if (cutter_speed < 0 || cutter_speed > 1)
		// {
		// 	ROS_INFO_STREAM("Cutter speed exceeds max/min speed!");
		// }
		// else

		int theta_x = joint_1;
		ss << "V38=" << theta_x;
		dsp_array[0] = ss.str();
		ss.str("");
		int theta_z = joint_2 - 75;
		ss << "V40=" << theta_z;
		dsp_array[1] = ss.str();
		ss.str("");
		int theta_y = joint_3 - 47.0;
		ss << "V39=" << theta_y;
		dsp_array[2] = ss.str();
		ss.str("");
		int theta_r = joint_4;
		ss << "V41=" << theta_r;
		dsp_array[3] = ss.str();
		ss.str("");
		dsp_array[4] = "V50=1";
		ss.str("");
		j_2 = 5;
	}
}

// swing control function (pass in swing_max_angle, swing_min_angle and number of swing)
void execute_swing(double swing_max, double swing_min, double swing_number)
{
	if (swing_max > joint_1_max || swing_min < joint_1_min)
	{
		ROS_INFO_STREAM("Please enter swing value (-110 ~ 110)!");
		j_2 = 0;
	}

	else
	{
		// passing swing cmd to dsp_array
		int swing_index = 0;
		for (int j = 0; j < swing_number; j++)
		{
			ss << "V38=" << swing_min;
			dsp_array[swing_index] = ss.str();
			ss.str("");
			swing_index++;
			dsp_array[swing_index] = "V50=1";
			ss.str("");
			swing_index++;
			ss << "V38=" << swing_max;
			dsp_array[swing_index] = ss.str();
			ss.str("");
			swing_index++;
			dsp_array[swing_index] = "V50=1";
			ss.str("");
			swing_index++;
		}
		j_2 = swing_number * 4;
	}
}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "move_arm");
	ros::NodeHandle nh;

	// sub to move_input
	ros::Subscriber move_sub = nh.subscribe("move_input", 10, &move_cb);
	// ros::Publisher read_pub = nh.advertise<std_msgs::String>("read", 1000);
	// sub to task_status
	ros::Publisher status_pub = nh.advertise<std_msgs::Int32>("task_status", 10);

	// create serial_port string
	std::string serial_port;
	// create baud rate
	int baud_rate;

	// get serial port and baud rate
	if (ros::param::has("/move_arm/serial_port"))
	{
		ros::param::get("/move_arm/serial_port", serial_port);
		ROS_INFO("serial_port: [%s]", serial_port.c_str());
	}
	else
	{
		serial_port = "/dev/ttyUSB0";
	}

	if (ros::param::has("/move_arm/baud_rate"))
	{
		ros::param::get("/move_arm/baud_rate", baud_rate);
		ROS_INFO("baud_rate: [%d]", baud_rate);
	}
	else
	{
		baud_rate = 9600;
	}

	try
	{
		ser.setPort(serial_port);
		ser.setBaudrate(baud_rate);
		serial::Timeout to = serial::Timeout::simpleTimeout(1000);
		ser.setTimeout(to);
		ser.open();
	}
	catch (serial::IOException &e)
	{
		ROS_ERROR_STREAM("Unable to open port ");
		return -1;
	}

	if (ser.isOpen())
	{
		ROS_INFO_STREAM("Serial Port initialized");
	}
	else
	{
		return -1;
	}

	ros::Rate loop_rate(10);
	while (ros::ok())
	{
		// condition to check when to transfer dsp cmd to serial port
		if (j_2 > 0 && i_2 < j_2 && swing_lock == 0)
		{
			dsp_cmd = dsp_array[i_2];
			ser.write(dsp_cmd);
			ser.write("\r\n");
			// if condition met then will wait for the swing to be executed completely
			if (dsp_cmd == "V50=1" && swing_flag == 1)
			{
				swing_lock = 1;
			}
			i_2++;
		}
		else if (i_2 > 0 && i_2 >= j_2)
		{
			i_2 = 0;
			j_2 = 0;
			swing_flag = 0;
			dsp_array->empty();
		}

		// mode switch
		switch (mode)
		{
		case 0:
			ROS_INFO_STREAM("Home mode");
			ser.write("V55=1");
			ser.write("\r\n");
			mode = -1;
			break;
		case 1:
			ROS_INFO_STREAM("Height control mode");
			execute_kinematic_h(move_array[1], move_array[2], move_array[3]);
			mode = -1;
			break;
		case 2:
			ROS_INFO_STREAM("Distance control mode");
			execute_kinematic_d(move_array[1], move_array[2], move_array[3]);
			mode = -1;
			break;
		case 3:
			ROS_INFO_STREAM("Swing control mode");
			execute_swing(move_array[1], move_array[2], move_array[3]);
			swing_flag = 1;
			mode = -1;
			break;
		case 100:
			// start dsp
			ROS_INFO_STREAM("Starting up...");
			ser.write("RUNP1");
			ser.write("\r\n");
			mode = -1;
			break;
		case 101:
			// free motor and stop dsp
			ROS_INFO_STREAM("Stopping...");
			dsp_array[0] = "FA";
			dsp_array[1] = "Q";
			j_2 = 2;
			mode = -1;
			break;
		default:
			break;
		}
		// read serial data
		if (ser.available())
		{
			result.data = ser.read(ser.available());
			ROS_INFO_STREAM("Read: " << result.data);
			// read_pub.publish(result);
			// cout << "Result data: " << result.data << endl;
			std::string feedback_s = result.data;
			// feedback task status
			if (feedback_s.find("123") != std::string::npos)
			{
				status.data = 1;
				status_pub.publish(status);
				// unlock swing lock
				if (swing_lock == 1)
				{
					swing_lock = 0;
				}
			}
			else
			{
				status.data = 0;
				status_pub.publish(status);
			}
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
}
