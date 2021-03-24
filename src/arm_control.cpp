#include "arm_control.h"

serial::Serial ser;
int mode = -1, i_2 = 0, j_2 = 0, swing_flag = 0, swing_lock = 0;
stringstream ss;
string dsp_cmd, s, ser_feedback;
string dsp_array[100];
double h_prev = 0.0, swing_wait_period = 0.0;
double move_array[5];
double va_array[6];
std_msgs::String result;

void write_callback(const std_msgs::String::ConstPtr &msg)
{
	ROS_INFO_STREAM(msg->data);
	ser.write(msg->data);
	ser.write("\r\n");
}

void move_cb(const std_msgs::Float64MultiArray &move_msg)
{
	for (int i = 0; i < 5; i++)
	{
		move_array[i] = move_msg.data[i];
		sleep(0.05);
	}
	mode = int(move_array[0]);
	// printf("mode: %d", mode);
}

void va_cb(const std_msgs::Float64MultiArray &va_msg)
{
	for (int i = 0; i < 6; i++)
	{
		va_array[i] = va_msg.data[i];
	}
}

void execute_kinematic_h(double h, double base_angle, double cutter_angle)
{
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
		// joint_1: -100 ~ 100, X(V38): -100 ~ 100
		double joint_1 = base_angle;
		printf("joint_1: %f \n", joint_1);
		// joint_4: -70 ~ 70, Y(V41): -70 ~ 70
		double joint_4 = cutter_angle;
		printf("joint_4: %f \n", joint_4);

		double d = -0.852 * h + 0.3085;
		printf("d: %f \n", d);
		double r = sqrt(h * h + d * d);
		printf("r: %f \n", r);
		double theta = (180.0 / PI) * acos((link_1 * link_1 + link_2 * link_2 - r * r) / (2 * link_1 * link_2));
		printf("theta: %f \n", theta);
		double phi = (180.0 / PI) * acos((link_1 * link_1 - link_2 * link_2 + r * r) / (2 * link_1 * r));
		printf("phi: %f \n", phi);
		double alpha = (180.0 / PI) * atan(h / d);
		printf("alpha: %f \n", alpha);
		// joint_2: 10 ~ 85, Z(V40): -65 ~ 10
		double joint_2 = phi - abs(alpha);
		printf("joint_2: %f \n", joint_2);
		// joint_3: 37 ~ 127, Y(V39): -10 ~ 80
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
		// printf("theta_x(V38): %d \n", theta_x);
		int theta_z = joint_2 - 75;
		ss << "V40=" << theta_z;
		dsp_array[1] = ss.str();
		ss.str("");
		// printf("theta_z(V40): %d \n", theta_z);
		int theta_y = joint_3 - 47.0;
		ss << "V39=" << theta_y;
		dsp_array[2] = ss.str();
		ss.str("");
		// printf("theta_y(V39): %d \n", theta_y);
		int theta_r = joint_4;
		ss << "V41=" << theta_r;
		dsp_array[3] = ss.str();
		ss.str("");
		// printf("theta_r(V41): %d \n", theta_r);
		dsp_array[4] = "V50=1";
		ss.str("");
		h_prev = h;
		j_2 = 5;
		sleep(0.1);
	}
}

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
		// joint_1: -100 ~ 100, X(V38): -100 ~ 100
		double joint_1 = base_angle;
		printf("joint_1: %f \n", joint_1);
		// joint_4: -70 ~ 70, Y(V41): -70 ~ 70
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
		// joint_2: 10 ~ 85, Z(V40): -65 ~ 10
		double joint_2 = phi - abs(alpha);
		printf("joint_2: %f \n", joint_2);
		// joint_3: 37 ~ 127, Y(V39): -10 ~ 80
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
		// printf("theta_x(V38): %d \n", theta_x);
		int theta_z = joint_2 - 75;
		ss << "V40=" << theta_z;
		dsp_array[1] = ss.str();
		ss.str("");
		// printf("theta_z(V40): %d \n", theta_z);
		int theta_y = joint_3 - 47.0;
		ss << "V39=" << theta_y;
		dsp_array[2] = ss.str();
		ss.str("");
		// printf("theta_y(V39): %d \n", theta_y);
		int theta_r = joint_4;
		ss << "V41=" << theta_r;
		dsp_array[3] = ss.str();
		ss.str("");
		// printf("theta_r(V41): %d \n", theta_r);
		dsp_array[4] = "V50=1";
		ss.str("");
		h_prev = h;
		j_2 = 5;
		sleep(0.1);
	}
}

void execute_swing(double swing_max, double swing_min, double swing_number)
{
	// printf("Swing max: %f", swing_max);
	// printf("Swing min: %f", swing_min);
	if (swing_max > joint_1_max || swing_min < joint_1_min)
	{
		ROS_INFO_STREAM("Please enter swing value (-110 ~ 110)!");
		j_2 = 0;
	}
	else
	{
		swing_wait_period = (abs(swing_max - swing_min) / 30.0) + 1.0;
		// cout << "swing_wait_period" << swing_wait_period;
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
		sleep(0.1);
	}
}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "move_arm");
	ros::NodeHandle nh;

	ros::Subscriber write_sub = nh.subscribe("write", 1000, write_callback);
	ros::Subscriber move_sub = nh.subscribe("move_input", 10, &move_cb);
	ros::Subscriber va_sub = nh.subscribe("va_value", 10, &va_cb);
	ros::Publisher read_pub = nh.advertise<std_msgs::String>("read", 1000);
	// ros::Publisher cutter_speed_pub = nh.advertise<std_msgs::Float64>("/cutter_speed_perecnt", 100);
	// std_msgs::Float64 cutter_speed_to_nano;

	std::string serial_port;
	int baud_rate;

	if (nh.getParam("serial_port", serial_port) == false)
	{
		serial_port = "/dev/ttyUSB0";
	}

	if (nh.getParam("baud_rate", baud_rate) == false)
	{
		baud_rate = SERIAL_BAUD_RATE;
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
		// cout << "swing_lock: " << swing_lock << endl;
		if (j_2 > 0 && i_2 < j_2 && swing_lock == 0)
		{
			dsp_cmd = dsp_array[i_2];
			ser.write(dsp_cmd);
			ser.write("\r\n");
			if (dsp_cmd == "V50=1" && swing_flag == 1)
			{
				swing_lock = 1;
			}
			sleep(0.1);
			i_2++;
		}
		else if (i_2 > 0 && i_2 >= j_2)
		{
			i_2 = 0;
			j_2 = 0;
			swing_flag = 0;
			dsp_array->empty();
		}

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
			ROS_INFO_STREAM("Starting up...");
			ser.write("RUNP1");
			ser.write("\r\n");
			mode = -1;
			break;
		case 101:
			ROS_INFO_STREAM("Stopping...");
			ser.write("STOPA");
			ser.write("\r\n");
			mode = -1;
			break;
		default:
			break;
		}
		if (ser.available())
		{
			result.data = ser.read(ser.available());
			ROS_INFO_STREAM("Read: " << result.data);
			read_pub.publish(result);
			// cout << "Result data: " << result.data << endl;
			std::string feedback_s = result.data;
			if (swing_lock == 1)
			{
				if (feedback_s.find("123") != std::string::npos)
				{
					cout << "Received feedback!" << endl;
					swing_lock = 0;
				}
				else
				{
					cout << "..." << endl;
				}
			}
		}
	ros::spinOnce();
	loop_rate.sleep();
	}
}
