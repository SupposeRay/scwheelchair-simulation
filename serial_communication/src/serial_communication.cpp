#include <ros/ros.h> // header file containing ROS
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>
#include <vector>
//#include <array>
#include <string>
#include <termios.h>
#include <boost/asio.hpp> // Include boost library function
#include <boost/bind.hpp>
#include "std_msgs/String.h"			//Ros defined String data type
#include "std_msgs/Float32MultiArray.h" // ROS message used for joystick data
#include "geometry_msgs/Twist.h"		// ROS message used for linear and angular velocities
#include "geometry_msgs/Quaternion.h"   // ROS message used for angular orientation
#include "nav_msgs/Odometry.h"			// ROS message used for odometry data
#include "sensor_msgs/Imu.h"			// ROS message used for linear and angular accelerations
#include "geometry_msgs/Pose2D.h"
#include <time.h>
#include <chrono>
#include <math.h>
#include <scat_libs/rosmsg.h>
#include <scat_libs/base_utils.h>


using namespace std;
using namespace boost::asio; // Define a namespace for subsequent read and write operations

#define ENCODERCIR 16384.
#define SWAP_INT16(a) a //   (  (a)<<8 | ( (a)>>8 ) & 0x00FF )
#define PACKAGE_START 0xfffb

//// global variables (for use in functions)
// Robot position in odom frame
float x_odom = 0, y_odom = 0, theta_odom = 0;
// Received command velocity
geometry_msgs::Twist cmd_vel;
// Median filter buffers
std::deque<float> buffer_x;
std::deque<float> buffer_y;
ros::Time time_last_cmd;

//// global parameters (for use in functions & parameters read out from file)
std::string usb_port_ = "/dev/ttyUSB0";
float wheel_acc_limit_ = 10.0;
bool publish_odom_tf_ = false;
bool apply_vel_filter_ = true;
int vel_filter_size_ = 5;
std::string odom_update_method_ = "odom";

//// Calibrated sensor parameters
// Odometry parameters
float calibration_v_angular_ = 1.0;
float calibration_v_linear_ = 1.0;
float calibration_cmd_ang_ = 1.0;
float calibration_cmd_lin_ = 1.0;

// IMU parameters
float bias_acc_x_ = 0.0;
float bias_acc_y_ = 0.0;
float bias_acc_z_ = 0.0;
float bias_gyro_x_ = 0.0;
float bias_gyro_y_ = 0.0;
float bias_gyro_z_ = 0.0;
float cal_imu_acc_ = 1.0;
float cal_imu_gyro_ = 1.0;

// IMU covariance matrices, measured using rosbag during experiment while driving around and calculated using numpy. 0 entry means negligible.
std::vector<double> CovOrient = {-1, -1, -1, // Orientation not given by IMU, set Covariance to -1 and estimates to 0. 
								-1, -1, -1,
								-1, -1, -1};
// std::vector<double> CovOrient = {1, 0, 0,
// 								0, 1, 0,
// 								0, 0, 1};								
std::vector<double> CovGyro = {0.002, 0, 0,
								0, 0.002, 0,
								0, 0, 0.002};
std::vector<double> CovAcc = {0.002, 0, 0,
								0, 0.002, 0,
								0, 0, 0.34};
std::vector<double> CovOdom = {0.2, 0, 0, 0, 0, 0, 
								0, 0.2, 0, 0, 0, 0, 
								0, 0, 0.01, 0, 0, 0,
								0, 0, 0, 0.01, 0, 0,
								0, 0, 0, 0, 0.01, 0,
								0, 0, 0, 0, 0, 0.4};
std::vector<double> CovVel = {0.1, 0, 0, 0, 0, 0, 
								0, 0.01, 0, 0, 0, 0, 
								0, 0, 0.01, 0, 0, 0,
								0, 0, 0, 0.01, 0, 0,
								0, 0, 0, 0, 0.01, 0,
								0, 0, 0, 0, 0, 0.1};

void medianFilter(float &x, float &y)
{
	buffer_x.push_back(x);
	buffer_y.push_back(y);
	if (buffer_x.size() > vel_filter_size_)
	{
		buffer_x.pop_front();
		buffer_y.pop_front();
	}
	float average_x = std::accumulate(buffer_x.begin(), buffer_x.end(), 0.0) / buffer_x.size();
	float average_y = std::accumulate(buffer_y.begin(), buffer_y.end(), 0.0) / buffer_y.size();

	x = average_x;
	y = average_y;
}

void cmdCallback(const geometry_msgs::Twist msg_cmd)
{
	cmd_vel.linear.x = msg_cmd.linear.x;
	cmd_vel.angular.z = msg_cmd.angular.z;
	time_last_cmd = ros::Time::now();
}

void odomCallback(const nav_msgs::Odometry msg_odom)
{
	x_odom = msg_odom.pose.pose.position.x;
	y_odom = msg_odom.pose.pose.position.y;
	theta_odom = tf::getYaw(msg_odom.pose.pose.orientation);
}

void poseCallback(const geometry_msgs::Pose2D msg_pose)
{
	x_odom = msg_pose.x;
	y_odom = msg_pose.y;
	theta_odom = msg_pose.theta;
}

bool readParameters(ros::NodeHandle &node_handle)
{
	if (!node_handle.getParam("usb_port", usb_port_))
		ROS_WARN_STREAM("Parameter usb_port not set for serial_communication. Using default setting: " << usb_port_);
	if (!node_handle.getParam("odom_update_method", odom_update_method_))
		ROS_WARN_STREAM("Parameter odom_update_method not set for serial_communication. Using default setting: " << odom_update_method_);
	if (!node_handle.getParam("wheel_acc_limit", wheel_acc_limit_))
		ROS_WARN_STREAM("Parameter wheel_acc_limit not set for serial_communication. Using default setting: " << wheel_acc_limit_);
	if (!node_handle.getParam("publish_odom_tf", publish_odom_tf_))
		ROS_WARN_STREAM("Parameter publish_odom_tf not set for serial_communication. Using default setting: " << publish_odom_tf_);
	if (!node_handle.getParam("calibration_v_angular", calibration_v_angular_))
		ROS_WARN_STREAM("Parameter calibration_v_angular not set for serial_communication. Using default setting: " << calibration_v_angular_);
	if (!node_handle.getParam("calibration_v_linear", calibration_v_linear_))
		ROS_WARN_STREAM("Parameter calibration_v_linear not set for serial_communication. Using default setting: " << calibration_v_linear_);
	if (!node_handle.getParam("calibration_cmd_ang", calibration_cmd_ang_))
		ROS_WARN_STREAM("Parameter calibration_cmd_ang not set for serial_communication. Using default setting: " << calibration_cmd_ang_);
	if (!node_handle.getParam("calibration_cmd_lin", calibration_cmd_lin_))
		ROS_WARN_STREAM("Parameter calibration_cmd_lin not set for serial_communication. Using default setting: " << calibration_cmd_lin_);
	if (!node_handle.getParam("apply_vel_filter", apply_vel_filter_))
		ROS_WARN_STREAM("Parameter apply_vel_filter not set for serial_communication. Using default setting: " << apply_vel_filter_);
	if (!node_handle.getParam("vel_filter_size", vel_filter_size_))
		ROS_WARN_STREAM("Parameter vel_filter_size not set for serial_communication. Using default setting: " << vel_filter_size_);
	if (!node_handle.getParam("bias_acc_x", bias_acc_x_))
		ROS_WARN_STREAM("Parameter bias_acc_x not set for serial_communication. Using default setting: " << bias_acc_x_);
	if (!node_handle.getParam("bias_acc_y", bias_acc_y_))
		ROS_WARN_STREAM("Parameter bias_acc_y not set for serial_communication. Using default setting: " << bias_acc_y_);
	if (!node_handle.getParam("bias_acc_z", bias_acc_z_))
		ROS_WARN_STREAM("Parameter bias_acc_z not set for serial_communication. Using default setting: " << bias_acc_z_);
	if (!node_handle.getParam("bias_gyro_x", bias_gyro_x_))
		ROS_WARN_STREAM("Parameter bias_gyro_x not set for serial_communication. Using default setting: " << bias_gyro_x_);
	if (!node_handle.getParam("bias_gyro_y", bias_gyro_y_))
		ROS_WARN_STREAM("Parameter bias_gyro_y not set for serial_communication. Using default setting: " << bias_gyro_y_);
	if (!node_handle.getParam("bias_gyro_z", bias_gyro_z_))
		ROS_WARN_STREAM("Parameter bias_gyro_z not set for serial_communication. Using default setting: " << bias_gyro_z_);
	if (!node_handle.getParam("cal_imu_gyro", cal_imu_gyro_))
		ROS_WARN_STREAM("Parameter cal_imu_gyro not set for serial_communication. Using default setting: " << cal_imu_gyro_);
	if (!node_handle.getParam("cal_imu_acc", cal_imu_acc_))
		ROS_WARN_STREAM("Parameter cal_imu_acc not set for serial_communication. Using default setting: " << cal_imu_acc_);
	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "serial_communication"); // Initialize the node
	ros::NodeHandle node_handle("~");

	// Read Parameters
	if (!readParameters(node_handle))
	{
		ROS_ERROR("Could not read parameters for serial_communication");
		ros::requestShutdown();
	}

	// Subscribers
	ros::Subscriber cmd_vel_sub = node_handle.subscribe("/cmd_vel", 1, cmdCallback);
	if (odom_update_method_ == "pose2D")
		ros::Subscriber scan_pose_sub = node_handle.subscribe("/pose2D", 1, poseCallback);
	else if (odom_update_method_ == "odom")
		ros::Subscriber odom_sub = node_handle.subscribe("/ekf/odom", 1, odomCallback);

	// Publishers
	ros::Publisher joystick_pub = node_handle.advertise<std_msgs::Float32MultiArray>("/user/joy", 1);
	ros::Publisher velocity_pub = node_handle.advertise<geometry_msgs::Twist>("/velocities", 1);
	ros::Publisher acceleration_pub = node_handle.advertise<sensor_msgs::Imu>("/imu", 1);
	ros::Publisher velocity_raw_pub = node_handle.advertise<std_msgs::Float32MultiArray>("/velocities_raw", 1);
	ros::Publisher odom_pub = node_handle.advertise<nav_msgs::Odometry>("/encoders/odom", 1);
	ros::Publisher odom2d_pub = node_handle.advertise<geometry_msgs::Pose2D>("/encoders/pose2D", 1);
	tf::TransformBroadcaster odom_broadcaster;

	// Variables
	int16_t buf[11];		// Define the received data
	int16_t velocitybuf[3]; //define wheelchair speed including side1 x and side2 y as well as one header for data verification.
	uint16_t counter1 = 0, counter2 = 0;
	float AccX, AccY, AccZ, GyroX, GyroY, GyroZ;
	float f_right, f_left, v_right, v_left, v_right_prev, v_left_prev, v_right_cmd, v_left_cmd;
	std::vector<float> JoystickValue = {0, 0};
	float encoder_left, encoder_right, encoder_left_prev, encoder_right_prev;
	// Parameters
	float wheel_diameter = 0.25;
	float base_width = 0.5;
	float SYSTEM_FREQUENCY = 50.0;
	float VelocityMax = 0.45 * 126 / 60 * wheel_diameter * M_PI; // converts from m/s to pwm pulses per second
	float max_dt_cmd = 1.0;
	// // Data quality check
	// float variance_x,variance_y,mean_x,mean_y;
	// int Period_Count=500;
	// int count=0;
	// std::vector<float> x_data(Period_Count);
	// std::vector<float> y_data(Period_Count);

	// Set up
	io_service iosev;
	serial_port sp(iosev, usb_port_); // Define the serial port of the transmission // serial_port sp(iosev, "/dev/USB_MCU");
	sp.set_option(serial_port::baud_rate(115200));
	sp.set_option(serial_port::flow_control());
	sp.set_option(serial_port::parity());
	sp.set_option(serial_port::stop_bits());
	sp.set_option(serial_port::character_size(8));
	ROS_INFO_STREAM("initializing serial connection");
	bool flag_beginning = false;
	while (!flag_beginning)
	{
		uint8_t temp[1];
		read(sp, buffer(temp));
		if (temp[0] == 0xcd)
		{
			read(sp, buffer(temp));
			if (temp[0] == 0xab)
			{
				flag_beginning = true;
			}
		}
	}
	ROS_INFO_STREAM("achieved connection to MCU");

	auto time_now = std::chrono::system_clock::now();
	auto time_prev = std::chrono::system_clock::now();
	ros::Time timestamp;

	while (ros::ok())
	{
		timestamp = ros::Time::now();
		time_now = std::chrono::system_clock::now();

		ros::spinOnce(); // check for incoming messages
		// static uint32_t cnt = 0;
		// ROS_INFO_STREAM("cnt: " << (cnt++) / 200.0 ) ;

		ros::Duration timediff_cmd = timestamp - time_last_cmd; // negative if message received in last spinonce
		if (timediff_cmd.toSec() > max_dt_cmd)
		{
			v_left_cmd = 0;
			v_right_cmd = 0;
		}
		else
		{
			v_left_cmd = cmd_vel.linear.x * calibration_cmd_lin_ - cmd_vel.angular.z * (base_width / 4) * calibration_cmd_ang_;
			v_right_cmd = cmd_vel.linear.x * calibration_cmd_lin_ + cmd_vel.angular.z * (base_width / 4) * calibration_cmd_ang_;
		}

		velocitybuf[0] = v_left_cmd / VelocityMax * 500; // Convert to MCU velocity range: sends pwm signals at 1500, +/- 500
		velocitybuf[1] = v_right_cmd / VelocityMax * 500;
		velocitybuf[2] = 0xFFFB;
		write(sp, buffer(velocitybuf)); //write the speed for cmd_val

		static uint8_t buf_temp[22];

		read(sp, buffer(buf_temp)); // block

		for (uint8_t i = 0; i < 11; i++)
		{
			//ROS_INFO_STREAM((uint16_t)buf_temp[i]);
			buf[i] = buf_temp[2 * i + 1] << 8 | buf_temp[2 * i];
			// ROS_INFO_STREAM(buf[i]);
		}
		if (buf[10] != int16_t(0xabcd))
		{
			std::cout << "data lost skip this transmission, with buf " << std::hex << buf[10] << std::endl;
			continue;
		}
		// Implicit conversion from uint16 to float
		JoystickValue[0] = -(buf[1] / 2048.0 - 1.0)/0.7;
		JoystickValue[1] = -(buf[0] / 2048.0 - 1.0)/0.7;

		AccX = buf[4] / 16384.0 * cal_imu_acc_ - bias_acc_x_;
		AccY = buf[5] / 16384.0 * cal_imu_acc_ - bias_acc_y_;
		AccZ = buf[6] / 16384.0 * cal_imu_acc_ - bias_acc_z_;
		GyroX = buf[7] / 121.0 * cal_imu_gyro_ - bias_gyro_x_;
		GyroY = buf[8] / 121.0 * cal_imu_gyro_ - bias_gyro_y_;
		GyroZ = buf[9] / 121.0 * cal_imu_gyro_ - bias_gyro_z_;

		//! To do implememt realiable formulaus for computing velocity
		encoder_left = buf[3];  // encoder1 is left
		encoder_right = buf[2]; // encoder2 is right

		float diff_enc_left = encoder_left - encoder_left_prev;
		float diff_enc_right = encoder_right - encoder_right_prev;
		if (fabs(diff_enc_right) <= ENCODERCIR / 2)
			f_right = diff_enc_right;
		else if (fabs(diff_enc_right) < -ENCODERCIR / 2)
			f_right = diff_enc_right + ENCODERCIR;
		else
			f_right = diff_enc_right - ENCODERCIR;

		if (fabs(diff_enc_left) <= ENCODERCIR / 2)
			f_left = diff_enc_left;
		else if (fabs(diff_enc_left) < -ENCODERCIR / 2)
			f_left = diff_enc_left + ENCODERCIR;
		else
			f_left = diff_enc_left - ENCODERCIR;

		v_right = -f_right / ENCODERCIR * M_PI * wheel_diameter * SYSTEM_FREQUENCY;
		v_left = f_left / ENCODERCIR * M_PI * wheel_diameter * SYSTEM_FREQUENCY;

		// Sometimes data gets lost and spikes are seen in the velocity readouts.
		// This is solve by limiting the max difference between subsequent velocity readouts.
		if (fabs(v_right - v_right_prev) * SYSTEM_FREQUENCY > wheel_acc_limit_)
			v_right = v_right_prev;
		if (fabs(v_left - v_left_prev) * SYSTEM_FREQUENCY > wheel_acc_limit_)
			v_left = v_left_prev;

		// deadzone the velocities, to avoid accumulation of noise in steady position
		if (fabs(v_right) < 0.01 || fabs(v_right) > 5 || isnan(v_right))
			v_right = 0.0;
		if (fabs(v_left) < 0.01 || fabs(v_left) > 5 || isnan(v_left))
			v_left = 0.0;

		// Apply medianfilter to velocities
		if (apply_vel_filter_)
			medianFilter(v_left, v_right);

		// Odometry calculation
		float v_linear = (v_right + v_left) / 2 * calibration_v_linear_/2;
		float v_angular = (v_right - v_left) / ( base_width / 4) * calibration_v_angular_/2;
		std::chrono::duration<float> elapsed_seconds = time_now - time_prev;
		float dt = elapsed_seconds.count();
		float delta_x = v_linear * cos(theta_odom) * dt;
		float delta_y = v_linear * sin(theta_odom) * dt;
		float delta_th = v_angular * dt;
		x_odom += delta_x;
		y_odom += delta_y;
		theta_odom += delta_th;

		// if(count>=Period_Count){
		// 	mean_x=mean_y=variance_x=variance_y=0.;
		//     for(int i=0;i<Period_Count;i++){
		// 		mean_x+=x_data[i];
		// 		mean_y+=y_data[i];
		// 	}
		// 	mean_x/=Period_Count;
		// 	mean_y/=Period_Count;
		// 	for(int i=0;i<Period_Count;i++){
		// 		variance_x+=(x_data[i]-mean_x)*(x_data[i]-mean_x);
		// 		variance_y+=(y_data[i]-mean_y)*(y_data[i]-mean_y);
		// 	}
		// 	variance_x/=Period_Count;
		// 	variance_y/=Period_Count;
		// 	count=0;
		// 	std::cout<<"x variance "<<std::sqrt(variance_x)<<" y variance "<<std::sqrt(variance_y)<<std::endl;
		// }else{
		// 	x_data[count]=JoystickValue[0];
		// 	y_data[count]=JoystickValue[1];
		// 	count+=1;
		// }

		// Publish Joystick data
		joystick_pub.publish(rosmsg::makeFloat32MultiArray(JoystickValue));

		// Publish raw Velocity data
		std::vector<float> velocity_raw = {v_left, v_right};
		velocity_raw_pub.publish(rosmsg::makeFloat32MultiArray(velocity_raw));

		// Publish Velocity data
		velocity_pub.publish(rosmsg::makeTwist(v_linear, v_angular));

		// Publish Odometry Pose2D
		odom2d_pub.publish(rosmsg::makePose2D(x_odom, y_odom, theta_odom));

		// Publish Odometry message
		odom_pub.publish(rosmsg::makeOdometry(rosmsg::makeHeader("odom", timestamp),
											  x_odom, y_odom, theta_odom, v_linear, v_angular,
											  CovOdom, CovVel));

		// Publish IMU data
		acceleration_pub.publish(rosmsg::makeIMU(rosmsg::makeHeader("base_footprint", timestamp),
												0,0,0,
												GyroX,GyroY,GyroZ,
												AccX,AccY,AccZ,
												CovOrient,CovGyro,CovAcc));

		// Send Odom transform (ONLY IF NO OTHER ODOM PUBLISHER ACTIVE!!)
		// (e.g. laser_scan_matcher or robot_pose_ekf)
		if (publish_odom_tf_)
		{
			tf::Quaternion odom_quat = tf::createQuaternionFromYaw(theta_odom);
			odom_broadcaster.sendTransform(
				tf::StampedTransform(
					tf::Transform(odom_quat, tf::Vector3(x_odom, y_odom, 0.0)),
					timestamp, "odom", "base_footprint"));
		}

		time_prev = time_now;
		encoder_left_prev = encoder_left;
		encoder_right_prev = encoder_right;
		v_right_prev = v_right;
		v_left_prev = v_left;
	}
	iosev.run();
	return 0;
}
