#ifndef _GPS_ARDUINO_BRIDGE_H_
#define _GPS_ARDUINO_BRIDGE_H_

#include "ros/ros.h"
#include "std_msgs/Int64.h"
#include "std_msgs/Int16MultiArray.h"
#include "gps_common/GPSFix.h"
#include "serial/serial.h"
#include <math.h>
#include <iostream>
#include <sstream>

using namespace std;

class GPSArduinoBridge {
private:
ros::NodeHandle nh;

ros::Publisher gps_pub;

string serial_port;
int serial_baud;
string serial_buffer;
serial::Serial serial_ref;
void waitForPacket(const string packet);

// bool euler_data_received;
// bool gyro_data_received;
// bool linaccel_data_received;
// bool quat_data_received;
//
// double initial_euler_roll;
// double initial_euler_pitch;
// double initial_euler_yaw;
//
// double euler_roll;
// double euler_pitch;
// double euler_yaw;
//
// double prev_euler_roll;
// double prev_euler_pitch;
// double prev_euler_yaw;
//
// int system_status;
// int accel_status;
// int gyro_status;
// int mag_status;
//
// int prev_system_status;
// int prev_accel_status;
// int prev_gyro_status;
// int prev_mag_status;

ros::Time debug_info_prev_time;
ros::Duration debug_info_delay;

sensor_msgs::Imu imu_msg;
void eulerToQuat(sensor_msgs::Imu &imu_msg, float roll, float pitch, float yaw);
void parseImuMessage();

gps_common::GPSFix gps_msg;
void parseGPSMessage();

public:
GPSArduinoBridge(ros::NodeHandle* nodehandle);

static const string IMU_FRAME_ID;
static const string CHILD_FRAME_ID;

static const string NODE_NAME;
static const string PACKET_END;
static const string HELLO_MESSAGE;
static const string READY_MESSAGE;
static const string START_COMMAND;
static const string STOP_COMMAND;

static const string GPS_MESSAGE_HEADER;
static const string MESSAGE_DELIMITER;

static const string ENCODER_MESSAGE_HEADER;

static const size_t MOTOR_COMMAND_MESSAGE_LEN;

static const float JUMP_WARN_THRESHOLD;

int run();
};

struct Error : exception
{
	char text[1000];

	Error(char const* fmt, ...) __attribute__((format(printf,2,3))) {
		va_list ap;
		va_start(ap, fmt);
		vsnprintf(text, sizeof text, fmt, ap);
		va_end(ap);
	}

	char const* what() const throw() {
		return text;
	}
};

#endif // _GPS_ARDUINO_BRIDGE_H_
