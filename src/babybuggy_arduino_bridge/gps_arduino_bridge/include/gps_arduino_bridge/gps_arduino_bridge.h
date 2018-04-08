#ifndef _GPS_ARDUINO_BRIDGE_H_
#define _GPS_ARDUINO_BRIDGE_H_

#include <ctime>
#include <iostream>

#include <boost/date_time/gregorian/gregorian.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include "ros/ros.h"
#include "std_msgs/Int64.h"
#include "std_msgs/Int16MultiArray.h"
#include "gps_common/GPSFix.h"
#include "sensor_msgs/NavSatFix.h"
#include "serial/serial.h"
#include <math.h>
#include <iostream>
#include <sstream>

using namespace std;
namespace bd = boost::gregorian;
namespace pt = boost::posix_time;

class GPSArduinoBridge {
private:
	ros::NodeHandle nh;  // ROS node handle

	// GPS data publishers and data message
	ros::Publisher gps_pub;  // publish gps_common GPSFix messages
	ros::Publisher navsat_pub;  // publish sensor_msgs NavSatFix messages

	gps_common::GPSFix gps_msg;
	sensor_msgs::NavSatFix navsat_msg;

	// Serial connection variables
	string serial_port;
	int serial_baud;
	string serial_buffer;
	serial::Serial serial_ref;

	// How often to report the GPS's status to the console or log
	ros::Time debug_info_prev_time;
	ros::Duration debug_info_delay;

	// Wait for the packet header specified with a timeout
	void waitForPacket(const string packet);

	// Helper method for parsing the GPS's data
	void parseGPSMessage();

	// scalar containers for the time. Translated to unix timestamps
	int days;
	int months;
	int years;
	int hours;
	int minutes;
	int seconds;
	int milliseconds;

public:
	GPSArduinoBridge(ros::NodeHandle* nodehandle);

	// The frame name of the IMU. Links to the base_link frame
	static const string GPS_FRAME_ID;
	static const string CHILD_FRAME_ID;

	// Important constants for the node's initialization
	static const string NODE_NAME;
	static const string PACKET_END;  // character that every packet ends with
	static const string HELLO_MESSAGE;  // the message to expect when the microcontroller starts up
	static const string READY_MESSAGE;  // message signalling that the microcontroller is ready to receive commands
	static const string START_COMMAND;  // packet to send to the microcontroller to tell it to start
	static const string STOP_COMMAND;  // packet to send to the microcontroller to tell it to stop

	// GPS data packet properties
	static const string GPS_MESSAGE_HEADER;  // the string that all imu data packets start with
	static const string MESSAGE_DELIMITER;  // IMU data segments are separated by this character

	int run();
};

// Custom error messages. Easier than returning from multiple functions to return an exit code
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
