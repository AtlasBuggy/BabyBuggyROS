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
#include "serial/serial.h"
#include <math.h>
#include <iostream>
#include <sstream>

using namespace std;
namespace bd = boost::gregorian;
namespace pt = boost::posix_time;

class GPSArduinoBridge {
private:
	ros::NodeHandle nh;

	ros::Publisher gps_pub;

	string serial_port;
	int serial_baud;
	string serial_buffer;
	serial::Serial serial_ref;
	void waitForPacket(const string packet);

	ros::Time debug_info_prev_time;
	ros::Duration debug_info_delay;

	gps_common::GPSFix gps_msg;
	void parseGPSMessage();

	int days;
	int months;
	int years;
	int hours;
	int minutes;
	int seconds;
	int milliseconds;

public:
	GPSArduinoBridge(ros::NodeHandle* nodehandle);

	static const string GPS_FRAME_ID;
	static const string CHILD_FRAME_ID;

	static const string NODE_NAME;
	static const string PACKET_END;
	static const string HELLO_MESSAGE;
	static const string READY_MESSAGE;
	static const string START_COMMAND;
	static const string STOP_COMMAND;

	static const string GPS_MESSAGE_HEADER;
	static const string MESSAGE_DELIMITER;

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
