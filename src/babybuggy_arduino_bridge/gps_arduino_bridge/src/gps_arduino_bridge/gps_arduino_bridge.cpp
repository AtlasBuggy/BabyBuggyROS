#include <gps_arduino_bridge/gps_arduino_bridge.h>

// string parsing macros
#define STR_TO_FLOAT(string)  strtof((string).c_str(), 0)
#define STR_TO_INT(string)  string_to_int64(string)

// Constant definitions
const string GPSArduinoBridge::GPS_FRAME_ID = "adafruit_gps";
const string GPSArduinoBridge::CHILD_FRAME_ID = "base_link";

const string GPSArduinoBridge::NODE_NAME = "gps_arduino_bridge";
const string GPSArduinoBridge::PACKET_END = "\n";

const string GPSArduinoBridge::HELLO_MESSAGE = "hello!" + PACKET_END;
const string GPSArduinoBridge::READY_MESSAGE = "ready!" + PACKET_END;
const string GPSArduinoBridge::START_COMMAND = "g" + PACKET_END;
const string GPSArduinoBridge::STOP_COMMAND = "s" + PACKET_END;
const string GPSArduinoBridge::GPS_MESSAGE_HEADER = "gps";
const string GPSArduinoBridge::MESSAGE_DELIMITER = "\t";


// convert a string to a 64-bit integer
long long string_to_int64(string s) {
	stringstream ss(s);
	long long integer = 0;
	ss >> integer;
	return integer;
}

// translate scalar values into a unix timestamp object
float to_unix_time(int years, int months, int days, int hours, int minutes, int seconds, int milliseconds)
{
	if (years == 0 || months == 0 || days == 0) {
		return 0.0;
	}
	using namespace boost::posix_time;
	static ptime epoch(boost::gregorian::date(1970, 1, 1));
	pt::time_duration t_dur(hours, minutes, seconds, milliseconds * 1000);
	boost::gregorian::date date(years, months, days);
	float secs = (ptime(date, t_dur) - epoch).total_milliseconds() / 1000;
	return secs;
}

GPSArduinoBridge::GPSArduinoBridge(ros::NodeHandle* nodehandle) : nh(*nodehandle)
{
	float _debug_info_delay;

	// pull parameters from the launch file
	nh.param<string>("serial_port", serial_port, "usb-Silicon_Labs_CP2104_USB_to_UART_Bridge_Controller_00FEBA3D-if00-port0");
	nh.param<int>("serial_baud", serial_baud, 9600);
	nh.param<float>("debug_info_delay", _debug_info_delay, 1.0);

	gps_pub = nh.advertise<gps_common::GPSFix>("/AdafruitGPS", 5);
	navsat_pub = nh.advertise<sensor_msgs::NavSatFix>("/GpsNavSat", 5);

	years = 0;
	months = 0;
	days = 0;
	hours = 0;
	minutes = 0;
	seconds = 0;
	milliseconds = 0;

	debug_info_prev_time = ros::Time::now();
	debug_info_delay = ros::Duration(_debug_info_delay);
}


void GPSArduinoBridge::waitForPacket(const string packet)
{
	ros::Time begin = ros::Time::now();
	ros::Duration timeout = ros::Duration(15.0);

	while ((ros::Time::now() - begin) < timeout)
	{
		if (serial_ref.available()) {
			serial_buffer = serial_ref.readline();
			ROS_DEBUG("buffer: %s", serial_buffer.c_str());

			if (serial_buffer.compare(packet) == 0) {
				ROS_INFO("Arduino sent %s!", packet.c_str());
				return;
			}
		}
	}

	throw Error("Timeout reached. Serial buffer didn't contain '%s', buffer: %s", packet.c_str(), serial_buffer.c_str());
}

int GPSArduinoBridge::run()
{
	try
	{
		serial_ref.setPort(serial_port);
		serial_ref.setBaudrate(serial_baud);
		serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
		serial_ref.setTimeout(timeout);
		serial_ref.open();
	}
	catch (serial::IOException e)
	{
		ROS_ERROR_STREAM("Unable to open port");
		ROS_ERROR_STREAM(serial_port);
		return -1;
	}

	waitForPacket(HELLO_MESSAGE);
	waitForPacket(READY_MESSAGE);

	serial_ref.write(START_COMMAND);

	ros::Rate clock_rate(15);  // 15 Hz

	while (ros::ok())
	{
		ros::spinOnce();
		clock_rate.sleep();

		if (serial_ref.available())
		{
			serial_buffer = serial_ref.readline();

			if (serial_buffer.at(0) == '-') {
				ROS_WARN("message: %s", serial_buffer.substr(1).c_str());
				continue;
			}

			ROS_DEBUG("buffer: %s", serial_buffer.c_str());

			// Parse GPS segment
			if (serial_buffer.length() > GPS_MESSAGE_HEADER.size() &&
			    serial_buffer.compare(0, GPS_MESSAGE_HEADER.size(), GPS_MESSAGE_HEADER) == 0) {
				parseGPSMessage();
			}
		}
	}

	serial_ref.write(STOP_COMMAND);

	return 0;
}

void GPSArduinoBridge::parseToken(string token)
{
	// parse differently based on the first character of the segment (e.g. 'ga40.442535' is latitude in degrees)
	switch (token.at(0)) {
		case 't':
			switch (token.at(1)) {
				case 'd': days = STR_TO_INT(token.substr(2)); break;  // days
				case 'b': months = STR_TO_INT(token.substr(2)); break;  // months
				case 'y': years = STR_TO_INT(token.substr(2)); break;  // years
				case 'H': hours = STR_TO_INT(token.substr(2)); break;  // hours
				case 'M': minutes = STR_TO_INT(token.substr(2)); break;  // minutes
				case 'S': seconds = STR_TO_INT(token.substr(2)); break;  // seconds
				case 's': milliseconds = STR_TO_INT(token.substr(2)); break;  // milliseconds
			}
			break;
		case 'f':
			switch (token.at(1)) {
				case 'f':
					if (STR_TO_INT(token.substr(2)) > 0) {
						gps_msg.status.status = gps_msg.status.STATUS_FIX;
						navsat_msg.status.status = navsat_msg.status.STATUS_FIX;
					}
					else {
						gps_msg.status.status = gps_msg.status.STATUS_NO_FIX;
						navsat_msg.status.status = navsat_msg.status.STATUS_NO_FIX;
					}
					break;
				case 'q': ROS_DEBUG("fix quality: %lld", STR_TO_INT(token.substr(2))); break;
			}
			break;
		case 'g':
			switch (token.at(1)) {
				case 'a':
					gps_msg.latitude = STR_TO_FLOAT(token.substr(2));
					navsat_msg.latitude = gps_msg.latitude;
					break;
				case 'o':
					gps_msg.longitude = STR_TO_FLOAT(token.substr(2));
					navsat_msg.longitude = gps_msg.longitude;
					break;
			}
			break;
		case 'x':
			switch (token.at(1)) {
				case 's': gps_msg.speed = STR_TO_FLOAT(token.substr(2)) * 1000; break;  // convert km/s to m/s
				case 'l':
					gps_msg.altitude = STR_TO_FLOAT(token.substr(2));
					navsat_msg.altitude = gps_msg.altitude;
					break;
				case 'm': gps_msg.status.satellites_used = STR_TO_INT(token.substr(2)); break;
				case 'h': gps_msg.hdop = STR_TO_INT(token.substr(2)); break;
			}
			break;
		default:
			ROS_WARN("Invalid segment type! Segment: '%s', packet: '%s'", token.c_str(), serial_buffer.c_str());
			break;
	}
}

void GPSArduinoBridge::parseGPSMessage()
{
	// strip off header and the trailing newline character
	serial_buffer = serial_buffer.substr(GPS_MESSAGE_HEADER.size() + 1, serial_buffer.size() - 1);

	// set message header with frame name and time
	gps_msg.header.frame_id = GPS_FRAME_ID;
	navsat_msg.header.frame_id = GPS_FRAME_ID;
	gps_msg.header.stamp = ros::Time::now();
	navsat_msg.header.stamp = ros::Time::now();

	// loop through the buffer until the end is reached
    // pos is the next MESSAGE_DELIMITER character
	size_t pos = 0;
	string token;
	while ((pos = serial_buffer.find(MESSAGE_DELIMITER)) != string::npos)
	{
		// extract the next segment of data (serial_buffer will be erased up to pos at the end)
		token = serial_buffer.substr(0, pos);
		if (token.size() == 0 || token.compare("nop") == 0) {
			return;
		}

		parseToken(token);

		// erase up to the end of the current token plus the delimiter character
		serial_buffer.erase(0, pos + MESSAGE_DELIMITER.length());
	}

	// parse the rest of the serial buffer except for end character
    token = serial_buffer.substr(0, serial_buffer.length() - 1);
    parseToken(token);

	if (ros::Time::now() - debug_info_prev_time > debug_info_delay) {
        debug_info_prev_time = ros::Time::now();
        ROS_INFO(
			"GPS lat: %0.6f, long: %0.6f, fix: %s",
			navsat_msg.latitude,
			navsat_msg.longitude,
			gps_msg.status.status == gps_msg.status.STATUS_FIX ? "true" : "false");
    }

	// Only publish data if the GPS things the data is valid
	if (gps_msg.status.status == gps_msg.status.STATUS_FIX) {
		gps_msg.time = to_unix_time(years, months, days, hours, minutes, seconds, milliseconds);

		navsat_pub.publish(navsat_msg);
		gps_pub.publish(gps_msg);
	}
}
