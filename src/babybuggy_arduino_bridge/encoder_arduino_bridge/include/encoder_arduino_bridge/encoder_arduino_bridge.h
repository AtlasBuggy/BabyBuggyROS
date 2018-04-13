#ifndef _ENCODER_ARDUINO_BRIDGE_H_
#define _ENCODER_ARDUINO_BRIDGE_H_

#include "ros/ros.h"
#include "std_msgs/Int64.h"
#include "serial/serial.h"
#include <math.h>
#include <iostream>
#include <sstream>

using namespace std;

class EncoderArduinoBridge {
private:
    ros::NodeHandle nh;  // ROS node handle

    // Encoder data publishers and data messages
    ros::Publisher enc1_pub;
    ros::Publisher enc2_pub;
    std_msgs::Int64 enc1_msg;
    std_msgs::Int64 enc2_msg;

    // Serial connection variables
    string serial_port;
    int serial_baud;
    string serial_buffer;
    serial::Serial serial_ref;

    // Wait for the packet header specified with a timeout
    void waitForPacket(const string packet);

    // Helper methods for parsing the encoder's data
    void parseEncoderMessage();
    int64_t parseSegmentedInt64(string s);

public:
    EncoderArduinoBridge(ros::NodeHandle* nodehandle);

    // Important constants for the node's initialization
    static const string NODE_NAME;
    static const string PACKET_END;  // character that every packet ends with
    static const string HELLO_MESSAGE;  // the message to expect when the microcontroller starts up
    static const string READY_MESSAGE;  // message signalling that the microcontroller is ready to receive commands
    static const string START_COMMAND;  // packet to send to the microcontroller to tell it to start
    static const string STOP_COMMAND;  // packet to send to the microcontroller to tell it to stop

    // encoder data packet properties
    static const string ENCODER_MESSAGE_HEADER;  // the string that all encoder data packets start with
    static const string MESSAGE_DELIMITER;  // encoder data segments are separated by this character
    static const string INT64_SEGMENT_DELIMITER;

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

    char const* what() const throw() { return text; }
};

#endif // _ENCODER_ARDUINO_BRIDGE_H_
