#ifndef _NABORIS_ARDUINO_BRIDGE_H_
#define _NABORIS_ARDUINO_BRIDGE_H_

#include "ros/ros.h"
#include "std_msgs/Int64.h"
#include "std_msgs/Int16MultiArray.h"
#include "sensor_msgs/Imu.h"
#include "serial/serial.h"
#include <math.h>
#include <iostream>
#include <sstream>

using namespace std;

class NaborisArduinoBridge {
private:
    ros::NodeHandle nh;

    ros::Publisher imu_pub;
    ros::Publisher left_encoder_pub;
    ros::Publisher right_encoder_pub;

    ros::Subscriber motor_command_sub;

    string serial_port;
    int serial_baud;
    string serial_buffer;
    serial::Serial serial_ref;
    void waitForPacket(const string packet);

    float euler_roll;
    float euler_pitch;
    float euler_yaw;

    sensor_msgs::Imu imu_msg;
    void eulerToQuat(sensor_msgs::Imu &imu_msg, float roll, float pitch, float yaw);
    void parseImuMessage();

    std_msgs::Int64 right_encoder_msg;
    std_msgs::Int64 left_encoder_msg;
    void parseEncoderMessage();

    void motor_command_callback(const std_msgs::Int16MultiArray& motor_commands);

public:
    NaborisArduinoBridge(ros::NodeHandle* nodehandle);

    static const string IMU_FRAME_ID;
    static const string CHILD_FRAME_ID;

    static const string NODE_NAME;
    static const string PACKET_END;
    static const string HELLO_MESSAGE;
    static const string READY_MESSAGE;
    static const string START_COMMAND;
    static const string STOP_COMMAND;

    static const string IMU_MESSAGE_HEADER;
    static const string MESSAGE_DELIMITER;

    static const string ENCODER_MESSAGE_HEADER;

    static const size_t MOTOR_COMMAND_MESSAGE_LEN;

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

    char const* what() const throw() { return text; }
};

#endif // _NABORIS_ARDUINO_BRIDGE_H_
