#ifndef _BNO055_ARDUINO_BRIDGE_H_
#define _BNO055_ARDUINO_BRIDGE_H_

#include "ros/ros.h"
#include "std_msgs/Int64.h"
#include "std_msgs/Int16MultiArray.h"
#include "sensor_msgs/Imu.h"
#include "serial/serial.h"
#include <math.h>
#include <iostream>
#include <sstream>

using namespace std;

class Bno055ArduinoBridge {
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

    bool euler_data_received;
    bool gyro_data_received;
    bool linaccel_data_received;
    bool quat_data_received;

    float euler_roll;
    float euler_pitch;
    float euler_yaw;

    float prev_euler_roll;
    float prev_euler_pitch;
    float prev_euler_yaw;

    int system_status;
    int accel_status;
    int gyro_status;
    int mag_status;

    int prev_system_status;
    int prev_accel_status;
    int prev_gyro_status;
    int prev_mag_status;

    ros::Time debug_info_prev_time;
    ros::Duration debug_info_delay;

    sensor_msgs::Imu imu_msg;
    void eulerToQuat(sensor_msgs::Imu &imu_msg, float roll, float pitch, float yaw);
    void parseImuMessage();

    std_msgs::Int64 right_encoder_msg;
    std_msgs::Int64 left_encoder_msg;
    void parseEncoderMessage();

    void motor_command_callback(const std_msgs::Int16MultiArray& motor_commands);

public:
    Bno055ArduinoBridge(ros::NodeHandle* nodehandle);

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

    char const* what() const throw() { return text; }
};

#endif // _BNO055_ARDUINO_BRIDGE_H_
