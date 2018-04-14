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
    ros::NodeHandle nh;  // ROS node handle

    // IMU data publisher and data message
    ros::Publisher imu_pub;
    sensor_msgs::Imu imu_msg;

    // Serial connection variables
    string serial_port;
    int serial_baud;
    string serial_buffer;
    serial::Serial serial_ref;

    // checks if data was received. Prevents publishing of uninitialized data
    bool euler_data_received;
    bool gyro_data_received;
    bool linaccel_data_received;
    bool quat_data_received;

    // Offset the starting value of the IMU if it's not 0
    double initial_euler_roll;
    double initial_euler_pitch;
    double initial_euler_yaw;

    // The current scalar values of orientation reported by the IMU
    double euler_roll;
    double euler_pitch;
    double euler_yaw;

    // Orientation reported by the IMU in the previous check.
    // For checking if the IMU skipped suddenly indicating an error.
    double prev_euler_roll;
    double prev_euler_pitch;
    double prev_euler_yaw;

    // Current status reported by the IMU. Data should be ignored if less than 1
    int system_status;
    int accel_status;
    int gyro_status;
    int mag_status;

    // IMU status in the previous check. For printing to the console if the status changed
    int prev_system_status;
    int prev_accel_status;
    int prev_gyro_status;
    int prev_mag_status;

    // How often to report the IMU's status to the console or log
    ros::Time debug_info_prev_time;
    ros::Duration debug_info_delay;

    // Wait for the packet header specified with a timeout
    void waitForPacket(const string packet);

    // Helper methods for parsing the IMU's data
    void eulerToQuat(sensor_msgs::Imu &imu_msg, float roll, float pitch, float yaw);
    void parseToken(string token);
    void parseImuMessage();

public:
    Bno055ArduinoBridge(ros::NodeHandle* nodehandle);

    // The frame name of the IMU. Links to the base_link frame
    static const string IMU_FRAME_ID;
    static const string CHILD_FRAME_ID;

    // Important constants for the node's initialization
    static const string NODE_NAME;
    static const string PACKET_END;  // character that every packet ends with
    static const string HELLO_MESSAGE;  // the message to expect when the microcontroller starts up
    static const string READY_MESSAGE;  // message signalling that the microcontroller is ready to receive commands
    static const string START_COMMAND;  // packet to send to the microcontroller to tell it to start
    static const string STOP_COMMAND;  // packet to send to the microcontroller to tell it to stop

    // IMU data packet properties
    static const string IMU_MESSAGE_HEADER;  // the string that all imu data packets start with
    static const string MESSAGE_DELIMITER;  // IMU data segments are separated by this character

    static const float JUMP_WARN_THRESHOLD;  // if the IMU's euler angles jump more than this value between checks, warn the user

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

#endif // _BNO055_ARDUINO_BRIDGE_H_
