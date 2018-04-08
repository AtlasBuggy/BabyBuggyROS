#ifndef _ERROR_CHECKER_H_
#define _ERROR_CHECKER_H_

#include <math.h>
#include "ros/ros.h"
#include <tf/transform_datatypes.h>
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/Int64.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/Odometry.h"

using namespace std;

void check_statement(bool statement, const char* error_message_format, ...) {
    if (!statement)
    {
        va_list argptr;
        va_start(argptr, error_message_format);
        vfprintf(stderr, error_message_format, argptr);
        va_end(argptr);
        throw std::runtime_error(error_message_format);
    }
}

class ErrorChecker {
private:
    ros::NodeHandle nh;  // ROS node handle

    // Data subscribers
    ros::Subscriber gps_sub;
    ros::Subscriber imu_sub;
    ros::Subscriber enc_sub;
    ros::Subscriber odom_sub;
    ros::Subscriber lidar_sub;
    ros::Subscriber bearing_sub;

    void GPSCallback(const sensor_msgs::NavSatFix& msg);
    void IMUCallback(const sensor_msgs::Imu& msg);
    void EncoderCallback(const std_msgs::Int64& msg);
    void OdomCallback(const nav_msgs::Odometry& msg);
    void LidarCallback(const sensor_msgs::LaserScan& msg);
    void BearingCallback(const geometry_msgs::PoseWithCovarianceStamped& msg);

    sensor_msgs::NavSatFix gps_msg;
    sensor_msgs::Imu imu_msg;
    nav_msgs::Odometry odom_msg;
    sensor_msgs::LaserScan laser_msg;
    geometry_msgs::PoseWithCovarianceStamped bearing_msg;

    ros::Time start_time;

    double yaw, pitch, roll;
    bool gps_status;
    double latitude, longitude;
    double bearing;
    int64_t encoder_ticks, largest_tick, smallest_tick;
    ros::Time enc_current_time;

public:
    ErrorChecker(ros::NodeHandle* nodehandle);

    int run();
};

#endif // _ERROR_CHECKER_H_
