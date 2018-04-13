#ifndef _BABYBUGGY_ODOMETRY_H_
#define _BABYBUGGY_ODOMETRY_H_

#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/Int64.h"
#include "nav_msgs/Odometry.h"
#include "robot_localization/SetDatum.h"
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <math.h>

using namespace std;

class BabybuggyOdometry {
private:
    ros::NodeHandle nh;  // ROS node handle

    // Data subscribers
    ros::Subscriber gps_sub;
    ros::Subscriber imu_sub;
    ros::Subscriber enc1_sub;
    ros::Subscriber enc2_sub;

    // This node produces naive odometry measurements (odometry with drift over time)
    ros::Publisher odom_pub;

    // republishes IMU messages with covariances
    ros::Publisher imu_pub;

    // publish sensor_msgs NavSatFix messages
    ros::Publisher navsat_pub;

    // publish gps bearing messages
    ros::Publisher bearing_pub;

    nav_msgs::Odometry odom_msg;
    sensor_msgs::Imu imu_msg;
    sensor_msgs::NavSatFix gps_covariance_msg;  // used only to store gps covariances
    double bearing_covariance;  // only important value is the yaw covariance
    geometry_msgs::PoseWithCovarianceStamped bearing_msg;  // used for storing GPS bearing

    vector<double> bearing_vector;  // store previous bearing values for averaging

    // This node also feeds into robot_localization. When GPS data is received, set the datum
    ros::ServiceClient client;
    robot_localization::SetDatum srv;
    bool datum_set;

    // Broadcasts the robot's transforms of different sensors and reference frames
    // tf::TransformBroadcaster tf_broadcaster;

    // Current scalar and vector values for measurements of the robot
    double odom_x, odom_y, odom_yaw;
    double roll, pitch, yaw;
    ros::Time prev_time;
    int64_t prev_encoder1_ticks, encoder1_ticks;
    int64_t prev_encoder2_ticks, encoder2_ticks;
    tf::Quaternion current_imu_orientation;
    bool enc_data_received, imu_data_received;
    double initial_compass_yaw_deg;

    double enc_ticks_to_m;
    double wheel_encoder_radius;
    double wheel_encoder_dist;
    int ticks_per_rotation;

    // transformation from the robot's frame to the odom frame
    tf::Transform odometry_transform;

    void GPSCallback(const sensor_msgs::NavSatFix& msg);
    void IMUCallback(const sensor_msgs::Imu& msg);
    void Encoder1Callback(const std_msgs::Int64& msg);
    void Encoder2Callback(const std_msgs::Int64& msg);

    double calculateBearing(sensor_msgs::NavSatFix currentMsg, sensor_msgs::NavSatFix prevMsg);

public:
    BabybuggyOdometry(ros::NodeHandle* nodehandle);

    // Names of various frames of the robot
    static const string BASE_LINK_FRAME_NAME;
    static const string ODOM_FRAME_NAME;
    static const string ENC_FRAME_NAME;
    static const string LASER_FRAME_NAME;
    static const string IMU_FRAME_NAME;
    static const string GPS_FRAME_NAME;

    //distance from imu to laser, meters, roughly measured with a tape measure
    static const float IMU_LASER_X;
    static const float IMU_LASER_Y;
    static const float IMU_LASER_Z;

    //distance from gps to laser, meters, roughly measured with a tape measure
    static const float GPS_LASER_X;
    static const float GPS_LASER_Y;
    static const float GPS_LASER_Z;

    static const size_t NUM_ROWS_ODOM_COVARIANCE;
    static const size_t NUM_ROWS_GPS_COVARIANCE;
    static const size_t LEN_IMU_COVARIANCE;

    static const size_t BEARING_ROLL_AVERAGE_SIZE;

    int run();
};

#endif // _BABYBUGGY_ODOMETRY_H_
