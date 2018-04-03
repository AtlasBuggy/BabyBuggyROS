#ifndef _ROBOT_TFS_H_
#define _ROBOT_TFS_H_

#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/Float64.h"
#include "nav_msgs/Odometry.h"
#include "robot_localization/SetDatum.h"
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

using namespace std;

class RobotTFs{
private:
    ros::NodeHandle nh;  // ROS node handle

    // Data subscribers
    ros::Subscriber gps_sub;
    ros::Subscriber imu_sub;
    ros::Subscriber enc_sub;

    // This node produces naive odometry measurements (odometry with drift over time)
    ros::Publisher odom_pub;

    // publish sensor_msgs NavSatFix messages
    // ros::Publisher navsat_pub;

    nav_msgs::Odometry odom_msg;

    // This node also feeds into robot_localization. When GPS data is received, set the datum
    ros::ServiceClient client;
    robot_localization::SetDatum srv;
    bool datum_set;

    // Broadcasts the robot's transforms of different sensors and reference frames
    tf::TransformBroadcaster tf_broadcaster;

    // Current scalar and vector values for measurements of the robot
    double odom_x, odom_y;
    double roll, pitch, yaw;
    double banked_dist;  // distance relative to the last measurement
    tf::Quaternion current_imu_orientation;
    bool enc_data_received, imu_data_received;

    // transformation from the robot's frame to the odom frame
    tf::Transform odometry_transform;

    void GPSCallback(const sensor_msgs::NavSatFix& msg);
    void IMUCallback(const sensor_msgs::Imu& msg);
    void EncoderCallback(const std_msgs::Float64& msg);

    void run();

public:
    RobotTFs(ros::NodeHandle* nodehandle);

    // Names of various frames of the robot
    static const string BASE_LINK_FRAME_NAME;
    static const string ODOM_FRAME_NAME;
    // static const string LASER_FRAME_NAME;
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

    // How often to report this node's status to the console or log
    ros::Time debug_info_prev_time;
    static const ros::Duration DEBUG_INFO_DELAY;
};

#endif // _ROBOT_TFS_H_
