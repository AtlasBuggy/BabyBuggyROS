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
    ros::NodeHandle nh;

    ros::Subscriber gps_sub;
    ros::Subscriber imu_sub;
    ros::Subscriber enc_sub;

    ros::Publisher odom_pub;

    ros::ServiceClient client;
    robot_localization::SetDatum srv;
    nav_msgs::Odometry odom_msg;

    tf::TransformBroadcaster tf_broadcaster;

    bool datum_set;

    float odom_x, odom_y;
    double roll, pitch, yaw;
    float banked_dist;
    tf::Quaternion robot_imu_orientation;
    tf::Quaternion robot_gps_orientation;

    tf::Quaternion current_imu_orientation;

    tf::Transform static_trans_imu;
    tf::Transform static_trans_gps;
    tf::Transform static_trans_laser;

    tf::Transform odometry_transform;

    bool enc_data_received, imu_data_received;

    void GPSCallback(const sensor_msgs::NavSatFix& msg);
    void IMUCallback(const sensor_msgs::Imu& msg);
    void EncoderCallback(const std_msgs::Float64& msg);

    void run();

public:
    RobotTFs(ros::NodeHandle* nodehandle);

    static const string BASE_LINK_FRAME_NAME;
    static const string ODOM_FRAME_NAME;
    static const string LASER_FRAME_NAME;
    static const string IMU_FRAME_NAME;
    static const string GPS_FRAME_NAME;

    //distance from imu to laser, meters, roughly measured with a tape measure
    static const float IMU_LASER_X;
    static const float IMU_LASER_Y;
    static const float IMU_LASER_Z;

    static const float GPS_LASER_X;
    static const float GPS_LASER_Y;
    static const float GPS_LASER_Z;

    ros::Time debug_info_prev_time;
    static const ros::Duration DEBUG_INFO_DELAY;
};

#endif // _ROBOT_TFS_H_
