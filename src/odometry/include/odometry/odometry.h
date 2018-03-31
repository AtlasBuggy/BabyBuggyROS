#ifndef _ODOMETRY_H_
#define _ODOMETRY_H_

#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/Float64.h"
#include "nav_msgs/Odometry.h"
#include "robot_localization/SetDatum.h"
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

using namespace std;

class Odometry{
private:
  ros::NodeHandle nh;

  ros::Subscriber gps_sub;
  ros::Subscriber imu_sub;
  ros::Subscriber enc_sub;
  ros::Publisher odom_pub;
  ros::Publisher gps_pub;
  ros::Publisher imu_pub;

  ros::ServiceClient client;
  robot_localization::SetDatum srv;
  nav_msgs::Odometry odom_msg;

  bool datum_set;

  float odom_x, odom_y;
  double roll, pitch, yaw;
  float banked_dist;
  tf::Quaternion q;

  bool enc_data_received, imu_data_received;

  void GPSCallback(const sensor_msgs::NavSatFix& msg);
  void IMUCallback(const sensor_msgs::Imu& msg);
  void EncoderCallback(const std_msgs::Float64& msg);

public:
  Odometry(ros::NodeHandle* nodehandle);

  static const string ODOM_NODE_NAME;
  static const string ODOM_FRAME_ID;
  static const string ODOM_CHILD_FRAME_ID;
};

#endif // _ODOMETRY_H_