#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/PoseStamped.h"

ros::Publisher imu_pub;
ros::Publisher imu_pose_pub;
ros::Subscriber imu_sub;

void IMUCallback(const sensor_msgs::Imu& msg)
{
  sensor_msgs::Imu new_msg;

  new_msg.header.frame_id = "base_link";
  new_msg.header.stamp = ros::Time::now();
  new_msg.orientation = msg.orientation;
  new_msg.orientation_covariance = msg.orientation_covariance;
  new_msg.angular_velocity = msg.angular_velocity;
  new_msg.angular_velocity_covariance = msg.angular_velocity_covariance;
  new_msg.linear_acceleration = msg.linear_acceleration;
  new_msg.linear_acceleration_covariance = msg.linear_acceleration_covariance;

  geometry_msgs::PoseStamped pose_msg;

  pose_msg.header.frame_id = "imu";
  pose_msg.header.stamp = ros::Time::now();
  pose_msg.pose.position.x = 0;
  pose_msg.pose.position.y = 0;
  pose_msg.pose.position.z = 0;
  pose_msg.pose.orientation = msg.orientation;

  imu_pose_pub.publish(pose_msg);
  imu_pub.publish(new_msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;

  imu_sub = n.subscribe("/BNO055", 1000, IMUCallback);
  imu_pub = n.advertise<sensor_msgs::Imu>("/BNO055_trans", 1000);
  imu_pose_pub = n.advertise<geometry_msgs::PoseStamped>("/IMU_pose", 1000);

  ros::spin();
  return 0;
}

