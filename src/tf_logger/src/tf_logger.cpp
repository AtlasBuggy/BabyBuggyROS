#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
#include <stdio.h>
#include <math.h>

tf::Quaternion q;
tf::Transform static_trans;
tf::Transform trans;

//transform data
float tr_x = 0; //m, overall motion in x plane
float tr_y = 0; //m, overall motion in y plane
float dt = 0; //seconds, change in time
float prev_t = 0; //seconds, previous time 
float prev_theta = 0; //radians, keeps track of previous angle
float theta = 0; //radians, calculates current angle
float banked_dist = 0.0; //m, banks distance

//distance from imu to laser
const float imu_laser_x = -0.1016; //m, roughly measured with a tape measure
const float imu_laser_y = 0.0762; //m, roughly measured with a tape measure

void scanCallback(const sensor_msgs::LaserScan& msg){
  static tf::TransformBroadcaster br;

  trans.setOrigin(tf::Vector3(tr_x, tr_y, 0.0));
  trans.setRotation(q);

  br.sendTransform(tf::StampedTransform(trans, ros::Time::now(), "laser", "base_link"));
  br.sendTransform(tf::StampedTransform(static_trans, ros::Time::now(), "base_link", "odom"));
}

void imuCallback(const sensor_msgs::Imu& imu_msg) {
  q = tf::Quaternion(imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z, imu_msg.orientation.w);
  float dtheta_z = imu_msg.angular_velocity.z;
  dt = ros::Time::now().toSec() - prev_t;
  theta += dtheta_z * dt;
  float mid_theta = (theta - prev_theta) / 2;

  tr_x += cos(mid_theta) * banked_dist;
  tr_y += sin(mid_theta) * banked_dist;
  banked_dist = 0;

  prev_t = ros::Time::now().toSec();
  prev_theta = theta;
}

void encoderCallback(const std_msgs::Float64& flt_msg) {
  banked_dist += flt_msg.data;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "tf_logger");

  q.setRPY(0, 0, 0);
  static_trans.setOrigin(tf::Vector3(imu_laser_x, imu_laser_y, 0.0));
  static_trans.setRotation(q);

  ros::NodeHandle node;
  ros::Subscriber scanSub = node.subscribe("/scan", 10, &scanCallback);
  ros::Subscriber encoderSub = node.subscribe("/encoder", 10, &encoderCallback);
  ros::Subscriber imuSub = node.subscribe("/BNO055", 10, &imuCallback);
  prev_t = ros::Time::now().toSec();

  ros::spin();
  return 0;
}