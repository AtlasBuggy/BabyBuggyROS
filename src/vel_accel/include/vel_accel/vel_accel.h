#ifndef _VELACCEL_H_
#define _VELACCEL_H_

#include "ros/ros.h"
#include "std_msgs/Float64.h"

using namespace std;

class VelAccel{
private:
  ros::NodeHandle nh;

  ros::Subscriber enc_sub;
  ros::Publisher vel_pub;
  ros::Publisher accel_pub;

  float prev_enc;
  float prev_vel;
  float prev_time;

  float velocity;
  float acceleration;

  void EncoderCallback(const std_msgs::Float64& msg);
public:
  VelAccel(ros::NodeHandle* nh);
};

#endif