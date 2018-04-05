#include "vel_accel/vel_accel.h"

VelAccel::VelAccel(ros::NodeHandle* nodehandle) : nh(*nodehandle)
{
  enc_sub = nh.subscribe("/encoder", 1000, &VelAccel::EncoderCallback, this);
  vel_pub = nh.advertise<std_msgs::Float64>("robot_vel", 1000);
  accel_pub = nh.advertise<std_msgs::Float64>("robot_accel", 1000);

  prev_enc = -1;
  prev_vel = 10000;
  ros::spin();
}

void VelAccel::EncoderCallback(const std_msgs::Float64& msg)
{
  if (prev_enc == -1){
    prev_enc = msg.data;
    prev_time = ros::Time::now().toSec();
  }
  else{
    float cur_time = ros::Time::now().toSec();
    velocity = (msg.data - prev_enc)/(1000*(cur_time - prev_time));

    // publish velocity
    std_msgs::Float64 vel_msg;
    vel_msg.data = velocity;
    vel_pub.publish(vel_msg);

    if (prev_vel != 10000){
      acceleration = (velocity - prev_vel)/(cur_time - prev_time);

      // publish acceleration
      std_msgs::Float64 accel_msg;
      accel_msg.data = acceleration;
      accel_pub.publish(accel_msg);
    }

    prev_time = cur_time;
    prev_vel = velocity;
    prev_enc = msg.data;
  }
}