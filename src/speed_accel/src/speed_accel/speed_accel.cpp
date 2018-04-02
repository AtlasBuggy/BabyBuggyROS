#include "speed_accel/speed_accel.h"

SpeedAccel::SpeedAccel(ros::NodeHandle* nodehandle) : nh(*nodehandle)
{
    enc_sub = nh.subscribe("/encoder", 1000, &SpeedAccel::EncoderCallback, this);
    speed_pub = nh.advertise<std_msgs::Float64>("robot_speed", 1000);
    accel_pub = nh.advertise<std_msgs::Float64>("robot_accel", 1000);

    prev_enc = -1;
    prev_speed = 10000;
    ros::spin();
}

void SpeedAccel::EncoderCallback(const std_msgs::Float64& msg)
{
    if (prev_enc == -1)
    {
        prev_enc = msg.data;
        prev_time = ros::Time::now().toSec();
    }
    else
    {
        float cur_time = ros::Time::now().toSec();
        speed = (msg.data - prev_enc)/(1000*(cur_time - prev_time));

        // publish speed
        std_msgs::Float64 speed_msg;
        speed_msg.data = speed;
        speed_pub.publish(speed_msg);

        if (prev_speed != 10000){
            acceleration = (speed - prev_speed)/(cur_time - prev_time);

            // publish acceleration
            std_msgs::Float64 accel_msg;
            accel_msg.data = acceleration;
            accel_pub.publish(accel_msg);
        }

        prev_time = cur_time;
        prev_speed = speed;
        prev_enc = msg.data;
    }
}
