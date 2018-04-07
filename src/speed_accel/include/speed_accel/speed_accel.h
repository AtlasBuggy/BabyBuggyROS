#ifndef _SPEED_ACCEL_H_
#define _SPEED_ACCEL_H_

#include "ros/ros.h"
#include "std_msgs/Float64.h"

using namespace std;

class SpeedAccel
{
private:
    ros::NodeHandle nh;

    ros::Subscriber enc_sub;
    ros::Publisher speed_pub;
    ros::Publisher accel_pub;

    float prev_speed;
    float prev_time;

    float speed;
    float acceleration;

    void EncoderCallback(const std_msgs::Float64& msg);
public:
    SpeedAccel(ros::NodeHandle* nh);
};

#endif  // _SPEED_ACCEL_H_
