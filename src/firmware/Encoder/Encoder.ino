
//Publishes ROS Encoder messages
#include <ros.h>
#include <std_msgs/Int64.h>
#include <Encoder.h>

Encoder encoder1(3, 4);


ros::NodeHandle nh;

// const float pi = 3.14159265358979323846;
// const float wheel_radius = 30.825;
// const int ticks_per_rotation = 256;
const String publish_stream = "encoder";

int prev_ticks = 0;
int encoder_read = 0;

std_msgs::Int64 enc_msg;
ros::Publisher encoder("encoder", &enc_msg);

void setup() {
    nh.initNode();
    nh.advertise(encoder);
}

void loop() {
    encoder_read = encoder1.read();
    enc_msg.data = encoder_read - prev_ticks;
    prev_ticks = encoder_read;

    encoder.publish(&enc_msg);
    nh.spinOnce();
}

// //Publishes ROS Encoder messages
// #include <ros.h>
// #include <std_msgs/Float64.h>
// #include <Encoder.h>
//
// Encoder encoder1(3, 4);
//
//
// ros::NodeHandle nh;
//
// const float pi = 3.14159265358979323846;
// const float wheel_radius = 30.825;
// const int ticks_per_rotation = 256;
// const String publish_stream = "encoder";
//
// int prev_ticks = 0;
// float tick_distance;
//
// std_msgs::Float64 flt_msg;
// ros::Publisher encoder("encoder", &flt_msg);
//
// void setup() {
//     tick_distance =  2 * wheel_radius * pi / ticks_per_rotation;
//     nh.initNode();
//     nh.advertise(encoder);
// }
//
// void loop() {
//     int encoder_read = encoder1.read();
//     int ticks = encoder_read - prev_ticks;
//
//     flt_msg.data = -(ticks * tick_distance);  // flip the encoder so forward is positive
//     encoder.publish(&flt_msg);
//     nh.spinOnce();
//
//     prev_ticks = encoder_read;
// }
