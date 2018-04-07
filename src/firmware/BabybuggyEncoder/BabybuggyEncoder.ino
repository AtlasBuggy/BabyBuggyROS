
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
    if (encoder_read < prev_ticks) {  // if int overflow occurred
        encoder1.write(0);
        encoder_read = encoder1.read();
    }
    enc_msg.data = encoder_read;
    prev_ticks = encoder_read;

    if (encoder_read - prev_ticks > 0) {
        encoder.publish(&enc_msg);
    }
    nh.spinOnce();
    delay(1);
}
