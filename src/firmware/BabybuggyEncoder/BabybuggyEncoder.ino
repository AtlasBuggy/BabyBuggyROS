//Publishes ROS Encoder messages
#include <ros.h>
#include <std_msgs/Int64.h>
#include <Encoder.h>

Encoder encoder1(3, 4);

ros::NodeHandle nh;

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
    if (abs(encoder_read - prev_ticks) > 0x100000) {  // if int overflow occurred
        encoder1.write(0);
        prev_ticks = 0;
        encoder_read = 0;
    }
    enc_msg.data = encoder_read - prev_ticks;
    prev_ticks = encoder_read;

    encoder.publish(&enc_msg);
    nh.spinOnce();
}
