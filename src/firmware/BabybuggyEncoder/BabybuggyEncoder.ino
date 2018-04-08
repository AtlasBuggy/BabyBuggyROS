//Publishes ROS Encoder messages
#include <ros.h>
#include <std_msgs/Int64.h>
#include <Encoder.h>

Encoder encoder1(4, 3);  // reverse encoder direction

ros::NodeHandle nh;

int64_t prev_ticks = -1;
int64_t encoder_read = 0;

uint64_t prev_time = millis();

std_msgs::Int64 enc_msg;
ros::Publisher encoder("encoder", &enc_msg);

void setup() {
    nh.initNode();
    nh.advertise(encoder);
}

void loop() {
    encoder_read = encoder1.read();
    if (prev_time > millis()) {
        prev_time = millis();
    }
    if (encoder_read != prev_ticks || millis() - prev_time > 1000) {
        prev_time = millis();
        enc_msg.data = encoder_read - prev_ticks;
        prev_ticks = encoder_read;

        encoder.publish(&enc_msg);
    }
    nh.spinOnce();
    delay(1);
}
