//accepts int32 messages [0,1023] representing how far to steer, and actuates them
// 400 is middle
// inputs 0 - 1023 for absolute steering
// 2048 left for relative steering
// 4096 right

#include <ros.h>
#include <Servo.h>
#include <std_msgs/Int32.h>
#include "DualVNH5019MotorShield.h"

#define POT_PIN A2

DualVNH5019MotorShield md;
ros::NodeHandle nh;
int steering_requested_value = 400;

Servo brake_servo;

void steer(const std_msgs::Int32& msg){
    steering_requested_value = msg.data;
}

void set_brakes(const std_msgs::Int32& msg)
{
    int brake_value = msg.data;
    if (brake_value < 0) {
        brake_value = 0;
    }
    if (brake_value > 180) {
        brake_value = 180;
    }
    brake_servo.write(brake_value);
}

ros::Subscriber<std_msgs::Int32> steeringSub("steering", steer);
ros::Subscriber<std_msgs::Int32> brakeSub("braking", set_brakes);

void turn_left() {
    md.setM1Speed(400);
}

void turn_right() {
    md.setM1Speed(-400);
}

void stop_motor(){
    md.setM1Speed(0);
}

void setup() {
    pinMode(POT_PIN, INPUT);
    brake_servo.attach(3);

    md.init();
    nh.initNode();
    nh.subscribe(steeringSub);
    nh.subscribe(brakeSub);
}

void loop(){
    // absolute steering
    if (steering_requested_value <= 1023) {
        int diff = steering_requested_value - analogRead(POT_PIN);

        if(abs(diff) > 5){
            if(diff > 0){
                turn_right();
            }else{
                turn_left();
            }
        }
        else{
            stop_motor();
        }
    }

    else if (steering_requested_value == 1500){
        stop_motor();
    }

    else {
        // relative steering
        if (steering_requested_value == 2048) {
            turn_left();
        } else if (steering_requested_value == 4096) {
            turn_right();
        }
        else {
            stop_motor();
        }
    }

    delay(10);
    nh.spinOnce();
}
