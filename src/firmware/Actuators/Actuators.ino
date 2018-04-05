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
#define SERVO_PIN 3

DualVNH5019MotorShield md;
ros::NodeHandle nh;
int steering_requested_value = 400;
int braking_requested_value = 0;
int current_brake_value = 0;

Servo brake_servo;

void steer(const std_msgs::Int32& msg){
    steering_requested_value = msg.data;
}

void set_brakes(const std_msgs::Int32& msg)
{
    braking_requested_value = (int)msg.data;
    if (braking_requested_value < 0) {
        braking_requested_value = 0;
    }
    if (braking_requested_value > 180) {
        braking_requested_value = 180;
    }
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
    md.init();
    nh.initNode();
    nh.subscribe(steeringSub);
    nh.subscribe(brakeSub);

    pinMode(POT_PIN, INPUT);
    brake_servo.attach(SERVO_PIN);
}

void loop(){
    // absolute steering
    if (steering_requested_value <= 1023)
    {
        int diff = steering_requested_value - analogRead(POT_PIN);

        if (abs(diff) > 5) {
            if (diff > 0) {
                turn_right();
            }
            else {
                turn_left();
            }
        }
        else {
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
        }
        else if (steering_requested_value == 4096) {
            turn_right();
        }
        else {
            stop_motor();
        }
    }

    if (braking_requested_value != current_brake_value) {
        current_brake_value = braking_requested_value;
        brake_servo.write(braking_requested_value);
    }

    delay(10);
    nh.spinOnce();
}
