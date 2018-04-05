#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <stdio.h>
#include <math.h>

#define LEFT 2048
#define RIGHT 4096
#define CENTER 400
#define STOP 1500

int steer_instruction = STOP;

void joyCallback(const sensor_msgs::Joy& joy_msg) {
    if (joy_msg.buttons[3] == 1) {
        steer_instruction = CENTER;
    } else if (joy_msg.buttons[4] == joy_msg.buttons[5]) {
    steer_instruction = STOP;
  } else if (joy_msg.buttons[4] > joy_msg.buttons[5]) {
    steer_instruction = LEFT;
  } else {
    steer_instruction = RIGHT;
  }
}

bool in_progress = false;
double start_time:
double change_time;
double end_time;

void perform(double instruction) {
    if (!in_progress) {
      start_time = ros::Time::now().toSec();
      change_time = ros::Time::now().toSec() + (instruction / 2);
      end_time = ros::Time::now().toSec() + instruction;
      in_progress = true;
      steer_instruction = RIGHT;
    } else {
      if (ros::Time::now().toSec() >= end_time) {
        in_progress = false;
        steer_instruction = STOP;
      } else if (ros::Time::now().toSec() >= change_time) {
        steer_instruction = CENTER;
      }
    }
}


int main(int argc, char** argv) {

    double instruction = 5.0;
    ros::init(argc, argv, "steering");

    ros::NodeHandle node;
    ros::Publisher steerPub = node.advertise<std_msgs::Int32>("/steering", 10);

    ros::Rate loop_rate(10);

    std_msgs::Int32 steer_msg;

    while (node.ok())
    {
      perform(instruction);
      steer_msg.data = steer_instruction;
      steerPub.publish(steer_msg);
      ros::spinOnce();
      loop_rate.sleep();
    }
    return 0;
}