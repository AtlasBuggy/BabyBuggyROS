#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int32.h>
#include <stdio.h>
#include <math.h>

// Designed exclusively for Logitech Gamepad F310

int steer_instruction = 1500;

void joyCallback(const sensor_msgs::Joy& joy_msg) {

}


int main(int argc, char** argv) {

    ros::init(argc, argv, "controller");

    ros::NodeHandle node;
    ros::Subscriber joySub = node.subscribe("/joy", 10, &joyCallback);

    ros::Publisher steerPub = node.advertise<std_msgs::Int32>("/steering", 10);

    std_msgs::Int32 steer_msg;

    while (node.ok())
    {
		steer_msg.data = steer_instruction;

    	steerPub.publish(steer_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
}