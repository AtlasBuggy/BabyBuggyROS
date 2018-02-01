//accepts int32 messages [0,1023] representing how far to steer, and actuates them
#include <ros.h>
#include <std_msgs/Int32.h>

#define motorPin1 D5
#define motorPin2 D4

#define potPin A0

ros::NodeHandle nh;
std_msgs::Int32 int_msg;

void steer(const std_msgs::Int32::ConstPtr& msg){
  int requestVal = msg.data;
  int currVal = analogRead(analogPin);
  while(((int diff=requestVal-currVal)*diff)<25){
    if(diff>0){
      digitalWrite(motorPin1,1);
      digitalWrite(motorPin2,0);
    }else{
      digitalWrite(motorPin1,0);
      digitalWrite(motorPin2,1);
    }
    delay(10);
  }
}

void setup() {
  pinMode(potPin, INPUT);
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);

  nh.initNode();
  ros::Subscriber steeringWheel = nh.subscribe("chatter", 10, steer);
  ros::spin();
}

void loop(){}
