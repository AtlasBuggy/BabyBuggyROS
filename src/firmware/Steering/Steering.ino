//accepts int32 messages [0,1023] representing how far to steer, and actuates them
// 400 is middle
// inputs 0 - 1023 for absolute steering
// 2048 left for relative steering
// 4096 right

#include <ros.h>
#include <std_msgs/Int32.h>

#define motorPin1 D5
#define motorPin2 D4

#define potPin A0

ros::NodeHandle nh;
std_msgs::Int32 int_msg;

void turn_right () {
  digitalWrite(motorPin1,1);
  digitalWrite(motorPin2,0);
}

void turn_left () {
  digitalWrite(motorPin1,0);
  digitalWrite(motorPin2,1);
}


void steer(const std_msgs::Int32::ConstPtr& msg){
  int requestVal = msg.data;
  // absolute steering
  if (requestVal <= 1100) {
    int currVal = analogRead(analogPin);
    while(((int diff=requestVal-currVal)*diff)<25){
      if(diff>0){
        turn_right();
      }else{
        turn_left();
      }
      delay(10);
    }
  } else {
  // relative steering
    if (requestVal == 2048) {
      turn_left();
    } else if (requestVal == 4096) {
      turn_right();
    } else {
      //error?
    }
  }
}

void setup() {
  pinMode(potPin, INPUT);
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);

  nh.initNode();
  ros::Subscriber steeringWheel = nh.subscribe("steering", 10, steer);
  ros::spin();
}

void loop(){}
