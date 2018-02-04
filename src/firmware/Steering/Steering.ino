//accepts int32 messages [0,1023] representing how far to steer, and actuates them
#include <ros.h>
#include <std_msgs/Int32.h>


#define PWM_PIN A2
#define DIR_PIN 4
#define BRK_PIN 3

#define A_PIN A1

ros::NodeHandle nh;
std_msgs::Int32 int_msg;
int goal;

void steer(const std_msgs::Int32::ConstPtr& msg){
  goal = msg.data;
}

void setup() {

  pinMode(PWM_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(BRK_PIN, OUTPUT);
  pinMode(A_PIN, INPUT);

  digitalWrite(BRK_PIN, LOW);
  goal = analogRead(A_PIN);
  analogWrite(PWM_PIN, 0);

  nh.initNode();
  ros::Subscriber steeringWheel = nh.subscribe("steering", 10, steer);
  ros::spin();
}

void loop() {
  int current = analogRead(A_PIN);
  if (current - goal < 10 && current - goal > -10) {
    analogWrite(PWM_PIN, 0);
  } else {
    if (current < goal) {
      digitalWrite(DIR_PIN, HIGH);
      analogWrite(PWM_PIN, 255);
    } else {
      digitalWrite(DIR_PIN, LOW);
      analogWrite(PWM_PIN, 255);
    }
  }
}
