//accepts int32 messages [0,1023] representing how far to steer, and actuates them
#include <ros.h>
#include <std_msgs/Int32.h>

// 0 is left
// 1023 is right
// 400 is center

#define PWM_PIN A2
#define DIR_PIN 4
#define BRK_PIN 3

#define A_PIN A1

ros::NodeHandle nh;

std_msgs::Int32 int_msg;
int goal;

void steer(const std_msgs::Int32& msg){
  goal = msg.data;
}

ros::Subscriber<std_msgs::Int32> steeringWheel("steering", steer);

void setup() {

  pinMode(PWM_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(BRK_PIN, OUTPUT);
  pinMode(A_PIN, INPUT);

  digitalWrite(BRK_PIN, LOW);
  goal = analogRead(A_PIN);
  goal = 400;
  analogWrite(PWM_PIN, 0);
  
  nh.initNode();
  nh.subscribe(steeringWheel);
}

void loop() {
  nh.spinOnce();
  int current = analogRead(A_PIN);
  if (current - goal < 5 && current - goal > -5) {
    analogWrite(PWM_PIN, 0);
  } else {
    if (current < goal) {
      digitalWrite(DIR_PIN, LOW);
      analogWrite(PWM_PIN, 255);
    } else {
      digitalWrite(DIR_PIN, HIGH);
      analogWrite(PWM_PIN, 255);
    }
  }
}
