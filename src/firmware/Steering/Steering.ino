//accepts int32 messages [0,1023] representing how far to steer, and actuates them
// 400 is middle
// inputs 0 - 1023 for absolute steering
// 2048 left for relative steering
// 4096 right

#include <ros.h>
#include <std_msgs/Int32.h>

#define DIR_PIN 4
#define BRAKE_PIN 3
#define PWM_PIN A2
#define POT_PIN A1

ros::NodeHandle nh;
int requestVal;

void steer(const std_msgs::Int32& msg){
  requestVal = msg.data; 
}

ros::Subscriber<std_msgs::Int32> steeringSub ("steering", steer);

void turn_left() {
  digitalWrite(DIR_PIN, HIGH);
  digitalWrite(PWM_PIN, HIGH);
}

void turn_right() {
  digitalWrite(DIR_PIN, LOW);
  digitalWrite(PWM_PIN, HIGH);
}

void stop_motor(){
  digitalWrite(PWM_PIN, LOW);
}

void setup() {
  pinMode(DIR_PIN, OUTPUT);
  pinMode(BRAKE_PIN, OUTPUT);
  pinMode(PWM_PIN, OUTPUT);
  pinMode(POT_PIN, INPUT);

  nh.initNode();
  nh.subscribe(steeringSub);
}

void loop(){
  // absolute steering
  if (requestVal <= 1100) {
    int diff = requestVal - analogRead(POT_PIN);
    
    if((diff * diff) > 25){
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
  
  else {
  // relative steering
    if (requestVal == 2048) {
      turn_left();
    } else if (requestVal == 4096) {
      turn_right();
    }
  }
  
  delay(10);
  nh.spinOnce();
}
