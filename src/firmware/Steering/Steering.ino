//accepts int32 messages [0,1023] representing how far to steer, and actuates them
// 400 is middle
// inputs 0 - 1023 for absolute steering
// 2048 left for relative steering
// 4096 right

#include <ros.h>
#include <std_msgs/Int32.h>
#include "DualVNH5019MotorShield.h"

#define M1INA 2
#define M1INB 4
#define M1ENDIAG 7
#define M2INA 7
#define M2INB 8
#define M2ENDIAG 12
#define M1CS A0
#define M2CS A2

#define POT_PIN A1

DualVNH5019MotorShield md(M1INA, M1INB, M1ENDIAG, M1CS,
                          M2INA, M2INB, M2ENDIAG, M2CS);
ros::NodeHandle nh;
int requestVal = 400;

void steer(const std_msgs::Int32& msg){
  requestVal = msg.data;
}

ros::Subscriber<std_msgs::Int32> steeringSub ("steering", steer);

void turn_left() {
  md.setM1Speed(-400);
}

void turn_right() {
  md.setM1Speed(400);
}

void stop_motor(){
  md.setM1Speed(0);
}

void setup() {
  pinMode(POT_PIN, INPUT);
  
  md.init();
  nh.initNode();
  nh.subscribe(steeringSub);
}

void loop(){
  // absolute steering
  if (requestVal <= 1023) {
    int diff = requestVal - analogRead(POT_PIN);

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

  else if (requestVal == 1500){
    stop_motor();
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