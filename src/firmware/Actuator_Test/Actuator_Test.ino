#include "DualVNH5019MotorShield.h"

#define POT_PIN A2

DualVNH5019MotorShield md;

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
  // put your setup code here, to run once:
  md.init();
  pinMode(POT_PIN, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  turn_left();
  delay(10);
}
