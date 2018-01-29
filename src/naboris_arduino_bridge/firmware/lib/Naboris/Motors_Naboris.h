#ifndef __MOTORS_NABORIS_H__
#define __MOTORS_NABORIS_H__


#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Adafruit_MotorShield.h>


#define NUM_MOTORS 4


#define MOTOR1 1
#define MOTOR2 0
#define MOTOR3 2
#define MOTOR4 3

#define SERVO_PIN 9

void set_motor_speed(int motor_num);
void set_motor_goal(int motor_num, int speed);
void set_motors(int speed1, int speed2, int speed3, int speed4);
void ping();
void stop_motors();
void release_motors();
void updateMotors();

#endif  // __MOTORS_NABORIS_H__
