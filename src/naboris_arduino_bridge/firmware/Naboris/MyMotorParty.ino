/*
This is a test sketch for the Adafruit assembled Motor Shield for Arduino v2
It won't work with v1.x motor shields! Only for the v2's with built in PWM
control

For use with the Adafruit Motor Shield v2
---->	http://www.adafruit.com/products/1438

This sketch creates a fun motor party on your desk *whiirrr*
Connect a unipolar/bipolar stepper to M3/M4
Connect a DC motor to M1
Connect a hobby servo to SERVO1
*/

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Servo.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61);


// And connect a DC motor to port M1
Adafruit_DCMotor *motor_1 = AFMS.getMotor(1);
Adafruit_DCMotor *motor_2 = AFMS.getMotor(2);
Adafruit_DCMotor *motor_3 = AFMS.getMotor(3);
Adafruit_DCMotor *motor_4 = AFMS.getMotor(4);

// We'll also test out the built in Arduino Servo library
Servo servo1;
Servo servo2;

int increment = 3;
int max_speed = 150;
int increment_delay = 1;

int m1_speed = 0;
int m2_speed = 0;
int m3_speed = 0;
int m4_speed = 0;

void setup() {
    Serial.begin(9600);           // set up Serial library at 9600 bps
    Serial.println("MMMMotor party!");

    AFMS.begin();  // create with the default frequency 1.6KHz
    //AFMS.begin(1000);  // OR with a different frequency, say 1KHz

    // Attach a servo to pin #10
    servo1.attach(10);
    servo2.attach(9);

    set_motors(0, 0, 0, 0);
    motor_1->run(RELEASE);
    motor_2->run(RELEASE);
    motor_3->run(RELEASE);
    motor_4->run(RELEASE);

}

// top left, top right, bottom left, bottom right
void set_motors(int speed2, int speed1, int speed3, int speed4)
{
    if (speed1 > 0) {
        motor_1->run(FORWARD);
    }
    else {
        motor_1->run(BACKWARD);
    }
    m1_speed = abs(speed1);
    motor_1->setSpeed(m1_speed);

    if (speed2 > 0) {
        motor_2->run(FORWARD);
    }
    else {
        motor_2->run(BACKWARD);
    }
    m2_speed = abs(speed2);
    motor_2->setSpeed(m2_speed);

    if (speed3 > 0) {
        motor_3->run(FORWARD);
    }
    else {
        motor_3->run(BACKWARD);
    }
    m3_speed = abs(speed3);
    motor_3->setSpeed(m3_speed);

    if (speed4 > 0) {
        motor_4->run(FORWARD);
    }
    else {
        motor_4->run(BACKWARD);
    }
    m4_speed = abs(speed4);
    motor_4->setSpeed(m4_speed);
}

void drive(int angle, int speed)
{
    angle %= 360;

    if (0 <= angle && angle < 90) {
        int fraction_speed = -2 * speed / 90 * angle + speed;
        set_motors(speed, fraction_speed, fraction_speed, speed);
    }
    else if (90 <= angle && angle < 180) {
        int fraction_speed = -2 * speed / 90 * (angle - 90) + speed;
        set_motors(fraction_speed, -speed, -speed, fraction_speed);
    }
    else if (180 <= angle && angle < 270) {
        int fraction_speed = 2 * speed / 90 * (angle - 180) - speed;
        set_motors(-speed, fraction_speed, fraction_speed, -speed);
    }
    else if (270 <= angle && angle < 360) {
        int fraction_speed = 2 * speed / 90 * (angle - 270) - speed;
        set_motors(fraction_speed, speed, speed, fraction_speed);
    }
}

void spin(int speed) {
    set_motors(speed, -speed, speed, -speed);
}

void stop()
{
    while (m1_speed > 0 || m2_speed > 0 || m3_speed > 0 || m4_speed > 0)
    {
        if (m1_speed > 0) {
            motor_1->setSpeed(m1_speed);
            m1_speed -= increment;
        }
        if (m2_speed > 0) {
            motor_2->setSpeed(m2_speed);
            m2_speed -= increment;
        }
        if (m3_speed > 0) {
            motor_3->setSpeed(m3_speed);
            m3_speed -= increment;
        }
        if (m4_speed > 0) {
            motor_4->setSpeed(m4_speed);
            m4_speed -= increment;
        }
        delay(increment_delay);
    }
    motor_1->setSpeed(m1_speed);
    motor_2->setSpeed(m2_speed);
    motor_3->setSpeed(m3_speed);
    motor_4->setSpeed(m4_speed);
}

void hard_stop() {
    set_motors(0, 0, 0, 0);
}

void release()
{
    motor_1->run(RELEASE);
    motor_2->run(RELEASE);
    motor_3->run(RELEASE);
    motor_4->run(RELEASE);
}

void loop() {
    for (int angle = 0; angle < 360; angle++){
        drive(angle, max_speed);
        delay(1);
    }
    stop();
    delay(250);

    spin(max_speed); delay(1000);
    stop(); delay(250);
    spin(-max_speed); delay(1000);
    stop(); delay(250);
}
