#include "Motors_Naboris.h"

/* ----------------------------- *
* Motor shield global variables *
* ----------------------------- */

// #define ENABLE_MOTOR_TIMEOUT_PINGS

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61);

unsigned int speed_increment = 40;

struct MotorStruct {
    Adafruit_DCMotor* af_motor;
    int speed;
    int goal_speed;
    byte run_state;
};

MotorStruct init_motor(int motor_num) {
    MotorStruct new_motor;
    new_motor.af_motor = AFMS.getMotor(motor_num);
    new_motor.speed = 0;
    new_motor.goal_speed = 0;
    return new_motor;
}
MotorStruct* motors = new MotorStruct[NUM_MOTORS];

#ifdef ENABLE_MOTOR_TIMEOUT_PINGS
uint32_t ping_timer = millis();
#endif


/* ---------------------- *
 * Servo global variables *
 * ---------------------- */

Servo head_servo;
uint32_t servo_timer = millis();
bool servos_attached = false;

void attach_servo()
{
    if (!servos_attached) {
        servos_attached = true;
        head_servo.attach(SERVO_PIN);
        servo_timer = millis();
    }
}

void detach_servo()
{
    if (servos_attached) {
        servos_attached = false;
        head_servo.detach();
        servo_timer = millis();
    }
}



void set_motor_speed(int motor_num)
{
    if (motors[motor_num].speed > 0) {
        motors[motor_num].af_motor->run(FORWARD);
    }
    else if (motors[motor_num].speed == 0) {
        motors[motor_num].af_motor->run(BRAKE);
    }
    else {
        motors[motor_num].af_motor->run(BACKWARD);
    }
    motors[motor_num].af_motor->setSpeed(abs(motors[motor_num].speed));
}

void set_motor_goal(int motor_num, int speed) {
    motors[motor_num].goal_speed = speed;
    if (motors[motor_num].goal_speed > 0) {
        if (motors[motor_num].goal_speed < 0) {
            motors[motor_num].goal_speed = 0;
        }
        if (motors[motor_num].goal_speed > 255) {
            motors[motor_num].goal_speed = 255;
        }
    }
    else {
        if (motors[motor_num].goal_speed > 0) {
            motors[motor_num].goal_speed = 0;
        }
        if (motors[motor_num].goal_speed < -255) {
            motors[motor_num].goal_speed = -255;
        }
    }
}

// top left, top right, bottom left, bottom right
void set_motors(int speed1, int speed2, int speed3, int speed4)
{
    set_motor_goal(MOTOR1, speed1);  // top left
    set_motor_goal(MOTOR2, speed2);  // top right
    set_motor_goal(MOTOR3, speed3);  // bottom left
    set_motor_goal(MOTOR4, speed4);  // bottom right
}

#ifdef ENABLE_MOTOR_TIMEOUT_PINGS
void ping() {
    ping_timer = millis();
}
#endif

void stop_motors() {
    set_motors(0, 0, 0, 0);
}

void release_motors()
{
    for (int motor_num = 0; motor_num < NUM_MOTORS; motor_num++)
    {
        motors[motor_num].goal_speed = 0;
        motors[motor_num].speed = 0;
        motors[motor_num].af_motor->run(RELEASE);
    }
}

void updateMotors()
{
    for (int motor_num = 0; motor_num < NUM_MOTORS; motor_num++)
    {
        set_motor_speed(motor_num);

        if (motors[motor_num].speed < motors[motor_num].goal_speed) {
            motors[motor_num].speed += speed_increment;
        }
        else {
            motors[motor_num].speed -= speed_increment;
        }

        if (abs(motors[motor_num].speed - motors[motor_num].goal_speed) < 2 * speed_increment) {
            motors[motor_num].speed = motors[motor_num].goal_speed;
        }
    }
}
