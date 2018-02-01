
#include "BNO055_Naboris.cpp"
#include "Encoder_Naboris.cpp"
#include "Lights_Naboris.cpp"
#include "Motors_Naboris.cpp"

#include "SerialManager.h"

SerialManager manager;

/* ------------------------ *
 * General global variables *
 * ------------------------ */

void setup() {
    manager.begin();

    AFMS.begin();
    strip.begin();

    strip.show();

    for (int motor_num = 1; motor_num <= NUM_MOTORS; motor_num++) {
        motors[motor_num - 1] = init_motor(motor_num);
    }

    // fadeColors(0, 0, 0, SIGNAL_COLOR, SIGNAL_COLOR, SIGNAL_COLOR, 1, SIGNAL_DELAY, SIGNAL_INCREMENT);
    fadeColors(0, 0, 0, 0, SIGNAL_COLOR, 0, SIGNAL_CYCLES, SIGNAL_DELAY, SIGNAL_INCREMENT);
    // fadeColors(SIGNAL_COLOR, 0, SIGNAL_COLOR, 0, 0, 0, 1, SIGNAL_DELAY, SIGNAL_INCREMENT);

    Serial.print("hello!\n");

    initIMU();

    Serial.print("ready!\n");
}

void loop()
{
    if (manager.available())
    {
        int status = manager.readSerial();

        String command = manager.getCommand();

        if (status == 2)  // start event
        {
            fadeColors(0, 0, 0, 0, 0, SIGNAL_COLOR, SIGNAL_CYCLES, SIGNAL_DELAY, SIGNAL_INCREMENT);
            setColor(strip.Color(1, 1, 1));

            stop_motors();
            rightEncoder.write(0);
            leftEncoder.write(0);
            oldLeftPosition = -1;
            oldRightPosition = -1;
            attach_servo();
            head_servo.write(90);
        }
        else if (status == 1)  // stop event
        {
            stop_motors();
            release_motors();
            detach_servo();

            for (int index = 0; index < NUM_LEDS; index++) {
                strip.setPixelColor(index, 0);
            }
            strip.show();

            fadeColors(0, 0, 0, SIGNAL_COLOR, 0, 0, SIGNAL_CYCLES, SIGNAL_DELAY, SIGNAL_INCREMENT);
        }
        else if (status == 0)  // user command
        {
            if (command.charAt(0) == 'd') {  // drive command
                int m1 = command.substring(1, 5).toInt();
                int m2 = command.substring(5, 9).toInt();
                int m3 = command.substring(9, 13).toInt();
                int m4 = command.substring(13, 17).toInt();

                set_motors(m1, m2, m3, m4);
                #ifdef ENABLE_MOTOR_TIMEOUT_PINGS
                ping();
                #endif
            }

            else if (command.charAt(0) == 'h') {  // stop motors command
                stop_motors();
            }
            else if (command.equals("rainbow")) {
                stop_motors();
                detach_servo();
                delay(5);

                rainbowCycle(1);
                setColor(strip.Color(1, 1, 1));

                delay(5);
                attach_servo();
            }
            else if (command.charAt(0) == 'r') {  // release command
                release_motors();
            }
            else if (command.charAt(0) == 'c') {  // servo command
                int servo_value = command.substring(1, 4).toInt();

                attach_servo();
                head_servo.write(servo_value);
            }
            else if (command.charAt(0) == 'o') {  // pixel command
                int led_num = command.substring(1, 4).toInt();
                if (led_num < 0) {
                    led_num = 0;
                }
                int r = command.substring(4, 7).toInt();
                int g = command.substring(7, 10).toInt();
                int b = command.substring(10, 13).toInt();
                if (command.length() > 13)
                {
                    int stop_num = command.substring(13, 16).toInt();
                    if (stop_num > NUM_LEDS) {
                        stop_num = NUM_LEDS;
                    }
                    for (int index = led_num; index < stop_num; index++) {
                        strip.setPixelColor(index, strip.Color(r, g, b));
                    }
                }
                else {
                    strip.setPixelColor(led_num, strip.Color(r, g, b));
                }
            }
            else if (command.charAt(0) == 'f' && command.length() == 13) {  // command leds to fade to a color
                int cycle_num = command.substring(1, 4).toInt();
                int r = command.substring(4, 7).toInt();
                int g = command.substring(7, 10).toInt();
                int b = command.substring(10, 13).toInt();
                // fadeColors(SIGNAL_COLOR, SIGNAL_COLOR, SIGNAL_COLOR, 1, SIGNAL_DELAY, SIGNAL_INCREMENT);
                fadeColors(0, 0, 0, r, g, b, cycle_num, SIGNAL_DELAY, SIGNAL_INCREMENT);
            }
            else if (command.charAt(0) == 'x') {  // show command
                detach_servo();
                delay(5);
                strip.show();
                delay(5);
                attach_servo();
            }
        }
    }

    if (!manager.isPaused()) {
        #ifdef ENABLE_MOTOR_TIMEOUT_PINGS
        if (ping_timer > millis())  ping_timer = millis();
        if ((millis() - ping_timer) > 500) {
            stop_motors();
            ping_timer = millis();
        }
        #endif

        if (servo_timer > millis())  servo_timer = millis();
        if (servos_attached && (millis() - servo_timer) > 500)  {
            detach_servo();
        }

        updateMotors();
        updateEncoders();
        updateIMU();

        // 100Hz update rate for imu
        delay(10);
    }
}
