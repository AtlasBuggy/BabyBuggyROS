#include <BNO055_Naboris.h>
#include <SerialManager.h>

SerialManager manager;

/* ------------------------ *
 * General global variables *
 * ------------------------ */

void setup() {
    manager.begin();

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

        }
        else if (status == 1)  // stop event
        {

        }
        else if (status == 0)  // user command
        {
            
        }
    }

    if (!manager.isPaused()) {
        updateIMU();
        // 100Hz update rate for imu
        delay(10);
    }
}
