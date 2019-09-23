#include <Encoder.h>
#include <SerialManager.h>

#define USE_ENCODER1
// #define USE_ENCODER2  // put encoder 2 on a different microcontroller

#ifdef USE_ENCODER1
Encoder encoder1(2, 3);
int64_t encoder1_read = 0;
#endif

#ifdef USE_ENCODER2
Encoder encoder2(5, 4);
int64_t encoder2_read = 0;
#endif

uint64_t prev_time = millis();

SerialManager manager;

void setup() {
    Serial.begin(115200);

    Serial.print("hello!\n");
    Serial.print("ready!\n");
}

void print_int64(int64_t value)
{
    int32_t part1 = value >> 32;
    int32_t part2 = value & 0xffffffff;
    Serial.print(part1);
    Serial.print("|");
    Serial.print(part2);
}

void loop() {
    if (manager.available()) {
        int status = manager.readSerial();
        String command = manager.getCommand();

        if (status == 2)  // start event
        {
            #ifdef USE_ENCODER1
            encoder1.write(0);
            #endif

            #ifdef USE_ENCODER2
            encoder2.write(0);
            #endif
        }
        // else if (status == 1)  // stop event
        // {
        //
        // }
        // if (status == 0)  // user command
        // {
        //
        // }
    }

    if (!manager.isPaused()) {
        #ifdef USE_ENCODER1
        encoder1_read = encoder1.read();
        #endif

        #ifdef USE_ENCODER2
        encoder2_read = encoder2.read();
        #endif

        if (prev_time > millis()) {
            prev_time = millis();
        }
        if (millis() - prev_time > 100) {
            prev_time = millis();

            Serial.print("enc\tt");
            Serial.print(millis());

            #ifdef USE_ENCODER1
            Serial.print("\ta");
            print_int64(encoder1_read);
            #endif

            #ifdef USE_ENCODER2
            Serial.print("\tb");
            print_int64(encoder2_read);
            #endif

            Serial.print('\n');
        }
        // delay(1);
    }
}
