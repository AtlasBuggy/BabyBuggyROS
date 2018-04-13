#include <Encoder.h>
#include <SerialManager.h>

Encoder encoder1(5, 3);  // reverse encoder direction
Encoder encoder2(6, 4);


int64_t prev1_ticks = -1;
int64_t encoder1_read = 0;

int64_t prev2_ticks = -1;
int64_t encoder2_read = 0;

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
            encoder1.write(0);
            encoder2.write(0);
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
        encoder1_read = encoder1.read();
        encoder2_read = encoder2.read();
        if (prev_time > millis()) {
            prev_time = millis();
        }
        if (encoder1_read != prev1_ticks || encoder2_read != prev2_ticks || millis() - prev_time > 1000) {
            prev_time = millis();

            Serial.print("enc\tt");
            Serial.print(millis());

            if (encoder1_read != prev1_ticks) {
                Serial.print("\t1");
                print_int64(encoder1_read);
            }
            if (encoder2_read != prev2_ticks) {
                Serial.print("\t2");
                print_int64(encoder2_read);
            }
            Serial.print('\n');

            prev1_ticks = encoder1_read;
            prev2_ticks = encoder2_read;
        }
        delay(1);
    }
}
