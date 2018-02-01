#include <Battery.h>

Battery battery(4900, 5200, A0);

void setup() {
	Serial.begin(9600);
	while (!Serial);
	battery.begin();
}

void loop() {
	// Serial.println(analogRead(A0));
	// delay(100);
	digitalWrite(13, HIGH);
	delay(500);
	digitalWrite(13, LOW);
	delay(500);

	Serial.print("Battery voltage is ");
	Serial.print(battery.voltage());
	Serial.print(" (");
	Serial.print(battery.level());
	Serial.println(")");
}
