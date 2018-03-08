
#include <SerialManager.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

// If you're using a GPS module:
// Connect the GPS Power pin to 5V
// Connect the GPS Ground pin to ground
// If using software serial (sketch example default):
//   Connect the GPS TX (transmit) pin to Digital 3
//   Connect the GPS RX (receive) pin to Digital 2
// If using hardware serial (e.g. Arduino Mega):
//   Connect the GPS TX (transmit) pin to Arduino RX1, RX2 or RX3
//   Connect the GPS RX (receive) pin to matching TX1, TX2 or TX3

// If you're using the Adafruit GPS shield, change
// SoftwareSerial gpsSerial(3, 2); -> SoftwareSerial gpsSerial(8, 7);
// and make sure the switch is set to SoftSerial

// If using software serial, keep this line enabled
// (you can change the pin numbers to match your wiring):
SoftwareSerial gpsSerial(3, 2);

// If using hardware serial (e.g. Arduino Mega), comment out the
// above SoftwareSerial line, and enable this line instead
// (you can change the Serial number to match your wiring):

//HardwareSerial gpsSerial = Serial1;

SerialManager manager;

Adafruit_GPS GPS(&gpsSerial);

#define GPS_UPDATE_RATE_HZ 5
unsigned long gps_update_delay = 1000 / GPS_UPDATE_RATE_HZ;

uint32_t timer = millis();

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences.
#define GPSECHO false

// this keeps track of whether we're using the interrupt
// off by default!
boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy

#define GPS_BAUD_RATE 9600

void initGPS(){
	// 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
	GPS.begin(GPS_BAUD_RATE);

	// For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
	// the parser doesn't care about other sentences at this time
	if (GPS_UPDATE_RATE_HZ == 1) {
		// turn on RMC (recommended minimum) and GGA (fix data) including altitude
		GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
		GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
		GPS.sendCommand(PMTK_API_SET_FIX_CTL_1HZ);
		// For the parsing code to work nicely and have time to sort thru the data, and
		// print it out we don't suggest using anything higher than 1 Hz
	}
	else if (GPS_UPDATE_RATE_HZ == 5) {
		GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
		GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);
		GPS.sendCommand(PMTK_API_SET_FIX_CTL_5HZ);
	}
	else if (GPS_UPDATE_RATE_HZ == 10) {
		// turn on RMC (recommended minimum)
		GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
		GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);
		GPS.sendCommand(PMTK_API_SET_FIX_CTL_5HZ);
	}

	// Request updates on antenna status, comment out to keep quiet
	GPS.sendCommand(PGCMD_ANTENNA);

	// the nice thing about this code is you can have a timer0 interrupt go off
	// every 1 millisecond, and read data from the GPS for you. that makes the
	// loop code a heck of a lot easier!
	useInterrupt(false);

	delay(1000);
	// Ask for firmware version
	gpsSerial.println(PMTK_Q_RELEASE);
}

// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
	char c = GPS.read();
	// if you want to debug, this is a good time to do it!
	#ifdef UDR0
	if (GPSECHO)
		if (c) UDR0 = c;
	// writing direct to UDR0 is much much faster than Serial.print
	// but only one character can be written at a time.
	#endif
}

void useInterrupt(boolean v) {
	if (v) {
		// Timer0 is already used for millis() - we'll just interrupt somewhere
		// in the middle and call the "Compare A" function above
		OCR0A = 0xAF;
		TIMSK0 |= _BV(OCIE0A);
		usingInterrupt = true;
	} else {
		// do not call the interrupt function COMPA anymore
		TIMSK0 &= ~_BV(OCIE0A);
		usingInterrupt = false;
	}
}

void updateGPS(){


	// in case you are not using the interrupt above, you'll
	// need to 'hand query' the GPS, not suggested :(
	// if (!usingInterrupt) {
	//     // read data from the GPS in the 'main loop'
	//     char c = GPS.read();
	//     // if you want to debug, this is a good time to do it!
	//     if (GPSECHO)
	//     if (c) Serial.print(c);
	// }

	// if a sentence is received, we can check the checksum, parse it...
	if (GPS.newNMEAreceived()) {
		// a tricky thing here is if we print the NMEA sentence, or data
		// we end up not listening and catching other sentences!
		// so be very wary if using OUTPUT_ALLDATA and trying to print out data
		//Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false

		if (!GPS.parse(GPS.lastNMEA()))     // this also sets the newNMEAreceived() flag to false
			return;          // we can fail to parse a sentence in which case we should just wait for another
	}

	// if millis() or timer wraps around, we'll just reset it
	if (timer > millis()) timer = millis();

	// approximately every "gps_update_delay" seconds or so, print out the current stats
	if (millis() - timer > gps_update_delay) {
		Serial.print("gps\t");
		timer = millis();     // reset the timer

		Serial.print("t");
		Serial.print(GPS.hour, DEC); Serial.print(':');
		Serial.print(GPS.minute, DEC); Serial.print(':');
		Serial.print(GPS.seconds, DEC); Serial.print(':');
		Serial.print(GPS.milliseconds);

		Serial.print("\td");
		Serial.print(GPS.day, DEC); Serial.print('/');
		Serial.print(GPS.month, DEC); Serial.print("/20");     // assuming this won't be around for 100 years
		Serial.print(GPS.year, DEC);

		Serial.print("\tf");
		Serial.print((int)GPS.fix); Serial.print(",");
		Serial.print((int)GPS.fixquality);
		if (GPS.fix) {
			Serial.print("\tl");
			Serial.print(GPS.latitude, 6); Serial.print(",");
			Serial.print(GPS.lat); Serial.print(",");
			Serial.print(GPS.longitude, 6); Serial.print(",");
			Serial.print(GPS.lon);

			Serial.print("\tg");
			Serial.print(GPS.latitudeDegrees, 6);
			Serial.print(",");
			Serial.print(GPS.longitudeDegrees, 6);

			Serial.print("\tx");
			Serial.print(GPS.speed); Serial.print(",");
			Serial.print(GPS.angle); Serial.print(",");
			Serial.print(GPS.altitude); Serial.print(",");
			Serial.print((int)GPS.satellites);
			Serial.print('\n');
		}
		else {
			Serial.print('\n');
		}
	}
}

void setup()
{
	manager.begin();
	Serial.print("hello!\n");

	initGPS();
	Serial.print("ready!\n");

	delay(50);
	GPS.standby();
	delay(50);
}

void loop()  // run over and over again
{
	if(manager.available()) {
		int status = manager.readSerial();
		if (status == 2) {          // start event
			useInterrupt(true);
			GPS.wakeup();
		}
		else if (status == 1) {     // stop event
			GPS.standby();
			useInterrupt(false);
		}
		else if (status == 0) {    // user command
			useInterrupt(false);
		}
	}

	if(!manager.isPaused()) {
		updateGPS();
		delay(gps_update_delay);
	}
}
