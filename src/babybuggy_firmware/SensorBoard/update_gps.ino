#include "nmea_parser.hpp"

#define GPS_PRECISION     8
#define GPS_DEBUG_LIGHT   0

double gps_vals[3];

NMEA_Parser parser;

bool init_gps() {
  Serial2.begin(BAUD_RATE);

  gps_vals[0] = 0;
  gps_vals[1] = 0;
  gps_vals[2] = -999;
  
  return true;
}

volatile bool inEvent = false;

void serialEvent2() {
  bool newData = false;
  char c;
  
  while(Serial2.available()) {
    c = Serial2.read();
    newData |= parser.encode(c);
  }
  
  if(newData) {
    double latitude = parser.getLatitude();
    double longitude = parser.getLongitude();
    double altitude = parser.getAltitude();

    gps_vals[0] = latitude;
    gps_vals[1] = longitude;
    gps_vals[2] = altitude;
  }
}

void write_gps_vals() {
  for(int i = 0; i < 3; i++) {
    Serial.print(gps_vals[i], GPS_PRECISION);
    Serial.print("\t");
  }
}
