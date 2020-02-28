#if (_TEST_GPS_)

#include "NMEAParser.hpp"
#include "Venus838.hpp"

long gps_vals[6];
char printableValue[16];

#if (_GPS_DEBUGGER_)
#include "Queue.h"
DataQueue<String> q(100);
#endif


NMEAParser parser;
Venus838 gps(BAUD_RATE, false);

String errorcodes[5] = {"NORMAL", "NACK", "TIMEOUT", "INVALIDARG", "UNKNOWN"};

long lastutc = 0;

bool init_gps() {
  
  //NS-HP output NMEA message in 115200 bps
  Serial2.begin(BAUD_RATE);

  //gps.setBaudRate(115200, 0);

  gps_vals[2] = -999;
  
  return true;
}

volatile bool inEvent = false;

void serialEvent2() {
  bool newData = false;
  char c = gps.read();
  if(parser.encode(c)) {
    long latitude_u, latitude_l; // each variable is separated into two longs to improve precision
    long longitude_u, longitude_l;
    parser.getLatitude(&latitude_u, &latitude_l); // pass reference to latitude_u and latitude_l to getLatitude()
    parser.getLongitude(&longitude_u, &longitude_l);

    long altitude = parser.getAltitude(); // altitude in cm
    long altitude_u = altitude / 100;
    long altitude_l = (altitude < 0 ? -1 : 1) * altitude % 100;

    gps_vals[0] = latitude_u;
    gps_vals[0] = latitude_l;

    gps_vals[2] = longitude_u;
    gps_vals[3] = longitude_l;

    gps_vals[4] = altitude_u;
    gps_vals[5] = altitude_l;
  }
}

void write_gps_vals() {
  for(int i = 0; i < 3; i++) {
    sprintf(printableValue, "%d.%07d\t", gps_vals[2 * i], gps_vals[2 * i + 1]);
    Serial.print(printableValue);
  }
  
#if (_GPS_DEBUGGER_)
  Serial.print("d[");
  while(!q.isEmpty()) {
    Serial.print(q.dequeue());
    Serial.print(",");
  }
  Serial.print("]/d\t");
#endif
}
#endif
