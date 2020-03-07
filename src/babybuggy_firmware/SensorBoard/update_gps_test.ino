#if (_TEST_GPS_)

#include "nmea_parser.hpp"

double gps_vals[3];
char printableValue[16];

#if (_GPS_DEBUGGER_)
#include "Queue.h"
DataQueue<String> q(100);
#endif


NMEA_Parser parser;

long lastutc = 0;

bool sendGPSMessage(char msg[]) {
    int len = 84;
    bool sent = false;

    for(int i = 0; i < len; i++) sent |= parser.encode(msg[i]);
    sent |= parser.encode(0x0D);
    sent |= parser.encode(0x0A);
    return sent;
}


bool init_gps() {
  
  //NS-HP output NMEA message in 115200 bps
  Serial2.begin(BAUD_RATE);

  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);

  //gps.setBaudRate(115200, 0);

  gps_vals[2] = -999;
//  Serial.print("Send temp message: ");
//  Serial.println(sendGPSMessage("$GPGGA,202434.000,2447.0936188,N,12100.5253729,E,4,22,0.6,96.186,M,19.600,M,,0000*65"));
  
  return true;
}

volatile bool inEvent = false;

void serialEvent2() {
  digitalWrite(13, LOW);
  bool newData = false;
  char c;
  while(Serial2.available()) {
    c = Serial2.read();
    newData |= parser.encode(c);
  }
//  Serial.print("error flag: ");
//  Serial.println(parser.getErrorFlag());
//  Serial.println(parser.flag);
//  Serial.println(parser.sentence);
//  parser.clearErrorFlag();
  if(newData) {
    digitalWrite(13, HIGH);
    double latitude;
    double longitude;
    latitude = parser.getLatitude();
    longitude = parser.getLongitude();

    double altitude = parser.getAltitude();

    gps_vals[0] = latitude;

    gps_vals[1] = longitude;

    gps_vals[2] = altitude;
  }
}

void write_gps_vals() {
  for(int i = 0; i < 3; i++) {
    //sprintf(printableValue, "%d.%04d\t", gps_vals[2 * i], gps_vals[2 * i + 1]);
    //Serial.print(printableValue);
    Serial.print(gps_vals[i], 8);
    Serial.print("\t");
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
