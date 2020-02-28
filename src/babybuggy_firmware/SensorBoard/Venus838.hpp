/*
    Venus838.hpp - Library for configuration of Skytraq's Venus838 chipset
    Reed A. Foster, June 2017.
*/

#ifndef Venus838_h
#define Venus838_h

#include "Arduino.h"
#include "HardwareSerial.h"

// Uncomment this line to enable debugging over Serial.
//#define GPS_DEBUG_BAUDRATE 115200

// NMEA strings enum definitions (also used by NMEAParser)
#define NMEA_GGA 0
#define NMEA_GSA 1
#define NMEA_GSV 2
#define NMEA_GLL 3
#define NMEA_RMC 4
#define NMEA_VTG 5
#define NMEA_ZDA 6
#define NMEA_UNKNOWN 7
#define uint unsigned int

#define GPS_ACK_TIMEOUT_MS 1000 // default wait time for how long the sender should wait for ack

#define GPS_DEFAULT_BAUDRATE 9600 // default baud rate of GPS receiver (should be 9600)

class Venus838
{
public:

    Venus838(long baudrate, bool reset); // pass hardwareserial port by reference

    // Receiver configuration/command methods
    char setBaudRate(int baudrate, char attribute);
    char setUpdateRate(int frequency, char attribute);
    char resetReceiver(bool reboot);
    char querySoftwareVersion();
    char cfgNMEA(char messagename, bool enable, char attribute);
    char cfgNMEA(char nmeabyte, char attribute);
    char cfgPowerSave(bool enable, char attribute);
    char cfgPPS(char mode, char attribute);

    // Serial wrapper methods
    bool available();
    char read();

private:

    // error codes
    const int GPS_NORMAL = 0;
    const int GPS_NACK = 1;
    const int GPS_TIMEOUT = 2;
    const int GPS_INVALIDARG = 3;
    const int GPS_UNKNOWN = 4;

    const long int _baudrates[6] = {4800, 9600, 19200, 38400, 57600, 115200};
    char _nmeastate = 0b1111111; // stores current configuration of which NMEA strings are enabled
    HardwareSerial _gpsSerial = Serial2;

    int _getBaudRate();
    char _sendCommand(char messageid, char *messagebody, int bodylen); // uses default timeout specified by TIMEOUTMS
    char _sendCommand(char messageid, char *messagebody, int bodylen, uint timemout);
    char _sendPacket(char *packet, int size, uint timeout);

    // debug
    void _printPacket(char *packet, int size);
    void _debug(const char *message);
    void _debug(int number);
};

#endif
