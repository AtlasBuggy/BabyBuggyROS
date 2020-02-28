/*
    Venus838.cpp - Library for configuration of Skytraq's Venus838 chipset
    Reed A. Foster, June 2017.
*/

#include "Arduino.h"
#include "HardwareSerial.h"
#include "Venus838.hpp"

// Receiver startup sequence:
// 1. match baudrate of receiver
// 2. perform reset and match default baudrate of receiver (if requested)
// 3. change baudrate of receiver
// 4. match new baudrate of receiver

Venus838::Venus838(long baudrate, bool reset)
{
    #ifdef GPS_DEBUG_BAUDRATE
    Serial.begin(GPS_DEBUG_BAUDRATE);
    Serial.println("Debuggging initialized");
    #endif
    /*
    _gpsSerial = gpsSerial;
    int currentbaudrate = _getBaudRate();
    _gpsSerial.begin(currentbaudrate);
    if (reset)
    {
        resetReceiver(true);
    }
    delay(100);
    setBaudRate(baudrate, 0);
    */
    _nmeastate = 0b1111111; // enable all nmea messages;
}


// Attribute:
// 0 = only update RAM
// 1 = update RAM and flash
char Venus838::setBaudRate(int baudrate, char attribute)
{
    _debug("Setting baud rate\n");
    char messagebody[3];
    memset(messagebody, 0, 3);
    messagebody[0] = 0x00; // COM port 1
    bool baudrateset = false;
    for (int i = 0; i < 6; i++)
    {
        if (baudrate == _baudrates[i])
        {
            messagebody[1] = i;
            baudrateset = true;
        }
    }
    if (!baudrateset)
    {
        return GPS_INVALIDARG;
    }
    messagebody[2] = attribute;
    char code = _sendCommand(0x05, messagebody, 3); // messageid = 5
    if (code == GPS_NORMAL)
    {
        _gpsSerial.end();
        _gpsSerial.begin(baudrate); // restart the serial port to the new baudrate
        return code;
    }
    return code;
}

// Attribute:
// 0 = only update RAM
// 1 = update RAM and flash
char Venus838::setUpdateRate(int frequency, char attribute)
{
    _debug("Setting update rate\n");
    char messagebody[2];
    memset(messagebody, 0, 2);
    messagebody[0] = frequency;
    messagebody[1] = attribute;
    return _sendCommand(0x0E, messagebody, 2);
}

char Venus838::querySoftwareVersion()
{
    _debug("Querying software version\n");
    char messagebody[1];
    memset(messagebody, 0, 1);
    messagebody[0] = 1;
    return _sendCommand(0x02, messagebody, 1);
}

char Venus838::resetReceiver(bool reboot)
{
    _debug("Resetting receiver\n");
    char messagebody[1];
    memset(messagebody, 0, 1);
    messagebody[0] = reboot ? 1 : 0;
    char code = _sendCommand(0x04, messagebody, 1, 10000);
    if (code == GPS_NORMAL)
    {
        delay(500);
        _gpsSerial.end();
        _gpsSerial.begin(GPS_DEFAULT_BAUDRATE);
    }
    return code;
}

// Attribute:
// 0 = only update RAM
// 1 = update RAM and flash
char Venus838::cfgNMEA(char messagename, bool enable, char attribute)
{
    _debug("Configuring a NMEA string\n");
    if (enable)
        _nmeastate |= 1 << messagename;
    else
        _nmeastate &= ~(1 << messagename);
    return cfgNMEA(_nmeastate, attribute);
}

// Attribute:
// 0 = only update RAM
// 1 = update RAM and flash
// 2 = temporarily enabled
char Venus838::cfgPowerSave(bool enable, char attribute)
{
    _debug("Configuring Power Save mode\n");
    char messagebody[2];
    memset(messagebody, 0, 2);
    messagebody[0] = enable ? 1 : 0;
    messagebody[1] = attribute;
    return _sendCommand(0x0C, messagebody, 2);
}

// Mode:
// 0 = off
// 1 = on only when 3D fix
// 2 = on when at least 1 SV
// Attribute:
// 0 = only update RAM
// 1 = update RAM and flash
char Venus838::cfgPPS(char mode, char attribute)
{
    _debug("Configuring 1PPS output\n");
    char messagebody[2];
    memset(messagebody, 0, 2);
    messagebody[0] = mode;
    messagebody[1] = attribute;
    return _sendCommand(0x3E, messagebody, 2);
}

// Attribute:
// 0 = only update RAM
// 1 = update RAM and flash
char Venus838::cfgNMEA(char nmeabyte, char attribute)
{
    _debug("Configuring all NMEA strings\n");
    _nmeastate = nmeabyte;
    char messagebody[8];
    memset(messagebody, 0, 8);
    messagebody[1] = (_nmeastate >> NMEA_GSA) & 1;
    messagebody[0] = (_nmeastate >> NMEA_GGA) & 1;
    messagebody[2] = (_nmeastate >> NMEA_GSV) & 1;
    messagebody[3] = (_nmeastate >> NMEA_GLL) & 1;
    messagebody[4] = (_nmeastate >> NMEA_RMC) & 1;
    messagebody[5] = (_nmeastate >> NMEA_VTG) & 1;
    messagebody[6] = (_nmeastate >> NMEA_ZDA) & 1;
    messagebody[7] = attribute;
    return _sendCommand(0x08, messagebody, 8);
}

bool Venus838::available()
{
    return _gpsSerial.available();
}

char Venus838::read()
{
    return _gpsSerial.read();
}

int Venus838::_getBaudRate()
{
    _debug("Autodetecting baud rate\n");

    uint i = 0;
    bool baudratefound = false;
    while (!baudratefound)
    {
        _gpsSerial.begin(_baudrates[i]);

        _debug("trying baudrate ");
        _debug(_baudrates[i]);

        char response = querySoftwareVersion();

        if (response == GPS_NORMAL)
        {
            baudratefound = true;
        }
        else if (i < sizeof(_baudrates) - 1)
        {
            i++;
        }
        else
        {
            i = 0;
        }
        _gpsSerial.end();
    }
    return _baudrates[i];
}

char Venus838::_sendCommand(char messageid, char *messagebody, int bodylen)
{
    return _sendCommand(messageid, messagebody, bodylen, GPS_ACK_TIMEOUT_MS);
}

char Venus838::_sendCommand(char messageid, char *messagebody, int bodylen, uint timeout)
{
    _debug("sending command\n");
    // Assemble Packet
    int packetlength = 8 + bodylen;
    char packet[packetlength];
    memset(packet, 0, packetlength);

    packet[0] = 0xA0; // start sequence
    packet[1] = 0xA1;

    packet[2] = (char) ((bodylen + 1) >> 8); // payload length includes message id
    packet[3] = (char) bodylen + 1;

    packet[4] = messageid;

    // calculate checksum
    char checksum = messageid;
    for (int i = 5; i < packetlength - 3; i++)
    {
        packet[i] = messagebody[i - 5];
        checksum ^= packet[i];
    }
    packet[packetlength - 3] = checksum;

    packet[packetlength - 2] = 0x0D; // terminate command with crlf
    packet[packetlength - 1] = 0x0A;

    // Send Packet
    _printPacket(packet, packetlength);

    char code = _sendPacket(packet, packetlength, timeout / 2);
    _debug("response code ");
    _debug(code);

    if (code != GPS_NORMAL)
    {
        _debug("failed, trying again\n");
        code = _sendPacket(packet, packetlength, timeout / 2);
        _debug("response code ");
        _debug(code);
    }
    return code;
}

char Venus838::_sendPacket(char *packet, int size, uint timeout)
{
    char c = 0;
    char last = 0;
    bool response = false;
    _gpsSerial.write(packet, size);
    // wait for repsonse
    for(uint start = millis(); millis() - start < timeout;)
    {
        while (_gpsSerial.available())
        {
            c = _gpsSerial.read();
            if (last == 0xA0 and c == 0xA1 and response == false)
                response = true;
            if (response and last == 0x83)
            {
                if (c == packet[4]) // packet[4] = messageid
                    return GPS_NORMAL;
                else
                    return GPS_UNKNOWN;
            }
            else if (response and last == 0x84)
            {
                if (c == packet[4]) // packet[4] = messageid
                    return GPS_NACK;
                else
                    return GPS_UNKNOWN;
            }
            last = c;
        }
    }
    return GPS_TIMEOUT;
}

void Venus838::_printPacket(char *packet, int size)
{
    #ifdef GPS_DEBUG_BAUDRATE
    Serial.print("assembled Packet: {");
    for (int i = 0; i < size; i++)
    {
        char hexval[4];
        sprintf(hexval, "0x%02X", packet[i]);
        Serial.print(hexval);
        if (i < size - 1) {Serial.print(", ");}
    }
    Serial.println("}");
    #endif
}

void Venus838::_debug(const char *message)
{
    #ifdef GPS_DEBUG_BAUDRATE
    Serial.print(message);
    #endif
}

void Venus838::_debug(int number)
{
    #ifdef GPS_DEBUG_BAUDRATE
    Serial.println(number);
    #endif
}
