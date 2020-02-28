/*
    NMEAParser.hpp - Library for parsing NMEA strings from a GPS
    Decodes GGA, GSA, GSV, and RMC strings
    Reed A. Foster, July 2017.
*/

#ifndef NMEAParser_h
#define NMEAParser_h

#include "Arduino.h"
#include "Venus838.hpp"

#ifndef Venus838_h
#define NMEA_GGA 0
#define NMEA_GSA 1
#define NMEA_GSV 2
#define NMEA_GLL 3
#define NMEA_RMC 4
#define NMEA_VTG 5
#define NMEA_ZDA 6
#define NMEA_UNKNOWN 7
#endif

#define NMEASENTENCE_MAXLENGTH 120
#define NMEASENTENCE_MAXTERMS 25

class NMEAParser
{
public:

    enum {
        GPS_INVALID_DOP = 0xFFFFFFFF,       GPS_INVALID_ANGLE = 999999999,
        GPS_INVALID_ALTITUDE = 999999999,   GPS_INVALID_DATE = 0,
        GPS_INVALID_TIME = 0xFFFFFFFF,      GPS_INVALID_SPEED = 999999999,
        GPS_INVALID_FIX_TIME = 0xFFFFFFFF,  GPS_INVALID_SATELLITES = 0xFF,
        GPS_INVALID_AGE = 0xFFFFFFFF,       GPS_INVALID_SNR = 0xFFFFFFFF
    };

    NMEAParser();

    // returns true if a NMEA sentence has successfully been decoded
    bool encode(char c);

    inline unsigned long getTime() {return _time;}
    inline unsigned short getDate() {return _date;}
    inline void getLatitude(long *upper, long *lower) {*upper = _latitude_upper; *lower = _latitude_lower;}
    inline void getLongitude(long *upper, long *lower) {*upper = _longitude_upper; *lower = _longitude_lower;}
    inline long getAltitude() {return _altitude;}
    inline unsigned short getPDOP() {return _pdop;}
    inline unsigned short getVDOP() {return _vdop;}
    inline unsigned short getHDOP() {return _hdop;}
    inline unsigned char getNSats() {return _numsats;}
    inline unsigned char getFixQuality() {return _fixquality;}
    inline unsigned char getFixType() {return _fixtype;}
    inline unsigned long getSpeed() {return _speed;}
    inline unsigned short getCourse() {return _course;}
    inline unsigned long timeAge() {return millis() - _last_time_fix;}
    inline unsigned long positionAge() {return millis() - _last_position_fix;}
    inline unsigned char getNSatsVisible() {return _numsats_visible;}
    inline unsigned long getSNR() {return _snr_avg;}

private:

    int _termcmp(const char *str1, const char *str2);
    int _hexToInt(char hex);
    long _parse_decimal(char *p);
    void _parse_degrees(char *p, long *upper, long *lower);
    bool _log_sentence();

    char _sentence[NMEASENTENCE_MAXLENGTH];
    int  _char_offset;

    const char _GPGGA_TERM[7] = "$GPGGA";
    const char _GPGLL_TERM[7] = "$GPGLL";
    const char _GPGSA_TERM[7] = "$GPGSA";
    const char _GPGSV_TERM[7] = "$GPGSV";
    const char _GPRMC_TERM[7] = "$GPRMC";
    const char _GPVTG_TERM[7] = "$GPVTG";
    const char _GPZDA_TERM[7] = "$GPZDA";

    // GGA Variables
    unsigned long   _time; // UTC time in hundredths of a second
    long            _latitude_lower, _latitude_upper; // latitude in ten millionths of a degree
    long            _longitude_lower, _longitude_upper; // longitude in ten millionths of a degree
    unsigned char   _fixquality;
    unsigned char   _numsats; // number of satellites used for fix
    unsigned short  _hdop; // horizontal dilution of position (scaled by 100, i.e. 120 corresponds to a dop of 1.2)
    long            _altitude; // altitude in centimeters

    // GSA Variables
    unsigned char   _fixtype;
    unsigned short  _pdop; // positional dop (same scale as hdop)
    unsigned short  _vdop; // vertical dop (same scale as hdop)

    // GSV Variables
    unsigned char   _numsats_visible; // number of satellites visible to gps
    unsigned char   _gsv_sentence, _gsv_sentences; // counter and total for gsv messages
    unsigned char   _snr_count; // count of satellites with Signal to Noise Ratio
    unsigned long   _snr_total, _new_snr_total; // sum of Signal to Noise Ratios (C/No, in dB) from all satellites
    unsigned long   _snr_avg; // average snr of gsv message (over all sentences) scaled by 100 (i.e. 4500 corresponds to an average SNR of 45)

    // RMC Variables
    unsigned long   _speed; // speed in hundredths of kph
    unsigned short  _course; // course in hundredths of a degree
    unsigned short  _date; // UTC date

    // Add variables for additional NMEA sentences here

    // Other status
    unsigned long   _last_time_fix;
    unsigned long   _last_position_fix;
};

#endif
