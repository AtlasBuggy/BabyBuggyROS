/*
    NMEAParser.cpp - Library for parsing NMEA strings from a GPS
    Decodes GGA, GSA, GSV, and RMC strings
    Reed A. Foster, July 2017.
*/

#include "NMEAParser.hpp"

NMEAParser::NMEAParser() :

    _char_offset(0),

    _time(GPS_INVALID_TIME),
    _latitude_lower(GPS_INVALID_ANGLE),
    _latitude_upper(GPS_INVALID_ANGLE),
    _longitude_lower(GPS_INVALID_ANGLE),
    _longitude_upper(GPS_INVALID_ANGLE),
    _fixquality(-1),
    _numsats(GPS_INVALID_SATELLITES),
    _hdop(GPS_INVALID_DOP),
    _altitude(GPS_INVALID_ALTITUDE),

    _fixtype(0),
    _pdop(GPS_INVALID_DOP),
    _vdop(GPS_INVALID_DOP),

    _numsats_visible(GPS_INVALID_SATELLITES),
    _gsv_sentence(0),
    _gsv_sentences(0),
    _snr_count(0),
    _snr_total(GPS_INVALID_SNR),
    _new_snr_total(0),
    _snr_avg(GPS_INVALID_SNR),

    _speed(GPS_INVALID_SPEED),
    _course(GPS_INVALID_ANGLE),
    _date(GPS_INVALID_DATE),

    _last_time_fix(GPS_INVALID_FIX_TIME),
    _last_position_fix(GPS_INVALID_FIX_TIME)
{
    _sentence[0] = 0;
}

bool NMEAParser::encode(char c)
{
    bool new_data = false;
    if (c == '\n')
    {
        if (_sentence[_char_offset - 4] == '*') // checksum exists
        {
            int checksum = _hexToInt(_sentence[_char_offset - 2]) + _hexToInt(_sentence[_char_offset - 3]) * 16;
            _char_offset -= 5;
            while (_sentence[_char_offset] && _sentence[_char_offset] != '$')
                checksum ^= _sentence[_char_offset--];
            if (checksum == 0) // checksum is valid
            {
                new_data = _log_sentence();
            }
            _char_offset = 0;
            _sentence[0] = 0;
        }
    }
    else
        _sentence[_char_offset++] = c;
    return new_data;
}

bool NMEAParser::_log_sentence()
{
    long logtime = millis();

    // determine sentence type
    int sentence_type;
    char title[7];
    for (int i = 0; i < 6; i++)
        title[i] = _sentence[i];
    title[6] = 0;
    if (!_termcmp(title, _GPGGA_TERM))
        sentence_type = NMEA_GGA;
    else if (!_termcmp(title, _GPGSA_TERM))
        sentence_type = NMEA_GSA;
    else if (!_termcmp(title, _GPGSV_TERM))
        sentence_type = NMEA_GSV;
    else if (!_termcmp(title, _GPRMC_TERM))
        sentence_type = NMEA_RMC;

    // Add additional NMEA sentence type here

    else
        sentence_type = NMEA_UNKNOWN;

    if (sentence_type == NMEA_UNKNOWN)
        return false;

    char *p;
    p = _sentence;

    while (*p++ != ',') ; // skip to next term

    // verify validity of sentences (if possible)
    bool data_valid = false;
    switch (sentence_type)
    {
        case NMEA_GGA:
            for (int i = 0; i < 5; i++) // skip over sentence until start of 5th term
                while (*p++ != ',') ;
            data_valid = *p > '0';
            break;
        case NMEA_GSA:
            for (int i = 0; i < 1; i++)
                while (*p++ != ',') ;
            data_valid = *p > '1';
            break;
        case NMEA_GSV:
            data_valid = true;
            break;
        case NMEA_RMC:
            for (int i = 0; i < 1; i++)
                while (*p++ != ',') ;
            data_valid = *p == 'A';
            break;

        // Add validity checking for additional NMEA sentences here

    }
    p = _sentence;

    if (!data_valid)
        return false;

    while (*p++ != ',') ; //advance to first term after sentence id/title

    // decode data
    int term_number = 0;
    bool decoded = false;
    while (!decoded)
    {
        switch (sentence_type)
        {
            case NMEA_GGA:
                switch (term_number)
                {
                    case 0: // UTC Time
                        _last_time_fix = logtime;
                        _time = _parse_decimal(p);
                        break;
                    case 1: // Latitude
                        _last_position_fix = logtime;
                        _parse_degrees(p, &_latitude_upper, &_latitude_lower);
                        break;
                    case 2: // Latitude Indicator
                        _latitude_upper = *p == 'S' ? -_latitude_upper : _latitude_upper;
                        break;
                    case 3: // Longitude
                        _parse_degrees(p, &_longitude_upper, &_longitude_lower);
                        break;
                    case 4: // Longitude Indicator
                        _longitude_upper = *p == 'W' ? -_longitude_upper : _longitude_upper;
                        break;
                    case 5: // Fix Quality
                        _fixquality = *p - '0';
                        break;
                    case 6: // Number of Satellites (tracked/used for fix)
                        _numsats = (*p - '0') * 10 + *(p + 1) - '0';
                        break;
                    case 7: // HDOP
                        _hdop = (short) _parse_decimal(p);
                        break;
                    case 8: // Altitude
                        _altitude = _parse_decimal(p);
                        break;
                }
                break;
            case NMEA_GSA:
                switch (term_number)
                {
                    case 1: // Fix Type
                        _fixtype = *p - '0';
                        break;
                    case 14: // PDOP
                        _pdop = (short) _parse_decimal(p);
                        break;
                    case 15: // HDOP
                        _hdop = (short) _parse_decimal(p);
                        break;
                    case 16: // VDOP
                        _vdop = (short) _parse_decimal(p);
                        break;
                }
                break;
            case NMEA_GSV:
                switch (term_number)
                {
                    case 0: // GSV Sentence Count
                        _gsv_sentences = *p - '0';
                        break;
                    case 1: // GSV Current Sentence Number
                        _gsv_sentence = *p - '0';
                        break;
                    case 2: // Number of Satellites (in view)
                        _numsats_visible = (*p - '0') * 10 + *(p + 1) - '0';
                        break;
                    case 6: // SNR 1
                    case 10: // SNR 2
                    case 14: // SNR 3
                    case 18: // SNR 4
                        if (*p >= '0' && *p <= '9' && *(p + 1) >= '0' && *(p + 1) <= '9')
                            _snr_count++;
                        _new_snr_total += _parse_decimal(p);
                        char *ptemp;
                        ptemp = p;
                        while (*ptemp != ',' && *ptemp != '*') ptemp++;
                        if ((_gsv_sentence == _gsv_sentences) && (*ptemp ==  '*')) // check to see if multiline gsv message is complete
                        {
                            _snr_total = _new_snr_total;
                            _snr_avg = _snr_total / _snr_count;
                            _new_snr_total = 0;
                            _snr_count = 0;
                        }
                        break;
                }
                break;
            case NMEA_RMC:
                switch (term_number)
                {
                    case 0: // UTC Time
                        _last_time_fix = logtime;
                        _time = _parse_decimal(p);
                        break;
                    case 2: // Latitude
                        _last_position_fix = logtime;
                        _parse_degrees(p, &_latitude_upper, &_latitude_lower);
                        break;
                    case 3: // Latitude Indicator
                        _latitude_upper = *p == 'S' ? -_latitude_upper : _latitude_upper;
                        break;
                    case 4: // Longitude
                        _parse_degrees(p, &_longitude_upper, &_longitude_lower);
                        break;
                    case 5: // Longitude Indicator
                        _longitude_upper = *p == 'W' ? -_longitude_upper : _longitude_upper;
                        break;
                    case 6: // Speed
                        _speed = _parse_decimal(p);
                        break;
                    case 7: // Course
                        _course = _parse_decimal(p);
                        break;
                    case 8: // UTC Date
                        _date = _parse_decimal(p) / 100;
                        break;
                }
                break;

            // Add functionality for additional NMEA sentences here

        }
        while (*p != ',' && *p != '*')
            p++;
        p++;
        term_number++;
        if (*(p - 1) == '*')
        {
            decoded = true;
        }
        if (term_number > NMEASENTENCE_MAXTERMS)
            return false;
    }
    return true;
}

int NMEAParser::_termcmp(const char *str1, const char *str2)
{
    while (*str1 && (*str1 == *str2))
        str1++, str2++;
    return (*str1 > *str2) - (*str2 > *str1);
}

int NMEAParser::_hexToInt(char hex)
{
    if (hex >= 'A' && hex <= 'F')
        return hex - 'A' + 10;
    else if (hex >= 'a' && hex <= 'f')
        return hex - 'a' + 10;
    else
        return hex - '0';
}

long NMEAParser::_parse_decimal(char *p)
{
    bool neg = *p == '-';
    if (neg) p++;
    long ret = 0L;
    while (*p >= '0' && *p <= '9')
        ret = ret * 10L + *p++ - '0';
    ret *= 100L;
    if (*p++ == '.')
    {
        if (*p >= '0' && *p <= '9')
        {
            ret += 10L * (*p++ - '0');
            if (*p >= '0' && *p <= '9')
                ret += *p - '0';
        }
    }
    return neg ? -ret : ret;
}

void NMEAParser::_parse_degrees(char *p, long *upper, long *lower)
{
    long deg = 0L;
    while ((*(p + 2) != '.') && (*p >= '0' && *p <= '9'))
        deg = deg * 10L + *p++ - '0';
    *upper = deg;

    long min = (*p++ - '0') * 10;
    min += *p++ - '0';
    p++;
    while ((*p >= '0' && *p <= '9'))
        min = min * 10L + *p++ - '0';

    min *= 100L;

    *lower = min / 6;
}
