/*
 * NMEA Parser
 *    Justin Brick
 *
 * 2020.03.09
 *
 * Parses NMEA Messages 
 */

#include "nmea_parser.hpp"

NMEA_Parser::NMEA_Parser() {
    writePos = 0;
    error_flag = ERR_NONE;
}

/*
    Determines whether the sentence in sentence is finished
    Checks that the last two charactesr are a carriage return
    And a line feed.

    returns true if finished, false otherwise
*/
bool NMEA_Parser::isSentenceFinished() {
    if(writePos > 2) {
        char cr = sentence[writePos - 2]; // should be carriage return  0x0D
        char lf = sentence[writePos - 1]; // should be line feed        0x0A
        return cr == '\r' && lf == '\n';
    }
    return false;
}

/*
    Explodes the given char phrase[] by the char d.
    The result is saved into the tokens array.
    numTokens is incremented by the number of different tokens.

    Return true if successful, sets the error_flag to ERR_TOKEN_OVERFLOW
        and returns false if otherwise
*/
bool NMEA_Parser::explodePhrase(char d, char phrase[], int *numTokens) {
    char token[SENTENCE_SIZE];
    int i = 0;
    int j = 0;

    while(phrase[i] != '\0' && phrase[i] != '\r') {
        if(phrase[i] == d) {
            // token delimit found
            token[j++] = 0;

            for(int x = 0; x < j; x++) tokens[*numTokens][x] = token[x];
            (*numTokens)++;

            if(*numTokens == TOKEN_SIZE) {
                error_flag = ERR_TOKEN_OVERFLOW;
                return false;
            }
            j = 0;
        }
        else token[j++] = phrase[i];
        
        i++;
    }

    token[j++] = 0;
    
    for(int x = 0; x < j; x++) tokens[*numTokens][x] = token[x];
    (*numTokens)++;

    return true;
}

/*
    Checks whether a given msg matches the checksum int by
    XORing all the chars together.

    The raw sentence should appear as:
        msg*XX
    where msg is msg, and XX is the hex representation of checksum

    Returns true if valid, sets the error_flag to ERR_CHECKSUM
        and returns false otherwise
*/
bool NMEA_Parser::validChecksum(char msg[], int checksum) {
    int i = 1;

    while(msg[i] != 0) checksum = checksum ^ msg[i++];

    if(checksum != 0) {
        error_flag = ERR_CHECKSUM;
        return false;
    }
    return true;
}

/*
    Encodes the given char into the sentence, then tries to evaluate
    the sentence if it is finished. If a sentence is finished, then
    the sentence is cleared after attempting to evaluate it.

    Read a byte in from the GPS and then encode it with this function.

    Returns true if finished and succesfully evaluated
    Returns false otherwise, Sets error flag to:
        ERR_CHECKSUM if checksum is invalid
        ERR_CHECKSUM_MALFORM if checksum couldn't be found
        ERR_TOKEN_OVERFLOW on explosion overflow
        ERR_MSG_MALFORM on incorrect number of tokens
        ERR_UNKNOWN_MSG on unknown message type
*/
bool NMEA_Parser::encode(char c) {
    sentence[writePos++] = c;
    bool out = false;
    if(isSentenceFinished()) {
        // sentence should be complete
        writePos = 0;
        int numTokens = 0;

        // break apart the checksum and the message
        if(explodePhrase(CHECKSUM_DELIMIT, sentence, &numTokens)) {
            if(numTokens != 2) {
                error_flag = ERR_CHECKSUM_MALFORM;
                return false;
            }

            // convert checksum byte to int
            int checksum = 16 * hexToInt(tokens[1][0]) + hexToInt(tokens[1][1]);

            if(validChecksum(tokens[0], checksum)) {
                // checksum is valid
                numTokens = 0;

                // break apart the tokens/arguments of the message
                if(explodePhrase(TOKEN_DELIMIT, tokens[0], &numTokens)) {
                    eval lex = findEvaluator(tokens[0]);
                    out = (this->*lex)(tokens, numTokens);
                }
            }
        }

        for(int i = 0; i < SENTENCE_SIZE; i++) sentence[i] = '\0';

        writePos = 0;
    }
    return out;
}

/* 

    Evaluators 

    Takes tokens and writes the values into the corresponding variables.
    Returns true if successful, returns false and sets error flag to 
        ERR_MSG_MALFORM otherwise.

*/

bool NMEA_Parser::evalGGA(char tokens[TOKEN_SIZE][SENTENCE_SIZE], int numTokens) {
    if(numTokens < 13) {
        error_flag = ERR_MSG_MALFORM;
        return false;
    }
    char *time = tokens[1];
    hours = decToInt(*(time + 0)) * 10 + decToInt(*(time + 1));
    minutes = decToInt(*(time + 2)) * 10 + decToInt(*(time + 3));
    seconds = wordToDouble(time + 4);

    char *lat = tokens[2];
    north = *tokens[3] == 'N';
    latitude_deg = decToInt(*(lat + 0)) * 10 + decToInt(*(lat + 1));
    latitude_min = wordToDouble(lat + 2);
    latitude = (double) latitude_deg + ((double) latitude_min / 60.0);
    if(!north) latitude = -latitude;

    char *lon = tokens[4];
    west = *tokens[5] == 'W';
    longitude_deg = decToInt(*(lon + 0)) * 100 + decToInt(*(lon + 1)) * 10 + decToInt(*(lon + 2));
    longitude_min = wordToDouble(lon + 3);
    longitude = (double) longitude_deg + ((double) longitude_min / 60.0);
    if(west) longitude = -longitude;

    gps_quality = decToInt(*tokens[6]);
    satellites = (int) wordToDouble(tokens[7]);
    hdop = wordToDouble(tokens[8]);

    altitude = wordToDouble(tokens[9]);

    geoidal_seperation = wordToDouble(tokens[10]);
    age = wordToDouble(tokens[11]);
    station_id = (int) wordToDouble(tokens[12]);

    return true;
}

bool NMEA_Parser::evalGLL(char tokens[TOKEN_SIZE][SENTENCE_SIZE], int numTokens) {
    if(numTokens < 8) {
        error_flag = ERR_MSG_MALFORM;
        return false;
    }
    char *lat = tokens[1];
    north = *tokens[2] == 'N';
    latitude_deg = decToInt(*(lat + 0)) * 10 + decToInt(*(lat + 1));
    latitude_min = wordToDouble(lat + 2);
    latitude = (double) latitude_deg + ((double) latitude_min / 60.0);
    if(!north) latitude = -latitude;

    char *lon = tokens[3];
    west = *tokens[4] == 'W';
    longitude_deg = decToInt(*(lon + 0)) * 100 + decToInt(*(lon + 1)) * 10 + decToInt(*(lon + 2));
    longitude_min = wordToDouble(lon + 3);
    longitude = (double) longitude_deg + ((double) longitude_min / 60.0);
    if(west) longitude = -longitude;

    char *time = tokens[5];
    hours = decToInt(*(time + 0)) * 10 + decToInt(*(time + 1));
    minutes = decToInt(*(time + 2)) * 10 + decToInt(*(time + 3));
    seconds = wordToDouble(time + 4);

    valid = tokens[6][0] == 'A';
    mode_indicator = tokens[7][0];

    return true;
}

bool NMEA_Parser::evalGSA(char tokens[TOKEN_SIZE][SENTENCE_SIZE], int numTokens) {
    if(numTokens < 7) {
        error_flag = ERR_MSG_MALFORM;
        return false;
    }
    automatic_dimension = tokens[1][0] == 'A';
    fix_type = decToInt(tokens[2][0]);

    for(int i = 0; i < 12; i++) satellites_used[i] = 0;
    for(int i = 3; i < numTokens - 3; i++) satellites_used[i - 3] = (int) wordToLong(tokens[i]);

    vdop = wordToDouble(tokens[numTokens - 1]);
    hdop = wordToDouble(tokens[numTokens - 2]);
    pdop = wordToDouble(tokens[numTokens - 3]);
    return true;
}

bool NMEA_Parser::evalGSV(char tokens[TOKEN_SIZE][SENTENCE_SIZE], int numTokens) {
    if(numTokens < 8) {
        error_flag = ERR_MSG_MALFORM;
        return false;
    }
    num_msg = decToInt(tokens[1][0]);
    sequence = decToInt(tokens[2][0]);
    satellites = (int) wordToLong(tokens[3]);
    satellite_id = (int) wordToLong(tokens[4]);

    elevation = (int) wordToLong(tokens[5]);
    azimuth = (int) wordToLong(tokens[6]);
    snr = (int) wordToLong(tokens[7]);
    return true;
}

bool NMEA_Parser::evalRMC(char tokens[TOKEN_SIZE][SENTENCE_SIZE], int numTokens) {
    if(numTokens < 11) {
        error_flag = ERR_MSG_MALFORM;
        return false;
    }
    char *time = tokens[1];
    hours = decToInt(*(time + 0)) * 10 + decToInt(*(time + 1));
    minutes = decToInt(*(time + 2)) * 10 + decToInt(*(time + 3));
    seconds = wordToDouble(time + 4);

    valid = tokens[2][0] == 'A';

    char *lat = tokens[3];
    north = *tokens[4] == 'N';
    latitude_deg = decToInt(*(lat + 0)) * 10 + decToInt(*(lat + 1));
    latitude_min = wordToDouble(lat + 2);
    latitude = (double) latitude_deg + ((double) latitude_min / 60.0);
    if(!north) latitude = -latitude;

    char *lon = tokens[5];
    west = *tokens[6] == 'W';
    longitude_deg = decToInt(*(lon + 0)) * 100 + decToInt(*(lon + 1)) * 10 + decToInt(*(lon + 2));
    longitude_min = wordToDouble(lon + 3);
    longitude = (double) longitude_deg + ((double) longitude_min / 60.0);
    if(west) longitude = -longitude;

    ground_speed_knots = wordToDouble(tokens[7]);
    course_speed = wordToDouble(tokens[8]);

    char *date = tokens[9];
    day = decToInt(*(time + 0)) * 10 + decToInt(*(time + 1));
    month = decToInt(*(time + 2)) * 10 + decToInt(*(time + 3));
    year = decToInt(*(time + 4)) * 10 + decToInt(*(time + 5));

    mode_indicator = tokens[10][0];

    return true;
}

bool NMEA_Parser::evalVTG(char tokens[TOKEN_SIZE][SENTENCE_SIZE], int numTokens) {
    if(numTokens < 5) {
        error_flag = ERR_MSG_MALFORM;
        return false;
    }
    course_speed = wordToDouble(tokens[1]);
    ground_speed_knots = wordToDouble(tokens[2]);
    ground_speed_km = wordToDouble(tokens[3]);

    mode_indicator = tokens[4][0];
    return true;
}

bool NMEA_Parser::evalZDA(char tokens[TOKEN_SIZE][SENTENCE_SIZE], int numTokens) {
    if(numTokens < 7) {
        error_flag = ERR_MSG_MALFORM;
        return false;
    }
    char *time = tokens[1];
    hours = decToInt(*(time + 0)) * 10 + decToInt(*(time + 1));
    minutes = decToInt(*(time + 2)) * 10 + decToInt(*(time + 3));
    seconds = wordToDouble(time + 4);

    day = (int) wordToLong(tokens[2]);
    month = (int) wordToLong(tokens[3]);
    year = (int) wordToLong(tokens[4]);

    local_hour = (int) wordToLong(tokens[5]);
    local_min = (int) wordToLong(tokens[6]);

    return true;
}

/*
    This is the default evaluator if the message type could not be
    determined.
*/
bool NMEA_Parser::evalNone(char tokens[TOKEN_SIZE][SENTENCE_SIZE], int numTokens) {
    error_flag = ERR_UNKNOWN_MSG;
    return false;
}

/*
    Returns the evaluator given a tag input. In a NMEA sentence, the tag
    should be the first token.

    If the type could not be determined, returns evalNone.
*/
NMEA_Parser::eval NMEA_Parser::findEvaluator(char tag[]) {
    if(cmpCharArray(tag, GPGGA_TERM, 7, 7) == 0) return &NMEA_Parser::evalGGA;
    if(cmpCharArray(tag, GPGLL_TERM, 7, 7) == 0) return &NMEA_Parser::evalGLL;

    if(cmpCharArray(tag, GPGSA_TERM, 7, 7) == 0
    || cmpCharArray(tag, GLGSA_TERM, 7, 7) == 0
    || cmpCharArray(tag, BDGSA_TERM, 7, 7) == 0) return &NMEA_Parser::evalGSA;

    if(cmpCharArray(tag, GPGSV_TERM, 7, 7) == 0
    || cmpCharArray(tag, GLGSV_TERM, 7, 7) == 0
    || cmpCharArray(tag, BDGSV_TERM, 7, 7) == 0) return &NMEA_Parser::evalGSV;

    if(cmpCharArray(tag, GPRMC_TERM, 7, 7) == 0) return &NMEA_Parser::evalRMC;
    if(cmpCharArray(tag, GPVTG_TERM, 7, 7) == 0) return &NMEA_Parser::evalVTG;
    if(cmpCharArray(tag, GPZDA_TERM, 7, 7) == 0) return &NMEA_Parser::evalZDA;
    
    return &NMEA_Parser::evalNone;
}

/*

    Utility functions

*/

/*
    Compares two char arrays M and N. if the lengths are not equal, then m - n.
    Returns the ORs of the difference between the chars otherwise.
*/
int NMEA_Parser::cmpCharArray(const char M[], const char N[], const int m, const int n) {
    int delta = 0;

    if(m - n != 0) return m - n;

    for(int i = 0; i < m && i < n; i++) {
        delta |= M[i] - N[i];
    }

    return delta;
}

/*
    Takes a hexadecimal digit and converts it to an int.
    Case insensitive.

    Returns -1 if not a valid digit.
*/
int NMEA_Parser::hexToInt(char hex) {
    if (hex >= 'A' && hex <= 'F') return hex - 'A' + 10;
    else if (hex >= 'a' && hex <= 'f') return hex - 'a' + 10;
    return decToInt(hex);
}

/*
    Converts a decimal digit into an int.

    Returns -1 if not a valid digit.
*/
int NMEA_Parser::decToInt(char dec) {
    if('0' <= dec && dec <= '9') return dec - '0';
    return -1;
}

/*
    Parses a word into a long. If there is a decimal point, the decimal
    point is ignored. Runs until there is a non-digit/non-period character
*/
long NMEA_Parser::wordToLong(char *dec) {
    bool neg = *dec == '-';
    if(neg) dec++;

    long result = 0;
    int d = 0;

    while(*dec == '.' || (d = decToInt(*dec)) != -1) {
        if(*dec++ == '.') continue;
        result = result * 10 + d;
    }

    return neg ? -result : result;
}

/*
    Parses a word into a double. Runs until there is a non-digit character
*/
double NMEA_Parser::wordToDouble(char *dec) {
    bool neg = *dec == '-';
    if(neg) dec++;

    long result = 0;
    int d = 0;

    while((d = decToInt(*dec)) != -1) {
        result = result * 10 + d;
        dec++;
    }

    if(*dec != '.') return result;
    dec++;

    double offset = 1.0;
    while((d = decToInt(*dec)) != -1) {
        result = result * 10 + d;
        offset = offset * 10.0;
        dec++;
    }

    double out = ((double) result) / offset;
    return neg ? -out : out;
}
