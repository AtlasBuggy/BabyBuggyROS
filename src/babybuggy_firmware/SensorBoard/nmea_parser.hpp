#define SENTENCE_SIZE       100

#define TOKEN_SIZE          15
#define TOKEN_DELIMIT       ','
#define CHECKSUM_DELIMIT    '*'

#define ERR_NONE            0
#define ERR_CHECKSUM        1
#define ERR_UNKNOWN_MSG     2
#define ERR_TOKEN_OVERFLOW  3
#define ERR_MSG_MALFORM     4

class NMEA_Parser {
    public:
        typedef bool (NMEA_Parser::*eval)(char tokens[TOKEN_SIZE][SENTENCE_SIZE], int numTokens);

        NMEA_Parser();
        char sentence[SENTENCE_SIZE];
        int flag;

        bool encode(char c);

        inline unsigned getErrorFlag() { return error_flag; }
        inline void clearErrorFlag() { error_flag = ERR_NONE; }

        // Generic Message
        inline int getHours() { return hours; }
        inline int getMinutes() { return minutes; }
        inline double getSecounds() { return seconds; }

        inline double getLatitude() { return latitude; }
        inline int getLatitudeDegrees() { return latitude_deg; }
        inline double getLatitudeMinutes() { return latitude_min; }
        inline bool isNorth() { return north; }
        inline double getLongitude() { return longitude; }
        inline int getLongitudeDegrees() { return longitude_deg; }
        inline double getLongitudeMinutes() { return longitude_min; }
        inline bool isWest() { return west; }

        inline int getSatellites() { return satellites; }
        inline char getModeIndicator() { return mode_indicator; }

        // GGA
        inline int getGPSQuality() { return gps_quality; }
        inline double getHDOP() { return hdop; }

        inline double getAltitude() { return altitude; }

        inline double getGeoidalSeperation() { return geoidal_seperation; }
        inline double getAgeDifferential() { return age; }
        inline int getStationID() { return station_id; }

        // GLL
        inline bool isValid() { return valid; }

        // GSA
        inline bool isAutomaticDimension() { return automatic_dimension; }
        inline int getFixType() { return fix_type; }
        inline int *getSatellitesUsed() { return satellites_used; }
        inline double getPDOP() { return pdop; }
        inline double getVDOP() { return vdop; }

        // GSV
        inline int getNumberOfMessages() { return num_msg; }
        inline int getSequence() { return sequence; }
        inline int getElevation() { return elevation; }
        inline int getAzimuth() { return azimuth; }
        inline int getSNR() { return snr; }

        // RMC / VTG
        inline double getGroundSpeedKnots() { return ground_speed_knots; }
        inline double getGroundSpeedKm() { return ground_speed_km; }
        inline double getCourseSpeed() { return course_speed; }

        // ZDA
        inline int getDay() { return day; }
        inline int getMonth() { return month; }
        inline int getYear() { return year; }
        inline int getLocalHour() { return local_hour; }
        inline int getLocalMinute() { return local_min; }
        
    private:
        unsigned error_flag = ERR_NONE;
        int writePos = 0;

        const char GPGGA_TERM[7] = "$GPGGA";
        const char GPGLL_TERM[7] = "$GPGLL";
        const char GPGSA_TERM[7] = "$GPGSA";
        const char GPGSV_TERM[7] = "$GPGSV";
        const char GPRMC_TERM[7] = "$GPRMC";
        const char GPVTG_TERM[7] = "$GPVTG";
        const char GPZDA_TERM[7] = "$GPZDA";

        // generic
        int hours;
        int minutes;
        double seconds;

        double latitude = 0;
        int latitude_deg = 0;
        double latitude_min = 0;
        bool north = true;

        double longitude = 0;
        int longitude_deg = 0;
        double longitude_min = 0;
        bool west = true;

        int satellites;
        char mode_indicator;

        // GGA
        int gps_quality;
        double hdop;

        double altitude = 0;

        double geoidal_seperation;
        double age;
        int station_id;

        // GLL
        bool valid;

        // GSA
        bool automatic_dimension;
        int fix_type;
        int satellites_used[12];
        double pdop;
        double vdop;

        // GSV
        int num_msg;
        int sequence;
        int satellite_id;
        int elevation; // degrees
        int azimuth;
        int snr;

        // RMC / VTG
        double ground_speed_knots;
        double ground_speed_km; // VTG
        double course_speed; // degrees

        // ZDA
        int day;
        int month;
        int year;
        int local_hour;
        int local_min;


        bool isSentenceFinished();
        bool explodeSentence(char d, char sentence[], int *numTokens);
        bool validChecksum(char msg[], int checksum);

        // evaluators
        eval findEvaluator(char tag[]);
        bool evalGGA(char tokens[TOKEN_SIZE][SENTENCE_SIZE], int numTokens);
        bool evalGLL(char tokens[TOKEN_SIZE][SENTENCE_SIZE], int numTokens);
        bool evalGSA(char tokens[TOKEN_SIZE][SENTENCE_SIZE], int numTokens);
        bool evalGSV(char tokens[TOKEN_SIZE][SENTENCE_SIZE], int numTokens);
        bool evalRMC(char tokens[TOKEN_SIZE][SENTENCE_SIZE], int numTokens);
        bool evalVTG(char tokens[TOKEN_SIZE][SENTENCE_SIZE], int numTokens);
        bool evalZDA(char tokens[TOKEN_SIZE][SENTENCE_SIZE], int numTokens);
        inline bool evalNone(char tokens[TOKEN_SIZE][SENTENCE_SIZE], int numTokens) { return !(error_flag = ERR_UNKNOWN_MSG); }

        // utility functions
        void copyCharArray(char tokens[][SENTENCE_SIZE], char token[SENTENCE_SIZE], int num, int j);
        int cmpCharArray(const char M[], const char N[], const int m, const int n);
        int hexToInt(char hex);
        int decToInt(char dec);
        double wordToDouble(char *dec);
        long wordToLong(char *dec);
};
