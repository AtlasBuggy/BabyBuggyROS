#include <Arduino.h>

#define DEFAULT_RATE 115200
#define LED13 13
#define PACKET_END '\n'

class SerialManager {
public:
    SerialManager();
    SerialManager(int baud);
    void begin();
    int readSerial();
    String getCommand();
    bool isPaused();

    void changeBaud(int newBaud);
    bool available();

    bool unpause();
    bool pause();

private:
    String _command;
    bool _paused;
    int _baud;
};
