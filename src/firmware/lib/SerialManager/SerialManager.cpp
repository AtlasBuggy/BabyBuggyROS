
#include <SerialManager.h>
// #define DEBUG

SerialManager::SerialManager()
{
    _command = "";
    _paused = true;
    _baud = DEFAULT_RATE;
}

SerialManager::SerialManager(int baud)
{
    _command = "";
    _paused = true;
    _baud = baud;
}

void SerialManager::begin()
{
    Serial.begin(_baud);
    Serial.setTimeout(100);  // 100 ms
    #ifdef DEBUG
    Serial.print("BAUD is ");
    Serial.println(DEFAULT_RATE);
    #endif
}

bool SerialManager::available() {
    return Serial.available() > 0;
}

int SerialManager::readSerial()
{
    if (_paused) {
        delay(100);  // minimize activity while paused
    }
    _command = Serial.readStringUntil('\n');
    #ifdef DEBUG
    Serial.println(_command);
    #endif

    if (_command.equals("g")) {
        if (unpause()) return 2;
        else return -1;
    }
    else if (_command.equals("s")) {
        if (pause()) return 1;
        else return -1;
    }
    else {
        if (!_paused) return 0;
        else return -1;
    }
}

void SerialManager::changeBaud(int newBaud)
{
    if (newBaud == _baud) {
        #ifdef DEBUG
        Serial.println("new baud not different");
        #endif
        return;
    }
    #ifdef DEBUG
    Serial.println("changing baud");
    #endif

    _baud = newBaud;

    delay(50);
    Serial.end();
    delay(50);
    Serial.begin(newBaud);
}

String SerialManager::getCommand() {
    return _command;
}

bool SerialManager::isPaused() {
    return _paused;
}


bool SerialManager::unpause()
{
    if (_paused) {
        Serial.print("\nstarting\n");
        _paused = false;
        return true;
    }
    else {
        return false;
    }
}

bool SerialManager::pause()
{
    if (!_paused) {
        Serial.print("\nstopping\n");
        _paused = true;
        return true;
    }
    else {
        return false;
    }
}
