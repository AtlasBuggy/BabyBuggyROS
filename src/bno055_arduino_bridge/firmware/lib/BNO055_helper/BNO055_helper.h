
#ifndef __BNO055_HELPER_H__
#define __BNO055_HELPER_H__

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <EEPROM.h>

/* ----------------------- *
 * BNO055 global variables *
 * ----------------------- */

#define INCLUDE_FILTERED_DATA
// #define USE_QUATERNIONS
// #define INCLUDE_MAG_DATA
#define INCLUDE_GYRO_DATA
// #define INCLUDE_ACCEL_DATA
#define INCLUDE_LINACCEL_DATA

#define BNO055_SAMPLERATE_DELAY_MS (100)

void displaySensorDetails(void);
void displaySensorStatus(void);

void displayCalStatus(void);
void displaySensorOffsets(const adafruit_bno055_offsets_t &calibData);
void initIMU();
void updateIMU();

#endif  // __BNO055_HELPER_H__
