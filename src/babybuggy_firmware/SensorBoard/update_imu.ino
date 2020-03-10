#include <Wire.h>
#include "libraries/Adafruit_Sensor.h"
#include "libraries/Adafruit_BNO055.cpp"
#include "libraries/utility/imumaths.h"

/* This driver reads raw data from the BNO055

   Connections
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 3.3V DC
   Connect GROUND to common ground

   History
   =======
   2015/MAR/03  - First release (KTOWN)
*/

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);
imu::Quaternion quat;

double imu_vals[4];
int safe_imu = 0;

bool isSafe_IMU() {
  return safe_imu == 4;
}

void init_imu() {
  /* Initialise the sensor */
  if(!bno.begin()) {    // DEFAULT  = bno.OPERATION_MODE_NDOF
                        // OTHER    = bno.OPERATION_MODE_COMPASS
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  bno.setExtCrystalUse(true);
}

bool update_imu() {
  safe_imu = 0;
  quat = bno.getQuat();
  imu_vals[safe_imu++] = quat.x();
  imu_vals[safe_imu++] = quat.y();
  imu_vals[safe_imu++] = quat.z();
  imu_vals[safe_imu++] = quat.w();
 
  return true;
}

void write_imu_vals() {
  for(int i = 0; i < 4; i++) {
    Serial.print(imu_vals[i], 4);
    Serial.print("\t");
  }
}
