//#include "update_gps"
//#include "update_imu"

//#include "libraries/DuneBuggyIMU.cpp"

#define _BAUD_RATE_   115200

#define _RUN_IMU_         1
#define _RUN_GPS_         1
#define _RUN_ENCODER_     1
#define _RUN_RF_          0

//PINS:
//Note: 2, 3, 18, 19, 20, 21 pins for interrupts

const int FREQUENCY = 10; //Hertz (must divide 1000 evenly)

unsigned long lastTime = 0;

void setup() {
  Serial.begin(_BAUD_RATE_);

#if (_RUN_IMU_)
  //Initialize IMU: IMU code currently prints some outputs to the main Serial
  init_imu();
#endif
#if (_RUN_GPS_)
  //Initialize GPS: GPS uses Serial1 to send info to board  
  init_gps();
#endif
#if (_RUN_ENCODER_)
  init_encoder();
#endif
#if (_RUN_RF_)
  init_rf();
#endif

  lastTime = millis();
}

void sendMessage() {

#if (_RUN_IMU_)
  write_imu_vals();
#endif
#if (_RUN_GPS_)
  write_gps_vals();
#endif
#if (_RUN_ENCODER_)
  write_encoder_vals();
#endif
#if (_RUN_RF_)
  write_rf_vals();
#endif

  Serial.print("\n");
}

void loop() {
  if (millis() - lastTime >= 1000 / FREQUENCY) {
    
#if (_RUN_IMU_)
    update_imu();
#endif
#if (_RUN_RF_)
    update_rf();
#endif
    sendMessage();
    lastTime = millis();
  }

}
