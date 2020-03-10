
#define BAUD_RATE   115200

#define _RUN_IMU_         1
#define _RUN_GPS_         1
#define _RUN_ENCODER_     1
#define _RUN_RF_          0

//Hertz (must divide 1000 evenly)
#define MESSAGE_FREQUENCY 10

//PINS:
//Note: 2, 3, 18, 19, 20, 21 pins for interrupts

unsigned long lastMessage = 0;

void setup() {
  Serial.begin(BAUD_RATE);

  //Initialize IMU: IMU code currently prints some outputs to the main Serial
  if(_RUN_IMU_) init_imu();
  //Initialize GPS: GPS uses Serial1 to send info to board  
  if(_RUN_GPS_) init_gps();
  if(_RUN_ENCODER_) init_encoder();
  if(_RUN_RF_) init_rf();

  lastMessage = millis();
}

void sendMessage() {
  if(_RUN_IMU_) write_imu_vals();
  if(_RUN_GPS_) write_gps_vals();
  if(!_RUN_GPS_) Serial.print("0.0\t0.0\t0.0\t");
  if(_RUN_ENCODER_) write_encoder_vals();
  if(_RUN_RF_) write_rf_vals();

  Serial.print("\n");
}

void loop() {
  if (millis() - lastMessage >= 1000 / MESSAGE_FREQUENCY) {
    if(_RUN_IMU_) update_imu();
    if(_RUN_RF_) update_rf();
    
    sendMessage();
    lastMessage = millis();
  }
}
