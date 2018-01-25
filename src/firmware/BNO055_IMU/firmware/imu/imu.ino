#include <ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32MultiArray.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

ros::NodeHandle nh;

//sensor_msgs::Imu imu_msg;
std_msgs::Float32MultiArray message;
ros::Publisher chatter("/imu",&message);

Adafruit_BNO055 bno = Adafruit_BNO055();

void setup(){
  nh.initNode();
  message.layout.dim = (std_msgs::MultiArrayDimension *)
  malloc(sizeof(std_msgs::MultiArrayDimension) * 2);
  message.layout.dim[0].size = 10;
  message.layout.dim[0].stride = 1*10;
  message.layout.data_offset = 0;
  message.layout.dim_length = 1;
  message.data_length = 10;
  message.data = (float *)malloc(sizeof(float)*10);
  nh.advertise(chatter);
}

void loop(){
  
  //Quaternion
  imu::Quaternion quat = bno.getQuat();
  message.data[0] = quat.x();

  message.data[1] = quat.y();

  message.data[2] = quat.z();

  message.data[3] = quat.w();

  //angular velocity
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  message.data[4] = gyro.x();

  message.data[5] = gyro.y();

  message.data[6] = gyro.z();

  //linear acceleration
  imu::Vector<3> linaccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  message.data[7] = linaccel.x();

  message.data[8] = linaccel.y();

  message.data[9] = linaccel.z();
  chatter.publish(&message);
  nh.spinOnce();
}
  
