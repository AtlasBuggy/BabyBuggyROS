#include <ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

ros::NodeHandle nh;

//sensor_msgs::Imu imu_msg;
std_msgs::Float32 message;
ros::Publisher chatter("/imu",&message);

Adafruit_BNO055 bno = Adafruit_BNO055();

void setup(){
  nh.initNode();
  nh.advertise(chatter);
}

void loop(){
  //Quaternion
  imu::Quaternion quat = bno.getQuat();
  message.data = quat.x();
  chatter.publish(&imu_msg);
  nh.spinOnce();
  delay(100);

  message.data = quat.y();
  chatter.publish(&imu_msg);
  nh.spinOnce();
  delay(100);

  message.data = quat.z();
  chatter.publish(&imu_msg);
  nh.spinOnce();
  delay(100);

  message.data = quat.w();
  chatter.publish(&imu_msg);
  nh.spinOnce();
  delay(100);

  //angular velocity
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  message.data = gyro.x();
  chatter.publish(&imu_msg);
  nh.spinOnce();
  delay(100);

  message.data = gyro.y();
  chatter.publish(&imu_msg);
  nh.spinOnce();
  delay(100);

  message.data = gyro.z();
  chatter.publish(&imu_msg);
  nh.spinOnce();
  delay(100);

  //linear acceleration
  imu::Vector<3> linaccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  message.data = linaccel.x();
  chatter.publish(&imu_msg);
  nh.spinOnce();
  delay(100);
  message.data = linaccel.y();
  chatter.publish(&imu_msg);
  nh.spinOnce();
  delay(100);
  message.data = linaccel.z();
  chatter.publish(&imu_msg);
  nh.spinOnce();
  delay(100);
  
}
  
