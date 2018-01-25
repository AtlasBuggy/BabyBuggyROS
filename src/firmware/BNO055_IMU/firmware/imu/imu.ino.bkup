#include <ros.h>
#include <sensor_msgs/Imu.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

ros::NodeHandle nh;

sensor_msgs::Imu imu_msg;
ros::Publisher chatter("/imu",&imu_msg);

Adafruit_BNO055 bno = Adafruit_BNO055();

void setup(){
  nh.initNode();
  nh.advertise(chatter);
}

void loop(){
  //Quaternion
  imu::Quaternion quat = bno.getQuat();
  imu_msg.Quaternion.x = quat.x();
  imu_msg.Quaternion.y = quat.y();
  imu_msg.Quaternion.z = quat.z();
  imu_msg.Quaternion.w = quat.w();

  //angular velocity
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu_msg.angular_velocity.x = gyro.x();
  imu_msg.angular_velocity.y = gyro.y();
  imu_msg.angular_velocity.z = gyro.z();

  //linear acceleration
  imu::Vector<3> linaccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  imu_msg.linear_acceleration.x = linaccel.x();
  imu_msg.linear_acceleration.y = linaccel.y();
  imu_msg.linear_acceleration.z = linaccel.z();
  
  chatter.publish(&imu_msg);
  nh.spinOnce();
  delay(100);
}
	
