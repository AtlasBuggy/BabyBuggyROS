//Publishes ROS Encoder messages

#include <ros.h>
#include <std_msgs/Float64.h>
#include <Encoder.h>
#include <Atlasbuggy.h>

Atlasbuggy robot("Quadrature-Encoder");
Encoder encoder1(3, 4);
//int sensorState =s 0, lastState=0
int ticks = 0;

ros::NodeHandle nh;

const float pi = 3.14159265358979323846;
const float wheel_radius = 30.825;
const int ticks_per_rotation = 256;
const String publish_stream = "encoder";

boolean speedPublish = false;
float publish_value;

std_msgs::float flt_msg;
ros::Publisher encoder(publish_stream, &flt_msg);

void setup() {
  if (speedPublish) {
    publish_value = wheel_radius * pi / ticks_per_rotation;
  } else {
    publish_value = wheel_radius * pi;
  }
  flt_msg.data = publish_value;
  robot.begin();
  nh.initNode();
  nh.advertise(encoder);
}
void loop(){
  // read the state of the pushbutton value:
  // sensorState = digitalRead(SENSORPIN);

  robot.readSerial();
  int status = robot.readSerial();
  
  if (status == 2) {  // start event
    encoder1.write(0);
    ticks++;
  }
  if (speedPublish && tick > 0) {
    encoder.publish( &str_flt );
    nh.spinOnce();
  } else if (!speedPublish && tick == ticks_per_rotation) {
    encoder.publish( &str_flt );
    nh.spinOnce();
  }
}
