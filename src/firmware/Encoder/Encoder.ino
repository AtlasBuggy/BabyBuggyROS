//Publishes ROS Encoder messages
#include <ros.h>
#include <std_msgs/Float64.h>
#include <Encoder.h>

Encoder encoder1(3, 4);
//int sensorState =s 0, lastState=0

ros::NodeHandle nh;

const float pi = 3.14159265358979323846;
const float wheel_diameter = 30.825;
const int ticks_per_rotation = 256;
const String publish_stream = "encoder";

boolean speedPublish = false;
int prev_ticks = 0;
float tick_distance;

std_msgs::Float64 flt_msg;
ros::Publisher encoder("encoder", &flt_msg);

void setup() {
  tick_distance =  wheel_diameter * pi / ticks_per_rotation;
  nh.initNode();
  nh.advertise(encoder);
}
void loop(){
  // read the state of the pushbutton value:
  // sensorState = digitalRead(SENSORPIN);
  int encoder_read = encoder1.read();
  int ticks = encoder_read - prev_ticks;

  flt_msg.data = -(ticks * tick_distance);
  encoder.publish ( &flt_msg );
  nh.spinOnce();
  
  prev_ticks = encoder_read;
}