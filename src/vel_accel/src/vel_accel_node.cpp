#include "vel_accel/vel_accel.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  ros::NodeHandle nh;

  VelAccel m(&nh);

  return 0;
}