#include "checkpoint_loader/checkpoint_loader.h"

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "loader");
  ros::NodeHandle nh;

  CheckpointLoader m (&nh);

  ros::spin();
  return 0;
}