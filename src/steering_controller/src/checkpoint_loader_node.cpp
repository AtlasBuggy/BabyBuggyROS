#include "checkpoint_loader/checkpoint_loader.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "loader");
  ros::NodeHandle nh("~");

  CheckpointLoader m (&nh);

  ros::spin();
  return 0;
}