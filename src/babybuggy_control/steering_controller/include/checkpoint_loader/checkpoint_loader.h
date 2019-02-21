#ifndef _CHECKPOINT_LOADER_H_
#define _CHECKPOINT_LOADER_H_

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "steering_controller/RelativePose.h"

#include <iostream>
#include <fstream>

using namespace std;

class CheckpointLoader{
private:
  ros::NodeHandle nh;

  ros::Subscriber cur_pose_sub;
  ros::Publisher rel_pose_pub;

  float cur_pose_x;
  float cur_pose_y;
  float cur_pose_th;

  string checkpoint_dir;
  string filename;
  vector<float> checkpoint_x;
  vector<float> checkpoint_y;
  vector<float> checkpoint_th;
  int cur_checkpoint_index;

  void CurPoseCallback(const nav_msgs::Odometry& msg);

  int findCurrentCheckpoint();
  void loadCheckpoints();
public:
  CheckpointLoader(ros::NodeHandle* nodehandle);
  void run();
};

#endif // _CHECKPOINT_LOADER_H_