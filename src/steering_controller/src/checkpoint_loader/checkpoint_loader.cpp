#include "checkpoint_loader/checkpoint_loader.h"

CheckpointLoader::CheckpointLoader(ros::NodeHandle* nodehandle) : nh(*nodehandle)
{
  // initialize subscriber and publisher
  cur_pose_sub = nh.subscribe("/cur_pose", 100, &CheckpointLoader::CurPoseCallback, this);
  rel_pose_pub = nh.advertise<steering_controller::RelativePose>("rel_pose", 100);

  // initial current poses
  cur_pose_x = 0;
  cur_pose_y = 0;
  cur_pose_th = 500;

  // get filename from parameter
  nh.param<std::string>("filename", filename, "checkpoints.txt");
  nh.param<std::string>("checkpoint_dir", checkpoint_dir, "/home/nvidia/code/BabyBuggyROS/checkpoints");

  // load checkpoints from file
  loadCheckpoints();
}

void CheckpointLoader::run()
{
  // check if pose has been updated through callback
  if(cur_pose_th <= 360){

  }
  else{
    ROS_INFO("Current pose has not been published.\n");
  }
}

void CheckpointLoader::loadCheckpoints()
{
  string file_path = checkpoint_dir + "/" + filename;

  ROS_INFO("%s\n", file_path.c_str());

  ifstream inFile(file_path.c_str());

  if (!inFile.is_open()) {
      ROS_INFO("%s does not exist.\n", filename.c_str());
      ROS_INFO("Shutting down node.\n");
      ros::shutdown();
  }

  ROS_INFO("%s successfully opened.", filename.c_str());

  float tmp;
  while (inFile >> tmp){
    ROS_INFO("%f\n", tmp);
    
  }

  inFile.close();
}

void CheckpointLoader::CurPoseCallback(const nav_msgs::Odometry& msg)
{
  return;
}