#include "robot_tfs/robot_tfs.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "odometry");
    ros::NodeHandle nh("~");

    RobotTFs broadcaster(&nh);

    ros::spin();
    return 0;
}
