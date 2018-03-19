#include "odometry/odometry.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, Odometry::ODOM_NODE_NAME);
    ros::NodeHandle nh("~");

    Odometry broadcaster(&nh);
    ros::Rate clock_rate(60);

    while(ros::ok()){
      ros::spinOnce();
      clock_rate.sleep();
    }
}
