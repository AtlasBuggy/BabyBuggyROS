#include "odometry/odometry.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "odometry");
    ros::NodeHandle nh("~");

    Odometry broadcaster(&nh);

    ros::spin();
    return 0;
}
