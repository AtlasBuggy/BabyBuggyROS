#include "babybuggy_odometry/babybuggy_odometry.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "odometry");
    ros::NodeHandle nh("~");

    BabybuggyOdometry broadcaster(&nh);

    int err = broadcaster.run();

    return err;
}
