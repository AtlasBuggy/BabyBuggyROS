#include "speed_accel/speed_accel.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "speed_accel");
    ros::NodeHandle nh;

    SpeedAccel m(&nh);

    return 0;
}
