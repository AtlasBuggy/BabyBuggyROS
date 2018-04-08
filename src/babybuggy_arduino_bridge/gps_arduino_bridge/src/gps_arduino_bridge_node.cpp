
#include "gps_arduino_bridge/gps_arduino_bridge.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, GPSArduinoBridge::NODE_NAME);
    ros::NodeHandle nh("~");

    GPSArduinoBridge broadcaster(&nh);
    int err = broadcaster.run();

    return err;
}
