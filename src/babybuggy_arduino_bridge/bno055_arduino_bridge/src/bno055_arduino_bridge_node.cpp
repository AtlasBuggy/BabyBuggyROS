
#include "bno055_arduino_bridge/bno055_arduino_bridge.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, Bno055ArduinoBridge::NODE_NAME);
    ros::NodeHandle nh("~");

    Bno055ArduinoBridge broadcaster(&nh);
    int err = broadcaster.run();

    return err;
}
