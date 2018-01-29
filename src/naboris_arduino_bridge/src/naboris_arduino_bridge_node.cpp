
#include "naboris_arduino_bridge/naboris_arduino_bridge.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, NaborisArduinoBridge::NODE_NAME);
    ros::NodeHandle nh;

    NaborisArduinoBridge broadcaster(&nh);
    int err = broadcaster.run();

    return err;
}
