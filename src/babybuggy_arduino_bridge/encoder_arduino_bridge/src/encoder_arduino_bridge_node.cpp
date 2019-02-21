
#include "encoder_arduino_bridge/encoder_arduino_bridge.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, EncoderArduinoBridge::NODE_NAME);
    ros::NodeHandle nh("~");

    EncoderArduinoBridge broadcaster(&nh);
    int err = broadcaster.run();

    return err;
}
