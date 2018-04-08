#include "babybuggy_error_checker/babybuggy_error_checker.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "error_checker");
    ros::NodeHandle nh("~");

    ErrorChecker broadcaster(&nh);

    return broadcaster.run();
}
