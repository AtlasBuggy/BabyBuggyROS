#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

using namespace std;

tf::TransformBroadcaster* odom_broadcaster;
tf::Transform odom_trans;
tf::Quaternion current_orientation;

void odometryCallback(const nav_msgs::Odometry& odom_msg)
{
    odom_trans.setOrigin(tf::Vector3(
        odom_msg.pose.pose.position.x,
        odom_msg.pose.pose.position.y,
        0.0
    ));

    current_orientation.setValue(
        odom_msg.pose.pose.orientation.x,
        odom_msg.pose.pose.orientation.y,
        odom_msg.pose.pose.orientation.z,
        odom_msg.pose.pose.orientation.w
    );

    odom_trans.setRotation(current_orientation);

    odom_broadcaster->sendTransform(
        tf::StampedTransform(odom_trans, ros::Time::now(), "odom", "base_link")
    );
}


int main(int argc, char** argv){
    ros::init(argc, argv, "odometry_tf_converter");

    ros::NodeHandle n;
    odom_broadcaster = new tf::TransformBroadcaster();

    string odom_sub_name;
    n.param<string>("odom_sub_name", odom_sub_name, "/naive_odom");

    ros::Subscriber odom_sub = n.subscribe(odom_sub_name, 1000, &odometryCallback);

    ros::spin();
}
