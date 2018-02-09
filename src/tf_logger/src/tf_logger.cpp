#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include <stdio.h>
#include <math.h>

tf::Quaternion q;
tf::Transform static_trans_imu;
tf::Transform static_trans_laser;
tf::Transform trans;

//transform data
float tr_x = 0; //m, overall motion in x plane
float tr_y = 0; //m, overall motion in y plane
float dt = 0; //seconds, change in time
float prev_t = 0; //seconds, previous time
float prev_theta = 0; //radians, keeps track of previous angle
float theta = 0; //radians, calculates current angle
float banked_dist = 0.0; //m, banks distance

//distance from imu to laser, meters, roughly measured with a tape measure
const float imu_laser_x = 0.1016;
const float imu_laser_y = 0.0762;
const float imu_laser_z = 0.095;

//distance from base_link to imu

void scanCallback(const sensor_msgs::LaserScan& msg){

    static tf::TransformBroadcaster br;

    trans.setOrigin(tf::Vector3(tr_x, tr_y, 0.0));
    trans.setRotation(q);

    br.sendTransform(tf::StampedTransform(static_trans_laser, ros::Time::now(), "base_link", "laser"));
    br.sendTransform(tf::StampedTransform(static_trans_imu, ros::Time::now(), "imu", "base_link"));
    br.sendTransform(tf::StampedTransform(trans, ros::Time::now(), "odom", "imu"));
}

void imuCallback(const sensor_msgs::Imu& imu_msg)
{
    q.setValue(
        imu_msg.orientation.x,
        imu_msg.orientation.y,
        imu_msg.orientation.z,
        imu_msg.orientation.w
    );
    double roll, pitch, yaw;

    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);

    // ROS_INFO("ROLL: %f", roll);

    /*
    float dtheta_z = imu_msg.angular_velocity.z;
    dt = ros::Time::now().toSec() - prev_t;
    theta += dtheta_z * dt;
    */

    /*
    theta = roll < 0 ? roll + 2*M_PI : roll;

    float dt = theta - prev_theta;
    if(dt > M_PI){
    dt -= 2*M_PI;

    float mid_theta = prev_theta + dt;
    */

    banked_dist /= 1000.0;
    tr_x += cos(yaw) * banked_dist;
    tr_y += sin(yaw) * banked_dist;
    banked_dist = 0;

    // prev_t = ros::Time::now().toSec();
    prev_theta = theta;
}

void encoderCallback(const std_msgs::Float64& flt_msg) {
    banked_dist += flt_msg.data;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "tf_logger");

    q.setRPY(0, 0, 0);
    static_trans_imu.setOrigin(tf::Vector3(imu_laser_x, imu_laser_y, imu_laser_z));
    static_trans_imu.setRotation(q);

    static_trans_laser.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    static_trans_laser.setRotation(q);

    ros::NodeHandle node;
    ros::Subscriber scanSub = node.subscribe("/scan", 10, &scanCallback);
    ros::Subscriber encoderSub = node.subscribe("/encoder", 10, &encoderCallback);
    ros::Subscriber imuSub = node.subscribe("/BNO055", 10, &imuCallback);
    ros::Publisher posePub = node.advertise<geometry_msgs::PoseStamped>("/robot_pose", 10);

    ros::Rate loop_rate(10);

    prev_t = ros::Time::now().toSec();

    geometry_msgs::Quaternion orient_msg;
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.pose.position.z = 0;
    pose_msg.header.frame_id = "base_link";
    // orient_msg.header.frame_id = "orientation_link";

    while (node.ok())
    {
        pose_msg.header.stamp = ros::Time::now();
        // orient_msg.header.stamp = ros::Time::now();
        // ROS_INFO("x: %f, y: %f", tr_x, tr_y);

        orient_msg.x = q.x();
        orient_msg.y = q.y();
        orient_msg.z = q.z();
        orient_msg.w = q.w();

        pose_msg.pose.position.x = tr_x;
        pose_msg.pose.position.y = tr_y;
        pose_msg.pose.orientation = orient_msg;

        posePub.publish(pose_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    // ros::spin();
    return 0;
}
