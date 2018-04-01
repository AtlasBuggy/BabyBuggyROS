#include <odometry/odometry.h>

const string RobotTFs::BASE_LINK_FRAME_NAME = "base_link";
const string RobotTFs::ODOM_FRAME_NAME = "odom";
const string RobotTFs::LASER_FRAME_NAME = "laser";
const string RobotTFs::IMU_FRAME_NAME = "imu";
const string RobotTFs::GPS_FRAME_NAME = "gps";

const float RobotTFs::IMU_LASER_X = 0.1016;
const float RobotTFs::IMU_LASER_Y = 0.0762;
const float RobotTFs::IMU_LASER_Z = 0.095;

const float RobotTFs::GPS_LASER_X = 0.0;
const float RobotTFs::GPS_LASER_Y = 0.0;
const float RobotTFs::GPS_LASER_Z = 0.0;

const ros::Duration RobotTFs::DEBUG_INFO_DELAY = ros::Duration(1.0);

RobotTFs::RobotTFs(ros::NodeHandle* nodehandle):nh(*nodehandle)
{
    // setup subscribers
    gps_sub = nh.subscribe("/GpsNavSat", 1000, &RobotTFs::GPSCallback, this);
    imu_sub = nh.subscribe("/BNO055", 1000, &RobotTFs::IMUCallback, this);
    enc_sub = nh.subscribe("/encoder", 1000, &RobotTFs::EncoderCallback, this);

    // setup odom publisher
    odom_pub = nh.advertise<nav_msgs::RobotTFs>("/naive_odom", 1000);

    // setup client for SetDatum service
    client = nh.serviceClient<robot_localization::SetDatum>("/datum");
    datum_set = false;

    // initialize odometry
    odom_x = 0;
    odom_y = 0;

    robot_imu_orientation.setRPY(0, 0, 0);
    static_trans_imu.setOrigin(tf::Vector3(IMU_LASER_X, IMU_LASER_Y, IMU_LASER_Z));
    static_trans_imu.setRotation(robot_imu_orientation);

    robot_gps_orientation.setRPY(0, 0, 0);
    static_trans_gps.setOrigin(tf::Vector3(GPS_LASER_X, GPS_LASER_Y, GPS_LASER_Z));
    static_trans_gps.setRotation(robot_gps_orientation);

    current_imu_orientation.setRPY(0, 0, 0);

    // initialize odom_msg
    odom_msg.header.frame_id = ODOM_FRAME_NAME;
    odom_msg.child_frame_id = BASE_LINK_FRAME_NAME;

    for (size_t i = 0; i < 36; i++) {
        odom_msg.pose.covariance[i] = 0;
    }

    // Set covariance to identity multiplied by scaling factor
    size_t identity_col = 0;
    for (size_t row = 0; row < 36; row += 6) {
        odom_msg.pose.covariance[row + identity_col] = 1E6;
        identity_col++;
    }

    tf_broadcaster = tf::TransformBroadcaster();

    tf_broadcaster.sendTransform(tf::StampedTransform(static_trans_laser, ros::Time::now(), BASE_LINK_FRAME_NAME, LASER_FRAME_NAME));
    tf_broadcaster.sendTransform(tf::StampedTransform(static_trans_gps, ros::Time::now(), GPS_FRAME_NAME, BASE_LINK_FRAME_NAME));
    tf_broadcaster.sendTransform(tf::StampedTransform(static_trans_imu, ros::Time::now(), IMU_FRAME_NAME, BASE_LINK_FRAME_NAME));
}

void RobotTFs::IMUCallback(const sensor_msgs::Imu& msg)
{
    tf::Quaternion tmp;

    tmp.setValue(
        msg.orientation.x,
        msg.orientation.y,
        msg.orientation.z,
        msg.orientation.w
    );

    tf::Matrix3x3 m(tmp);
    m.getEulerYPR(yaw, pitch, roll);
    current_imu_orientation.setRPY(roll, pitch, yaw);

    odom_x += cos(yaw) * banked_dist;
    odom_y += sin(yaw) * banked_dist;
    banked_dist = 0;

    if (enc_data_received)
    {
        odometry_transform.setOrigin(tf::Vector3(odom_x, odom_y, 0.0));
        odometry_transform.setRotation(current_imu_orientation);

        transform_broadcaster.sendTransform(tf::StampedTransform(odometry_transform, ros::Time::now(), ODOM_FRAME_NAME, IMU_FRAME_NAME));

        odom_pub.header.stamp = ros::Time::now();

        // fill out and publish odom data
        odom_msg.pose.pose.position.x = odom_x;
        odom_msg.pose.pose.position.y = odom_y;
        odom_msg.pose.pose.position.z = 0;

        odom_msg.pose.pose.orientation.x = current_imu_orientation.x();
        odom_msg.pose.pose.orientation.z = current_imu_orientation.z();
        odom_msg.pose.pose.orientation.y = current_imu_orientation.y();
        odom_msg.pose.pose.orientation.w = current_imu_orientation.w();

        odom_pub.publish(odom_msg);
    }

    if (ros::Time::now() - debug_info_prev_time > DEBUG_INFO_DELAY) {
        debug_info_prev_time = ros::Time::now();
        ROS_INFO("Odom pose | X: %f, Y: %f, yaw: %f", odom_x, odom_y, yaw * 180 / M_PI);
    }
}

void RobotTFs::GPSCallback(const sensor_msgs::NavSatFix& msg)
{
    if (msg.status.status == msg.status.STATUS_FIX) {
        if (!datum_set) {
            srv.request.geo_pose.position.latitude = msg.latitude;
            srv.request.geo_pose.position.longitude = msg.longitude;
            srv.request.geo_pose.position.altitude = msg.altitude;

            srv.request.geo_pose.orientation.x = current_imu_orientation.x();
            srv.request.geo_pose.orientation.y = current_imu_orientation.y();
            srv.request.geo_pose.orientation.z = current_imu_orientation.z();
            srv.request.geo_pose.orientation.w = current_imu_orientation.w();

            if(client.call(srv)){
                ROS_INFO("gps datum service call - success!\n");
            }
            else{
                ROS_INFO("gps datum service call - failed!\n");
            }

            datum_set = true;
            ROS_INFO("gps datum service call - datum set!\n");
        }
    }
    else {
        datum_set = false;
    }
}

void RobotTFs::EncoderCallback(const std_msgs::Float64& msg)
{
    banked_dist += msg.data / 1000;
    enc_data_received = true;
}
