#include <robot_tfs/robot_tfs.h>

const string RobotTFs::BASE_LINK_FRAME_NAME = "base_link";
const string RobotTFs::ODOM_FRAME_NAME = "odom";
// const string RobotTFs::LASER_FRAME_NAME = "laser";
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
    odom_pub = nh.advertise<nav_msgs::Odometry>("/naive_odom", 1000);
    // navsat_pub = nh.advertise<sensor_msgs::NavSatFix>("/raw_gps_navsat", 1000);

    // setup client for SetDatum service
    client = nh.serviceClient<robot_localization::SetDatum>("/datum");
    datum_set = false;

    // pull parameters from launch file
    nh.param<double>("initial_compass_yaw_deg", initial_compass_yaw_deg, 0.0);

    // initialize odometry
    odom_x = 0.0;
    odom_y = 0.0;
    banked_dist = 0.0;

    // Initial orientation is always 0
    current_imu_orientation.setRPY(0.0, 0.0, 0.0);

    // initialize odom_msg
    odom_msg.header.frame_id = ODOM_FRAME_NAME;
    odom_msg.child_frame_id = BASE_LINK_FRAME_NAME;

    for (size_t i = 0; i < 36; i++) {
        odom_msg.pose.covariance[i] = 0;
    }

    // Set covariance to identity multiplied by scaling factor
    size_t identity_col = 0;
    for (size_t row = 0; row < 36; row += 6) {
        odom_msg.pose.covariance[row + identity_col] = 1E-3;
        identity_col++;
    }

    tf_broadcaster = tf::TransformBroadcaster();
}

void RobotTFs::IMUCallback(const sensor_msgs::Imu& msg)
{
    // Convert sensor_msgs quaternion to tf quaternion
    tf::Quaternion tmp;

    tmp.setValue(
        msg.orientation.x,
        msg.orientation.y,
        msg.orientation.z,
        msg.orientation.w
    );

    // extract 3x3 rotation matrix from quaternion
    tf::Matrix3x3 m(tmp);
    m.getEulerYPR(yaw, pitch, roll);  // convert to ypr and set current_imu_orientation
    current_imu_orientation.setRPY(roll, pitch, yaw);

    yaw = fmod(-yaw - M_PI / 2, 2 * M_PI);

    // Use yaw and the encoder's banked_dist to calculate
    odom_x += cos(yaw) * banked_dist;
    odom_y += sin(yaw) * banked_dist;
    banked_dist = 0.0;  // don't add redundant distances to the x, y position

    // Only produce odometry messages if both sensors (encoders and IMU) are initialized and producing data
    if (enc_data_received)
    {
        // Form the odometry transform and broadcast it
        odometry_transform.setOrigin(tf::Vector3(odom_x, odom_y, 0.0));
        odometry_transform.setRotation(current_imu_orientation);

        tf_broadcaster.sendTransform(tf::StampedTransform(odometry_transform, ros::Time::now(), ODOM_FRAME_NAME, BASE_LINK_FRAME_NAME));

        // Form the odometry message with the IMU's orientation and accumulated distance in x and y
        odom_msg.header.stamp = ros::Time::now();

        // fill out and publish odom data
        odom_msg.pose.pose.position.x = odom_x;
        odom_msg.pose.pose.position.y = odom_y;
        odom_msg.pose.pose.position.z = 0.0;

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
    // Wait for the GPS to produce data that's valid and set robot_localization's datum
    if (msg.status.status == msg.status.STATUS_FIX && msg.latitude != 0.0 && msg.longitude != 0.0) {
        if (!datum_set) {
            srv.request.geo_pose.position.latitude = msg.latitude;
            srv.request.geo_pose.position.longitude = msg.longitude;
            srv.request.geo_pose.position.altitude = msg.altitude;

            tf::Quaternion tmp;
            tmp.setRPY(0.0, 0.0, initial_compass_yaw_deg * M_PI / 180.0);

            srv.request.geo_pose.orientation.x = tmp.x();
            srv.request.geo_pose.orientation.y = tmp.y();
            srv.request.geo_pose.orientation.z = tmp.z();
            srv.request.geo_pose.orientation.w = tmp.w();

            if(client.call(srv)){
                ROS_INFO("gps datum service call - success! lat: %0.6f, long: %0.6f\n", msg.latitude, msg.longitude);
            }
            else{
                ROS_INFO("gps datum service call - failed!\n");
            }

            datum_set = true;
            ROS_INFO("gps datum service call - datum set!\n");
        }

        // msg.child_frame_id = BASE_LINK_FRAME_NAME;
        //
        // navsat_pub.publish(msg);
    }
    else {
        datum_set = false;
    }
}

void RobotTFs::EncoderCallback(const std_msgs::Float64& msg)
{
    // append the encoder's distance to banked_dist. The arduino produces distances relative to the last measurement.
    banked_dist += msg.data / 1000.0;
    enc_data_received = true;
}
