#include <babybuggy_odometry/babybuggy_odometry.h>

const string BabybuggyOdometry::BASE_LINK_FRAME_NAME = "base_link";
const string BabybuggyOdometry::ODOM_FRAME_NAME = "odom";
// const string BabybuggyOdometry::LASER_FRAME_NAME = "laser";
const string BabybuggyOdometry::IMU_FRAME_NAME = "imu";
const string BabybuggyOdometry::GPS_FRAME_NAME = "GPS";

const float BabybuggyOdometry::IMU_LASER_X = 0.1016;
const float BabybuggyOdometry::IMU_LASER_Y = 0.0762;
const float BabybuggyOdometry::IMU_LASER_Z = 0.095;

const float BabybuggyOdometry::GPS_LASER_X = 0.0;
const float BabybuggyOdometry::GPS_LASER_Y = 0.0;
const float BabybuggyOdometry::GPS_LASER_Z = 0.0;

const size_t BabybuggyOdometry::NUM_ROWS_ODOM_COVARIANCE = 6;
const size_t BabybuggyOdometry::NUM_ROWS_GPS_COVARIANCE = 3;

const size_t BabybuggyOdometry::BEARING_ROLL_AVERAGE_SIZE = 3;

BabybuggyOdometry::BabybuggyOdometry(ros::NodeHandle* nodehandle):nh(*nodehandle)
{
    // setup subscribers
    gps_sub = nh.subscribe("/GpsNavSat", 5, &BabybuggyOdometry::GPSCallback, this);
    imu_sub = nh.subscribe("/BNO055", 100, &BabybuggyOdometry::IMUCallback, this);
    enc_sub = nh.subscribe("/encoder", 1000, &BabybuggyOdometry::EncoderCallback, this);

    // setup odom publisher
    odom_pub = nh.advertise<nav_msgs::Odometry>("/naive_odom", 100);
    navsat_pub = nh.advertise<sensor_msgs::NavSatFix>("/raw_gps_navsat", 5);
    bearing_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/gps_bearing", 5);

    // setup client for SetDatum service
    client = nh.serviceClient<robot_localization::SetDatum>("/datum");
    datum_set = false;

    // pull parameters from launch file
    nh.param<double>("initial_compass_yaw_deg", initial_compass_yaw_deg, 0.0);
    nh.param<double>("bearing_covariance", bearing_covariance, 0.0);
    nh.param<double>("wheel_radius", wheel_radius, 1.0);
    nh.param<int>("ticks_per_rotation", ticks_per_rotation, 1);

    // initialize odometry
    odom_x = 0.0;
    odom_y = 0.0;
    encoder_ticks = 0;
    prev_encoder_ticks = 0;

    ros::Time prev_time = ros::Time::now();

    enc_ticks_to_m = 2.0 * M_PI * wheel_radius / (double)ticks_per_rotation;
    ROS_INFO("Encoder ticks to meters: %f", enc_ticks_to_m);

    // Initial orientation is always 0
    current_imu_orientation.setRPY(0.0, 0.0, 0.0);

    // initialize odom_msg
    odom_msg.header.frame_id = ODOM_FRAME_NAME;
    odom_msg.child_frame_id = BASE_LINK_FRAME_NAME;

    bearing_msg.header.frame_id = GPS_FRAME_NAME;

    size_t identity_col;

    XmlRpc::XmlRpcValue _odom_pos_launch_covariances;
    if (nh.hasParam("odom_pos_covariances")) {
        nh.getParam("odom_pos_covariances", _odom_pos_launch_covariances);
        ROS_ASSERT(_odom_pos_launch_covariances.getType() == XmlRpc::XmlRpcValue::TypeArray);
        ROS_ASSERT(_odom_pos_launch_covariances.size() == NUM_ROWS_ODOM_COVARIANCE * NUM_ROWS_ODOM_COVARIANCE);

        ROS_INFO("Using launch file's odometry position covariances");
        for (size_t i = 0; i < _odom_pos_launch_covariances.size(); i++) {
            odom_msg.pose.covariance[i] = _odom_pos_launch_covariances[i];
        }
    }
    else {
        ROS_INFO("Using default odometry covariances");
        // Set covariance to identity multiplied by scaling factor
        identity_col = 0;
        for (size_t row = 0; row < NUM_ROWS_ODOM_COVARIANCE * NUM_ROWS_ODOM_COVARIANCE; row += NUM_ROWS_ODOM_COVARIANCE) {
            odom_msg.pose.covariance[row + identity_col] = 0.5;
            identity_col++;
        }
    }

    XmlRpc::XmlRpcValue _odom_vel_launch_covariances;
    if (nh.hasParam("odom_vel_covariances")) {
        nh.getParam("odom_vel_covariances", _odom_vel_launch_covariances);
        ROS_ASSERT(_odom_vel_launch_covariances.getType() == XmlRpc::XmlRpcValue::TypeArray);
        ROS_ASSERT(_odom_vel_launch_covariances.size() == NUM_ROWS_ODOM_COVARIANCE * NUM_ROWS_ODOM_COVARIANCE);

        ROS_INFO("Using launch file's odometry covariances");
        for (size_t i = 0; i < _odom_vel_launch_covariances.size(); i++) {
            odom_msg.twist.covariance[i] = _odom_vel_launch_covariances[i];
        }
    }
    else {
        ROS_INFO("Using default odometry velocity covariances");
        // Set covariance to identity multiplied by scaling factor
        identity_col = 0;
        for (size_t row = 0; row < NUM_ROWS_ODOM_COVARIANCE * NUM_ROWS_ODOM_COVARIANCE; row += NUM_ROWS_ODOM_COVARIANCE) {
            odom_msg.pose.covariance[row + identity_col] = 0.5;
            identity_col++;
        }
    }

    XmlRpc::XmlRpcValue _gps_launch_covariances;
    if (nh.hasParam("gps_covariances")) {
        nh.getParam("gps_covariances", _gps_launch_covariances);
        ROS_ASSERT(_gps_launch_covariances.getType() == XmlRpc::XmlRpcValue::TypeArray);
        ROS_ASSERT(_gps_launch_covariances.size() == NUM_ROWS_GPS_COVARIANCE * NUM_ROWS_GPS_COVARIANCE);

        ROS_INFO("Using launch file's gps covariances");
        for (size_t i = 0; i < _gps_launch_covariances.size(); i++) {
            gps_covariance_msg.position_covariance[i] = _gps_launch_covariances[i];
        }
    }
    else {
        ROS_INFO("Using default GPS covariances");
        // Set covariance of the gps (diagonal only. Assuming independence)
        identity_col = 0;
        for (size_t row = 0; row < NUM_ROWS_GPS_COVARIANCE * NUM_ROWS_GPS_COVARIANCE; row += NUM_ROWS_GPS_COVARIANCE) {
            gps_covariance_msg.position_covariance[row + identity_col] = 0.25;
            identity_col++;
        }
    }

    // tf_broadcaster = tf::TransformBroadcaster();
}


void BabybuggyOdometry::IMUCallback(const sensor_msgs::Imu& msg)
{
    // Convert sensor_msgs quaternion to tf quaternion
    static tf::Quaternion tmp;

    tmp.setValue(
        msg.orientation.x,
        msg.orientation.y,
        msg.orientation.z,
        msg.orientation.w
    );

    // extract 3x3 rotation matrix from quaternion
    tf::Matrix3x3 m(tmp);
    double prev_yaw = yaw;
    double prev_pitch = pitch;
    double prev_roll = roll;
    m.getEulerYPR(yaw, pitch, roll);  // convert to ypr and set current_imu_orientation

    yaw *= -1;
    // yaw = 2.0 * M_PI - yaw;
    // yaw = fmod(-yaw, 2 * M_PI);
    // if (yaw < 0.0) {
    //     yaw += 2.0 * M_PI;
    // }

    // Use yaw and the encoder's banked_dist to calculate x y
    double banked_dist = encoder_ticks * enc_ticks_to_m;
    double delta_dist = banked_dist - prev_encoder_ticks * enc_ticks_to_m;

    prev_encoder_ticks = encoder_ticks;

    double delta_x = cos(yaw) * delta_dist;
    double delta_y = sin(yaw) * delta_dist;

    odom_x += delta_x;
    odom_y += delta_y;

    ros::Time current_time = ros::Time::now();
    ros::Duration delta_duration = prev_time - current_time;
    double dt = delta_duration.toSec();
    prev_time = current_time;

    double velocity_x = delta_x / dt;
    double velocity_y = delta_y / dt;

    if (roll - prev_roll > M_PI) {
        prev_roll -= 2 * M_PI;
    }
    if (roll - prev_roll < -M_PI) {
        prev_roll += 2 * M_PI;
    }
    
    if (pitch - prev_pitch > M_PI) {
        prev_pitch -= 2 * M_PI;
    }
    if (pitch - prev_pitch < -M_PI) {
        prev_pitch += 2 * M_PI;
    }
    
    if (yaw - prev_yaw > M_PI) {
        prev_yaw -= 2 * M_PI;
    }
    if (yaw - prev_yaw < -M_PI) {
        prev_yaw += 2 * M_PI;
    }
    
    double angular_x = roll - prev_roll / dt;
    double angular_y = pitch - prev_pitch / dt;
    double angular_z = yaw - prev_yaw / dt;

    // update orientation with adjusted yaw values
    current_imu_orientation.setRPY(roll, pitch, yaw);

    // Only produce odometry messages if both sensors (encoders and IMU) are initialized and producing data
    if (enc_data_received)
    {
        // Form the odometry message with the IMU's orientation and accumulated distance in x and y
        odom_msg.header.stamp = ros::Time::now();

        // fill out and publish odom data
        odom_msg.pose.pose.position.x = odom_x;
        odom_msg.pose.pose.position.y = odom_y;
        odom_msg.pose.pose.position.z = 0.0;

        odom_msg.twist.twist.linear.x = velocity_x;
        odom_msg.twist.twist.linear.y = velocity_y;
        odom_msg.twist.twist.linear.z = 0.0;

        odom_msg.pose.pose.orientation.x = current_imu_orientation.x();
        odom_msg.pose.pose.orientation.y = current_imu_orientation.y();
        odom_msg.pose.pose.orientation.z = current_imu_orientation.z();
        odom_msg.pose.pose.orientation.w = current_imu_orientation.w();

        odom_msg.twist.twist.angular.x = angular_x;
        odom_msg.twist.twist.angular.y = angular_y;
        odom_msg.twist.twist.angular.z = angular_z;

        odom_pub.publish(odom_msg);
    }
}

double BabybuggyOdometry::calculateBearing(sensor_msgs::NavSatFix currentMsg, sensor_msgs::NavSatFix prevMsg)
{
    // pulled from https://www.dougv.com/2009/07/calculating-the-bearing-and-compass-rose-direction-between-two-latitude-longitude-coordinates-in-php/
    double long2_rad = currentMsg.longitude * M_PI / 180.0;
    double long1_rad = prevMsg.longitude * M_PI / 180.0;
    double lat2_rad = currentMsg.latitude * M_PI / 180.0;
    double lat1_rad = prevMsg.latitude * M_PI / 180.0;

    double d_long_rad = long2_rad - long1_rad;
    double d_phi = log(
        tan(lat2_rad / 2.0 + M_PI / 4.0) /
        tan(lat1_rad / 2.0 + M_PI / 4.0)
    );

    if (abs(d_long_rad) > M_PI) {
        if (d_long_rad > 0) {
            d_long_rad = d_long_rad - 2 * M_PI;
        }
        else {
            d_long_rad = d_long_rad + 2 * M_PI;
        }
    }

    return fmod(atan2(d_long_rad, d_phi) + 2 * M_PI, 2 * M_PI);
}

void BabybuggyOdometry::GPSCallback(const sensor_msgs::NavSatFix& msg)
{
    static sensor_msgs::NavSatFix prev_msg;
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
                ROS_INFO("gps datum service call - success! lat: %0.6f, long: %0.6f", msg.latitude, msg.longitude);
            }
            else{
                ROS_INFO("gps datum service call - failed!");
            }

            datum_set = true;
            ROS_INFO("gps datum service call - datum set!");
        }

        if (msg.latitude != prev_msg.latitude && msg.longitude != prev_msg.longitude)
        {
            if (bearing_vector.size() == 0) {
                bearing_vector.push_back(initial_compass_yaw_deg * M_PI / 180.0);
                ROS_INFO("Initializing bearing rolling average");
            }
            else if (bearing_vector.size() == 1) {
                double current_bearing = -calculateBearing(msg, prev_msg);
                bearing_vector.push_back(current_bearing);
                ROS_INFO("First bearing value: %0.4f", current_bearing);
            }
            else {
                double current_bearing = -calculateBearing(msg, prev_msg);
                bearing_vector.push_back(current_bearing);

                double sum_bearings = 0.0;
                for (size_t i = 0; i < bearing_vector.size(); i++) {

                    sum_bearings += bearing_vector[i];
                }
                double avg_bearing = sum_bearings / (double)bearing_vector.size();

                tf::Quaternion bearing_quat;
                bearing_quat.setRPY(0.0, 0.0, avg_bearing);

                bearing_msg.header.stamp = ros::Time::now();
                bearing_msg.pose.pose.orientation.x = bearing_quat.x();
                bearing_msg.pose.pose.orientation.y = bearing_quat.y();
                bearing_msg.pose.pose.orientation.z = bearing_quat.z();
                bearing_msg.pose.pose.orientation.w = bearing_quat.w();

                size_t identity_col = 0;
                for (size_t row = 0; row < 36; row += 6) {
                    bearing_msg.pose.covariance[row + identity_col] = 1e-9;
                    identity_col++;
                }
                bearing_msg.pose.covariance[35] = bearing_covariance;  // last element is the yaw covariance

                bearing_pub.publish(bearing_msg);

                if (bearing_vector.size() >= BEARING_ROLL_AVERAGE_SIZE) {
                    bearing_vector.erase(bearing_vector.begin());
                }
            }

            prev_msg = msg;
            prev_msg.position_covariance = gps_covariance_msg.position_covariance;
            prev_msg.position_covariance_type = msg.COVARIANCE_TYPE_APPROXIMATED;
        }

        navsat_pub.publish(prev_msg);
    }
    else {
        datum_set = false;
    }
}


void BabybuggyOdometry::EncoderCallback(const std_msgs::Int64& msg)
{
    // append the encoder's distance to encoder_ticks. The arduino produces distances relative to the last measurement.
    encoder_ticks += msg.data;
    enc_data_received = true;
}
