#include <babybuggy_error_checker/babybuggy_error_checker.h>

ErrorChecker::ErrorChecker(ros::NodeHandle* nodehandle):nh(*nodehandle)
{
    // setup subscribers
    gps_sub = nh.subscribe("/raw_gps_navsat", 5, &ErrorChecker::GPSCallback, this);
    imu_sub = nh.subscribe("/BNO055", 10, &ErrorChecker::IMUCallback, this);
    enc1_sub = nh.subscribe("/encoder1_raw", 10, &ErrorChecker::Encoder1Callback, this);
    enc2_sub = nh.subscribe("/encoder2_raw", 10, &ErrorChecker::Encoder2Callback, this);
    odom_sub = nh.subscribe("/encoder_odom", 10, &ErrorChecker::OdomCallback, this);
    lidar_sub = nh.subscribe("/scan", 10, &ErrorChecker::LidarCallback, this);
    bearing_sub = nh.subscribe("/gps_bearing", 5, &ErrorChecker::BearingCallback, this);

    yaw = 0.0;
    pitch = 0.0;
    roll = 0.0;

    gps_status = false;
    latitude = 0.0;
    longitude = 0.0;
    bearing = 0.0;

    encoder1_ticks = 0;
    largest1_tick = 0;
    smallest1_tick = 0;

    encoder2_ticks = 0;
    largest2_tick = 0;
    smallest2_tick = 0;

    start_time = ros::Time::now();
    enc1_current_time = ros::Time::now();
    enc2_current_time = ros::Time::now();

    for (int i = 0; i < laser_msg.ranges.size(); i++) {
        laser_msg.ranges[i] = 0.0;
    }
}

int ErrorChecker::run()
{
    ros::Rate clock_rate(1);  // run loop at 1 Hz
    int num_nans, num_inf = 0;

    while (ros::ok())
    {
        // let ROS process any events
        ros::spinOnce();
        clock_rate.sleep();

        ros::Time current_time = ros::Time::now();

        ROS_INFO("\n\t\t\t------ Error checking (%f) ------", (current_time - start_time).toSec());

        if (gps_status) {
            if (current_time - gps_msg.header.stamp > ros::Duration(10.0)) {
                ROS_WARN("No GPS message received for 10 seconds");
            }
            ROS_INFO("GPS | latitude: %0.6f\t\tlongitude: %0.6f\t\tbearing: %0.4f", latitude, longitude, bearing);
        }

        if (current_time - odom_msg.header.stamp > ros::Duration(2.0)) {
            ROS_WARN("No odometry message received for 2 seconds");
        }

        ROS_INFO("Odometry | x: %f\ty: %f\tyaw: %f\tvx: %f\tvy: %f\tva: %f",
            odom_msg.pose.pose.position.x,
            odom_msg.pose.pose.position.y,
            yaw * 180 / M_PI,
            odom_msg.twist.twist.linear.x,
            odom_msg.twist.twist.linear.y,
            odom_msg.twist.twist.angular.z
        );

        if (current_time - enc1_current_time > ros::Duration(2.0)) {
            ROS_WARN("No message received from encoder 1 for 2 seconds");
        }

        if (current_time - enc2_current_time > ros::Duration(2.0)) {
            ROS_WARN("No message received from encoder 2 for 2 seconds");
        }

        if (current_time - laser_msg.header.stamp > ros::Duration(5.0)) {
            ROS_WARN("No LIDAR messages received for 5 seconds");
        }
        ROS_INFO("LIDAR | scan #%d", laser_msg.header.seq);

        //
        // Check the encoder's values
        //
        if (largest1_tick < encoder1_ticks) {
            largest1_tick = (int64_t)encoder1_ticks;
        }
        if (encoder1_ticks < smallest1_tick) {
            smallest1_tick = (int64_t)encoder1_ticks;
        }
        if (largest2_tick < encoder2_ticks) {
            largest2_tick = (int64_t)encoder2_ticks;
        }
        if (encoder2_ticks < smallest2_tick) {
            smallest2_tick = (int64_t)encoder2_ticks;
        }

        // the largest tick should be a lot bigger than the smallest tick after some time
        if (largest1_tick != 0) {
            if (largest1_tick > 1000 && abs(largest1_tick) < (abs(smallest1_tick) * 1000)) {
                ROS_WARN("Detected that the encoder is going backwards much more than it is going forwards. Min val: %li; Max val: %li", smallest1_tick, largest1_tick);
            }
        }
        if (largest2_tick != 0) {
            if (largest2_tick > 1000 && abs(largest2_tick) < (abs(smallest2_tick) * 1000)) {
                ROS_WARN("Detected that the encoder is going backwards much more than it is going forwards. Min val: %li; Max val: %li", smallest2_tick, largest2_tick);
            }
        }

        ROS_INFO("Encoder 1 | tick: %li, min: %li, max: %li", encoder1_ticks, smallest1_tick, largest1_tick);
        ROS_INFO("Encoder 2 | tick: %li, min: %li, max: %li", encoder2_ticks, smallest2_tick, largest2_tick);

        //
        // Check the lidar's values
        //
        if (laser_msg.header.seq > 25)  // do these checks after 25 scans
        {
            num_nans = 0;
            num_inf = 0;
            for (int i = 0; i < laser_msg.ranges.size(); i++)
            {
                if (laser_msg.ranges[i] != laser_msg.ranges[i]) {  // is NaN
                    ROS_WARN("Detected a NaN value at index %d!", i);
                    num_nans++;
                }
                else if (isinf(laser_msg.ranges[i])) {  // is inf
                    ROS_WARN("Detected a inf value at index %d!", i);
                    num_inf++;
                }
                else {
                    check_statement(laser_msg.ranges[i] > 0.0, "Scan element at index %d has a value less than 0! ('%f')", i, laser_msg.ranges[i]);
                    check_statement(laser_msg.ranges[i] < 32.8, "Scan element at index %d has a value more than 32.8! ('%f')", i, laser_msg.ranges[i]);
                }
            }
            check_statement(num_nans <= (int)(laser_msg.ranges.size() / 2), "Detected a large number (%d out of %d) of NaN values!", num_nans, laser_msg.ranges.size());
            check_statement(num_inf <= (int)(laser_msg.ranges.size() / 2), "Detected a large number (%d out of %d) of Inf values!", num_inf, laser_msg.ranges.size());
        }
    }
    return 0;
}


void ErrorChecker::GPSCallback(const sensor_msgs::NavSatFix& msg)
{
    gps_msg = msg;
    gps_status = msg.status.status == msg.status.STATUS_FIX;
    latitude = msg.latitude;
    longitude = msg.longitude;

    check_statement(gps_status, "odometry produced a GPS message that had no fix!");
    check_statement(latitude != 0.0, "odometry produced a GPS message that had an latitude of 0!");
    check_statement(longitude != 0.0, "odometry produced a GPS message that had an longitude of 0!");
}

void ErrorChecker::IMUCallback(const sensor_msgs::Imu& msg)
{
    imu_msg = msg;
    static tf::Quaternion tmp;

    tmp.setValue(
        msg.orientation.x,
        msg.orientation.y,
        msg.orientation.z,
        msg.orientation.w
    );

    // extract 3x3 rotation matrix from quaternion
    tf::Matrix3x3 m(tmp);
    m.getEulerYPR(yaw, pitch, roll);

    check_statement(0 <= yaw <= M_PI * 2, "yaw from IMU is out of range! (%f not in 0...2pi)", yaw);
    check_statement(0 <= pitch <= M_PI * 2, "pitch from IMU is out of range! (%f not in 0...2pi)", pitch);
    check_statement(0 <= roll <= M_PI * 2, "roll from IMU is out of range! (%f not in 0...2pi)", roll);
}

void ErrorChecker::Encoder1Callback(const std_msgs::Int64& msg) {
    enc1_current_time = ros::Time::now();
    encoder1_ticks = msg.data;
}

void ErrorChecker::Encoder2Callback(const std_msgs::Int64& msg) {
    enc2_current_time = ros::Time::now();
    encoder2_ticks = msg.data;
}

void ErrorChecker::OdomCallback(const nav_msgs::Odometry& msg)
{
    odom_msg = msg;
    check_statement(
        abs(odom_msg.pose.pose.position.x) < 15000.0 || abs(odom_msg.pose.pose.position.y) < 15000.0,
        "Odometry has exceeded the expected bounds: (x: %f, y: %f) > (15.0km, 15.0km)",
        odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y
    );

    check_statement(
        abs(odom_msg.twist.twist.linear.x) < 50.0 || abs(odom_msg.pose.pose.position.y) < 50.0,
        "Velocity has exceeded the expected bounds: (vx: %f, vy: %f) > (50.0m/s, 50.0m/s)",
        odom_msg.twist.twist.linear.x, odom_msg.twist.twist.linear.y
    );

    // check_statement(
    //     abs(odom_msg.twist.twist.angular.x) < 1.0 || abs(odom_msg.twist.twist.angular.y) < 1.0 || abs(odom_msg.twist.twist.angular.z) < 1.0,
    //     "Angular velocity has exceeded the expected bounds: (x: %f, y: %f, z: %f) > (1.0, 1.0, 1.0) rad/s",
    //     odom_msg.twist.twist.angular.x, odom_msg.twist.twist.angular.y, odom_msg.twist.twist.angular.z
    // );
}

void ErrorChecker::LidarCallback(const sensor_msgs::LaserScan& msg) {
    laser_msg = msg;
}

void ErrorChecker::BearingCallback(const geometry_msgs::PoseWithCovarianceStamped& msg)
{
    bearing_msg = msg;
    static tf::Quaternion tmp;
    static double _;

    tmp.setValue(
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w
    );

    // extract 3x3 rotation matrix from quaternion
    tf::Matrix3x3 m(tmp);
    m.getEulerYPR(bearing, _, _);

    check_statement(0 <= bearing <= M_PI * 2, "yaw from GPS bearing is out of range! (%f not in 0...2pi)", bearing);
}
