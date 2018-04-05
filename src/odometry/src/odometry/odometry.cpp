#include <odometry/odometry.h>

Odometry::Odometry(ros::NodeHandle* nodehandle):nh(*nodehandle)
{
  // setup subscribers
  gps_sub = nh.subscribe("/GpsNavSat", 1000, &Odometry::GPSCallback, this);
  imu_sub = nh.subscribe("/BNO055", 1000, &Odometry::IMUCallback, this);
  enc_sub = nh.subscribe("/encoder", 1000, &Odometry::EncoderCallback, this);

  // setup odom publisher
  odom_pub = nh.advertise<nav_msgs::Odometry>("/odometry/filtered", 1000);
  gps_pub = nh.advertise<sensor_msgs::NavSatFix>("/gps/fix", 1000);
  // imu_pub = nh.advertise<sensor_msgs::Imu>("/imu/data", 1000);

  // setup client for SetDatum service
  client = nh.serviceClient<robot_localization::SetDatum>("/datum");
  datum_set = false;

  // initialize odometry
  odom_x = 0;
  odom_y = 0;
  q.setRPY(0, 0, 0);

  // initialize odom_msg
  odom_msg.header.frame_id = "odom";
  odom_msg.child_frame_id = "base_link";
}

void Odometry::IMUCallback(const sensor_msgs::Imu& msg)
{
  static tf::TransformBroadcaster br;
  tf::Transform static_trans_gps;
  tf::Quaternion q_tmp;

  q_tmp.setRPY(0, 0, 0);
  static_trans_gps.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
  static_trans_gps.setRotation(q_tmp);
  br.sendTransform(tf::StampedTransform(static_trans_gps, ros::Time::now(), "base_link", "GPS"));

  // imu_pub.publish(msg);

  tf::Quaternion tmp;

  tmp.setValue(
      msg.orientation.x,
      msg.orientation.y,
      msg.orientation.z,
      msg.orientation.w
  );

  tf::Matrix3x3 m(tmp);
  m.getEulerYPR(yaw, pitch, roll);
  q.setRPY(roll, pitch, -yaw);

  odom_x += cos(-yaw) * banked_dist;
  odom_y += sin(-yaw) * banked_dist;
  banked_dist = 0;

  // fill out and publish odom data
  odom_msg.pose.pose.position.x = odom_x;
  odom_msg.pose.pose.position.y = odom_y;
  odom_msg.pose.pose.position.z = 0;

  odom_msg.pose.pose.orientation.x = q.x();
  odom_msg.pose.pose.orientation.z = q.z();
  odom_msg.pose.pose.orientation.y = q.y();
  odom_msg.pose.pose.orientation.w = q.w();

  for(int i = 0; i < 36; i++){
    odom_msg.pose.covariance[i] = 0;
  }

  // q.setRPY(roll, pitch, yaw);

  odom_pub.publish(odom_msg);

  imu_data_received = true;
}

void Odometry::GPSCallback(const sensor_msgs::NavSatFix& msg)
{
  gps_pub.publish(msg);

  if ((msg.status.status == msg.status.STATUS_FIX) && (!datum_set)){
    srv.request.geo_pose.position.latitude = msg.latitude;
    srv.request.geo_pose.position.longitude = msg.longitude;
    srv.request.geo_pose.position.altitude = msg.altitude;

    q.setRPY(roll, pitch, yaw-M_PI);
    srv.request.geo_pose.orientation.x = q.x();
    srv.request.geo_pose.orientation.y = q.y();
    srv.request.geo_pose.orientation.z = q.z();
    srv.request.geo_pose.orientation.w = q.w();
    q.setRPY(roll, pitch, -yaw);

    if(client.call(srv)){
      ROS_INFO("SUCCESS\n");
    }
    else{
      ROS_INFO("OH NO\n");
    }

    datum_set = true;
    ROS_INFO("DATUM SET\n");
  }
  if ((msg.status.status != msg.status.STATUS_FIX) && (datum_set)){
    datum_set = false;
  }
}

void Odometry::EncoderCallback(const std_msgs::Float64& msg)
{
  banked_dist += msg.data / 1000;
  enc_data_received = true;
}