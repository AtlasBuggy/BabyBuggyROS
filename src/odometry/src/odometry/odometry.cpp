#include <odometry/odometry.h>

Odometry::Odometry(ros::NodeHandle* nodehandle):nh(*nodehandle)
{
  // setup subscribers
  gps_sub = nh.subscribe("/GpsNavSat", 1000, &Odometry::GPSCallback, this);
  imu_sub = nh.subscribe("/BNO055", 1000, &Odometry::IMUCallback, this);
  enc_sub = nh.subscribe("/encoder", 1000, &Odometry::EncoderCallback, this);

  // setup odom publisher
  odom_pub = nh.advertise<nav_msgs::Odometry>("odometry/filtered", 1000);
  gps_pub = nh.advertise<sensor_msgs::NavSatFix>("gps/fix", 1000);

  // setup client for SetDatum service
  client = nh.serviceClient<robot_localization::SetDatum>("SetDatum");

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

  odom_pub.publish(odom_msg);

  imu_data_received = true;
}

void Odometry::GPSCallback(const sensor_msgs::NavSatFix& msg)
{
  gps_pub.publish(msg);

  if (msg.status.status == msg.status.STATUS_FIX){
    srv.request.geo_pose.position.latitude = msg.latitude;
    srv.request.geo_pose.position.longitude = msg.longitude;
    srv.request.geo_pose.position.altitude = msg.altitude;
    srv.request.geo_pose.orientation.x = q.x();
    srv.request.geo_pose.orientation.y = q.y();
    srv.request.geo_pose.orientation.z = q.z();
    srv.request.geo_pose.orientation.w = q.w();
    client.call(srv);
  }
}

void Odometry::EncoderCallback(const std_msgs::Float64& msg)
{
  banked_dist += msg.data / 1000;
  enc_data_received = true;
}