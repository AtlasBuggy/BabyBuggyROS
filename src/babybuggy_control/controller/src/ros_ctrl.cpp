#include "../include/controller/controller.hpp"
#include "ros/ros.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Float64.h"
#include "std_msgs/UInt16.h"

using namespace std;

Robot buggy(true);
deque<double> x_cord = {24.650000000000006, 30.97500000000001, 37.30000000000001, 44.750000000000014, 52.20000000000002, 61.10000000000001, 70.0, 76.67500000000001, 83.35000000000002, 88.40000000000002, 93.45000000000002, 99.40000000000002, 105.35000000000002, 111.70000000000002, 118.05000000000001, 124.12500000000001, 130.20000000000002, 136.90000000000003, 143.60000000000002, 147.125, 150.65, 153.65000000000003, 156.65000000000003, 158.3, 159.95, 160.65, 161.35000000000002, 161.57500000000002, 161.8, 161.675, 161.55, 163.10000000000002, 164.65000000000003, 165.90000000000003, 167.15000000000003, 168.45000000000002, 169.75, 171.3, 172.85000000000002, 175.15, 177.45, 180.075, 182.7, 186.175, 189.65000000000003, 192.12500000000003, 194.60000000000002, 197.55, 200.5, 203.925, 207.35000000000002, 210.8, 214.25, 219.75, 225.25, 229.275, 233.3, 236.70000000000002, 240.10000000000002, 242.75000000000003, 245.40000000000003, 248.50000000000003, 251.60000000000002, 255.15000000000003, 258.70000000000005, 263.25, 267.8, 272.4, 277.0, 284.65, 292.3, 299.35, 306.40000000000003, 311.32500000000005, 316.25, 321.125, 326.0, 328.625, 331.25, 334.20000000000005, 337.15000000000003, 339.35, 341.55, 344.35, 347.15000000000003, 350.25, 353.35, 355.75, 358.15000000000003, 361.25, 364.35, 366.975, 369.6, 371.975, 374.35, 376.8, 379.25, 381.57500000000005, 383.90000000000003, 384.55000000000007, 385.20000000000005, 385.1, 385.0, 385.25, 385.5, 385.1, 384.70000000000005, 383.07500000000005, 381.45000000000005, 379.475, 377.5, 375.5, 373.5, 371.82500000000005, 370.15000000000003, 367.95000000000005, 365.75, 363.725, 361.70000000000005, 358.55000000000007, 355.40000000000003, 353.80000000000007, 352.20000000000005, 346.70000000000005, 341.20000000000005, 335.77500000000003, 330.35, 324.725, 319.1, 313.40000000000003, 307.70000000000005, 302.6, 297.5, 291.85, 286.20000000000005, 282.70000000000005, 279.20000000000005, 276.725, 274.25, 271.9, 269.55, 266.07500000000005, 262.6, 258.75, 254.90000000000003, 251.82500000000002, 248.75, 246.0, 243.25, 240.35000000000002, 237.45000000000005, 234.35000000000002, 231.25, 229.375, 227.5, 225.05, 222.60000000000002, 221.3, 220.0, 217.625, 215.25, 213.75, 212.25, 210.10000000000002, 207.95000000000005, 205.57500000000002, 203.2, 199.75, 196.3, 192.275, 188.25, 182.925, 177.60000000000002, 173.25000000000003, 168.90000000000003, 165.40000000000003, 161.90000000000003, 155.77500000000003, 149.65, 144.825, 140.0, 134.575, 129.15, 125.75000000000001, 122.35000000000002, 121.57500000000002, 120.80000000000001, 119.72500000000001, 118.65, 117.97500000000001, 117.30000000000001, 116.775, 116.25, 115.55000000000001, 114.85000000000002, 114.15000000000002, 113.45000000000002, 113.52500000000002, 113.60000000000002, 113.52500000000002, 113.45000000000002, 112.92500000000001, 112.4, 112.12500000000001, 111.85000000000002, 112.00000000000001, 112.15, 112.87500000000001, 113.60000000000002, 113.72500000000002, 113.85000000000002, 114.45000000000002, 115.05000000000001, 115.20000000000002, 115.35000000000002, 115.47500000000002, 115.60000000000002, 115.95000000000002, 116.30000000000001, 116.72500000000001, 117.15, 117.25000000000001, 117.35000000000002, 117.72500000000002, 118.10000000000002, 118.25000000000001, 118.4, 119.12500000000001, 119.85000000000002, 120.17500000000001, 120.5, 121.95, 123.4, 124.12500000000001, 124.85000000000002, 125.57500000000002, 126.30000000000001, 127.525, 128.75, 129.55, 130.35000000000002, 130.75, 131.15, 131.8, 132.45000000000002, 132.55, 132.65, 132.85000000000002, 133.05, 133.07500000000002, 133.10000000000002, 133.20000000000002, 133.3, 133.72500000000002, 134.15, 134.60000000000002, 135.05};

deque<double> y_cord = {0.15000000000000568, 1.125000000000007, 2.1000000000000085, 3.07500000000001, 4.050000000000011, 5.57500000000001, 7.1000000000000085, 8.475000000000009, 9.850000000000009, 10.82500000000001, 11.800000000000011, 13.125000000000007, 14.450000000000003, 15.500000000000007, 16.55000000000001, 18.20000000000001, 19.85000000000001, 20.85000000000001, 21.85000000000001, 23.47500000000001, 25.10000000000001, 27.025000000000013, 28.950000000000017, 30.92500000000001, 32.900000000000006, 35.42500000000001, 37.95000000000002, 40.70000000000002, 43.45000000000002, 46.57500000000002, 49.70000000000002, 52.95000000000002, 56.20000000000002, 59.125000000000014, 62.05000000000001, 64.82500000000002, 67.60000000000002, 71.32500000000002, 75.05000000000001, 78.17500000000001, 81.30000000000001, 83.20000000000002, 85.10000000000002, 86.70000000000002, 88.30000000000001, 90.25000000000001, 92.20000000000002, 95.05000000000001, 97.9, 100.17500000000001, 102.45000000000002, 104.05000000000001, 105.65, 107.325, 109.0, 110.875, 112.75, 113.95, 115.15, 116.525, 117.9, 119.00000000000001, 120.10000000000002, 121.50000000000001, 122.9, 124.37500000000001, 125.85000000000002, 127.72500000000002, 129.60000000000002, 132.3, 135.0, 138.15, 141.3, 144.5, 147.70000000000002, 151.5, 155.3, 158.22500000000002, 161.15000000000003, 163.55, 165.95, 170.1, 174.25, 178.875, 183.5, 188.125, 192.75, 196.925, 201.10000000000002, 205.75000000000003, 210.40000000000003, 215.45000000000002, 220.5, 224.875, 229.25, 233.9, 238.55, 243.32500000000002, 248.10000000000002, 253.82500000000002, 259.55, 264.175, 268.8, 273.82500000000005, 278.85, 283.225, 287.6, 291.675, 295.75, 300.35, 304.95000000000005, 309.42500000000007, 313.90000000000003, 317.57500000000005, 321.25, 325.05, 328.85, 330.57500000000005, 332.3, 334.725, 337.15000000000003, 338.40000000000003, 339.65000000000003, 343.17500000000007, 346.70000000000005, 350.32500000000005, 353.95000000000005, 358.67500000000007, 363.40000000000003, 369.67500000000007, 375.95000000000005, 381.27500000000003, 386.6, 392.1, 397.6, 401.175, 404.75, 407.9, 411.05, 414.6, 418.15, 421.45, 424.75, 428.075, 431.4, 435.75, 440.1, 443.65000000000003, 447.20000000000005, 451.32500000000005, 455.45000000000005, 459.32500000000005, 463.20000000000005, 466.425, 469.65, 473.625, 477.6, 480.02500000000003, 482.45000000000005, 484.95000000000005, 487.45000000000005, 489.475, 491.5, 493.2, 494.9, 495.625, 496.35, 496.57500000000005, 496.80000000000007, 495.50000000000006, 494.20000000000005, 490.85, 487.5, 484.95, 482.4, 480.05, 477.70000000000005, 474.77500000000003, 471.85, 468.27500000000003, 464.70000000000005, 460.82500000000005, 456.95000000000005, 452.02500000000003, 447.1, 443.975, 440.85, 436.675, 432.5, 427.625, 422.75, 416.3, 409.85, 404.8, 399.75, 393.07500000000005, 386.40000000000003, 378.82500000000005, 371.25, 363.125, 355.0, 348.425, 341.85, 334.675, 327.5, 321.20000000000005, 314.90000000000003, 307.05000000000007, 299.20000000000005, 293.475, 287.75, 281.20000000000005, 274.65000000000003, 267.92500000000007, 261.20000000000005, 255.02500000000003, 248.85000000000002, 243.40000000000003, 237.95000000000005, 231.97500000000002, 226.0, 219.72500000000002, 213.45000000000005, 206.15000000000003, 198.85000000000002, 192.45000000000002, 186.05, 178.375, 170.7, 162.05, 153.4, 144.9, 136.4, 129.075, 121.75, 114.42500000000001, 107.10000000000002, 99.35000000000002, 91.60000000000002, 84.00000000000001, 76.4, 69.42500000000001, 62.45000000000002, 59.32500000000002, 56.20000000000002, 54.95000000000002, 53.70000000000002, 53.05000000000001, 52.400000000000006, 51.92500000000001, 51.45000000000002, 50.80000000000001, 50.150000000000006, 47.85000000000001, 45.55000000000001, 42.875000000000014, 40.20000000000002};

deque<double> dr_x_cord = {0, 16.5513, 33.1027, 49.6540, 66.2054, 82.7567, 99.3081, 115.8594, 132.4108, 148.9621};

deque<double> dr_y_cord = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

void amcl_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
	double new_x = msg->pose.pose.position.x;
	double new_y = msg->pose.pose.position.y;
	double new_z = msg->pose.pose.position.z;
	double siny_cosp = 2.0 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z + msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
	double cosy_cosp = 1.0 - 2.0 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y + msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);
	double new_ori = atan2(siny_cosp, cosy_cosp);
	buggy.amcl_update_pose(new_x, new_y, new_z, new_ori);
}

void dead_reckoning_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
	double dr_x = msg->pose.pose.position.x;
	double dr_y = msg->pose.pose.position.y;
	double dr_z = msg->pose.pose.position.z;
	double dr_siny_cosp = 2.0 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z + msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
	double dr_cosy_cosp = 1.0 - 2.0 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y + msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);
	double dr_ori = atan2(dr_siny_cosp, dr_cosy_cosp);
	buggy.dr_update_pose(dr_x, dr_y, dr_z, dr_ori);
}

void speed_callback(const std_msgs::Float64::ConstPtr& msg)
{
	double new_speed = msg->data;
	buggy.update_speed(new_speed);
}

void ch3_callback(const std_msgs::UInt16::ConstPtr& msg)
{
	if (msg->data > 1460) {
		buggy.is_manual = false;
	}
	else {
		buggy.is_manual = true;
	}
}

int main(int argc, char **argv)
{
	deque<pair<double, double>> path;
	for (int i = 0; i < x_cord.size(); i++)
	{
		path.push_back(make_pair(x_cord[i], y_cord[i]));
	}

	deque<pair<double, double>> dr_path;
	for (int i = 0; i < dr_x_cord.size(); i++)
	{
		dr_path.push_back(make_pair(dr_x_cord[i], dr_y_cord[i]));
	}	

	buggy.load_path(path);
	buggy.load_dr_path(dr_path);

	ros::init(argc, argv, "controller");
	ros::NodeHandle n;

	ros::Publisher steering_pub = n.advertise<std_msgs::Float64>("steering_angle", 10);
	ros::Publisher target_pose_pub = n.advertise<geometry_msgs::PointStamped>("target_point", 10);

	ros::Subscriber amcl_sub = n.subscribe<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", 10, amcl_callback);
	ros::Subscriber dead_reckoning_sub = n.subscribe<nav_msgs::Odometry>("odom", 10, dead_reckoning_callback);
	ros::Subscriber speed_sub = n.subscribe<std_msgs::Float64>("wheel_vel", 10, speed_callback);
	ros::Subscriber ch3_sub = n.subscribe<std_msgs::UInt16>("ch3", 10, ch3_callback);

	ros::Rate loop_rate(10);

	int count = 0;
	while (ros::ok())
	{
		if (buggy.index >= 9) {
			buggy.use_amcl = true;
		}

		if (buggy.is_manual) {
			ROS_INFO("MANUAL");
		}
		else {
			ROS_INFO("AUTONOMOUS");
		}

		geometry_msgs::PointStamped target_msg;
		target_msg.header.frame_id = "map";
		target_msg.point.x = x_cord[buggy.target_index];
		target_msg.point.y = y_cord[buggy.target_index];
		target_pose_pub.publish(target_msg);

		std_msgs::Float64 angle_msg;
		double angle = buggy.pp_control();
		angle_msg.data = angle;
		steering_pub.publish(angle_msg);

		ros::spinOnce();

		loop_rate.sleep();
		++count;
	}

	return 0;
}
