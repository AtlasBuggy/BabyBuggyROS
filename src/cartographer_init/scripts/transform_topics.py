#!/usr/bin/env python
import rospy
import sys

from sensor_msgs.msg import LaserScan, MultiEchoLaserScan
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

multi_echo_pub = None
odom_pub = None

wheel_vel = 0

def dead_reckoning_callback(msg):
    global odom_pub

    new_msg = Odometry()
    new_msg.header.frame_id = "odom"
    new_msg.child_frame_id = "base_link"
    new_msg.pose.pose.position = msg.position
    new_msg.pose.pose.orientation = msg.orientation

    odom_pub.publish(new_msg)

def wheel_vel_callback(msg):
    global wheel_vel
    wheel_vel = msg.data

def laser_scan_callback(msg):
    global multi_echo_pub

    new_msg = MultiEchoLaserScan()

    new_msg.header.frame_id = msg.header.frame_id

    new_msg.angle_min = msg.angle_min
    new_msg.angle_max = msg.angle_max
    new_msg.angle_increment = msg.angle_increment
    new_msg.time_increment = msg.time_increment
    new_msg.scan_time = msg.scan_time
    new_msg.range_min = msg.range_min
    new_msg.range_max = msg.range_max

    new_msg.ranges.echoes = msg.ranges
    new_msg.intensities.echoes = msg.intensities

    multi_echo_pub.publish(new_msg)

def main():
    global multi_echo_pub, odom_pub

    rospy.init_node('cartographer_transform', anonymous=True)

    multi_echo_pub = rospy.Publisher("echoes", MultiEchoLaserScan, queue_size=10)
    odom_pub = rospy.Publisher("odom", Odometry, queue_size=10)

    rospy.Subscriber("dead_reckon_odom", Pose, dead_reckoning_callback)
    rospy.Subscriber("wheel_vel", Float64, wheel_vel_callback)

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
