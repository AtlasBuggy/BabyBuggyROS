#!/usr/bin/env python
import rospy
import sys

from sensor_msgs.msg import LaserScan, MultiEchoLaserScan, LaserEcho
from nav_msgs.msg import Odometry

multi_echo_pub = None
odom_pub = None

def dead_reckoning_callback(msg):
    global odom_pub

    new_msg = Odometry()
    new_msg.header.stamp = rospy.Time.now()
    new_msg.header.frame_id = "odom_carto"
    new_msg.child_frame_id = "base_link"
    new_msg.pose = msg.pose

    odom_pub.publish(new_msg)

def laser_scan_callback(msg):
    global multi_echo_pub

    new_msg = MultiEchoLaserScan()

    new_msg.header.frame_id = "echoes"

    new_msg.angle_min = msg.angle_min
    new_msg.angle_max = msg.angle_max
    new_msg.angle_increment = msg.angle_increment
    new_msg.time_increment = msg.time_increment
    new_msg.scan_time = msg.scan_time
    new_msg.range_min = msg.range_min
    new_msg.range_max = 32.8

    new_msg.ranges = []
    new_msg.intensities = []

    ranges = msg.ranges
    intensities = msg.intensities

    for i in range(len(msg.ranges)):
        n = LaserEcho()
        m = LaserEcho()

        n.echoes = [ranges[i]]
        m.echoes = [0]

        new_msg.ranges.append(n)
        new_msg.intensities.append(m)

    multi_echo_pub.publish(new_msg)

def main():
    global multi_echo_pub
    global odom_pub

    rospy.init_node('cartographer_transform', anonymous=True)

    multi_echo_pub = rospy.Publisher("echoes", MultiEchoLaserScan, queue_size=10)
    odom_pub = rospy.Publisher("odom", Odometry, queue_size=10)

    rospy.Subscriber("dead_reckon_odom", Pose, dead_reckoning_callback)
    rospy.Subscriber("scan", LaserScan, laser_scan_callback)

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
