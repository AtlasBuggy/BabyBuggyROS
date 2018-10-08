#!/usr/bin/env python
import rospy
import sys

from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Pose

from tf.transformations import euler_from_quaternion, quaternion_from_euler

nu = 0.1
channel = sys.argv[1]
current_data = 0.0

def wheel_vel_callback(data):
    diff = data.data - current_data
    current_data += nu * diff
    pub.publish(current_data)

def imu_callback(data):

def main():
    global pf_pub
    rospy.init_node('particle_filter', anonymous=True)

    rospy.Subscriber("wheel_vel", Float64, wheel_vel_callback)
    rospy.Subscriber("BNO055", Imu, imu_callback)

    pf_pub = rospy.Publisher("pf_estimate", Pose, queue_size=10)

    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        msg = Pose()

        msg.position.x = state[0]
        msg.position.y = state[1]

        msg.orientation.x = orientation[0]
        msg.orientation.y = orientation[1]
        msg.orientation.z = orientation[2]
        msg.orientation.w = orientation[3]

        odom_pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
