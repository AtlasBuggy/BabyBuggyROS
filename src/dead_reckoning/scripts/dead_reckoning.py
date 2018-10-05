#!/usr/bin/env python
import rospy
import math

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64

from tf.transformations import euler_from_quaternion, quaternion_from_euler

p = 0.1
state = [0, 0, 0, 0] # x, y, th, v
prev_time = None
orientation = [0, 0, 0, 1]

odom_pub = None

def wheel_vel_callback(msg):
    global state
    state[3] = msg.data

def imu_callback(msg):
    global state, prev_time, orientation

    if prev_time == None:
        prev_time = rospy.get_time()
        return

    x = msg.orientation.x
    y = msg.orientation.y
    z = msg.orientation.z
    w = msg.orientation.w
    orientation = [x, y, z, w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation)

    state[2] += (yaw - state[2]) * p # smooth yaw reading

    cur_time = rospy.get_time()
    dt = cur_time - prev_time
    prev_time = cur_time

    state[0] = state[0] + state[3]*math.cos(state[2])*dt
    state[1] = state[1] + state[3]*math.sin(state[2])*dt

def main():
    global odom_pub, orientation, state
    rospy.init_node('dead_reckoning', anonymous=True)

    rospy.Subscriber("wheel_vel", Float64, wheel_vel_callback)
    rospy.Subscriber("BNO055", Imu, imu_callback)

    odom_pub = rospy.Publisher("dead_reckon_odom", Pose, queue_size=10)

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
