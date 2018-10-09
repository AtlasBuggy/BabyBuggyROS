#!/usr/bin/env python
import rospy
import math
import numpy as np

import tf2_ros

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Pose, TransformStamped
from std_msgs.msg import Float64

from tf.transformations import euler_from_quaternion, quaternion_from_euler

p = 0.1
state = [0, 0, 0, 0, 0, 0] # [x, y, z, v, w]
prev_time = None
orientation = [0, 0, 0, 1]

odom_pub = None

def wheel_vel_callback(msg):
    global state
    state[3] = msg.data

def ang_vel_callback(msg):
    global state
    state[4] = msg.data

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

    yawMatrix = np.matrix([
                [math.cos(yaw), -math.sin(yaw), 0],
                [math.sin(yaw), math.cos(yaw), 0],
                [0, 0, 1]
                ])

    pitchMatrix = np.matrix([
                [math.cos(pitch), 0, math.sin(pitch)],
                [0, 1, 0],
                [-math.sin(pitch), 0, math.cos(pitch)]
                ])

    rollMatrix = np.matrix([
                [1, 0, 0],
                [0, math.cos(roll), -math.sin(roll)],
                [0, math.sin(roll), math.cos(roll)]
                ])

    R = yawMatrix * pitchMatrix * rollMatrix
    vel_mat = np.matrix([[state[3]], [0], [0]])

    transf_vel = R * vel_mat

    # state[2] += (yaw - state[2]) * p # smooth yaw reading

    cur_time = rospy.get_time()
    dt = cur_time - prev_time
    prev_time = cur_time

    state[0] = state[0] + transf_vel[0,0]*dt
    state[1] = state[1] + transf_vel[1,0]*dt
    state[2] = state[2] + transf_vel[2,0]*dt

def main():
    global odom_pub, orientation, state
    rospy.init_node('dead_reckoning', anonymous=True)

    rospy.Subscriber("wheel_vel", Float64, wheel_vel_callback)
    rospy.Subscriber("ang_vel", Float64, ang_vel_callback)
    rospy.Subscriber("BNO055", Imu, imu_callback)

    odom_pub = rospy.Publisher("dead_reckon_odom", Pose, queue_size=10)

    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        # br = tf.TransformBroadcaster()
        # br.sendTransform(state[0:3], orientation, \
        #                  rospy.Time.now(), \
        #                  "odom", "base_link")

        broadcaster = tf2_ros.StaticTransformBroadcaster()
        static_transformStamped = TransformStamped()

        static_transformStamped.header.stamp = rospy.Time.now()
        static_transformStamped.header.frame_id = "odom"
        static_transformStamped.child_frame_id = "base_link"

        static_transformStamped.transform.translation.x = state[0]
        static_transformStamped.transform.translation.y = state[1]
        static_transformStamped.transform.translation.z = state[2]

        static_transformStamped.transform.rotation.x = orientation[0]
        static_transformStamped.transform.rotation.y = orientation[1]
        static_transformStamped.transform.rotation.z = orientation[2]
        static_transformStamped.transform.rotation.w = orientation[3]

        broadcaster.sendTransform(static_transformStamped)

        msg = Pose()
        msg.position.x = state[0]
        msg.position.y = state[1]
        msg.position.z = state[2]

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
