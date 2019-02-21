#!/usr/bin/env python
import rospy
import sys
import math
import time

from std_msgs.msg import Float64, Int64

# ROS variables
vel_pub = None

# Constants
p = 0.2
d = 0
enc_tpr = 1024.0
enc_radius = 0.030825
enc_ticks_to_m = 2.0 * math.pi * enc_radius / enc_tpr
wheel_dist = 0.152

# Calculation variables
left_dist = None
left_vel = 0
left_prev_time = 0
left_prev_err = None
left_updated = 0

right_dist = None
right_vel = 0
right_prev_time = 0
right_prev_err = None
right_updated = 0

ang_vel = 0
ang_vel_prev_time = 0

def enc1_callback(msg):
    global right_dist, right_vel, right_prev_time, right_prev_err
    global enc_ticks_to_m

    if right_dist == None:
        right_dist = msg.data * enc_ticks_to_m
        # right_prev_time = rospy.get_time()
        right_prev_time = time.time()
    else:
        cur_dist = msg.data * enc_ticks_to_m
        # cur_time = rospy.get_time()
        cur_time = time.time()

        d_dist = cur_dist - right_dist
        d_t = cur_time - right_prev_time

        right_dist = d_dist
        right_prev_time = cur_time

        cur_vel = d_dist / d_t
        err = cur_vel - right_vel

        right_vel = right_vel + p * err

def enc2_callback(msg):
    global left_dist, left_vel, left_prev_time, left_prev_err
    global enc_ticks_to_m

    # update left velocity and dist
    if left_dist == None:
        left_dist = msg.data * enc_ticks_to_m
        # left_prev_time = rospy.get_time()
        left_prev_time = time.time()
    else:
        cur_dist = msg.data * enc_ticks_to_m
        # cur_time = rospy.get_time()
        cur_time = time.time()

        d_dist = cur_dist - left_dist
        d_t = cur_time - left_prev_time

        left_dist = d_dist
        left_prev_time = cur_time

        cur_vel = d_dist / d_t
        err = cur_vel - left_vel

        left_vel = left_vel + p * err

def main():
    global vel_pub, left_vel, right_vel, wheel_dist
    global ang_vel, left_dist, right_dist, ang_vel_prev_time

    rospy.init_node('velocity_calculator', anonymous=True)

    rospy.Subscriber("encoder1_raw", Int64, enc1_callback) # right_vel
    rospy.Subscriber("encoder2_raw", Int64, enc2_callback) # left_vel

    vel_pub = rospy.Publisher("wheel_vel", Float64, queue_size=10)
    ang_vel_pub = rospy.Publisher("ang_vel", Float64, queue_size=10)

    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        # calculate and publish linear velocity
        # vel = (right_vel + left_vel)/2.0
        vel = left_vel
        vel_pub.publish(vel)

        # calculate and publish angular velocity
        if (right_dist != None) and (left_dist != None):
            cur_time = rospy.get_time()
            d_dist = (right_dist - left_dist)/(wheel_dist)
            d_t = cur_time - ang_vel_prev_time

            cur_ang_vel = d_dist / d_t
            ang_vel_prev_time = cur_time

            ang_vel += (cur_ang_vel - ang_vel)*p
            ang_vel_pub.publish(ang_vel)

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
