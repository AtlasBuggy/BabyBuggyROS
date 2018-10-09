#!/usr/bin/env python
import rospy
import sys
import math

from std_msgs.msg import Float64, Int64

# ROS variables
vel_pub = None

# Constants
p = 0.1
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

def enc1_callback(msg):
    global left_dist, left_vel, left_prev_time, left_prev_err
    global enc_ticks_to_m

    # update left velocity and dist
    if left_dist == None:
        left_dist = msg.data * enc_ticks_to_m
        left_prev_time = rospy.get_time()
    else:
        cur_dist = msg.data * enc_ticks_to_m
        cur_time = rospy.get_time()

        d_dist = cur_dist - left_dist
        d_t = cur_time - left_prev_time

        left_dist = cur_dist
        left_prev_time = cur_time

        cur_vel = d_dist / d_t
        err = cur_vel - left_vel

        if left_prev_err == None:
            left_prev_err = err

        left_vel += p * err + d * ((err - left_prev_err) / d_t)
        left_prev_err = err

def enc2_callback(msg):
    global right_dist, right_vel, right_prev_time, right_prev_err
    global enc_ticks_to_m

    if right_dist == None:
        right_dist = msg.data * enc_ticks_to_m
        right_prev_time = rospy.get_time()
    else:
        cur_dist = msg.data * enc_ticks_to_m
        cur_time = rospy.get_time()

        d_dist = cur_dist - right_dist
        d_t = cur_time - right_prev_time

        right_dist = cur_dist
        right_prev_time = cur_time

        cur_vel = d_dist / d_t
        err = cur_vel - right_vel

        if right_prev_err == None:
            right_prev_err = err

        right_vel += d * err + d * ((err - right_prev_err) / d_t)
        right_prev_err = err

def main():
    global vel_pub, left_vel, right_vel, wheel_dist
    rospy.init_node('velocity_calculator', anonymous=True)

    rospy.Subscriber("encoder1_raw", Int64, enc1_callback)
    rospy.Subscriber("encoder2_raw", Int64, enc2_callback)

    vel_pub = rospy.Publisher("wheel_vel", Float64, queue_size=10)
    ang_vel_pub = rospy.Publisher("ang_vel", Float64, queue_size=10)

    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        vel = (right_vel + left_vel)/2.0
        vel_pub.publish(vel)

        ang_vel = (right_vel - left_vel)/(wheel_dist)
        # ang_vel = math.atan2(math.sin(ang_vel), math.cos(ang_vel))

        ang_vel_pub.publish(ang_vel)

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
