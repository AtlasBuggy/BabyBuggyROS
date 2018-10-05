#!/usr/bin/env python
import rospy
import sys
import math

from std_msgs.msg import Float64, Int64

# ROS variables
vel_pub = None

# Constants
nu = 0.1
enc_tpr = 1024.0;
enc_radius = 0.030825;
enc_ticks_to_m = 2.0 * math.pi * enc_radius / enc_tpr;

# Calculation variables
left_dist = None
right_dist = None
left_vel = 0
right_vel = 0
left_prev_time = 0
right_prev_time = 0

def enc1_callback(msg):
    global left_dist, left_vel, left_prev_time
    global enc_ticks_to_m
    
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
        diff = cur_vel - left_vel
        left_vel += nu * diff
    
def enc2_callback(msg):
    global right_dist, right_vel, right_prev_time
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
        diff = cur_vel - right_vel
        right_vel += nu * diff

def main():
    global vel_pub, left_vel, right_vel
    rospy.init_node('velocity_calculator', anonymous=True)
    
    rospy.Subscriber("encoder1_raw", Int64, enc1_callback)
    rospy.Subscriber("encoder2_raw", Int64, enc2_callback)
    
    vel_pub = rospy.Publisher("wheel_vel", Float64, queue_size=10)
    
    rate = rospy.Rate(10) # 10hz
    
    while not rospy.is_shutdown():
        vel = (left_vel + right_vel)/2.0
        vel_pub.publish(vel)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass