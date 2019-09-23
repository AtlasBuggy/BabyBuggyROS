#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import UInt16, Int32, Float64

cur_angle = None
des_angle = None
manual = False

def angle_to_steering(ang):
    len = math.tan(ang + 0.0545) * 13 + 8.89
    cmd = int((len - 0.725)/0.01775)
    return cmd

def steering_angle_callback(msg):
    global des_angle
    des_angle = -msg.data

def ch3_callback(msg):
    global manual

    if msg.data < 1460:
	manual = True
    else:
	manual = False

def main():
    global des_angle, manual
    # In ROS, nodes are uniquely named. If two nodes  with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('manual_controller', anonymous=True)
    rospy.Subscriber("steering_angle", Float64, steering_angle_callback)
    rospy.Subscriber("ch3", UInt16, ch3_callback)
    steering_pub = rospy.Publisher("steering", Int32, queue_size=10)

    # spin() simply keeps python from exiting until this node is stopped
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        if des_angle is None:
            continue

        # publish steering cmd to /steering
        msg = Int32()
        msg.data = angle_to_steering(des_angle)
        print(msg.data)

	if not manual:
	        steering_pub.publish(msg)

        rate.sleep()

if __name__ == '__main__':
    main()
