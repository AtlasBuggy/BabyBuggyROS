#!/usr/bin/env python
import rospy
from std_msgs.msg import UInt16, Int32

LEFT = 2048
RIGHT = 4096
CENTER = 365
STOP = 1500

ch1_threshold = 1470
ch1_val = None

def get_steering_cmd():
    global ch1_val, ch1_threshold

    if ch1_val == None:
        return None

    diff = ch1_val - ch1_threshold

    if abs(diff) > 40:
        if diff > 0:
            return RIGHT
        else:
            return LEFT
    else:
        return CENTER

def ch1_callback(msg):
    global ch1_val
    ch1_val = msg.data

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('manual_controller', anonymous=True)

    rospy.Subscriber("ch1", UInt16, ch1_callback)
    steering_pub = rospy.Publisher("steering", Int32, queue_size=10)

    # spin() simply keeps python from exiting until this node is stopped
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        # get steering cmd
        steering_cmd = get_steering_cmd()

        # publish steering cmd to /steering
        if steering_cmd != None:
            msg = Int32()
            msg.data = steering_cmd
            steering_pub.publish(msg)

        rate.sleep()

if __name__ == '__main__':
    listener()
