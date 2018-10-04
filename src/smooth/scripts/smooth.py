#!/usr/bin/env python
import rospy
import sys
from std_msgs.msg import Float64

k = 0.1
if (len(sys.argv) < 2):
    print("Usage: smooth.py channel_name")
    raise ValueError('Please include as an argument the channel you want to smooth!')
channel = sys.argv[1]
speed = 0.0
pub = rospy.Publisher("smooth_" + channel, Float64, queue_size=10)

def smooth_callback(data):
    diff = data.data - speed
    speed += k * diff
    pub.publish(speed)

def smoother():
    rospy.init_node('smoother', anonymous=True)
    rospy.Subscriber(channel, Float64, smooth_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        smoother()
    except rospy.ROSInterruptException:
        pass