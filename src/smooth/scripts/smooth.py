#!/usr/bin/env python
import rospy
import sys
from std_msgs.msg import Float64, Int64

nu = 0.1
if (len(sys.argv) < 3):
    print("Usage: smooth.py [f/i] [channel_name]")
    raise ValueError('Please include as an argument the channel you want to smooth!')
if (len(sys.argv) >= 4):
    nu = float(sys.argv[3])
isFloat = (sys.arv[2] == "f")
channel = sys.argv[1]
current_data = 0.0
pub = rospy.Publisher("smooth_" + channel, Float64, queue_size=10)

def smooth_callback(data):
    global current_data
    diff = data.data - current_data
    current_data += nu * diff
    pub.publish(current_data)

def smoother():
    rospy.init_node('smoother', anonymous=True)
    if (isFloat):
        rospy.Subscriber(channel, Float64, smooth_callback)
    else
        rospy.Subscriber(channel, Int64, smooth_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        smoother()
    except rospy.ROSInterruptException:
        pass