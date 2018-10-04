#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Float64

k = 0.1
channel = "velocity"
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