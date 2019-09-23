#!/usr/bin/env python
import math
import rospy
from std_msgs.msg import String, Float64
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler

goal_pos = None
cur_pos = None

KP = 0.0

def pose_callback(msg):
	global cur_pos

	orientation = msg.pose.pose.orientation
	x = orientation.x
	y = orientation.y
	z = orientation.z
	w = orientation.w

	imu_quaternion = [x, y, z, w]
	(roll, pitch, yaw) = euler_from_quaternion(imu_quaternion)

	x = msg.pose.pose.position.x
	y = msg.pose.pose.position.y
	z = msg.pose.pose.position.z

	cur_pos = [x, y, z, roll, pitch, yaw]

def wp_callback(msg):
	global goal_pos
	goal_pos = [msg.position.x, msg.position.y]

def calc_steering():
	global cur_pos, goal_pos, KP

	x = cur_pos[0]
	y = cur_pos[1]
	th = cur_pos[5]

	x_g = goal_pos[0]
	y_g = goal_pos[1]

	y_r = math.cos(th)*(x_g - x) - math.sin(th)*(y_g - y)
	return KP * y_r;

def main():
	global KP
	rospy.init_node('pure_pursuit_controller', anonymous=True)

	rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, pose_callback)
	rospy.Subscriber('next_waypoint', Pose, wp_callback)

	ang_pub = rospy.Publisher("steering_angle", Float64, queue_size=10)

	KP = rospy.get_param('~KP')

	rate = rospy.Rate(10) # 10hz
	while not rospy.is_shutdown():
		# wait until received pose and goal pose
		if goal_pos == None or cur_pos == None:
			continue

		msg = Float64()
		msg.data = calc_steering()
		ang_pub.publish(msg)

		rate.sleep()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
