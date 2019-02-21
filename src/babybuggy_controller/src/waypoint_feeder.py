#!/usr/bin/env python
import math
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped

waypoints = []
waypoint_ind = 0
threshold = None

cur_pose = None

def dist(x1,y1,x2,y2):
    return math.sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2))

def pose_callback(msg):
    global cur_pose
    cur_pose = msg.pose

def get_next_waypoint():
    global cur_pose, threshold

    if cur_pose == None:
        return None

    x_pose = cur_pose.pose.position.x
    y_pose = cur_pose.pose.position.y

    wp_x = waypoints[waypoint_ind][0]
    wp_y = waypoints[waypoint_ind][1]

    if dist(x_pose, wp_x, y_pose, wp_y) < threshold:
        waypoint_ind = waypoint_ind + 1
        wp_x = waypoints[waypoint_ind][0]
        wp_y = waypoints[waypoint_ind][1]

    return (wp_x, wp_y)


def read_waypoints(filename):
    global waypoints

    f = open(filename, "r")
    for line in f:
        line = line.strip()
        split_line = line.split(" ")
        wp = (float(split_line[0]), float(split_line[1]))
        waypoints.append(wp)

def main():
    global threshold

    rospy.init_node('waypoint_feeder', anonymous=True)

    rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, pose_callback)
    waypoint_pub = rospy.Publisher('next_waypoint', Pose, queue_size=10)

    # get parameters from launch file
    filename = rospy.get_param('~filename')
    threshold = rospy.get_param('~threshold')

    # load waypoints from file
    read_waypoints(filename)

    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        # get next waypoint
        waypoint = get_next_waypoint()

        # publish waypoint to /next_waypoint if waypoint exists
        if waypoint != None:
            msg = Pose()
            msg.position.x = waypoint[0]
            msg.position.y = waypoint[1]
            waypoint_pub.publish(msg)

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
