#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import UInt32, UInt8

ROAD_NUM = 3        # the classification number of the road in segnet
STRAIGHT_TOL = 0.1  # the tolerance for what is considered straight on
ROAD_TOL = 0.1      # the tolerance for deciding if on the road
ROAD_RATIO = 0.3    # ratio of road to other semantic objects

def get_direction(width, height, image):
    # -1 = left
    # 0 = straight
    # 1 = right
    # 2 = could not find direction
    leftCount = 0
    rightCount = 0
    for i in range(height):
        for j in range(width):
            index = i * width + j
            if (image[index] != ROAD_NUM):
                # not a road segment, ignore
                continue
            if j < width / 2:
                leftCount += 1
            else:
                rightCount += 1

    roadRatio = float(leftCount + rightCount) / float(width * height)
    # print("ratio of road: " + str(roadRatio))

    if not (ROAD_RATIO - ROAD_TOL <= roadRatio <= ROAD_RATIO + ROAD_TOL):
        return 2

    lrRatio = float(rightCount) / float(leftCount)
    # print("left: " + str(leftCount) + "\nright: " + str(rightCount) +
    #         "\nratio: " + str(lrRatio))

    if (1 - STRAIGHT_TOL <= lrRatio <= 1 + STRAIGHT_TOL):
        return 0
    else:
        return 1 if rightCount > leftCount else -1

def segment_callback(data, pub):
    # the data is a sensor_msgs/Image with the following fields:
    # - uint32 height
    # - uint32 width
    # - uint8[] data, which is represented as a string of bytes
    w = data.width
    h = data.height
    image = list(bytearray(data.data))

    direction = get_direction(w, h, image)
    # print("direction: " + str(direction) + "\n")
    pub.publish(direction)

def listen_for_segmentation():
    pub = rospy.Publisher('vision/steering_angle', UInt8, queue_size=10)
    rospy.init_node('vision', anonymous=True)
    callback_lambda = lambda x: segment_callback(x, pub)
    rospy.Subscriber('segnet/class_mask', Image, callback_lambda)
    rospy.spin()

if __name__ == '__main__':
    try:
        listen_for_segmentation()
    except rospy.ROSInterruptException:
        pass
