import sys
import math
import rosbag
import time
import yaml
import os
import subprocess
from std_msgs.msg import Int64

wheel_radius = 30.825
ticks_per_rotation = 256

def status(length, percent):
    sys.stdout.write('\x1B[2K') # Erase entire current line
    sys.stdout.write('\x1B[0E') # Move to the beginning of the current line
    progress = "Progress: ["
    for i in range(0, length):
        if i < length * percent:
            progress += '='
        else:
            progress += ' '

    progress += "] " + str(round(percent * 100.0, 2)) + "%"
    sys.stdout.write(progress)
    sys.stdout.flush()

# rosbag_in_path = '/home/woz4tetra/BabyBuggyROS/bags/back_hills_only_encoder_calibration_2018-04-07-18-11-36.bag'
# rosbag_out_path = '/home/woz4tetra/BabyBuggyROS/bags/back_hills_only_encoder_calibration_2018-04-07-18-11-36-edited.bag'

# rosbag_in_path = '/home/woz4tetra/BabyBuggyROS/bags/back_hills_only_encoder_calibration_2018-04-07-18-03-42.bag'
# rosbag_out_path = '/home/woz4tetra/BabyBuggyROS/bags/back_hills_only_encoder_calibration_2018-04-07-18-03-42-edited.bag'

rosbag_in_path = '/home/woz4tetra/BabyBuggyROS/bags/indoors_short_distance_2018-04-07-19-32-53.bag'
rosbag_out_path = '/home/woz4tetra/BabyBuggyROS/bags/indoors_short_distance_2018-04-07-19-32-53-edited.bag'

info_dict = yaml.load(subprocess.Popen(['rosbag', 'info', '--yaml', rosbag_in_path], stdout=subprocess.PIPE).communicate()[0])
duration = info_dict['duration']
start_time = info_dict['start']

with rosbag.Bag(rosbag_out_path, 'w') as outbag:
    last_time = time.clock()
    for topic, msg, t in rosbag.Bag(rosbag_in_path).read_messages():
        # invert the laser
        # if topic == "/scan":
        #     angle_min = msg.angle_min
        #     angle_max = msg.angle_max
        #     msg.angle_min = angle_max
        #     msg.angle_max = angle_min
        # delete computed old robot pose odometry calculations
        if topic == "/encoder":
            msg = Int64(int(msg.data / (2 * wheel_radius * math.pi / ticks_per_rotation)))
        # if topic == "/robot_pose":
        #     continue
        #
        # # remove tf data
        # if topic == "/tf":
        #     continue

        # modify tf data
        # if topic == "/tf" and msg.transforms:
        #     remove_indices = []
        #     for index, tf in enumerate(msg.transforms):
        #         if tf.header.frame_id == "odom" and tf.child_frame_id == "imu":
        #             # tf.child_frame_id = "base_link"
        #             remove_indices.append(index)
        #
        #         elif tf.header.frame_id == "base_link" and tf.child_frame_id == "laser":
        #             remove_indices.append(index)
        #
        #         elif tf.header.frame_id == "imu" and tf.child_frame_id == "base_link":
        #             remove_indices.append(index)
        #
        #         # if tf.header.frame_id == "imu" and tf.child_frame_id == "base_link":
        #         #     tf.child_frame_id = "imu"
        #         #     tf.header.frame_id = "base_link"
        #
        #     if remove_indices:
        #         for index in remove_indices:
        #             msg.transforms.pop(index)
        outbag.write(topic, msg, msg.header.stamp if msg._has_header else t)

        if time.clock() - last_time > .1:
            percent = (t.to_sec() - start_time) / duration
            status(40, percent)
            last_time = time.clock()

status(40, 1)
print "\ndone"
