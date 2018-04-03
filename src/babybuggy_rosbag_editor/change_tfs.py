import sys
import rosbag
import time
import yaml
import os
import subprocess

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

rosbag_in_path = '/home/woz4tetra/BabyBuggyROS/bags/03-31-courserun.bag'
rosbag_out_path = '/home/woz4tetra/BabyBuggyROS/bags/03-31-courserun-edited.bag'

info_dict = yaml.load(subprocess.Popen(['rosbag', 'info', '--yaml', rosbag_in_path], stdout=subprocess.PIPE).communicate()[0])
duration = info_dict['duration']
start_time = info_dict['start']

with rosbag.Bag(rosbag_out_path, 'w') as outbag:
    last_time = time.clock()
    for topic, msg, t in rosbag.Bag(rosbag_in_path).read_messages():
        # This also replaces tf timestamps under the assumption
        # that all transforms in the message share the same timestamp
        if topic == "/robot_pose":
            continue
        if topic == "/tf" and msg.transforms:
            remove_index = -1
            for index, tf in enumerate(msg.transforms):
                if tf.header.frame_id == "odom" and tf.child_frame_id == "imu":
                    # tf.child_frame_id = "base_link"
                    remove_index = index

                if tf.header.frame_id == "imu" and tf.child_frame_id == "base_link":
                    tf.child_frame_id = "imu"
                    tf.header.frame_id = "base_link"

                    remove_index = index
            if remove_index >= 0:
                msg.transforms.pop(remove_index)
        outbag.write(topic, msg, msg.header.stamp if msg._has_header else t)

        if time.clock() - last_time > .1:
            percent = (t.to_sec() - start_time) / duration
            status(40, percent)
            last_time = time.clock()

status(40, 1)
print "\ndone"
