#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Int32
import serial

# Receives a message from the arduino through serial and parses the sensor
# data from that message

# Message Format:
#   <quatx: double> \t<quaty: double>\t<quatz: double>\t<quatw: double>
#   <latitude:  double>\t<longitude: double>\t<altitude: double>\t
#   <ch1: int>\t<ch2: int>\t<ch3: int>\t<ch4: int>\t<ch5: int>\t<ch6: int>\n

def parse_GPS(data):
    # parses GPS part of string in the form:
    GPS_msg = NavSatFix()
    GPS_msg.latitude = float(data[0])
    GPS_msg.longitude = float(data[1])
    GPS_msg.altitude = float(data[2])
    # GPS_msg.status.status = int(data[3]) # update flag
    return GPS_msg


def parse_encoders(data):
    # parses channel part of the string of the form:
    enc1_msg = Int32()
    enc2_msg = Int32()

    enc1_msg.data = int(data[0])
    enc2_msg.data = int(data[1])
    return [enc1_msg, enc2_msg]


def parse_IMU(data):
    # parses IMU part of the string of the form
    IMU_msg = Imu()
    IMU_msg.orientation.x = float(data[0])
    IMU_msg.orientation.y = float(data[1])
    IMU_msg.orientation.z = float(data[2])
    IMU_msg.orientation.w = float(data[3])
    return IMU_msg


def parse_sensors(ser):
    number_params = 10
    GPS_start = 4
    IMU_start = 0
    encoder_start = 7
    GPS_length = 3
    IMU_length = 4
    encoder_length = 2

    # define the publishers(IMU, GPS, ch1-ch6)
    imu_pub = rospy.Publisher('IMU', Imu, queue_size=10)
    gps_pub = rospy.Publisher('GPS', NavSatFix, queue_size=10)
    enc1_pub = rospy.Publisher('enc1', Int32, queue_size=10)
    enc2_pub = rospy.Publisher('enc2', Int32, queue_size=10)

    rospy.init_node('sensor_info', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():

        # read the line from the arduino, parse the data for each message
        line = ser.readline()

	rospy.loginfo(line)

        data = line.split('\t')

        # if the full data didn't come though, ignore
        if len(data) != number_params:
            continue

        # print("test")

        # parse the strings into ros messages
        imu_msg = parse_IMU(data[IMU_start : IMU_start + IMU_length])
        gps_msg = parse_GPS(data[GPS_start : GPS_start + GPS_length])
        enc_msgs = parse_encoders(data[encoder_start : encoder_start + encoder_length])


        # publish on all the topics
        imu_pub.publish(imu_msg)
        gps_pub.publish(gps_msg)
        enc1_pub.publish(enc_msgs[0])
        enc2_pub.publish(enc_msgs[1])

        rate.sleep()

if __name__ == '__main__':
    # initialize the serial connection to the arduino
    ser = serial.Serial()
    ser.baudrate = 115200
    ser.port = "/dev/serial/by-id/usb-Arduino__www.arduino.cc__0042_55834323633351F030F1-if00"
    ser.timeout = 1
    ser.open()

    try:
        parse_sensors(ser)
    except rospy.ROSInterruptException:
        ser.close()

