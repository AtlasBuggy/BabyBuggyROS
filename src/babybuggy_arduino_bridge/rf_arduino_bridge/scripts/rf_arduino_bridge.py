#!/usr/bin/env python
import rospy
from std_msgs.msg import UInt16
import serial

def parse_serial(ser):
    # define the publishers(IMU, GPS, ch1-ch6)
    ch1_pub = rospy.Publisher('ch1', UInt16, queue_size=10)
    ch2_pub = rospy.Publisher('ch2', UInt16, queue_size=10)
    ch3_pub = rospy.Publisher('ch3', UInt16, queue_size=10)
    ch4_pub = rospy.Publisher('ch4', UInt16, queue_size=10)
    ch5_pub = rospy.Publisher('ch5', UInt16, queue_size=10)
    ch6_pub = rospy.Publisher('ch6', UInt16, queue_size=10)
    

    rospy.init_node('rf_arduino_bridge', anonymous=True)
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        try:
            # read the line from the arduino, parse the data for each message
            line = ser.readline()
            # ser.write("received\n".encode())
            rospy.loginfo(line)

            data = line.strip().split('\t')

            ch1_pub.publish(int(float(data[0])))
            ch2_pub.publish(int(float(data[1])))
            ch3_pub.publish(int(float(data[2])))
            ch4_pub.publish(int(float(data[3])))
            ch5_pub.publish(int(float(data[4])))
            ch6_pub.publish(int(float(data[5])))
        except:
            continue

        rate.sleep()

if __name__ == '__main__':
    # initialize the serial connection to the arduino
    ser = serial.Serial()
    ser.baudrate = 115200
    ser.port = "usb-Adafruit_Adafruit_Metro_328_ADAOLANOp-if00-port0"
    ser.timeout = 1
    ser.open()

    try:
        parse_serial(ser)
    except rospy.ROSInterruptException:
        ser.close()

