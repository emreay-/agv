#!/usr/bin/env python
import rospy
from agv.msg import int16Array
import struct
import serial
from std_msgs.msg import Char

def get_rpm():
    pub = rospy.Publisher('encoder_values', int16Array)
    rospy.init_node('agvGet', anonymous=True)
    rate = rospy.Rate(20) # 20hz
    while not rospy.is_shutdown():
        values = port.read(20)
	for x in range(0,12):
		if values[x] == "#" and values[x+5] == "!":
			left_enc = (ord(values[x+1])+(ord(values[x+2])*256))
			if left_enc > 32767:
				left_enc = left_enc - 65536
			right_enc = (ord(values[x+3])+(ord(values[x+4])*256))
			if right_enc > 32767:
				right_enc = right_enc - 65536
			msg = (35, left_enc/100, right_enc/100, 33)
			rospy.loginfo(msg)
			pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    port = serial.Serial("/dev/ttyUSB1", baudrate = 115200, bytesize = 8, stopbits = 1, timeout = 0.5)
    port.open()
    get_rpm()
