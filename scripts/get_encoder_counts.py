#!/usr/bin/env python
import rospy
from agv.msg import int16Array
import struct
import serial
from std_msgs.msg import Char

def get_encoder_counts():
    pub = rospy.Publisher('encoder_values', int16Array)
    rospy.init_node('agvGet', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        values = port.read(30)
	for x in range(0,20):
		if values[x] == "#" and values[x+9] == "!":
			left_enc = (ord(values[x+1])+(ord(values[x+2])*256)+(ord(values[x+3])*65536)+(ord(values[x+4])*65536*256))
			if left_enc > 2147483647:
				left_enc = left_enc - 4294967295
			right_enc = (ord(values[x+5])+(ord(values[x+6])*256)+(ord(values[x+7])*65536)+(ord(values[x+8])*65536*256))
			if right_enc > 2147483647:
				right_enc = right_enc - 4294967295
			msg = (35, left_enc/400, right_enc/400, 33)
			rospy.loginfo(msg)
			pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    port = serial.Serial("/dev/ttyUSB1", baudrate = 115200, bytesize = 8, stopbits = 1, timeout = 2)
    port.open()
    get_encoder_counts()
