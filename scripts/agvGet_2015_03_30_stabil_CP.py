#!/usr/bin/env python
import rospy
from agv.msg import int16Array
import struct
import serial
from std_msgs.msg import Char

def agvGet():
    pub = rospy.Publisher('encoder_values', Char)
    rospy.init_node('agvGet', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        values = port.read(6)
	for x in range(0,6):
		print(ord(values[x]))        
		rospy.loginfo(ord(values[x]))
        	pub.publish(ord(values[x]))
        rate.sleep()

if __name__ == '__main__':
    port = serial.Serial("/dev/ttyUSB2", baudrate = 115200, bytesize = 8, stopbits = 1, timeout = 2)
    port.open()
    agvGet()
