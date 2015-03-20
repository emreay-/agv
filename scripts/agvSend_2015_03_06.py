#!/usr/bin/env python
import rospy
from agv.msg import uint8Array
from agv.msg import uint8ArrayDim
from std_msgs.msg import String
import struct
import serial

def agvSendTest():
    rospy.init_node('agvSendTest', anonymous=True)
    rospy.Subscriber("wheel_values", uint8Array, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

def callback(data):
    pub = rospy.Publisher('encoder',uint8ArrayDim)
    rospy.loginfo(data.data)
    port.write(data.data)
    temp = port.read(4)
    pub.publish(temp)

if __name__ == '__main__':
    port = serial.Serial("/dev/ttyUSB0", baudrate = 115200, bytesize = 8, stopbits = 1, timeout = 0.001)
    port.open()
    agvSendTest()
