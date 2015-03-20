#!/usr/bin/env python
import rospy
from agv.msg import uint8Array
import struct
import serial

def callback(data):
    rospy.loginfo(data.data)
    port = serial.Serial("/dev/ttyUSB0", baudrate = 115200, bytesize = 8, stopbits = 1, timeout = 2)
    port.open()
    port.write(data.data)
    port.close()
    
def agvSendTest():
    rospy.init_node('agvSendTest', anonymous=True)
    rospy.Subscriber("wheel_test_values", uint8Array, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    agvSendTest()
