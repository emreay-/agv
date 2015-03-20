#!/usr/bin/env python
# license removed for brevity
import rospy
from agv.msg import uint8Array

def publishValue(value, pub):
    data = uint8Array()
    data.data = value
    rospy.loginfo(data)
    pub.publish(data)


def wheelPubTest():
    pub = rospy.Publisher('wheel_test_values', uint8Array)
    rospy.init_node('wheelPubTest', anonymous=True)
    rate = rospy.Rate(5) # 10hz
    while not rospy.is_shutdown():
	values = (35,250,0,250,0,250,0,250,0,33)
	values2 = (35,250,0,250,0,0,0,250,0,33)
	for x in range(0, 10):        
		publishValue(values,pub)
		rate.sleep()

	for x in range(0, 10):        
		publishValue(values2,pub)
		rate.sleep()

        rate.sleep()

if __name__ == '__main__':
    try:
        wheelPubTest()
    except rospy.ROSInterruptException:
        pass
