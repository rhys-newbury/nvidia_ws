#!/usr/bin/env python
import rospy
import tf
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

debug = rospy.Publisher("/debug", String, queue_size=10)

def callback(data):
	debug.publish(str(data))

if __name__ == '__main__':
    rospy.init_node('tf_broadcaster')
    rospy.Subscriber('/scan', LaserScan, callback)
    rospy.spin()
